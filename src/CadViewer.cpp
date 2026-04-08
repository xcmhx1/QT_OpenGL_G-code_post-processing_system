#include "pch.h"

// CadViewer 实现文件
// 实现 CadViewer 模块，对应头文件中声明的主要行为和协作流程。
// CAD 主视图模块，负责 OpenGL 生命周期、输入接入、场景刷新和信号分发。

#include "CadViewer.h"

// CAD 模块内部依赖
#include "CadCrosshairBuilder.h"
#include "CadDocument.h"
#include "CadEntityPicker.h"
#include "CadEntityRenderer.h"
#include "CadInteractionConstants.h"
#include "CadItem.h"
#include "CadProcessVisualUtils.h"
#include "CadPreviewBuilder.h"
#include "CadViewTransform.h"

// Qt 核心模块
#include <QDragEnterEvent>
#include <QElapsedTimer>
#include <QDebug>
#include <QFont>
#include <QFontMetrics>
#include <QKeyEvent>
#include <QMimeData>
#include <QPainter>
#include <QSurfaceFormat>
#include <QWheelEvent>

// 标准库
#include <cmath>
#include <vector>

// 匿名命名空间，存放局部常量和辅助函数
namespace
{
    // 是否启用视图性能日志记录
    constexpr bool kEnableViewerPerfLogging = false;
    // 慢帧阈值（毫秒），超过此值会记录日志
    constexpr qint64 kSlowFrameThresholdMs = 16;

    // 检查是否为支持拖放的文件类型
    // @param localFile 本地文件路径
    // @return 如果文件类型受支持则返回 true
    bool isSupportedDropFile(const QString& localFile)
    {
        return localFile.endsWith(QStringLiteral(".dxf"), Qt::CaseInsensitive)
            || localFile.endsWith(QStringLiteral(".dwg"), Qt::CaseInsensitive)
            || localFile.endsWith(QStringLiteral(".bmp"), Qt::CaseInsensitive)
            || localFile.endsWith(QStringLiteral(".png"), Qt::CaseInsensitive)
            || localFile.endsWith(QStringLiteral(".jpg"), Qt::CaseInsensitive)
            || localFile.endsWith(QStringLiteral(".jpeg"), Qt::CaseInsensitive);
    }

    QVector3D buildPerpendicularDirection(const QVector3D& direction)
    {
        QVector3D perpendicular(-direction.y(), direction.x(), 0.0f);

        if (!qFuzzyIsNull(perpendicular.lengthSquared()))
        {
            perpendicular.normalize();
        }

        return perpendicular;
    }
}

// 构造函数：
// 配置 OpenGL 4.5 Core Profile，并启用鼠标跟踪与键盘焦点。
CadViewer::CadViewer(QWidget* parent)
    : QOpenGLWidget(parent)
{
    // 配置 OpenGL 表面格式
    QSurfaceFormat surfaceFormat = format();
    surfaceFormat.setRenderableType(QSurfaceFormat::OpenGL);
    surfaceFormat.setVersion(4, 5);
    surfaceFormat.setProfile(QSurfaceFormat::CoreProfile);
    surfaceFormat.setDepthBufferSize(24);
    surfaceFormat.setStencilBufferSize(8);
    setFormat(surfaceFormat);

    // 启用鼠标跟踪
    setMouseTracking(true);
    // 启用拖放
    setAcceptDrops(true);
    // 设置焦点策略
    setFocusPolicy(Qt::StrongFocus);
    // 设置光标样式
    ensureBlankCursor();

    // 设置控制器与当前视图的关联
    m_controller.setViewer(this);
    m_graphicsCoordinator.setTheme(m_theme);
}

// 析构时释放 OpenGL 资源。
// 需要先 makeCurrent()，确保当前上下文有效。
CadViewer::~CadViewer()
{
    if (context() != nullptr)
    {
        makeCurrent();
        m_sceneCoordinator.clearAllBuffers();
        m_graphicsCoordinator.destroy();
        doneCurrent();
    }
}

// 设置当前文档：
// - 清除选择状态
// - 标记缓冲需重建
// - 更新场景包围盒
// - 若 OpenGL 已初始化则立即重建 GPU 缓冲
// - 最后执行 fitScene()
// @param document 要设置的文档指针
void CadViewer::setDocument(CadDocument* document)
{
    // 绑定文档到场景协调器
    m_sceneCoordinator.bindDocument(document, this, &CadViewer::handleDocumentSceneChanged);
    // 清除选中实体
    setSelectedEntityId(0);

    // 如果图形协调器已初始化，则立即重建缓冲
    if (m_graphicsCoordinator.isInitialized())
    {
        makeCurrent();
        rebuildAllBuffers();
        doneCurrent();
    }

    // 适配场景
    fitScene();
    // 请求更新
    update();
}

// 设置当前编辑器
// @param editer 编辑器指针
void CadViewer::setEditer(CadEditer* editer)
{
    m_controller.setEditer(editer);
}

void CadViewer::setDefaultDrawingProperties(const QString& layerName, const QColor& color, int colorIndex)
{
    m_controller.setDefaultDrawingProperties(layerName, color, colorIndex);
    refreshCommandPrompt();
}

void CadViewer::setTheme(const AppThemeColors& theme)
{
    m_theme = theme;
    m_graphicsCoordinator.setTheme(theme);
    update();
}

void CadViewer::startDrawing(DrawType drawType)
{
    m_controller.beginDrawing(drawType, m_controller.drawState().drawingColor);
    update();
}

bool CadViewer::startMoveSelected()
{
    const bool handled = m_controller.beginMoveSelected();

    if (handled)
    {
        update();
    }

    return handled;
}

// 适配整个场景，并回到 2D 平面视图。
void CadViewer::fitScene()
{
    // 刷新场景边界
    m_sceneCoordinator.refreshBounds();

    // 如果没有有效边界，则返回
    if (!m_sceneCoordinator.hasBounds())
    {
        return;
    }

    // 相机适配整个场景
    m_camera.fitAll(m_sceneCoordinator.minPoint(), m_sceneCoordinator.maxPoint(), aspectRatio());
    // 重置视图交互控制器
    m_viewInteractionController.resetForFitScene();
    // 重置控制器
    m_controller.reset();
    // 请求更新
    update();
}

// 开始轨道旋转交互
void CadViewer::beginOrbitInteraction()
{
    m_viewInteractionController.beginOrbitInteraction(m_camera);
    update();
}

// 开始平移交互
void CadViewer::beginPanInteraction()
{
    m_viewInteractionController.beginPanInteraction();
    update();
}

// 更新轨道旋转交互
// @param screenDelta 屏幕坐标增量
void CadViewer::updateOrbitInteraction(const QPoint& screenDelta)
{
    m_viewInteractionController.updateOrbitInteraction
    (
        m_camera,
        m_sceneCoordinator.orbitCenter(),
        m_sceneCoordinator.hasBounds(),
        screenDelta
    );
    update();
}

// 更新平移交互
// @param screenDelta 屏幕坐标增量
void CadViewer::updatePanInteraction(const QPoint& screenDelta)
{
    m_viewInteractionController.updatePanInteraction(m_camera, pixelToWorldScale(), screenDelta);
    update();
}

// 结束视图交互
void CadViewer::endViewInteraction()
{
    m_viewInteractionController.endViewInteraction();
}

// 在屏幕位置选择实体
// @param screenPos 屏幕坐标点
void CadViewer::selectEntityAt(const QPoint& screenPos)
{
    setSelectedEntityId(pickEntity(screenPos));
    update();
}

// 在指定屏幕位置缩放
// @param screenPos 屏幕坐标点
// @param factor 缩放因子
void CadViewer::zoomAtScreenPosition(const QPoint& screenPos, float factor)
{
    // 计算屏幕坐标对应的近平面和远平面点
    const QVector3D nearPoint = screenToWorld(screenPos, -1.0f);
    const QVector3D farPoint = screenToWorld(screenPos, 1.0f);
    const QVector3D rayDirection = farPoint - nearPoint;
    const QVector3D planeNormal = m_camera.forwardDirection();

    // 默认锚点为相机目标点
    QVector3D anchor = m_camera.target;
    const float denominator = QVector3D::dotProduct(rayDirection, planeNormal);

    // 如果分母不为零，计算视线与相机平面的交点
    if (!qFuzzyIsNull(denominator))
    {
        const float t = QVector3D::dotProduct(m_camera.target - nearPoint, planeNormal) / denominator;
        anchor = nearPoint + rayDirection * t;
    }

    // 在交点处进行缩放
    m_camera.zoomAtPoint(factor, anchor);
    update();
}

// 重置到顶视图
void CadViewer::resetToTopView()
{
    m_viewInteractionController.resetToTopView(m_camera);
    update();
}

// 适配场景视图
void CadViewer::fitSceneView()
{
    fitScene();
}

// 获取当前视图模式
// @return 相机视图模式
CameraViewMode CadViewer::viewMode() const
{
    return m_viewInteractionController.viewMode();
}

// 获取当前交互模式
// @return 视图交互模式
ViewInteractionMode CadViewer::interactionMode() const
{
    return m_viewInteractionController.interactionMode();
}

// 是否应该忽略下一次轨道旋转增量
// @return 如果应该忽略则返回 true
bool CadViewer::shouldIgnoreNextOrbitDelta() const
{
    return m_viewInteractionController.shouldIgnoreNextOrbitDelta();
}

// 消耗忽略下一次轨道旋转增量的标志
void CadViewer::consumeIgnoreNextOrbitDelta()
{
    m_viewInteractionController.consumeIgnoreNextOrbitDelta();
}

// 请求视图更新
void CadViewer::requestViewUpdate()
{
    update();
}

// 获取当前选中的实体
// @return 选中实体指针，如果没有选中则返回 nullptr
CadItem* CadViewer::selectedEntity() const
{
    return findEntityById(m_selectedEntityId);
}

// 追加命令消息
// @param message 消息内容
void CadViewer::appendCommandMessage(const QString& message)
{
    if (!message.trimmed().isEmpty())
    {
        emit commandMessageAppended(message.trimmed());
    }
}

// 刷新命令提示
void CadViewer::refreshCommandPrompt()
{
    emit commandPromptChanged(m_controller.currentPrompt());
}

// 放大视图。
// @param factor 缩放因子，默认为 1.4
void CadViewer::zoomIn(float factor)
{
    m_camera.zoom(factor);
    update();
}

// 缩小视图。
// @param factor 缩放因子，默认为 1.4
void CadViewer::zoomOut(float factor)
{
    if (factor <= 0.0f)
    {
        return;
    }

    m_camera.zoom(1.0f / factor);
    update();
}

// 屏幕坐标 -> 世界坐标。
// depth 取值使用 NDC 空间：-1 近平面，+1 远平面。
// @param screenPos 屏幕坐标点
// @param depth NDC深度，默认为0
// @return 对应的世界坐标
QVector3D CadViewer::screenToWorld(const QPoint& screenPos, float depth) const
{
    return CadViewTransform::screenToWorld(m_camera, m_viewportWidth, m_viewportHeight, screenPos, depth);
}

// 屏幕坐标转地面平面坐标
// @param screenPos 屏幕坐标点
// @return 对应的地面平面坐标
QVector3D CadViewer::screenToGroundPlane(const QPoint& screenPos) const
{
    return CadViewTransform::screenToGroundPlane(m_camera, m_viewportWidth, m_viewportHeight, screenPos);
}

// 世界坐标 -> 屏幕坐标。
// @param worldPos 世界坐标
// @return 对应的屏幕坐标
QPoint CadViewer::worldToScreen(const QVector3D& worldPos) const
{
    return CadViewTransform::worldToScreen(m_camera, m_viewportWidth, m_viewportHeight, worldPos);
}

// OpenGL 初始化：
// - 初始化函数表
// - 设置基本渲染状态
// - 创建 shader / 网格 / 轴 / 轨道标记缓冲
// - 重建实体缓冲并适配场景
void CadViewer::initializeGL()
{
    // 初始化 OpenGL 函数
    initializeOpenGLFunctions();

    // 初始化图形协调器
    m_graphicsCoordinator.setTheme(m_theme);
    m_graphicsCoordinator.initialize();
    // 重建所有缓冲
    rebuildAllBuffers();
    // 适配场景
    fitScene();
    // 刷新命令提示
    refreshCommandPrompt();
}

// Qt 回调的逻辑尺寸变化。
// 实际绘制时 paintGL 还会按 devicePixelRatioF() 换算 framebuffer 尺寸。
// @param w 新宽度
// @param h 新高度
void CadViewer::resizeGL(int w, int h)
{
    m_viewportWidth = std::max(1, w);
    m_viewportHeight = std::max(1, h);
}

// 主绘制入口：
// 1. 设置视口
// 2. 清屏
// 3. 若缓冲脏则重建
// 4. 绘制网格、实体、坐标轴、轨道中心标记
void CadViewer::paintGL()
{
    // 性能计时器
    [[maybe_unused]] QElapsedTimer frameTimer;

    if constexpr (kEnableViewerPerfLogging)
    {
        frameTimer.start();
    }

    // 计算帧缓冲尺寸（考虑设备像素比）
    const int framebufferWidth = std::max(1, static_cast<int>(std::round(width() * devicePixelRatioF())));
    const int framebufferHeight = std::max(1, static_cast<int>(std::round(height() * devicePixelRatioF())));

    glClearColor
    (
        m_theme.viewerBackgroundColor.redF(),
        m_theme.viewerBackgroundColor.greenF(),
        m_theme.viewerBackgroundColor.blueF(),
        1.0f
    );

    // 准备帧
    m_graphicsCoordinator.prepareFrame(framebufferWidth, framebufferHeight);

    // 如果图形协调器未初始化，则返回
    if (!m_graphicsCoordinator.isInitialized())
    {
        return;
    }

    // 如果缓冲脏，则重建
    if (m_sceneCoordinator.buffersDirty())
    {
        rebuildAllBuffers();
    }

    // 计算视图投影矩阵
    const QMatrix4x4 viewProjection = m_camera.viewProjectionMatrix(aspectRatio());

    // 渲染各个组件
    renderGrid(viewProjection);
    renderEntities(viewProjection);
    renderTransientPrimitives(viewProjection);
    renderAxis(viewProjection);
    renderOrbitMarker(viewProjection);
    renderProcessOrderLabels();

    // 性能日志记录
    if constexpr (kEnableViewerPerfLogging)
    {
        const qint64 elapsedMs = frameTimer.elapsed();

        if (elapsedMs >= kSlowFrameThresholdMs)
        {
            qDebug() << "CadViewer::paintGL slow frame:" << elapsedMs << "ms";
        }
    }
}

// 鼠标按下事件处理：
// - 中键 + Shift：进入轨道旋转
// - 中键：进入平移
// - 左键：执行拾取
// @param event 鼠标事件
void CadViewer::mousePressEvent(QMouseEvent* event)
{
    ensureBlankCursor();
    // 记录光标位置
    m_cursorScreenPos = event->pos();
    // 更新十字准线显示状态
    m_showCrosshairOverlay = rect().contains(event->pos());

    // 尝试由控制器处理鼠标按下事件
    if (!m_controller.handleMousePress(event))
    {
        // 如果控制器未处理，则传递给父类
        QOpenGLWidget::mousePressEvent(event);
    }

    // 更新悬停世界位置
    updateHoveredWorldPosition(event->pos());
    // 刷新命令提示
    refreshCommandPrompt();
    // 请求更新
    update();
}

// 鼠标移动事件处理
// @param event 鼠标事件
void CadViewer::mouseMoveEvent(QMouseEvent* event)
{
    ensureBlankCursor();
    m_cursorScreenPos = event->pos();
    m_showCrosshairOverlay = rect().contains(event->pos());

    if (!m_controller.handleMouseMove(event))
    {
        QOpenGLWidget::mouseMoveEvent(event);
    }

    // 平移/轨道观察时优先保证视图响应，不在每次 move 上额外做坐标投影和状态栏刷新。
    if (interactionMode() == ViewInteractionMode::Idle)
    {
        updateHoveredWorldPosition(event->pos());
    }

    update();
}

// 鼠标离开事件处理
// @param event 离开事件
void CadViewer::leaveEvent(QEvent* event)
{
    m_showCrosshairOverlay = false;
    update();
    QOpenGLWidget::leaveEvent(event);
}

void CadViewer::enterEvent(QEnterEvent* event)
{
    ensureBlankCursor();
    m_showCrosshairOverlay = true;
    QOpenGLWidget::enterEvent(event);
    update();
}

void CadViewer::focusInEvent(QFocusEvent* event)
{
    ensureBlankCursor();
    QOpenGLWidget::focusInEvent(event);
}

// 鼠标释放中键后退出当前交互模式。
// @param event 鼠标事件
void CadViewer::mouseReleaseEvent(QMouseEvent* event)
{
    ensureBlankCursor();
    m_cursorScreenPos = event->pos();
    m_showCrosshairOverlay = rect().contains(event->pos());

    if (!m_controller.handleMouseRelease(event))
    {
        QOpenGLWidget::mouseReleaseEvent(event);
    }

    updateHoveredWorldPosition(event->pos());
    refreshCommandPrompt();
    update();
}

// 滚轮缩放：
// 1. 通过 screenToWorld 求出鼠标位置对应的近远平面点
// 2. 构造一条视线
// 3. 与经过 target、法向为 forward 的相机平面求交
// 4. 以该交点为锚点做缩放
// @param event 滚轮事件
void CadViewer::wheelEvent(QWheelEvent* event)
{
    ensureBlankCursor();
    if (!m_controller.handleWheel(event))
    {
        QOpenGLWidget::wheelEvent(event);
    }

    updateHoveredWorldPosition(event->position().toPoint());
}

// 键盘快捷键处理：
// F / Fit：适配场景
// T / Home：回二维顶视图
// +/-：缩放
// @param event 键盘事件
void CadViewer::keyPressEvent(QKeyEvent* event)
{
    ensureBlankCursor();
    if (!m_controller.handleKeyPress(event))
    {
        QOpenGLWidget::keyPressEvent(event);
    }

    refreshCommandPrompt();
    update();
}

// 拖拽进入事件处理
// @param event 拖拽事件
void CadViewer::dragEnterEvent(QDragEnterEvent* event)
{
    const QMimeData* mimeData = event->mimeData();

    if (mimeData == nullptr || !mimeData->hasUrls())
    {
        event->ignore();
        return;
    }

    // 检查所有 URL 是否包含支持的文件
    for (const QUrl& url : mimeData->urls())
    {
        const QString localFile = url.toLocalFile();

        if (isSupportedDropFile(localFile))
        {
            event->acceptProposedAction();
            return;
        }
    }

    event->ignore();
}

// 拖放事件处理
// @param event 拖放事件
void CadViewer::dropEvent(QDropEvent* event)
{
    const QMimeData* mimeData = event->mimeData();

    if (mimeData == nullptr || !mimeData->hasUrls())
    {
        event->ignore();
        return;
    }

    for (const QUrl& url : mimeData->urls())
    {
        const QString localFile = url.toLocalFile();

        if (isSupportedDropFile(localFile))
        {
            emit fileDropRequested(localFile);
            event->acceptProposedAction();
            return;
        }
    }

    event->ignore();
}

// 清空并重建所有实体缓冲。
// 常在切换文档或场景数据变化后调用。
void CadViewer::rebuildAllBuffers()
{
    m_sceneCoordinator.ensureGpuBuffersReady(m_graphicsCoordinator.isInitialized());
}

void CadViewer::ensureBlankCursor()
{
    if (cursor().shape() != Qt::BlankCursor)
    {
        setCursor(Qt::BlankCursor);
    }
}

// 绘制背景网格。
// 网格不参与深度测试，始终作为背景参考显示。
// @param viewProjection 视图投影矩阵
void CadViewer::renderGrid(const QMatrix4x4& viewProjection)
{
    if (!m_graphicsCoordinator.isInitialized())
    {
        return;
    }

    m_graphicsCoordinator.renderGrid(viewProjection);
}

// 绘制三轴：
// - X 红
// - Y 绿
// - Z 蓝
// 其中 Z 轴可根据 axesSwapped 状态切换实线/虚线。
// @param viewProjection 视图投影矩阵
void CadViewer::renderAxis(const QMatrix4x4& viewProjection)
{
    if (!m_graphicsCoordinator.isInitialized())
    {
        return;
    }

    m_graphicsCoordinator.renderAxis(viewProjection, m_camera.axesSwapped());
}

// 绘制所有实体。
// 当前实体数据默认已是世界坐标，因此直接用 VP 变换。
// 若实体被选中，则使用高亮色和更大的点尺寸。
// @param viewProjection 视图投影矩阵
void CadViewer::renderEntities(const QMatrix4x4& viewProjection)
{
    CadDocument* scene = m_sceneCoordinator.document();

    if (scene == nullptr || !m_graphicsCoordinator.isInitialized())
    {
        return;
    }

    // 渲染实体
    CadEntityRenderer::renderEntities
    (
        m_graphicsCoordinator.generalShader(),
        viewProjection,
        scene->m_entities,
        m_sceneCoordinator.renderCache(),
        m_selectedEntityId,
        m_theme
    );
}

// 绘制轨道旋转中心标记。
// 仅在"正在轨道旋转"且场景包围盒有效时显示。
// @param viewProjection 视图投影矩阵
void CadViewer::renderOrbitMarker(const QMatrix4x4& viewProjection)
{
    if (!m_graphicsCoordinator.isInitialized())
    {
        return;
    }

    m_graphicsCoordinator.renderOrbitMarker
    (
        viewProjection,
        m_sceneCoordinator.orbitCenter(),
        m_viewInteractionController.orbitMarkerVisible(m_sceneCoordinator.hasBounds())
    );
}

// 渲染临时图元，包括命令预览和十字准线
// @param viewProjection 视图投影矩阵
void CadViewer::renderTransientPrimitives(const QMatrix4x4& viewProjection)
{
    if (!m_graphicsCoordinator.isInitialized())
    {
        return;
    }

    // 视图平移/轨道观察时暂停 overlay 与命令预览构建，优先保证拖拽流畅度。
    if (interactionMode() != ViewInteractionMode::Idle)
    {
        return;
    }

    std::vector<TransientPrimitive> processPrimitives = buildProcessDirectionPrimitives();
    // 构建命令预览图元
    const std::vector<TransientPrimitive> commandPrimitives = buildTransientPrimitives();
    // 构建十字准线图元
    const std::vector<TransientPrimitive> crosshairPrimitives = CadCrosshairBuilder::buildCrosshairPrimitives
    (
        m_camera,
        m_viewportWidth,
        m_viewportHeight,
        width(),
        height(),
        m_cursorScreenPos,
        m_showCrosshairOverlay,
        m_viewInteractionController.crosshairSuppressed(),
        m_crosshairPlaneZ,
        CadInteractionConstants::kPickBoxHalfSizePixels * pixelToWorldScale(),
        CadInteractionConstants::kCrosshairHalfLengthWorld
    );

    // 如果没有临时图元，则返回
    processPrimitives.insert(processPrimitives.end(), commandPrimitives.begin(), commandPrimitives.end());

    if (processPrimitives.empty() && crosshairPrimitives.empty())
    {
        return;
    }

    // 渲染临时图元
    m_graphicsCoordinator.renderTransientPrimitives
    (
        viewProjection,
        processPrimitives,
        crosshairPrimitives
    );
}

void CadViewer::renderProcessOrderLabels()
{
    CadDocument* scene = m_sceneCoordinator.document();

    if (scene == nullptr || interactionMode() != ViewInteractionMode::Idle)
    {
        return;
    }

    struct ProcessOrderLabel
    {
        int order = -1;
        bool selected = false;
        QPoint center;
        QString text;
    };

    std::vector<ProcessOrderLabel> labels;
    labels.reserve(scene->m_entities.size());

    const CadItem* selectedItem = selectedEntity();

    for (const std::unique_ptr<CadItem>& entity : scene->m_entities)
    {
        if (entity == nullptr)
        {
            continue;
        }

        const CadProcessVisualInfo info = buildProcessVisualInfo(entity.get());

        if (!info.valid || info.processOrder < 0)
        {
            continue;
        }

        const QPoint screenPoint = worldToScreen(info.labelAnchor);

        if (screenPoint.x() < -24 || screenPoint.x() > width() + 24
            || screenPoint.y() < -24 || screenPoint.y() > height() + 24)
        {
            continue;
        }

        ProcessOrderLabel label;
        label.order = info.processOrder;
        label.selected = entity.get() == selectedItem;
        label.center = screenPoint;
        label.text = QString::number(info.processOrder + 1);
        labels.push_back(std::move(label));
    }

    if (labels.empty())
    {
        return;
    }

    std::sort
    (
        labels.begin(),
        labels.end(),
        [](const ProcessOrderLabel& left, const ProcessOrderLabel& right)
        {
            if (left.selected != right.selected)
            {
                return left.selected && !right.selected;
            }

            return left.order < right.order;
        }
    );

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setRenderHint(QPainter::TextAntialiasing, true);

    QFont labelFont = painter.font();
    labelFont.setPointSize(9);
    labelFont.setBold(true);
    painter.setFont(labelFont);

    const QFontMetrics metrics(labelFont);
    std::vector<QRect> occupiedRects;
    occupiedRects.reserve(labels.size());

    for (const ProcessOrderLabel& label : labels)
    {
        const QRect textRect = metrics.boundingRect(label.text);
        QRect bubbleRect
        (
            label.center.x() - textRect.width() / 2 - 7,
            label.center.y() - textRect.height() / 2 - 4,
            textRect.width() + 14,
            textRect.height() + 8
        );

        if (!label.selected)
        {
            bool overlaps = false;

            for (const QRect& occupiedRect : occupiedRects)
            {
                if (bubbleRect.adjusted(-2, -2, 2, 2).intersects(occupiedRect))
                {
                    overlaps = true;
                    break;
                }
            }

            if (overlaps)
            {
                continue;
            }
        }

        occupiedRects.push_back(bubbleRect);

        const QColor fillColor = label.selected ? m_theme.selectedProcessLabelFillColor : m_theme.processLabelFillColor;
        const QColor borderColor = label.selected ? m_theme.selectedProcessLabelBorderColor : m_theme.processLabelBorderColor;
        const QColor textColor = label.selected ? m_theme.selectedProcessLabelTextColor : m_theme.processLabelTextColor;

        painter.setPen(QPen(borderColor, 1.0));
        painter.setBrush(fillColor);
        painter.drawRoundedRect(bubbleRect, 6.0, 6.0);
        painter.setPen(textColor);
        painter.drawText(bubbleRect, Qt::AlignCenter, label.text);
    }
}

// 处理文档场景变化
void CadViewer::handleDocumentSceneChanged()
{
    // 标记缓冲脏
    m_sceneCoordinator.markBuffersDirty();
    // 刷新场景边界
    m_sceneCoordinator.refreshBounds();

    // 如果选中的实体已不存在，则清除选中状态
    if (findEntityById(m_selectedEntityId) == nullptr)
    {
        setSelectedEntityId(0);
    }

    // 如果图形协调器已初始化，则重建缓冲
    if (m_graphicsCoordinator.isInitialized())
    {
        makeCurrent();
        rebuildAllBuffers();
        doneCurrent();
    }

    update();
}

// 更新悬停世界位置
// @param screenPos 屏幕坐标
void CadViewer::updateHoveredWorldPosition(const QPoint& screenPos)
{
    emit hoveredWorldPositionChanged(screenToGroundPlane(screenPos));
}

void CadViewer::setSelectedEntityId(EntityId entityId)
{
    if (m_selectedEntityId == entityId)
    {
        return;
    }

    m_selectedEntityId = entityId;
    emit selectedEntityChanged(selectedEntity());
}

// 屏幕空间拾取：
// - 点实体：测鼠标点到投影点距离
// - 线实体：测鼠标点到各投影线段距离
// 返回距离最近且在阈值内的实体 ID。
// @param screenPos 屏幕坐标
// @return 命中的实体ID，0 表示未命中
EntityId CadViewer::pickEntity(const QPoint& screenPos) const
{
    CadDocument* scene = m_sceneCoordinator.document();

    if (scene == nullptr)
    {
        return 0;
    }

    return CadEntityPicker::pickEntity
    (
        scene->m_entities,
        m_camera.viewProjectionMatrix(aspectRatio()),
        m_viewportWidth,
        m_viewportHeight,
        screenPos,
        CadInteractionConstants::kPickBoxHalfSizePixels
    );
}

// 根据 ID 查找实体
// @param id 实体ID
// @return 实体指针，nullptr 表示未找到
CadItem* CadViewer::findEntityById(EntityId id) const
{
    return m_sceneCoordinator.findEntityById(id);
}

// 构建临时图元列表
// @return 临时图元列表
std::vector<TransientPrimitive> CadViewer::buildTransientPrimitives() const
{
    return CadPreviewBuilder::buildTransientPrimitives(m_controller.drawState(), selectedEntity());
}

std::vector<TransientPrimitive> CadViewer::buildProcessDirectionPrimitives() const
{
    CadDocument* scene = m_sceneCoordinator.document();

    if (scene == nullptr)
    {
        return {};
    }

    std::vector<TransientPrimitive> primitives;
    primitives.reserve(scene->m_entities.size());

    const CadItem* selectedItem = selectedEntity();
    const float pixelScale = std::max(pixelToWorldScale(), 1.0e-4f);
    const float shaftLength = pixelScale * 30.0f;
    const float wingLength = pixelScale * 12.0f;

    for (const std::unique_ptr<CadItem>& entity : scene->m_entities)
    {
        if (entity == nullptr)
        {
            continue;
        }

        const CadProcessVisualInfo info = buildProcessVisualInfo(entity.get());

        if (!info.valid || info.direction.lengthSquared() <= 1.0e-6f)
        {
            continue;
        }

        const QVector3D perpendicular = buildPerpendicularDirection(info.direction);

        if (perpendicular.lengthSquared() <= 1.0e-6f)
        {
            continue;
        }

        const QVector3D shaftStart = info.startPoint;
        const QVector3D shaftEnd = shaftStart + info.direction * shaftLength;
        const QVector3D wingBase = shaftEnd - info.direction * wingLength;
        const QVector3D wingOffset = perpendicular * (wingLength * 0.55f);

        TransientPrimitive primitive;
        primitive.primitiveType = GL_LINES;
        primitive.vertices =
        {
            shaftStart,
            shaftEnd,
            shaftEnd,
            wingBase + wingOffset,
            shaftEnd,
            wingBase - wingOffset
        };

        if (entity.get() == selectedItem)
        {
            primitive.color = QVector3D(1.0f, 0.78f, 0.30f);
        }
        else if (info.processOrder >= 0)
        {
            primitive.color = info.isReverse ? QVector3D(1.0f, 0.42f, 0.28f) : QVector3D(0.12f, 0.92f, 0.72f);
        }
        else
        {
            primitive.color = info.isReverse ? QVector3D(0.82f, 0.34f, 0.26f) : QVector3D(0.34f, 0.78f, 0.58f);
        }

        primitives.push_back(std::move(primitive));
    }

    return primitives;
}

// 计算当前视口宽高比
// @return 宽高比（宽度/高度）
float CadViewer::aspectRatio() const
{
    return CadViewTransform::aspectRatio(m_viewportWidth, m_viewportHeight);
}

// 将 1 个屏幕像素估算为多少世界单位。
// 当前基于正交投影 viewHeight 直接换算，适用于平移操作。
// @return 像素到世界的缩放比例
float CadViewer::pixelToWorldScale() const
{
    return CadViewTransform::pixelToWorldScale(m_camera, m_viewportHeight);
}
