#include "pch.h"

// 实现 CadViewer 模块，对应头文件中声明的主要行为和协作流程。
// CAD 主视图模块，负责 OpenGL 生命周期、输入接入、场景刷新和信号分发。
#include "CadViewer.h"

#include "CadCrosshairBuilder.h"
#include "CadDocument.h"
#include "CadEntityPicker.h"
#include "CadEntityRenderer.h"
#include "CadInteractionConstants.h"
#include "CadItem.h"
#include "CadPreviewBuilder.h"
#include "CadViewTransform.h"

#include <QDragEnterEvent>
#include <QElapsedTimer>
#include <QDebug>
#include <QKeyEvent>
#include <QMimeData>
#include <QSurfaceFormat>
#include <QWheelEvent>

#include <cmath>
#include <vector>

namespace
{
    constexpr bool kEnableViewerPerfLogging = false;
    constexpr qint64 kSlowFrameThresholdMs = 16;

    bool isSupportedDropFile(const QString& localFile)
    {
        return localFile.endsWith(QStringLiteral(".dxf"), Qt::CaseInsensitive)
            || localFile.endsWith(QStringLiteral(".dwg"), Qt::CaseInsensitive)
            || localFile.endsWith(QStringLiteral(".bmp"), Qt::CaseInsensitive)
            || localFile.endsWith(QStringLiteral(".png"), Qt::CaseInsensitive)
            || localFile.endsWith(QStringLiteral(".jpg"), Qt::CaseInsensitive)
            || localFile.endsWith(QStringLiteral(".jpeg"), Qt::CaseInsensitive);
    }
}

// 构造函数：
// 配置 OpenGL 4.5 Core Profile，并启用鼠标跟踪与键盘焦点。
CadViewer::CadViewer(QWidget* parent)
    : QOpenGLWidget(parent)
{
    QSurfaceFormat surfaceFormat = format();
    surfaceFormat.setRenderableType(QSurfaceFormat::OpenGL);
    surfaceFormat.setVersion(4, 5);
    surfaceFormat.setProfile(QSurfaceFormat::CoreProfile);
    surfaceFormat.setDepthBufferSize(24);
    surfaceFormat.setStencilBufferSize(8);
    setFormat(surfaceFormat);

    setMouseTracking(true);
    setAcceptDrops(true);
    setFocusPolicy(Qt::StrongFocus);
    setCursor(Qt::BlankCursor);

    m_controller.setViewer(this);
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
void CadViewer::setDocument(CadDocument* document)
{
    m_sceneCoordinator.bindDocument(document, this, &CadViewer::handleDocumentSceneChanged);
    m_selectedEntityId = 0;

    if (m_graphicsCoordinator.isInitialized())
    {
        makeCurrent();
        rebuildAllBuffers();
        doneCurrent();
    }

    fitScene();
    update();
}

void CadViewer::setEditer(CadEditer* editer)
{
    m_controller.setEditer(editer);
}

// 适配整个场景，并回到 2D 平面视图。
void CadViewer::fitScene()
{
    m_sceneCoordinator.refreshBounds();

    if (!m_sceneCoordinator.hasBounds())
    {
        return;
    }

    m_camera.fitAll(m_sceneCoordinator.minPoint(), m_sceneCoordinator.maxPoint(), aspectRatio());
    m_viewInteractionController.resetForFitScene();
    m_controller.reset();
    update();
}

void CadViewer::beginOrbitInteraction()
{
    m_viewInteractionController.beginOrbitInteraction(m_camera);
    update();
}

void CadViewer::beginPanInteraction()
{
    m_viewInteractionController.beginPanInteraction();
    update();
}

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

void CadViewer::updatePanInteraction(const QPoint& screenDelta)
{
    m_viewInteractionController.updatePanInteraction(m_camera, pixelToWorldScale(), screenDelta);
    update();
}

void CadViewer::endViewInteraction()
{
    m_viewInteractionController.endViewInteraction();
}

void CadViewer::selectEntityAt(const QPoint& screenPos)
{
    m_selectedEntityId = pickEntity(screenPos);
    update();
}

void CadViewer::zoomAtScreenPosition(const QPoint& screenPos, float factor)
{
    const QVector3D nearPoint = screenToWorld(screenPos, -1.0f);
    const QVector3D farPoint = screenToWorld(screenPos, 1.0f);
    const QVector3D rayDirection = farPoint - nearPoint;
    const QVector3D planeNormal = m_camera.forwardDirection();

    QVector3D anchor = m_camera.target;
    const float denominator = QVector3D::dotProduct(rayDirection, planeNormal);

    if (!qFuzzyIsNull(denominator))
    {
        const float t = QVector3D::dotProduct(m_camera.target - nearPoint, planeNormal) / denominator;
        anchor = nearPoint + rayDirection * t;
    }

    m_camera.zoomAtPoint(factor, anchor);
    update();
}

void CadViewer::resetToTopView()
{
    m_viewInteractionController.resetToTopView(m_camera);
    update();
}

void CadViewer::fitSceneView()
{
    fitScene();
}

CameraViewMode CadViewer::viewMode() const
{
    return m_viewInteractionController.viewMode();
}

ViewInteractionMode CadViewer::interactionMode() const
{
    return m_viewInteractionController.interactionMode();
}

bool CadViewer::shouldIgnoreNextOrbitDelta() const
{
    return m_viewInteractionController.shouldIgnoreNextOrbitDelta();
}

void CadViewer::consumeIgnoreNextOrbitDelta()
{
    m_viewInteractionController.consumeIgnoreNextOrbitDelta();
}

void CadViewer::requestViewUpdate()
{
    update();
}

CadItem* CadViewer::selectedEntity() const
{
    return findEntityById(m_selectedEntityId);
}

void CadViewer::appendCommandMessage(const QString& message)
{
    if (!message.trimmed().isEmpty())
    {
        emit commandMessageAppended(message.trimmed());
    }
}

void CadViewer::refreshCommandPrompt()
{
    emit commandPromptChanged(m_controller.currentPrompt());
}

// 放大视图。
void CadViewer::zoomIn(float factor)
{
    m_camera.zoom(factor);
    update();
}

// 缩小视图。
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
QVector3D CadViewer::screenToWorld(const QPoint& screenPos, float depth) const
{
    return CadViewTransform::screenToWorld(m_camera, m_viewportWidth, m_viewportHeight, screenPos, depth);
}

QVector3D CadViewer::screenToGroundPlane(const QPoint& screenPos) const
{
    return CadViewTransform::screenToGroundPlane(m_camera, m_viewportWidth, m_viewportHeight, screenPos);
}

// 世界坐标 -> 屏幕坐标。
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
    initializeOpenGLFunctions();

    m_graphicsCoordinator.initialize();
    rebuildAllBuffers();
    fitScene();
    refreshCommandPrompt();
}

// Qt 回调的逻辑尺寸变化。
// 实际绘制时 paintGL 还会按 devicePixelRatioF() 换算 framebuffer 尺寸。
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
    [[maybe_unused]] QElapsedTimer frameTimer;

    if constexpr (kEnableViewerPerfLogging)
    {
        frameTimer.start();
    }

    const int framebufferWidth = std::max(1, static_cast<int>(std::round(width() * devicePixelRatioF())));
    const int framebufferHeight = std::max(1, static_cast<int>(std::round(height() * devicePixelRatioF())));

    m_graphicsCoordinator.prepareFrame(framebufferWidth, framebufferHeight);

    if (!m_graphicsCoordinator.isInitialized())
    {
        return;
    }

    if (m_sceneCoordinator.buffersDirty())
    {
        rebuildAllBuffers();
    }

    const QMatrix4x4 viewProjection = m_camera.viewProjectionMatrix(aspectRatio());

    renderGrid(viewProjection);
    renderEntities(viewProjection);
    renderTransientPrimitives(viewProjection);
    renderAxis(viewProjection);
    renderOrbitMarker(viewProjection);

    if constexpr (kEnableViewerPerfLogging)
    {
        const qint64 elapsedMs = frameTimer.elapsed();

        if (elapsedMs >= kSlowFrameThresholdMs)
        {
            qDebug() << "CadViewer::paintGL slow frame:" << elapsedMs << "ms";
        }
    }
}

// 鼠标按下：
// - 中键 + Shift：进入轨道旋转
// - 中键：进入平移
// - 左键：执行拾取
void CadViewer::mousePressEvent(QMouseEvent* event)
{
    m_cursorScreenPos = event->pos();
    m_showCrosshairOverlay = rect().contains(event->pos());

    if (!m_controller.handleMousePress(event))
    {
        QOpenGLWidget::mousePressEvent(event);
    }

    updateHoveredWorldPosition(event->pos());
    refreshCommandPrompt();
    update();
}

// 鼠标移动：
// - 平移模式：按像素位移换算为世界位移
// - 旋转模式：围绕场景中心轨道旋转
void CadViewer::mouseMoveEvent(QMouseEvent* event)
{
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

void CadViewer::leaveEvent(QEvent* event)
{
    m_showCrosshairOverlay = false;
    update();
    QOpenGLWidget::leaveEvent(event);
}

// 鼠标释放中键后退出当前交互模式。
void CadViewer::mouseReleaseEvent(QMouseEvent* event)
{
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
void CadViewer::wheelEvent(QWheelEvent* event)
{
    if (!m_controller.handleWheel(event))
    {
        QOpenGLWidget::wheelEvent(event);
    }

    updateHoveredWorldPosition(event->position().toPoint());
}

// 键盘快捷键：
// F / Fit：适配场景
// T / Home：回二维顶视图
// +/-：缩放
void CadViewer::keyPressEvent(QKeyEvent* event)
{
    if (!m_controller.handleKeyPress(event))
    {
        QOpenGLWidget::keyPressEvent(event);
    }

    refreshCommandPrompt();
    update();
}

void CadViewer::dragEnterEvent(QDragEnterEvent* event)
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
            event->acceptProposedAction();
            return;
        }
    }

    event->ignore();
}

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

// 绘制背景网格。
// 网格不参与深度测试，始终作为背景参考显示。
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
void CadViewer::renderEntities(const QMatrix4x4& viewProjection)
{
    CadDocument* scene = m_sceneCoordinator.document();

    if (scene == nullptr || !m_graphicsCoordinator.isInitialized())
    {
        return;
    }

    CadEntityRenderer::renderEntities
    (
        m_graphicsCoordinator.generalShader(),
        viewProjection,
        scene->m_entities,
        m_sceneCoordinator.renderCache(),
        m_selectedEntityId
    );
}

// 绘制轨道旋转中心标记。
// 仅在“正在轨道旋转”且场景包围盒有效时显示。
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

    const std::vector<TransientPrimitive> commandPrimitives = buildTransientPrimitives();
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

    if (commandPrimitives.empty() && crosshairPrimitives.empty())
    {
        return;
    }

    m_graphicsCoordinator.renderTransientPrimitives
    (
        viewProjection,
        commandPrimitives,
        crosshairPrimitives
    );
}

void CadViewer::handleDocumentSceneChanged()
{
    m_sceneCoordinator.markBuffersDirty();
    m_sceneCoordinator.refreshBounds();

    if (findEntityById(m_selectedEntityId) == nullptr)
    {
        m_selectedEntityId = 0;
    }

    if (m_graphicsCoordinator.isInitialized())
    {
        makeCurrent();
        rebuildAllBuffers();
        doneCurrent();
    }

    update();
}

void CadViewer::updateHoveredWorldPosition(const QPoint& screenPos)
{
    emit hoveredWorldPositionChanged(screenToGroundPlane(screenPos));
}

// 屏幕空间拾取：
// - 点实体：测鼠标点到投影点距离
// - 线实体：测鼠标点到各投影线段距离
// 返回距离最近且在阈值内的实体 ID。
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

CadItem* CadViewer::findEntityById(EntityId id) const
{
    return m_sceneCoordinator.findEntityById(id);
}

std::vector<TransientPrimitive> CadViewer::buildTransientPrimitives() const
{
    return CadPreviewBuilder::buildTransientPrimitives(m_controller.drawState(), selectedEntity());
}

// 计算当前视口宽高比。
float CadViewer::aspectRatio() const
{
    return CadViewTransform::aspectRatio(m_viewportWidth, m_viewportHeight);
}

// 将 1 个屏幕像素估算为多少世界单位。
// 当前基于正交投影 viewHeight 直接换算，适用于平移操作。
float CadViewer::pixelToWorldScale() const
{
    return CadViewTransform::pixelToWorldScale(m_camera, m_viewportHeight);
}
