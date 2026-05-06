#include "pch.h"

// CadViewer 实现文件
// 实现 CadViewer 模块，对应头文件中声明的主要行为和协作流程。
// CAD 主视图模块，负责 OpenGL 生命周期、输入接入、场景刷新和信号分发。

#include "CadViewer.h"

// CAD 模块内部依赖
#include "CadCrosshairBuilder.h"
#include "CadDocument.h"
#include "CadEditer.h"
#include "CadEntityPicker.h"
#include "CadEntityRenderer.h"
#include "CadInteractionConstants.h"
#include "CadItem.h"
#include "CadProcessVisualUtils.h"
#include "CadPreviewBuilder.h"
#include "CadViewTransform.h"
#include "CadViewerUtils.h"

// Qt 核心模块
#include <QDragEnterEvent>
#include <QElapsedTimer>
#include <QDebug>
#include <QFont>
#include <QFontMetrics>
#include <QKeyEvent>
#include <QMimeData>
#include <QPainter>
#include <QPolygonF>
#include <QSurfaceFormat>
#include <QWheelEvent>

// 标准库
#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <vector>

// 匿名命名空间，存放局部常量和辅助函数
namespace
{
    // 是否启用视图性能日志记录
    constexpr bool kEnableViewerPerfLogging = false;
    // 慢帧阈值（毫秒），超过此值会记录日志
    constexpr qint64 kSlowFrameThresholdMs = 16;
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
    m_snapComputationTimer.start();

    m_overlappedHandlePopupTimer.setSingleShot(true);
    connect
    (
        &m_overlappedHandlePopupTimer,
        &QTimer::timeout,
        this,
        [this]()
        {
            if (interactionMode() != ViewInteractionMode::Idle
                || m_controller.drawState().hasActiveCommand()
                || m_overlappedHandleHoverState.candidateIndices.size() < 2)
            {
                return;
            }

            const CadItem* selectedItem = selectedEntity();

            if (selectedItem == nullptr
                || CadViewerUtils::toEntityId(selectedItem) != m_overlappedHandleHoverState.entityId)
            {
                return;
            }

            m_overlappedHandleHoverState.popupVisible = true;
            update();
        }
    );
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
    invalidateSnapCache();
    // 清除选中实体
    setSelectedEntityId(0);
    m_pendingProcessOrderSwapEntityId = 0;
    resetOverlappedHandleHoverState();
    hideSelectionWindowPreview();

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
    m_editer = editer;
    m_controller.setEditer(editer);
}

void CadViewer::setDefaultDrawingProperties(const QString& layerName, const QColor& color, int colorIndex)
{
    m_controller.setDefaultDrawingProperties(layerName, color, colorIndex);
    refreshCommandPrompt();
}

void CadViewer::invalidateSnapCache()
{
    ++m_snapContextRevision;
    m_snapResolveCache.valid = false;
}

void CadViewer::setBasePointSnapEnabled(bool enabled)
{
    m_basePointSnapEnabled = enabled;
    invalidateSnapCache();
    updateHoveredWorldPosition(m_cursorScreenPos);
    update();
}

void CadViewer::setControlPointSnapEnabled(bool enabled)
{
    m_controlPointSnapEnabled = enabled;
    invalidateSnapCache();
    updateHoveredWorldPosition(m_cursorScreenPos);
    update();
}

void CadViewer::setGridSnapEnabled(bool enabled)
{
    m_gridSnapEnabled = enabled;
    invalidateSnapCache();
    updateHoveredWorldPosition(m_cursorScreenPos);
    update();
}

void CadViewer::setEndpointSnapEnabled(bool enabled)
{
    m_endpointSnapEnabled = enabled;
    invalidateSnapCache();
    updateHoveredWorldPosition(m_cursorScreenPos);
    update();
}

void CadViewer::setMidpointSnapEnabled(bool enabled)
{
    m_midpointSnapEnabled = enabled;
    invalidateSnapCache();
    updateHoveredWorldPosition(m_cursorScreenPos);
    update();
}

void CadViewer::setCenterSnapEnabled(bool enabled)
{
    m_centerSnapEnabled = enabled;
    invalidateSnapCache();
    updateHoveredWorldPosition(m_cursorScreenPos);
    update();
}

void CadViewer::setIntersectionSnapEnabled(bool enabled)
{
    m_intersectionSnapEnabled = enabled;
    invalidateSnapCache();
    updateHoveredWorldPosition(m_cursorScreenPos);
    update();
}

void CadViewer::setOrthoEnabled(bool enabled)
{
    if (m_controller.orthoEnabled() == enabled)
    {
        return;
    }

    m_controller.setOrthoEnabled(enabled);
    updateHoveredWorldPosition(m_cursorScreenPos);
    emit orthoEnabledChanged(enabled);
    update();
}

void CadViewer::setPolarTrackingEnabled(bool enabled)
{
    if (m_controller.polarTrackingEnabled() == enabled)
    {
        return;
    }

    m_controller.setPolarTrackingEnabled(enabled);
    updateHoveredWorldPosition(m_cursorScreenPos);
    emit polarTrackingEnabledChanged(enabled);
    update();
}

void CadViewer::setTheme(const AppThemeColors& theme)
{
    m_theme = theme;
    m_graphicsCoordinator.setTheme(theme);
    update();
}

void CadViewer::setProcessVisualsVisible(bool visible)
{
    if (m_processVisualsVisible == visible)
    {
        return;
    }

    m_processVisualsVisible = visible;
    m_pendingProcessOrderSwapEntityId = 0;
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

bool CadViewer::startCopySelected()
{
    const bool handled = m_controller.beginCopySelected();

    if (handled)
    {
        update();
    }

    return handled;
}

bool CadViewer::startRotateSelected()
{
    const bool handled = m_controller.beginRotateSelected();

    if (handled)
    {
        update();
    }

    return handled;
}

bool CadViewer::startScaleSelected()
{
    const bool handled = m_controller.beginScaleSelected();

    if (handled)
    {
        update();
    }

    return handled;
}

bool CadViewer::startRectangularArraySelected()
{
    const bool handled = m_controller.beginRectangularArraySelected();

    if (handled)
    {
        update();
    }

    return handled;
}

bool CadViewer::startCircularArraySelected()
{
    const bool handled = m_controller.beginCircularArraySelected();

    if (handled)
    {
        update();
    }

    return handled;
}

bool CadViewer::startMirrorSelected()
{
    const bool handled = m_controller.beginMirrorSelected();

    if (handled)
    {
        update();
    }

    return handled;
}

bool CadViewer::startOffsetSelected()
{
    const bool handled = m_controller.beginOffsetSelected();

    if (handled)
    {
        update();
    }

    return handled;
}

bool CadViewer::startTrimSelected()
{
    const bool handled = m_controller.beginTrimSelected();

    if (handled)
    {
        update();
    }

    return handled;
}

bool CadViewer::startExtendSelected()
{
    const bool handled = m_controller.beginExtendSelected();

    if (handled)
    {
        update();
    }

    return handled;
}

bool CadViewer::startFilletSelected()
{
    const bool handled = m_controller.beginFilletSelected();

    if (handled)
    {
        update();
    }

    return handled;
}

bool CadViewer::startChamferSelected()
{
    const bool handled = m_controller.beginChamferSelected();

    if (handled)
    {
        update();
    }

    return handled;
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
    invalidateSnapCache();
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
    renderEntitySelectionOverlays();
    renderProcessOrderLabels();
    renderSelectionWindowPreview();
    renderOverlappedHandlePopup();
    renderDynamicInputOverlay();
    renderDynamicCommandOverlay();

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

    float minX = 0.0f;
    float maxX = 0.0f;
    float minY = 0.0f;
    float maxY = 0.0f;
    computeVisibleGroundBounds(minX, maxX, minY, maxY);

    m_graphicsCoordinator.renderGrid(viewProjection, minX, maxX, minY, maxY, currentGridStep());
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
    const std::vector<TransientPrimitive> selectedHandlePrimitives = buildSelectedEntityHandlePrimitives();
    const std::vector<TransientPrimitive> snapHighlightPrimitives = buildSnapHighlightPrimitives();
    // 构建命令预览图元
    const std::vector<TransientPrimitive> commandPrimitives = buildTransientPrimitives();
    const QPoint crosshairScreenPos = worldToScreen(resolveInteractiveWorldPosition(m_cursorScreenPos));
    // 构建十字准线图元
    const std::vector<TransientPrimitive> crosshairPrimitives = CadCrosshairBuilder::buildCrosshairPrimitives
    (
        m_camera,
        m_viewportWidth,
        m_viewportHeight,
        width(),
        height(),
        crosshairScreenPos,
        m_showCrosshairOverlay,
        m_viewInteractionController.crosshairSuppressed(),
        m_crosshairPlaneZ,
        CadInteractionConstants::kPickBoxHalfSizePixels * pixelToWorldScale(),
        CadInteractionConstants::kCrosshairHalfLengthWorld
    );

    // 如果没有临时图元，则返回
    processPrimitives.insert(processPrimitives.end(), selectedHandlePrimitives.begin(), selectedHandlePrimitives.end());
    processPrimitives.insert(processPrimitives.end(), snapHighlightPrimitives.begin(), snapHighlightPrimitives.end());
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


// 处理文档场景变化
void CadViewer::handleDocumentSceneChanged()
{
    invalidateSnapCache();
    m_pendingProcessOrderSwapEntityId = 0;
    // 标记缓冲脏
    m_sceneCoordinator.markBuffersDirty();
    // 刷新场景边界
    m_sceneCoordinator.refreshBounds();

    // 场景变化后重算选中集合，自动剔除已失效实体。
    setSelectedEntities(m_selectedEntityIds, m_selectedEntityId);

    if (m_selectionWindowPreview.visible)
    {
        updateSelectionWindowPreviewCandidates();
    }
    else
    {
        m_windowPreviewEntityIds.clear();
    }

    // 如果图形协调器已初始化，则重建缓冲
    if (m_graphicsCoordinator.isInitialized())
    {
        makeCurrent();
        rebuildAllBuffers();
        doneCurrent();
    }

    updateOverlappedHandleHoverState(m_cursorScreenPos);
    update();
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






