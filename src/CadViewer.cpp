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
    // 期望网格在屏幕上的视觉间距（像素）
    constexpr float kTargetGridSpacingPixels = 40.0f;
    // 单个方向上允许绘制的最大网格线数量
    constexpr float kMaxGridLineCountPerAxis = 240.0f;
    // 对象吸附屏幕距离阈值（像素）
    constexpr float kObjectSnapDistancePixels = 14.0f;
    // 网格吸附屏幕距离阈值（像素）
    constexpr float kGridSnapDistancePixels = 10.0f;
    // 窗口框选有效尺寸阈值（像素）。
    constexpr int kWindowSelectionMinimumPixels = 2;
    // 重叠夹点悬停弹框延迟（毫秒）。
    constexpr qint64 kOverlappedHandlePopupDelayMs = 1000;
    // 重叠夹点悬停判定允许的屏幕漂移阈值（像素）。
    constexpr int kOverlappedHandleStablePixels = 4;
    // 重叠夹点判定时额外放宽的屏幕命中系数。
    constexpr float kOverlappedHandlePickScale = 1.25f;

    struct HandlePickCandidate
    {
        int handleIndex = -1;
        float distanceSquared = std::numeric_limits<float>::max();
        int priority = 1;
    };

    struct OverlappedHandlePopupLayout
    {
        QRect panelRect;
        QVector<QRect> rowRects;
        QVector<QString> rowTexts;
        QRect badgeRect;
    };

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

    QVector<QVector3D> buildTriangleVertices(const QVector3D& center, QVector3D direction, float size)
    {
        direction.setZ(0.0f);

        if (direction.lengthSquared() <= 1.0e-6f)
        {
            direction = QVector3D(0.0f, 1.0f, 0.0f);
        }
        else
        {
            direction.normalize();
        }

        QVector3D perpendicular(-direction.y(), direction.x(), 0.0f);

        if (perpendicular.lengthSquared() <= 1.0e-6f)
        {
            perpendicular = QVector3D(1.0f, 0.0f, 0.0f);
        }
        else
        {
            perpendicular.normalize();
        }

        const QVector3D tip = center + direction * size;
        const QVector3D baseCenter = center - direction * (size * 0.8f);
        const QVector3D left = baseCenter + perpendicular * (size * 0.7f);
        const QVector3D right = baseCenter - perpendicular * (size * 0.7f);
        return { tip, left, right };
    }

    // 采用二分层级网格步长，保证细分后不会丢失上一级网格点。
    // 示例：100 -> 50 -> 25 -> 12.5 -> 6.25
    float chooseHierarchicalGridStep(float desiredStep)
    {
        constexpr float kGridStepAnchor = 100.0f;
        const float safeStep = std::max(desiredStep, 1.0e-6f);
        const float level = std::round(std::log2(safeStep / kGridStepAnchor));
        const float step = kGridStepAnchor * std::pow(2.0f, level);
        return std::max(step, 1.0e-6f);
    }

    QRect normalizedSelectionRect(const QPoint& anchorScreenPos, const QPoint& currentScreenPos)
    {
        return QRect(anchorScreenPos, currentScreenPos).normalized();
    }

    bool isPopupCycleModifierValid(Qt::KeyboardModifiers modifiers)
    {
        const Qt::KeyboardModifiers maskedModifiers = modifiers & ~(Qt::ShiftModifier);
        return maskedModifiers == Qt::NoModifier;
    }

    QVector<int> collectEditableHandleCandidates
    (
        const QVector<CadSelectionHandleInfo>& handles,
        const std::function<QPoint(const QVector3D&)>& worldToScreen,
        const QPoint& cursorScreenPos,
        float pickDistanceSquared
    )
    {
        QVector<HandlePickCandidate> candidates;

        for (int index = 0; index < handles.size(); ++index)
        {
            const CadSelectionHandleInfo& handle = handles.at(index);

            if (!handle.editable)
            {
                continue;
            }

            const QPoint handleScreenPos = worldToScreen(handle.position);
            const float dx = static_cast<float>(handleScreenPos.x() - cursorScreenPos.x());
            const float dy = static_cast<float>(handleScreenPos.y() - cursorScreenPos.y());
            const float distanceSquared = dx * dx + dy * dy;

            if (distanceSquared > pickDistanceSquared)
            {
                continue;
            }

            HandlePickCandidate candidate;
            candidate.handleIndex = index;
            candidate.distanceSquared = distanceSquared;
            candidate.priority = handle.isBasePoint ? 0 : 1;
            candidates.push_back(candidate);
        }

        std::sort
        (
            candidates.begin(),
            candidates.end(),
            [](const HandlePickCandidate& left, const HandlePickCandidate& right)
            {
                if (std::abs(left.distanceSquared - right.distanceSquared) > 1.0e-4f)
                {
                    return left.distanceSquared < right.distanceSquared;
                }

                if (left.priority != right.priority)
                {
                    return left.priority < right.priority;
                }

                return left.handleIndex < right.handleIndex;
            }
        );

        QVector<int> orderedIndices;
        orderedIndices.reserve(candidates.size());

        for (const HandlePickCandidate& candidate : candidates)
        {
            orderedIndices.push_back(candidate.handleIndex);
        }

        return orderedIndices;
    }

    QString handleTypeDisplayName(const CadItem* selectedItem, const CadSelectionHandleInfo& handle)
    {
        if (handle.isBasePoint)
        {
            return QStringLiteral("基点");
        }

        if (selectedItem == nullptr)
        {
            return QStringLiteral("拉伸点");
        }

        switch (selectedItem->m_type)
        {
        case DRW::ETYPE::POINT:
            return QStringLiteral("点位控制点");
        case DRW::ETYPE::LINE:
            return QStringLiteral("拉伸点");
        case DRW::ETYPE::CIRCLE:
            return QStringLiteral("半径控制点");
        case DRW::ETYPE::ARC:
            if (handle.pointIndex == 1)
            {
                return QStringLiteral("起点拉伸点");
            }

            if (handle.pointIndex == 2)
            {
                return QStringLiteral("半径控制点");
            }

            if (handle.pointIndex == 3)
            {
                return QStringLiteral("终点拉伸点");
            }

            return QStringLiteral("拉伸点");
        case DRW::ETYPE::ELLIPSE:
            if (handle.pointIndex == 1 || handle.pointIndex == 2)
            {
                return QStringLiteral("长轴控制点");
            }

            if (handle.pointIndex == 3 || handle.pointIndex == 4)
            {
                return QStringLiteral("短轴控制点");
            }

            if (handle.pointIndex == 5)
            {
                return QStringLiteral("起点拉伸点");
            }

            if (handle.pointIndex == 6)
            {
                return QStringLiteral("终点拉伸点");
            }

            return QStringLiteral("拉伸点");
        case DRW::ETYPE::POLYLINE:
        case DRW::ETYPE::LWPOLYLINE:
            return QStringLiteral("顶点拉伸点");
        default:
            return QStringLiteral("拉伸点");
        }
    }

    QString overlappedHandleRowText
    (
        const CadItem* selectedItem,
        const CadSelectionHandleInfo& handle,
        int ordinal
    )
    {
        return QStringLiteral("%1. %2")
            .arg(ordinal + 1)
            .arg(handleTypeDisplayName(selectedItem, handle));
    }

    bool computeOverlappedHandlePopupLayout
    (
        const CadItem* selectedItem,
        const QVector<CadSelectionHandleInfo>& handles,
        const QVector<int>& candidateIndices,
        int activeCandidateOrdinal,
        const QPoint& cursorScreenPos,
        const QSize& viewportSize,
        const QFontMetrics& metrics,
        OverlappedHandlePopupLayout& layout
    )
    {
        if (candidateIndices.size() < 2 || viewportSize.width() <= 0 || viewportSize.height() <= 0)
        {
            return false;
        }

        layout.rowTexts.clear();
        layout.rowTexts.reserve(candidateIndices.size());
        int maxTextWidth = 0;

        for (int ordinal = 0; ordinal < candidateIndices.size(); ++ordinal)
        {
            const int handleIndex = candidateIndices.at(ordinal);

            if (handleIndex < 0 || handleIndex >= handles.size())
            {
                continue;
            }

            const CadSelectionHandleInfo& handle = handles.at(handleIndex);
            const QString rowText = overlappedHandleRowText(selectedItem, handle, ordinal);
            layout.rowTexts.push_back(rowText);
            maxTextWidth = std::max(maxTextWidth, metrics.horizontalAdvance(rowText));
        }

        if (layout.rowTexts.size() < 2)
        {
            return false;
        }

        const int panelPaddingX = 14;
        const int panelPaddingY = 10;
        const int rowHeight = std::max(22, metrics.height() + 8);
        const int desiredPanelWidth = maxTextWidth + panelPaddingX * 2 + 24;
        const int maxPanelWidth = std::max(140, viewportSize.width() - 16);
        const int panelWidth = std::min(desiredPanelWidth, maxPanelWidth);
        const int panelHeight = panelPaddingY * 2 + rowHeight * layout.rowTexts.size();
        QPoint panelTopLeft = cursorScreenPos + QPoint(18, 16);

        if (panelTopLeft.x() + panelWidth > viewportSize.width() - 8)
        {
            panelTopLeft.setX(std::max(8, viewportSize.width() - panelWidth - 8));
        }

        if (panelTopLeft.y() + panelHeight > viewportSize.height() - 8)
        {
            panelTopLeft.setY(std::max(8, viewportSize.height() - panelHeight - 8));
        }

        layout.panelRect = QRect(panelTopLeft, QSize(panelWidth, panelHeight));
        layout.rowRects.clear();
        layout.rowRects.reserve(layout.rowTexts.size());

        for (int ordinal = 0; ordinal < layout.rowTexts.size(); ++ordinal)
        {
            const QRect rowRect
            (
                panelTopLeft.x() + panelPaddingX,
                panelTopLeft.y() + panelPaddingY + ordinal * rowHeight,
                panelWidth - panelPaddingX * 2,
                rowHeight
            );
            layout.rowRects.push_back(rowRect);
        }

        const int badgeRadius = 9;
        const QPoint badgeCenter = cursorScreenPos + QPoint(13, -13);
        layout.badgeRect = QRect
        (
            badgeCenter.x() - badgeRadius,
            badgeCenter.y() - badgeRadius,
            badgeRadius * 2,
            badgeRadius * 2
        );

        Q_UNUSED(activeCandidateOrdinal);
        return true;
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
    // 清除选中实体
    setSelectedEntityId(0);
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
    m_controller.setEditer(editer);
}

void CadViewer::setDefaultDrawingProperties(const QString& layerName, const QColor& color, int colorIndex)
{
    m_controller.setDefaultDrawingProperties(layerName, color, colorIndex);
    refreshCommandPrompt();
}

void CadViewer::setBasePointSnapEnabled(bool enabled)
{
    m_basePointSnapEnabled = enabled;
    updateHoveredWorldPosition(m_cursorScreenPos);
    update();
}

void CadViewer::setControlPointSnapEnabled(bool enabled)
{
    m_controlPointSnapEnabled = enabled;
    updateHoveredWorldPosition(m_cursorScreenPos);
    update();
}

void CadViewer::setGridSnapEnabled(bool enabled)
{
    m_gridSnapEnabled = enabled;
    updateHoveredWorldPosition(m_cursorScreenPos);
    update();
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
void CadViewer::selectEntityAt(const QPoint& screenPos, SelectionUpdateMode updateMode)
{
    const EntityId pickedId = pickEntity(screenPos);

    if (updateMode == SelectionUpdateMode::Toggle)
    {
        if (pickedId != 0)
        {
            QSet<EntityId> selectedIds = m_selectedEntityIds;
            EntityId preferredEntityId = m_selectedEntityId;

            if (selectedIds.contains(pickedId))
            {
                selectedIds.remove(pickedId);

                if (preferredEntityId == pickedId)
                {
                    preferredEntityId = 0;
                }
            }
            else
            {
                selectedIds.insert(pickedId);
                preferredEntityId = pickedId;
            }

            setSelectedEntities(selectedIds, preferredEntityId);
        }
    }
    else
    {
        setSelectedEntityId(pickedId);
    }

    update();
}

void CadViewer::selectEntitiesInWindow
(
    const QPoint& startScreenPos,
    const QPoint& endScreenPos,
    bool crossingSelection,
    SelectionUpdateMode updateMode
)
{
    CadDocument* scene = m_sceneCoordinator.document();

    if (scene == nullptr)
    {
        if (updateMode == SelectionUpdateMode::Replace)
        {
            setSelectedEntities(QSet<EntityId>(), 0);
        }

        update();
        return;
    }

    const QRect selectionRect = normalizedSelectionRect(startScreenPos, endScreenPos);

    if (selectionRect.width() < kWindowSelectionMinimumPixels || selectionRect.height() < kWindowSelectionMinimumPixels)
    {
        if (updateMode == SelectionUpdateMode::Replace)
        {
            setSelectedEntities(QSet<EntityId>(), 0);
        }

        update();
        return;
    }

    const std::vector<EntityId> pickedIds = CadEntityPicker::pickEntitiesByWindow
    (
        scene->m_entities,
        m_camera.viewProjectionMatrix(aspectRatio()),
        m_viewportWidth,
        m_viewportHeight,
        QRectF(selectionRect),
        crossingSelection
    );

    QSet<EntityId> selectedIds;
    selectedIds.reserve(static_cast<qsizetype>(pickedIds.size()));

    for (EntityId id : pickedIds)
    {
        if (id != 0)
        {
            selectedIds.insert(id);
        }
    }

    EntityId preferredEntityId = pickedIds.empty() ? 0 : pickedIds.front();

    if (updateMode == SelectionUpdateMode::Toggle)
    {
        QSet<EntityId> mergedSelection = m_selectedEntityIds;

        for (EntityId id : selectedIds)
        {
            if (mergedSelection.contains(id))
            {
                mergedSelection.remove(id);
            }
            else
            {
                mergedSelection.insert(id);
            }
        }

        preferredEntityId = m_selectedEntityId;

        for (EntityId id : pickedIds)
        {
            if (id != 0 && mergedSelection.contains(id))
            {
                preferredEntityId = id;
                break;
            }
        }

        if (preferredEntityId != 0 && !mergedSelection.contains(preferredEntityId))
        {
            preferredEntityId = 0;
        }

        setSelectedEntities(mergedSelection, preferredEntityId);
    }
    else
    {
        setSelectedEntities(selectedIds, preferredEntityId);
    }

    m_windowPreviewEntityIds.clear();
    update();
}

QVector<CadItem*> CadViewer::selectedEntities() const
{
    QVector<CadItem*> entities;
    CadDocument* scene = m_sceneCoordinator.document();

    if (scene == nullptr || m_selectedEntityIds.isEmpty())
    {
        return entities;
    }

    entities.reserve(static_cast<qsizetype>(m_selectedEntityIds.size()));

    for (const std::unique_ptr<CadItem>& entity : scene->m_entities)
    {
        if (entity == nullptr)
        {
            continue;
        }

        const EntityId id = CadViewerUtils::toEntityId(entity.get());

        if (m_selectedEntityIds.contains(id))
        {
            entities.push_back(entity.get());
        }
    }

    return entities;
}

void CadViewer::showSelectionWindowPreview(const QPoint& anchorScreenPos, const QPoint& currentScreenPos)
{
    m_selectionWindowPreview.visible = true;
    m_selectionWindowPreview.anchorScreenPos = anchorScreenPos;
    m_selectionWindowPreview.currentScreenPos = currentScreenPos;
    m_selectionWindowPreview.crossingSelection = currentScreenPos.x() < anchorScreenPos.x();
    updateSelectionWindowPreviewCandidates();
    update();
}

void CadViewer::hideSelectionWindowPreview()
{
    const bool hadVisiblePreview = m_selectionWindowPreview.visible;
    m_selectionWindowPreview.visible = false;

    if (m_windowPreviewEntityIds.isEmpty() && !hadVisiblePreview)
    {
        return;
    }

    m_windowPreviewEntityIds.clear();
    update();
}

void CadViewer::updateSelectionWindowPreviewCandidates()
{
    m_windowPreviewEntityIds.clear();

    if (!m_selectionWindowPreview.visible)
    {
        return;
    }

    CadDocument* scene = m_sceneCoordinator.document();

    if (scene == nullptr)
    {
        return;
    }

    const QRect selectionRect = normalizedSelectionRect
    (
        m_selectionWindowPreview.anchorScreenPos,
        m_selectionWindowPreview.currentScreenPos
    );

    if (selectionRect.width() < kWindowSelectionMinimumPixels || selectionRect.height() < kWindowSelectionMinimumPixels)
    {
        return;
    }

    const std::vector<EntityId> previewIds = CadEntityPicker::pickEntitiesByWindow
    (
        scene->m_entities,
        m_camera.viewProjectionMatrix(aspectRatio()),
        m_viewportWidth,
        m_viewportHeight,
        QRectF(selectionRect),
        m_selectionWindowPreview.crossingSelection
    );

    for (EntityId id : previewIds)
    {
        if (id != 0)
        {
            m_windowPreviewEntityIds.insert(id);
        }
    }
}

void CadViewer::resetOverlappedHandleHoverState()
{
    m_overlappedHandlePopupTimer.stop();
    m_overlappedHandleHoverState.entityId = 0;
    m_overlappedHandleHoverState.candidateIndices.clear();
    m_overlappedHandleHoverState.activeCandidateOrdinal = 0;
    m_overlappedHandleHoverState.anchorScreenPos = QPoint();
    m_overlappedHandleHoverState.popupVisible = false;
    m_overlappedHandleHoverState.hoverTimer.invalidate();
}

void CadViewer::updateOverlappedHandleHoverState(const QPoint& screenPos)
{
    if (interactionMode() != ViewInteractionMode::Idle
        || m_controller.drawState().hasActiveCommand()
        || m_selectionWindowPreview.visible
        || (m_controller.drawState().pressedButtons & Qt::LeftButton) != 0)
    {
        resetOverlappedHandleHoverState();
        return;
    }

    CadItem* selectedItem = selectedEntity();

    if (selectedItem == nullptr)
    {
        resetOverlappedHandleHoverState();
        return;
    }

    const QVector<CadSelectionHandleInfo> handles = buildSelectionHandleInfo(selectedItem);

    if (handles.isEmpty())
    {
        resetOverlappedHandleHoverState();
        return;
    }

    const float pickDistanceSquared = (kObjectSnapDistancePixels * kOverlappedHandlePickScale)
        * (kObjectSnapDistancePixels * kOverlappedHandlePickScale);
    const QVector<int> candidateIndices = collectEditableHandleCandidates
    (
        handles,
        [this](const QVector3D& worldPosition)
        {
            return worldToScreen(worldPosition);
        },
        screenPos,
        pickDistanceSquared
    );

    if (candidateIndices.size() < 2)
    {
        resetOverlappedHandleHoverState();
        return;
    }

    const EntityId entityId = CadViewerUtils::toEntityId(selectedItem);
    const bool sameEntity = m_overlappedHandleHoverState.entityId == entityId;
    const bool sameCandidates = m_overlappedHandleHoverState.candidateIndices == candidateIndices;
    const bool stableCursor = sameEntity
        && sameCandidates
        && (screenPos - m_overlappedHandleHoverState.anchorScreenPos).manhattanLength() <= kOverlappedHandleStablePixels;

    if (!stableCursor)
    {
        m_overlappedHandleHoverState.entityId = entityId;
        m_overlappedHandleHoverState.candidateIndices = candidateIndices;
        m_overlappedHandleHoverState.activeCandidateOrdinal = 0;
        m_overlappedHandleHoverState.anchorScreenPos = screenPos;
        m_overlappedHandleHoverState.popupVisible = false;
        m_overlappedHandleHoverState.hoverTimer.restart();
        m_overlappedHandlePopupTimer.start(static_cast<int>(kOverlappedHandlePopupDelayMs));
        return;
    }

    if (!m_overlappedHandleHoverState.popupVisible
        && m_overlappedHandleHoverState.hoverTimer.isValid()
        && m_overlappedHandleHoverState.hoverTimer.elapsed() >= kOverlappedHandlePopupDelayMs)
    {
        m_overlappedHandlePopupTimer.stop();
        m_overlappedHandleHoverState.popupVisible = true;
    }
    else if (!m_overlappedHandleHoverState.popupVisible && !m_overlappedHandlePopupTimer.isActive())
    {
        const qint64 elapsedMs = m_overlappedHandleHoverState.hoverTimer.isValid()
            ? m_overlappedHandleHoverState.hoverTimer.elapsed()
            : 0;
        const qint64 remainingMs = std::max<qint64>(1, kOverlappedHandlePopupDelayMs - elapsedMs);
        m_overlappedHandlePopupTimer.start(static_cast<int>(remainingMs));
    }

    m_overlappedHandleHoverState.activeCandidateOrdinal = std::clamp
    (
        m_overlappedHandleHoverState.activeCandidateOrdinal,
        0,
        static_cast<int>(m_overlappedHandleHoverState.candidateIndices.size()) - 1
    );
}

int CadViewer::resolveHoveredHandleIndex(const QVector<CadSelectionHandleInfo>& handles) const
{
    if (handles.isEmpty())
    {
        return -1;
    }

    const float pickDistanceSquared = (kObjectSnapDistancePixels * kOverlappedHandlePickScale)
        * (kObjectSnapDistancePixels * kOverlappedHandlePickScale);
    const QVector<int> candidateIndices = collectEditableHandleCandidates
    (
        handles,
        [this](const QVector3D& worldPosition)
        {
            return worldToScreen(worldPosition);
        },
        m_cursorScreenPos,
        pickDistanceSquared
    );

    if (candidateIndices.isEmpty())
    {
        return -1;
    }

    const CadItem* selectedItem = selectedEntity();
    const EntityId selectedEntityId = selectedItem != nullptr ? CadViewerUtils::toEntityId(selectedItem) : 0;

    if (candidateIndices.size() >= 2
        && m_overlappedHandleHoverState.entityId == selectedEntityId
        && m_overlappedHandleHoverState.candidateIndices == candidateIndices)
    {
        int resolvedOrdinal = std::clamp
        (
            m_overlappedHandleHoverState.activeCandidateOrdinal,
            0,
            static_cast<int>(candidateIndices.size()) - 1
        );
        return candidateIndices.at(resolvedOrdinal);
    }

    return candidateIndices.front();
}

bool CadViewer::handleOverlappedHandlePopupPress(const QPoint& screenPos)
{
    if (!m_overlappedHandleHoverState.popupVisible || m_overlappedHandleHoverState.candidateIndices.size() < 2)
    {
        return false;
    }

    const CadItem* selectedItem = selectedEntity();

    if (selectedItem == nullptr
        || CadViewerUtils::toEntityId(selectedItem) != m_overlappedHandleHoverState.entityId)
    {
        return false;
    }

    const QVector<CadSelectionHandleInfo> handles = buildSelectionHandleInfo(selectedItem);

    if (handles.isEmpty())
    {
        return false;
    }

    QFont popupFont = font();
    popupFont.setPointSize(9);
    const QFontMetrics metrics(popupFont);
    OverlappedHandlePopupLayout popupLayout;

    if (!computeOverlappedHandlePopupLayout
    (
        selectedItem,
        handles,
        m_overlappedHandleHoverState.candidateIndices,
        m_overlappedHandleHoverState.activeCandidateOrdinal,
        m_cursorScreenPos,
        size(),
        metrics,
        popupLayout
    ))
    {
        return false;
    }

    for (int ordinal = 0; ordinal < popupLayout.rowRects.size(); ++ordinal)
    {
        if (!popupLayout.rowRects.at(ordinal).contains(screenPos))
        {
            continue;
        }

        m_overlappedHandleHoverState.activeCandidateOrdinal = ordinal;
        m_overlappedHandleHoverState.popupVisible = true;
        m_overlappedHandleHoverState.anchorScreenPos = m_cursorScreenPos;
        m_overlappedHandleHoverState.hoverTimer.restart();
        m_overlappedHandlePopupTimer.stop();
        update();
        return true;
    }

    if (popupLayout.panelRect.contains(screenPos))
    {
        return true;
    }

    return false;
}

bool CadViewer::cycleOverlappedHandleCandidate(int step)
{
    if (m_overlappedHandleHoverState.candidateIndices.size() < 2 || step == 0)
    {
        return false;
    }

    const CadItem* selectedItem = selectedEntity();

    if (selectedItem == nullptr
        || CadViewerUtils::toEntityId(selectedItem) != m_overlappedHandleHoverState.entityId)
    {
        return false;
    }

    const int candidateCount = m_overlappedHandleHoverState.candidateIndices.size();
    int ordinal = m_overlappedHandleHoverState.activeCandidateOrdinal % candidateCount;

    if (ordinal < 0)
    {
        ordinal += candidateCount;
    }

    ordinal = (ordinal + step) % candidateCount;

    if (ordinal < 0)
    {
        ordinal += candidateCount;
    }

    m_overlappedHandleHoverState.activeCandidateOrdinal = ordinal;
    m_overlappedHandleHoverState.popupVisible = true;
    m_overlappedHandleHoverState.anchorScreenPos = m_cursorScreenPos;
    m_overlappedHandleHoverState.hoverTimer.restart();
    m_overlappedHandlePopupTimer.stop();
    update();
    return true;
}

void CadViewer::renderOverlappedHandlePopup()
{
    if (interactionMode() != ViewInteractionMode::Idle)
    {
        return;
    }

    const CadItem* selectedItem = selectedEntity();

    if (selectedItem == nullptr
        || CadViewerUtils::toEntityId(selectedItem) != m_overlappedHandleHoverState.entityId
        || m_overlappedHandleHoverState.candidateIndices.size() < 2)
    {
        return;
    }

    const QVector<CadSelectionHandleInfo> handles = buildSelectionHandleInfo(selectedItem);

    if (handles.isEmpty())
    {
        return;
    }

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setRenderHint(QPainter::TextAntialiasing, true);

    QFont popupFont = painter.font();
    popupFont.setPointSize(9);
    painter.setFont(popupFont);
    const QFontMetrics metrics(popupFont);

    OverlappedHandlePopupLayout popupLayout;

    if (!computeOverlappedHandlePopupLayout
    (
        selectedItem,
        handles,
        m_overlappedHandleHoverState.candidateIndices,
        m_overlappedHandleHoverState.activeCandidateOrdinal,
        m_cursorScreenPos,
        size(),
        metrics,
        popupLayout
    ))
    {
        return;
    }

    // 鼠标附近数量徽标：提示当前存在重叠夹点候选。
    painter.setPen(QPen(QColor(15, 22, 30, 210), 1.0));
    painter.setBrush(QColor(255, 186, 54, 235));
    painter.drawEllipse(popupLayout.badgeRect);
    painter.setPen(QColor(18, 24, 33));
    painter.drawText(popupLayout.badgeRect, Qt::AlignCenter, QString::number(m_overlappedHandleHoverState.candidateIndices.size()));

    if (!m_overlappedHandleHoverState.popupVisible)
    {
        return;
    }

    painter.setPen(QPen(QColor(94, 176, 255, 230), 1.0));
    painter.setBrush(QColor(20, 26, 34, 228));
    painter.drawRoundedRect(popupLayout.panelRect, 7.0, 7.0);

    const int rowCount = popupLayout.rowRects.size();
    const int activeOrdinal = std::clamp(m_overlappedHandleHoverState.activeCandidateOrdinal, 0, rowCount - 1);

    for (int ordinal = 0; ordinal < rowCount; ++ordinal)
    {
        const QRect rowRect = popupLayout.rowRects.at(ordinal);
        const QString rowText = ordinal < popupLayout.rowTexts.size()
            ? popupLayout.rowTexts.at(ordinal)
            : QStringLiteral("%1. 拉伸点").arg(ordinal + 1);
        const QString displayText = metrics.elidedText(rowText, Qt::ElideRight, std::max(20, rowRect.width() - 8));

        if (ordinal == activeOrdinal)
        {
            painter.setPen(Qt::NoPen);
            painter.setBrush(QColor(84, 166, 255, 96));
            painter.drawRoundedRect(rowRect.adjusted(0, 0, 0, -1), 4.0, 4.0);
        }

        painter.setPen(ordinal == activeOrdinal ? QColor(255, 248, 227) : QColor(220, 230, 242));
        painter.drawText(rowRect.adjusted(4, 0, -4, 0), Qt::AlignVCenter | Qt::AlignLeft, displayText);
    }

    const QString hintText = QStringLiteral("Tab/Shift+Tab 切换");
    const int hintTop = popupLayout.panelRect.bottom() + 5;
    QRect hintRect
    (
        popupLayout.panelRect.left(),
        hintTop,
        popupLayout.panelRect.width(),
        metrics.height() + 2
    );

    if (hintRect.bottom() > height() - 4)
    {
        hintRect.moveTop(popupLayout.panelRect.top() - hintRect.height() - 4);
    }

    painter.setPen(QColor(150, 172, 194, 228));
    painter.drawText(hintRect, Qt::AlignLeft | Qt::AlignVCenter, hintText);
}

void CadViewer::renderDynamicInputOverlay()
{
    if (interactionMode() != ViewInteractionMode::Idle)
    {
        return;
    }

    const CadDynamicInputOverlayState overlayState = m_controller.dynamicInputOverlayState();

    if (!overlayState.visible)
    {
        return;
    }

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setRenderHint(QPainter::TextAntialiasing, true);

    QFont titleFont = painter.font();
    titleFont.setPointSize(9);
    titleFont.setBold(true);

    QFont valueFont = painter.font();
    valueFont.setPointSize(9);

    const QFontMetrics titleMetrics(titleFont);
    const QFontMetrics valueMetrics(valueFont);
    const int horizontalPadding = 8;
    const int verticalPadding = 6;
    const int rowHeight = valueMetrics.height() + 6;
    const int titleHeight = titleMetrics.height();
    const int hintHeight = valueMetrics.height();
    const int valueColumnWidth = 140;
    const int labelColumnWidth = 58;
    const int panelWidth = horizontalPadding * 2 + labelColumnWidth + valueColumnWidth;
    const int rowCount = overlayState.expressionMode ? 1 : 2;
    const int panelHeight = verticalPadding * 2 + titleHeight + 4 + rowCount * rowHeight + 4 + hintHeight;

    QPoint panelTopLeft = m_cursorScreenPos + QPoint(16, 20);

    if (panelTopLeft.x() + panelWidth > width() - 6)
    {
        panelTopLeft.setX(std::max(6, width() - panelWidth - 6));
    }

    if (panelTopLeft.y() + panelHeight > height() - 6)
    {
        panelTopLeft.setY(std::max(6, m_cursorScreenPos.y() - panelHeight - 16));
    }

    const QRect panelRect(panelTopLeft, QSize(panelWidth, panelHeight));
    painter.setPen(QPen(QColor(88, 160, 245, 220), 1.0));
    painter.setBrush(QColor(16, 22, 30, 222));
    painter.drawRoundedRect(panelRect, 7.0, 7.0);

    QRect contentRect = panelRect.adjusted(horizontalPadding, verticalPadding, -horizontalPadding, -verticalPadding);

    painter.setFont(titleFont);
    painter.setPen(QColor(228, 238, 252, 235));
    painter.drawText(QRect(contentRect.left(), contentRect.top(), contentRect.width(), titleHeight), Qt::AlignLeft | Qt::AlignVCenter, overlayState.title);

    int rowTop = contentRect.top() + titleHeight + 4;

    auto drawValueRow = [&](int rowIndex, const QString& labelText, const QString& valueText, bool active)
    {
        QRect rowRect(contentRect.left(), rowTop + rowIndex * rowHeight, contentRect.width(), rowHeight);

        if (active)
        {
            painter.setPen(Qt::NoPen);
            painter.setBrush(QColor(82, 162, 245, 92));
            painter.drawRoundedRect(rowRect.adjusted(0, 0, 0, -1), 4.0, 4.0);
        }

        painter.setFont(valueFont);
        painter.setPen(active ? QColor(246, 251, 255) : QColor(212, 224, 239));
        painter.drawText(QRect(rowRect.left(), rowRect.top(), labelColumnWidth, rowRect.height()), Qt::AlignLeft | Qt::AlignVCenter, labelText);
        painter.drawText(QRect(rowRect.left() + labelColumnWidth, rowRect.top(), rowRect.width() - labelColumnWidth, rowRect.height()), Qt::AlignLeft | Qt::AlignVCenter, valueText);
    };

    if (overlayState.expressionMode)
    {
        const QString expressionText = overlayState.expressionText.trimmed().isEmpty()
            ? QStringLiteral("请输入表达式")
            : overlayState.expressionText;
        drawValueRow(0, QStringLiteral("表达式"), expressionText, true);
    }
    else
    {
        const QString xLabel = overlayState.xLocked ? QStringLiteral("X [锁]") : QStringLiteral("X");
        const QString yLabel = overlayState.yLocked ? QStringLiteral("Y [锁]") : QStringLiteral("Y");
        drawValueRow(0, xLabel, overlayState.xValueText, overlayState.xActive);
        drawValueRow(1, yLabel, overlayState.yValueText, overlayState.yActive);
    }

    const int hintTop = rowTop + rowCount * rowHeight + 4;
    const QString hintText = overlayState.stageHint.trimmed().isEmpty()
        ? QStringLiteral("Tab切换字段，Enter确认")
        : overlayState.stageHint;
    painter.setFont(valueFont);
    painter.setPen(QColor(156, 178, 202, 228));
    painter.drawText(QRect(contentRect.left(), hintTop, contentRect.width(), hintHeight), Qt::AlignLeft | Qt::AlignVCenter, hintText);
}

void CadViewer::renderDynamicCommandOverlay()
{
    if (interactionMode() != ViewInteractionMode::Idle)
    {
        return;
    }

    const CadDynamicCommandOverlayState overlayState = m_controller.dynamicCommandOverlayState();

    if (!overlayState.visible)
    {
        return;
    }

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setRenderHint(QPainter::TextAntialiasing, true);

    QFont titleFont = painter.font();
    titleFont.setPointSize(9);
    titleFont.setBold(true);

    QFont rowFont = painter.font();
    rowFont.setPointSize(9);

    const QFontMetrics titleMetrics(titleFont);
    const QFontMetrics rowMetrics(rowFont);
    const int horizontalPadding = 8;
    const int verticalPadding = 6;
    const int rowHeight = rowMetrics.height() + 6;
    const int titleHeight = titleMetrics.height();
    const int maxRows = std::min(6, static_cast<int>(overlayState.candidates.size()));
    const int panelWidth = 320;
    const int panelHeight = verticalPadding * 2 + titleHeight + 4 + maxRows * rowHeight + 4 + rowMetrics.height();

    QPoint panelTopLeft = m_cursorScreenPos + QPoint(18, 24);

    if (panelTopLeft.x() + panelWidth > width() - 6)
    {
        panelTopLeft.setX(std::max(6, width() - panelWidth - 6));
    }

    if (panelTopLeft.y() + panelHeight > height() - 6)
    {
        panelTopLeft.setY(std::max(6, m_cursorScreenPos.y() - panelHeight - 16));
    }

    const QRect panelRect(panelTopLeft, QSize(panelWidth, panelHeight));
    painter.setPen(QPen(QColor(90, 168, 252, 220), 1.0));
    painter.setBrush(QColor(18, 24, 34, 228));
    painter.drawRoundedRect(panelRect, 7.0, 7.0);

    const QRect contentRect = panelRect.adjusted(horizontalPadding, verticalPadding, -horizontalPadding, -verticalPadding);
    painter.setFont(titleFont);
    painter.setPen(QColor(238, 245, 255, 236));
    const QString titleText = QStringLiteral("命令: %1").arg(overlayState.inputText);
    painter.drawText(QRect(contentRect.left(), contentRect.top(), contentRect.width(), titleHeight), Qt::AlignLeft | Qt::AlignVCenter, titleText);

    painter.setFont(rowFont);
    const int rowsTop = contentRect.top() + titleHeight + 4;
    const int activeIndex = std::clamp(overlayState.activeCandidateIndex, 0, std::max(0, maxRows - 1));

    for (int row = 0; row < maxRows; ++row)
    {
        const QRect rowRect(contentRect.left(), rowsTop + row * rowHeight, contentRect.width(), rowHeight);
        const bool active = row == activeIndex;
        const QString rowText = overlayState.candidates.at(row);

        if (active)
        {
            painter.setPen(Qt::NoPen);
            painter.setBrush(QColor(82, 162, 246, 92));
            painter.drawRoundedRect(rowRect.adjusted(0, 0, 0, -1), 4.0, 4.0);
        }

        painter.setPen(active ? QColor(252, 253, 255) : QColor(214, 225, 239));
        painter.drawText(rowRect.adjusted(6, 0, -6, 0), Qt::AlignLeft | Qt::AlignVCenter, rowText);
    }

    const int hintTop = rowsTop + maxRows * rowHeight + 4;
    painter.setPen(QColor(156, 178, 202, 228));
    painter.drawText
    (
        QRect(contentRect.left(), hintTop, contentRect.width(), rowMetrics.height()),
        Qt::AlignLeft | Qt::AlignVCenter,
        overlayState.hintText
    );
}

bool CadViewer::pickSelectedHandle(const QPoint& screenPos, CadSelectionHandleInfo* outHandle) const
{
    const CadItem* selectedItem = selectedEntity();

    if (selectedItem == nullptr)
    {
        return false;
    }

    const QVector<CadSelectionHandleInfo> handles = buildSelectionHandleInfo(selectedItem);

    if (handles.isEmpty())
    {
        return false;
    }

    const float handlePickDistanceSquared = kObjectSnapDistancePixels * kObjectSnapDistancePixels;
    const QVector<int> candidateIndices = collectEditableHandleCandidates
    (
        handles,
        [this](const QVector3D& worldPosition)
        {
            return worldToScreen(worldPosition);
        },
        screenPos,
        handlePickDistanceSquared
    );

    if (candidateIndices.isEmpty())
    {
        return false;
    }

    int selectedIndex = candidateIndices.front();

    if (candidateIndices.size() >= 2
        && m_overlappedHandleHoverState.entityId == CadViewerUtils::toEntityId(selectedItem)
        && m_overlappedHandleHoverState.candidateIndices == candidateIndices)
    {
        int resolvedOrdinal = std::clamp
        (
            m_overlappedHandleHoverState.activeCandidateOrdinal,
            0,
            static_cast<int>(candidateIndices.size()) - 1
        );
        selectedIndex = candidateIndices.at(resolvedOrdinal);
    }

    if (outHandle != nullptr)
    {
        *outHandle = handles[selectedIndex];
    }

    return true;
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

void CadViewer::clearSelection()
{
    setSelectedEntityId(0);
    m_windowPreviewEntityIds.clear();
    resetOverlappedHandleHoverState();
    update();
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

QVector3D CadViewer::resolveInteractiveWorldPosition(const QPoint& screenPos) const
{
    return applySnapToGroundPosition(screenPos, screenToGroundPlane(screenPos));
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

    if (event->button() == Qt::LeftButton && handleOverlappedHandlePopupPress(event->pos()))
    {
        event->accept();
        updateHoveredWorldPosition(event->pos());
        return;
    }

    // 尝试由控制器处理鼠标按下事件
    if (!m_controller.handleMousePress(event))
    {
        // 如果控制器未处理，则传递给父类
        QOpenGLWidget::mousePressEvent(event);
    }

    // 更新悬停世界位置
    updateHoveredWorldPosition(event->pos());
    updateOverlappedHandleHoverState(event->pos());
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
        updateOverlappedHandleHoverState(event->pos());
    }
    else
    {
        resetOverlappedHandleHoverState();
    }

    update();
}

// 鼠标离开事件处理
// @param event 离开事件
void CadViewer::leaveEvent(QEvent* event)
{
    m_showCrosshairOverlay = false;
    resetOverlappedHandleHoverState();
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
    updateOverlappedHandleHoverState(event->pos());
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
    const bool dynamicCommandOverlayVisible = m_controller.dynamicCommandOverlayState().visible;

    if (!dynamicCommandOverlayVisible
        && event->key() == Qt::Key_Tab
        && isPopupCycleModifierValid(event->modifiers()))
    {
        const int step = (event->modifiers() & Qt::ShiftModifier) != 0 ? -1 : 1;

        if (cycleOverlappedHandleCandidate(step))
        {
            event->accept();
            return;
        }
    }

    if (!m_controller.handleKeyPress(event))
    {
        QOpenGLWidget::keyPressEvent(event);
    }

    updateOverlappedHandleHoverState(m_cursorScreenPos);
    refreshCommandPrompt();
    update();
}

bool CadViewer::focusNextPrevChild(bool next)
{
    Q_UNUSED(next);
    return false;
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
        label.selected = entity->m_isSelected;
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

void CadViewer::renderEntitySelectionOverlays()
{
    CadDocument* scene = m_sceneCoordinator.document();

    if (scene == nullptr || interactionMode() != ViewInteractionMode::Idle)
    {
        return;
    }

    if (m_selectedEntityIds.isEmpty() && m_windowPreviewEntityIds.isEmpty())
    {
        return;
    }

    const QMatrix4x4 viewProjection = m_camera.viewProjectionMatrix(aspectRatio());
    const QColor committedColor(0, 188, 255, 228);
    const QColor previewColor = m_selectionWindowPreview.crossingSelection
        ? QColor(66, 205, 104, 220)
        : QColor(74, 152, 250, 220);

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);

    for (const std::unique_ptr<CadItem>& entity : scene->m_entities)
    {
        if (entity == nullptr)
        {
            continue;
        }

        const EntityId id = CadViewerUtils::toEntityId(entity.get());
        const bool committedSelected = m_selectedEntityIds.contains(id);
        const bool previewSelected = m_windowPreviewEntityIds.contains(id);

        if (!committedSelected && !previewSelected)
        {
            continue;
        }

        const QVector<QVector3D>& worldVertices = entity->m_geometry.vertices;

        if (worldVertices.isEmpty())
        {
            continue;
        }

        QPolygonF projectedPolyline;
        projectedPolyline.reserve(worldVertices.size());
        bool validProjection = true;

        for (const QVector3D& worldVertex : worldVertices)
        {
            const QPointF screenPoint = CadViewerUtils::projectToScreen
            (
                worldVertex,
                viewProjection,
                m_viewportWidth,
                m_viewportHeight
            );

            if (!qIsFinite(screenPoint.x()) || !qIsFinite(screenPoint.y()))
            {
                validProjection = false;
                break;
            }

            projectedPolyline << screenPoint;
        }

        if (!validProjection || projectedPolyline.isEmpty())
        {
            continue;
        }

        const QColor mainColor = committedSelected ? committedColor : previewColor;
        const QColor glowColor = committedSelected
            ? QColor(255, 255, 255, 235)
            : QColor(232, 246, 255, 220);

        QPen glowPen(glowColor, committedSelected ? 2.4 : 2.0, Qt::SolidLine);
        QPen mainPen(mainColor, committedSelected ? 1.5 : 1.2, Qt::DashLine);
        mainPen.setDashPattern(committedSelected ? QList<qreal>{ 7.0, 3.0 } : QList<qreal>{ 5.0, 4.0 });

        painter.setBrush(Qt::NoBrush);

        if (projectedPolyline.size() == 1)
        {
            const QPointF center = projectedPolyline.front();
            painter.setPen(glowPen);
            painter.drawEllipse(center, 5.8, 5.8);
            painter.setPen(mainPen);
            painter.drawEllipse(center, 4.0, 4.0);
        }
        else
        {
            painter.setPen(glowPen);
            painter.drawPolyline(projectedPolyline);

            painter.setPen(mainPen);
            painter.drawPolyline(projectedPolyline);
        }

        // 仅绘制四角角标，避免完整包围框遮挡视野。
        const QRectF bounds = projectedPolyline.boundingRect().adjusted(-4.0, -4.0, 4.0, 4.0);
        const qreal maxCornerLength = 10.0;
        const qreal minCornerLength = 4.0;
        const qreal cornerLength = std::clamp
        (
            std::min(bounds.width(), bounds.height()) * 0.28,
            minCornerLength,
            maxCornerLength
        );

        QPen cornerPen(mainColor, committedSelected ? 1.4 : 1.2, Qt::SolidLine);
        painter.setPen(cornerPen);

        const qreal left = bounds.left();
        const qreal right = bounds.right();
        const qreal top = bounds.top();
        const qreal bottom = bounds.bottom();

        // 左上
        painter.drawLine(QPointF(left, top), QPointF(left + cornerLength, top));
        painter.drawLine(QPointF(left, top), QPointF(left, top + cornerLength));
        // 右上
        painter.drawLine(QPointF(right, top), QPointF(right - cornerLength, top));
        painter.drawLine(QPointF(right, top), QPointF(right, top + cornerLength));
        // 左下
        painter.drawLine(QPointF(left, bottom), QPointF(left + cornerLength, bottom));
        painter.drawLine(QPointF(left, bottom), QPointF(left, bottom - cornerLength));
        // 右下
        painter.drawLine(QPointF(right, bottom), QPointF(right - cornerLength, bottom));
        painter.drawLine(QPointF(right, bottom), QPointF(right, bottom - cornerLength));
    }
}

void CadViewer::renderSelectionWindowPreview()
{
    if (!m_selectionWindowPreview.visible || interactionMode() != ViewInteractionMode::Idle)
    {
        return;
    }

    const QRect selectionRect = normalizedSelectionRect
    (
        m_selectionWindowPreview.anchorScreenPos,
        m_selectionWindowPreview.currentScreenPos
    );

    if (selectionRect.width() < kWindowSelectionMinimumPixels || selectionRect.height() < kWindowSelectionMinimumPixels)
    {
        return;
    }

    const bool crossingSelection = m_selectionWindowPreview.crossingSelection;
    const QColor borderColor = crossingSelection
        ? QColor(66, 205, 104, 220)
        : QColor(74, 152, 250, 220);
    const QColor fillColor = crossingSelection
        ? QColor(66, 205, 104, 42)
        : QColor(74, 152, 250, 42);

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, false);

    QPen borderPen(borderColor, 1.0, Qt::DashLine);
    borderPen.setDashPattern({ 7.0, 4.0 });

    painter.setPen(borderPen);
    painter.setBrush(fillColor);
    painter.drawRect(selectionRect);
}

// 处理文档场景变化
void CadViewer::handleDocumentSceneChanged()
{
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

// 更新悬停世界位置
// @param screenPos 屏幕坐标
void CadViewer::updateHoveredWorldPosition(const QPoint& screenPos)
{
    emit hoveredWorldPositionChanged(resolveInteractiveWorldPosition(screenPos));
}

QVector3D CadViewer::applySnapToGroundPosition
(
    const QPoint& screenPos,
    const QVector3D& worldPos,
    bool* snapped,
    bool* objectSnap
) const
{
    if (snapped != nullptr)
    {
        *snapped = false;
    }

    if (objectSnap != nullptr)
    {
        *objectSnap = false;
    }

    struct SnapCandidate
    {
        QVector3D position;
        float distanceSquared = std::numeric_limits<float>::max();
        int priority = 1;
        bool valid = false;
    };

    SnapCandidate bestCandidate;
    const float snapDistanceSquared = kObjectSnapDistancePixels * kObjectSnapDistancePixels;

    if (m_basePointSnapEnabled || m_controlPointSnapEnabled)
    {
        CadItem* selectedItem = selectedEntity();

        if (selectedItem != nullptr)
        {
            const QVector<CadSelectionHandleInfo> handles = buildSelectionHandleInfo(selectedItem);

            for (const CadSelectionHandleInfo& handle : handles)
            {
                if ((handle.isBasePoint && !m_basePointSnapEnabled)
                    || (!handle.isBasePoint && !m_controlPointSnapEnabled))
                {
                    continue;
                }

                const QPoint handleScreenPos = worldToScreen(handle.position);
                const float dx = static_cast<float>(handleScreenPos.x() - screenPos.x());
                const float dy = static_cast<float>(handleScreenPos.y() - screenPos.y());
                const float distanceSquared = dx * dx + dy * dy;

                if (distanceSquared > snapDistanceSquared)
                {
                    continue;
                }

                const int priority = handle.isBasePoint ? 0 : 1;
                const bool sameDistance = std::abs(distanceSquared - bestCandidate.distanceSquared) <= 1.0e-4f;

                if (!bestCandidate.valid
                    || distanceSquared < bestCandidate.distanceSquared
                    || (sameDistance && priority < bestCandidate.priority))
                {
                    bestCandidate.position = handle.position;
                    bestCandidate.distanceSquared = distanceSquared;
                    bestCandidate.priority = priority;
                    bestCandidate.valid = true;
                }
            }
        }
    }

    if (bestCandidate.valid)
    {
        if (snapped != nullptr)
        {
            *snapped = true;
        }

        if (objectSnap != nullptr)
        {
            *objectSnap = true;
        }

        return QVector3D(bestCandidate.position.x(), bestCandidate.position.y(), 0.0f);
    }

    if (m_gridSnapEnabled)
    {
        const float gridStep = currentGridStep();
        const float snappedX = std::round(worldPos.x() / gridStep) * gridStep;
        const float snappedY = std::round(worldPos.y() / gridStep) * gridStep;
        const QVector3D snappedGridPosition(snappedX, snappedY, 0.0f);
        const QPoint snappedGridScreenPos = worldToScreen(snappedGridPosition);
        const float dx = static_cast<float>(snappedGridScreenPos.x() - screenPos.x());
        const float dy = static_cast<float>(snappedGridScreenPos.y() - screenPos.y());
        const float distanceSquared = dx * dx + dy * dy;
        const float gridSnapDistanceSquared = kGridSnapDistancePixels * kGridSnapDistancePixels;

        if (distanceSquared <= gridSnapDistanceSquared)
        {
            if (snapped != nullptr)
            {
                *snapped = true;
            }

            return snappedGridPosition;
        }
    }

    return QVector3D(worldPos.x(), worldPos.y(), 0.0f);
}

std::vector<TransientPrimitive> CadViewer::buildSnapHighlightPrimitives() const
{
    if (!m_showCrosshairOverlay)
    {
        return {};
    }

    bool snapped = false;
    bool objectSnap = false;
    const QVector3D snappedPosition = applySnapToGroundPosition
    (
        m_cursorScreenPos,
        screenToGroundPlane(m_cursorScreenPos),
        &snapped,
        &objectSnap
    );

    if (!snapped)
    {
        return {};
    }

    const QColor outerColor = objectSnap ? m_theme.selectedControlPointColor : m_theme.accentColor;
    const QColor innerColor = m_theme.viewerBackgroundColor;

    TransientPrimitive outerPoint;
    outerPoint.primitiveType = GL_POINTS;
    outerPoint.color =
    {
        static_cast<float>(outerColor.redF()),
        static_cast<float>(outerColor.greenF()),
        static_cast<float>(outerColor.blueF())
    };
    outerPoint.pointSize = objectSnap ? 15.0f : 13.0f;
    outerPoint.roundPoint = true;
    outerPoint.vertices = { snappedPosition };

    TransientPrimitive innerPoint;
    innerPoint.primitiveType = GL_POINTS;
    innerPoint.color =
    {
        static_cast<float>(innerColor.redF()),
        static_cast<float>(innerColor.greenF()),
        static_cast<float>(innerColor.blueF())
    };
    innerPoint.pointSize = objectSnap ? 7.0f : 5.5f;
    innerPoint.roundPoint = true;
    innerPoint.vertices = { snappedPosition };

    return { outerPoint, innerPoint };
}

void CadViewer::computeVisibleGroundBounds(float& minX, float& maxX, float& minY, float& maxY) const
{
    minX = std::numeric_limits<float>::max();
    maxX = -std::numeric_limits<float>::max();
    minY = std::numeric_limits<float>::max();
    maxY = -std::numeric_limits<float>::max();

    const int safeWidth = std::max(1, width());
    const int safeHeight = std::max(1, height());
    const std::array<QPoint, 4> corners =
    {
        QPoint(0, 0),
        QPoint(safeWidth - 1, 0),
        QPoint(0, safeHeight - 1),
        QPoint(safeWidth - 1, safeHeight - 1)
    };

    bool hasValidCorner = false;

    for (const QPoint& corner : corners)
    {
        const QVector3D position = screenToGroundPlane(corner);

        if (!std::isfinite(position.x()) || !std::isfinite(position.y()))
        {
            continue;
        }

        minX = std::min(minX, position.x());
        maxX = std::max(maxX, position.x());
        minY = std::min(minY, position.y());
        maxY = std::max(maxY, position.y());
        hasValidCorner = true;
    }

    if (!hasValidCorner)
    {
        const float halfHeight = m_camera.viewHeight * 0.5f;
        const float halfWidth = halfHeight * aspectRatio();
        minX = m_camera.target.x() - halfWidth;
        maxX = m_camera.target.x() + halfWidth;
        minY = m_camera.target.y() - halfHeight;
        maxY = m_camera.target.y() + halfHeight;
    }
}

float CadViewer::currentGridStep() const
{
    float minX = 0.0f;
    float maxX = 0.0f;
    float minY = 0.0f;
    float maxY = 0.0f;
    computeVisibleGroundBounds(minX, maxX, minY, maxY);

    float gridStep = chooseHierarchicalGridStep(pixelToWorldScale() * kTargetGridSpacingPixels);
    const float width = std::max(maxX - minX, 0.0f);
    const float height = std::max(maxY - minY, 0.0f);

    while ((width / gridStep) > kMaxGridLineCountPerAxis
        || (height / gridStep) > kMaxGridLineCountPerAxis)
    {
        gridStep *= 2.0f;
    }

    return gridStep;
}

void CadViewer::setSelectedEntityId(EntityId entityId)
{
    QSet<EntityId> ids;

    if (entityId != 0)
    {
        ids.insert(entityId);
    }

    setSelectedEntities(ids, entityId);
}

void CadViewer::setSelectedEntities(const QSet<EntityId>& entityIds, EntityId preferredEntityId)
{
    CadDocument* scene = m_sceneCoordinator.document();
    QSet<EntityId> filteredIds;
    EntityId resolvedPrimaryId = 0;

    if (scene != nullptr)
    {
        filteredIds.reserve(entityIds.size());

        for (const std::unique_ptr<CadItem>& entity : scene->m_entities)
        {
            if (entity == nullptr)
            {
                continue;
            }

            const EntityId id = CadViewerUtils::toEntityId(entity.get());
            const bool selected = entityIds.contains(id);
            entity->m_isSelected = selected;

            if (selected)
            {
                filteredIds.insert(id);
            }
        }
    }

    if (preferredEntityId != 0 && filteredIds.contains(preferredEntityId))
    {
        resolvedPrimaryId = preferredEntityId;
    }
    else if (!filteredIds.isEmpty() && scene != nullptr)
    {
        for (const std::unique_ptr<CadItem>& entity : scene->m_entities)
        {
            if (entity == nullptr)
            {
                continue;
            }

            const EntityId id = CadViewerUtils::toEntityId(entity.get());

            if (filteredIds.contains(id))
            {
                resolvedPrimaryId = id;
                break;
            }
        }
    }

    const bool selectionChanged = m_selectedEntityId != resolvedPrimaryId || m_selectedEntityIds != filteredIds;
    m_selectedEntityId = resolvedPrimaryId;
    m_selectedEntityIds = filteredIds;

    if (selectionChanged)
    {
        resetOverlappedHandleHoverState();
        emit selectedEntityChanged(selectedEntity());
    }
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
    return CadPreviewBuilder::buildTransientPrimitives(m_controller.drawState(), selectedEntity(), selectedEntities());
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

    const float pixelScale = std::max(pixelToWorldScale(), 1.0e-4f);
    const float headLength = pixelScale * 9.2f;
    const float headHalfWidth = pixelScale * 4.4f;

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

        // 按用户要求：箭头尖端直接锚定在加工起始点（不做前向偏移）。
        const QVector3D tip = info.startPoint;
        const QVector3D headBase = tip - info.direction * headLength;

        QVector3D arrowColor;
        if (entity->m_isSelected)
        {
            arrowColor = QVector3D(1.0f, 0.78f, 0.30f);
        }
        else if (info.processOrder >= 0)
        {
            arrowColor = info.isReverse ? QVector3D(1.0f, 0.42f, 0.28f) : QVector3D(0.12f, 0.92f, 0.72f);
        }
        else
        {
            arrowColor = info.isReverse ? QVector3D(0.82f, 0.34f, 0.26f) : QVector3D(0.34f, 0.78f, 0.58f);
        }

        // 仅保留小实心三角箭头头
        TransientPrimitive headPrimitive;
        headPrimitive.primitiveType = GL_TRIANGLES;
        headPrimitive.color = arrowColor;
        headPrimitive.vertices =
        {
            tip,
            headBase + perpendicular * headHalfWidth,
            headBase - perpendicular * headHalfWidth
        };
        primitives.push_back(std::move(headPrimitive));
    }

    return primitives;
}

std::vector<TransientPrimitive> CadViewer::buildSelectedEntityHandlePrimitives() const
{
    const CadItem* selectedItem = selectedEntity();

    if (selectedItem == nullptr)
    {
        return {};
    }

    const QVector<CadSelectionHandleInfo> handles = buildSelectionHandleInfo(selectedItem);

    if (handles.isEmpty())
    {
        return {};
    }

    const int hoveredHandleIndex = resolveHoveredHandleIndex(handles);

    TransientPrimitive basePointOuterPrimitive;
    basePointOuterPrimitive.primitiveType = GL_POINTS;
    basePointOuterPrimitive.color =
    {
        static_cast<float>(m_theme.selectedBasePointColor.redF()),
        static_cast<float>(m_theme.selectedBasePointColor.greenF()),
        static_cast<float>(m_theme.selectedBasePointColor.blueF())
    };
    basePointOuterPrimitive.pointSize = 13.0f;
    basePointOuterPrimitive.roundPoint = true;

    TransientPrimitive basePointInnerPrimitive;
    basePointInnerPrimitive.primitiveType = GL_POINTS;
    basePointInnerPrimitive.color =
    {
        static_cast<float>(m_theme.viewerBackgroundColor.redF()),
        static_cast<float>(m_theme.viewerBackgroundColor.greenF()),
        static_cast<float>(m_theme.viewerBackgroundColor.blueF())
    };
    basePointInnerPrimitive.pointSize = 6.2f;
    basePointInnerPrimitive.roundPoint = true;

    TransientPrimitive controlPointOuterPrimitive;
    controlPointOuterPrimitive.primitiveType = GL_POINTS;
    controlPointOuterPrimitive.color =
    {
        static_cast<float>(m_theme.selectedControlPointColor.redF()),
        static_cast<float>(m_theme.selectedControlPointColor.greenF()),
        static_cast<float>(m_theme.selectedControlPointColor.blueF())
    };
    controlPointOuterPrimitive.pointSize = 10.0f;
    controlPointOuterPrimitive.roundPoint = true;

    TransientPrimitive controlPointInnerPrimitive;
    controlPointInnerPrimitive.primitiveType = GL_POINTS;
    controlPointInnerPrimitive.color =
    {
        static_cast<float>(m_theme.viewerBackgroundColor.redF()),
        static_cast<float>(m_theme.viewerBackgroundColor.greenF()),
        static_cast<float>(m_theme.viewerBackgroundColor.blueF())
    };
    controlPointInnerPrimitive.pointSize = 4.4f;
    controlPointInnerPrimitive.roundPoint = true;

    TransientPrimitive trianglePrimitive;
    trianglePrimitive.primitiveType = GL_TRIANGLES;
    trianglePrimitive.color = controlPointOuterPrimitive.color;

    TransientPrimitive hoveredPointOuterPrimitive;
    hoveredPointOuterPrimitive.primitiveType = GL_POINTS;
    hoveredPointOuterPrimitive.color =
    {
        static_cast<float>(m_theme.accentColor.redF()),
        static_cast<float>(m_theme.accentColor.greenF()),
        static_cast<float>(m_theme.accentColor.blueF())
    };
    hoveredPointOuterPrimitive.pointSize = 17.0f;
    hoveredPointOuterPrimitive.roundPoint = true;

    TransientPrimitive hoveredPointInnerPrimitive;
    hoveredPointInnerPrimitive.primitiveType = GL_POINTS;
    hoveredPointInnerPrimitive.color = { 1.0f, 1.0f, 1.0f };
    hoveredPointInnerPrimitive.pointSize = 9.0f;
    hoveredPointInnerPrimitive.roundPoint = true;

    TransientPrimitive hoveredTrianglePrimitive;
    hoveredTrianglePrimitive.primitiveType = GL_TRIANGLES;
    hoveredTrianglePrimitive.color = hoveredPointOuterPrimitive.color;

    const float pixelScale = std::max(pixelToWorldScale(), 1.0e-4f);
    const float triangleSize = pixelScale * 6.5f;
    const float hoveredTriangleSize = pixelScale * 8.4f;

    for (int handleIndex = 0; handleIndex < handles.size(); ++handleIndex)
    {
        const CadSelectionHandleInfo& handle = handles.at(handleIndex);

        if (handle.shape == CadSelectionHandleShape::Triangle)
        {
            const QVector<QVector3D> triangleVertices = buildTriangleVertices(handle.position, handle.direction, triangleSize);
            trianglePrimitive.vertices += triangleVertices;

            if (handleIndex == hoveredHandleIndex)
            {
                const QVector<QVector3D> hoveredTriangleVertices = buildTriangleVertices
                (
                    handle.position,
                    handle.direction,
                    hoveredTriangleSize
                );
                hoveredTrianglePrimitive.vertices += hoveredTriangleVertices;
            }
        }
        else if (handle.isBasePoint)
        {
            basePointOuterPrimitive.vertices.push_back(handle.position);
            basePointInnerPrimitive.vertices.push_back(handle.position);
        }
        else
        {
            controlPointOuterPrimitive.vertices.push_back(handle.position);
            controlPointInnerPrimitive.vertices.push_back(handle.position);
        }

        if (handleIndex == hoveredHandleIndex)
        {
            hoveredPointOuterPrimitive.vertices.push_back(handle.position);
            hoveredPointInnerPrimitive.vertices.push_back(handle.position);
        }
    }

    std::vector<TransientPrimitive> primitives;

    if (!hoveredPointOuterPrimitive.vertices.isEmpty())
    {
        primitives.push_back(std::move(hoveredPointOuterPrimitive));
    }

    if (!controlPointOuterPrimitive.vertices.isEmpty())
    {
        primitives.push_back(std::move(controlPointOuterPrimitive));
    }

    if (!controlPointInnerPrimitive.vertices.isEmpty())
    {
        primitives.push_back(std::move(controlPointInnerPrimitive));
    }

    if (!hoveredPointInnerPrimitive.vertices.isEmpty())
    {
        primitives.push_back(std::move(hoveredPointInnerPrimitive));
    }

    if (!trianglePrimitive.vertices.isEmpty())
    {
        primitives.push_back(std::move(trianglePrimitive));
    }

    if (!hoveredTrianglePrimitive.vertices.isEmpty())
    {
        primitives.push_back(std::move(hoveredTrianglePrimitive));
    }

    if (!basePointOuterPrimitive.vertices.isEmpty())
    {
        primitives.push_back(std::move(basePointOuterPrimitive));
    }

    if (!basePointInnerPrimitive.vertices.isEmpty())
    {
        primitives.push_back(std::move(basePointInnerPrimitive));
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





