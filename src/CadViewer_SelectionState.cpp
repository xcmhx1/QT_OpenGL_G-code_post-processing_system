// CadViewer 选择状态实现
#include "pch.h"

#include "CadViewer.h"

#include "CadDocument.h"
#include "CadEntityPicker.h"
#include "CadInteractionConstants.h"
#include "CadItem.h"
#include "CadViewerUtils.h"

#include <memory>
#include <vector>

namespace
{
    constexpr int kWindowSelectionMinimumPixels = 2;

    QRect normalizedSelectionRect(const QPoint& anchorScreenPos, const QPoint& currentScreenPos)
    {
        return QRect(anchorScreenPos, currentScreenPos).normalized();
    }
}

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

void CadViewer::clearSelection()
{
    setSelectedEntityId(0);
    m_windowPreviewEntityIds.clear();
    resetOverlappedHandleHoverState();
    update();
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
        invalidateSnapCache();
        resetOverlappedHandleHoverState();
        emit selectedEntityChanged(selectedEntity());
    }
}
