// CadViewer 选择状态实现
#include "platform/pch.h"

#include "cad/view/CadViewer.h"

#include "cad/document/CadDocument.h"
#include "cad/view/interaction/CadEntityPicker.h"
#include "cad/view/interaction/CadInteractionConstants.h"
#include "cad/items/CadItem.h"
#include "cad/view/CadViewerUtils.h"

#include <QHash>

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
    const RenderEntityKey pickedRenderKey = pickRenderKey(screenPos);

    if (updateMode == SelectionUpdateMode::Toggle)
    {
        if (pickedRenderKey.valid())
        {
            QSet<RenderEntityKey> selectedRenderKeys = m_selectedRenderKeys;
            RenderEntityKey preferredRenderKey = m_selectedRenderKey;

            if (selectedRenderKeys.contains(pickedRenderKey))
            {
                selectedRenderKeys.remove(pickedRenderKey);

                if (preferredRenderKey == pickedRenderKey)
                {
                    preferredRenderKey = {};
                }
            }
            else
            {
                selectedRenderKeys.insert(pickedRenderKey);
                preferredRenderKey = pickedRenderKey;
            }

            setSelectedRenderKeys(selectedRenderKeys, preferredRenderKey);
        }
    }
    else
    {
        setSelectedRenderKey(pickedRenderKey);
    }

    update();
}

void CadViewer::selectEntitiesInWindow
(
    const QPoint& startScreenPos,
    const QPoint& endScreenPos,
    bool crossingSelection,
    SelectionUpdateMode updateMode,
    WindowSelectionTarget target
)
{
    CadDocument* scene = m_sceneCoordinator.document();

    if (scene == nullptr)
    {
        if (updateMode == SelectionUpdateMode::Replace)
        {
            setSelectedRenderKeys({}, {});
        }

        update();
        return;
    }

    const QRect selectionRect = normalizedSelectionRect(startScreenPos, endScreenPos);

    if (selectionRect.width() < kWindowSelectionMinimumPixels || selectionRect.height() < kWindowSelectionMinimumPixels)
    {
        if (updateMode == SelectionUpdateMode::Replace)
        {
            setSelectedRenderKeys({}, {});
        }

        update();
        return;
    }

    const std::vector<RenderEntityKey> pickedRenderKeys = CadEntityPicker::pickEntitiesByWindow
    (
        scene->m_entities,
        m_camera.viewProjectionMatrix(aspectRatio()),
        m_viewportWidth,
        m_viewportHeight,
        QRectF(selectionRect),
        crossingSelection
    );

    QSet<RenderEntityKey> selectedRenderKeys;
    selectedRenderKeys.reserve(static_cast<qsizetype>(pickedRenderKeys.size()));

    for (RenderEntityKey renderKey : pickedRenderKeys)
    {
        if (renderKey.valid())
        {
            selectedRenderKeys.insert(renderKey);
        }
    }

    ProcessUnitSelectionExpansion processUnitExpansion;

    if (target == WindowSelectionTarget::ProcessUnit)
    {
        processUnitExpansion = expandRenderKeysToProcessUnits(selectedRenderKeys);
        selectedRenderKeys = processUnitExpansion.expandedRenderKeys;
    }

    RenderEntityKey preferredRenderKey = pickedRenderKeys.empty()
        ? RenderEntityKey{} : pickedRenderKeys.front();

    if (updateMode == SelectionUpdateMode::Toggle)
    {
        QSet<RenderEntityKey> mergedSelection = m_selectedRenderKeys;

        if (target == WindowSelectionTarget::ProcessUnit
            && !processUnitExpansion.processUnitRenderKeyGroups.empty())
        {
            for (const QSet<RenderEntityKey>& unitRenderKeys
                : processUnitExpansion.processUnitRenderKeyGroups)
            {
                bool allSelected = !unitRenderKeys.isEmpty();

                for (RenderEntityKey renderKey : unitRenderKeys)
                {
                    if (!m_selectedRenderKeys.contains(renderKey))
                    {
                        allSelected = false;
                        break;
                    }
                }

                if (allSelected)
                {
                    for (RenderEntityKey renderKey : unitRenderKeys)
                    {
                        mergedSelection.remove(renderKey);
                    }
                }
                else
                {
                    mergedSelection.unite(unitRenderKeys);
                }
            }

            for (RenderEntityKey renderKey : processUnitExpansion.unassignedRenderKeys)
            {
                if (mergedSelection.contains(renderKey))
                {
                    mergedSelection.remove(renderKey);
                }
                else
                {
                    mergedSelection.insert(renderKey);
                }
            }
        }
        else
        {
            for (RenderEntityKey renderKey : selectedRenderKeys)
            {
                if (mergedSelection.contains(renderKey))
                {
                    mergedSelection.remove(renderKey);
                }
                else
                {
                    mergedSelection.insert(renderKey);
                }
            }
        }

        preferredRenderKey = m_selectedRenderKey;

        for (RenderEntityKey renderKey : pickedRenderKeys)
        {
            if (renderKey.valid() && mergedSelection.contains(renderKey))
            {
                preferredRenderKey = renderKey;
                break;
            }
        }

        if (preferredRenderKey.valid() && !mergedSelection.contains(preferredRenderKey))
        {
            preferredRenderKey = {};
        }

        setSelectedRenderKeys(mergedSelection, preferredRenderKey);
    }
    else
    {
        setSelectedRenderKeys(selectedRenderKeys, preferredRenderKey);
    }

    m_windowPreviewRenderKeys.clear();
    update();
}

CadViewer::ProcessUnitSelectionExpansion CadViewer::expandRenderKeysToProcessUnits
(
    const QSet<RenderEntityKey>& renderKeys
) const
{
    ProcessUnitSelectionExpansion expansion;
    expansion.expandedRenderKeys = renderKeys;
    expansion.unassignedRenderKeys = renderKeys;

    CadDocument* scene = m_sceneCoordinator.document();

    if (scene == nullptr || m_processPresentation == nullptr || renderKeys.isEmpty())
    {
        return expansion;
    }

    QHash<cadcam::geometry::EntityId, RenderEntityKey> renderKeyByEntityId;
    renderKeyByEntityId.reserve(static_cast<qsizetype>(scene->m_entities.size()));

    for (const std::unique_ptr<CadItem>& entity : scene->m_entities)
    {
        if (entity == nullptr || entity->m_entityId == 0)
        {
            continue;
        }

        const RenderEntityKey renderKey = CadViewerUtils::toRenderEntityKey(entity.get());

        if (renderKey.valid())
        {
            renderKeyByEntityId.insert(entity->m_entityId, renderKey);
        }
    }

    for (const cadcam::process::ProcessUnitPresentation& processUnit
        : m_processPresentation->processUnits)
    {
        bool unitHit = false;

        for (cadcam::geometry::EntityId entityId : processUnit.orderedMemberEntityIds)
        {
            const auto iterator = renderKeyByEntityId.constFind(entityId);

            if (iterator != renderKeyByEntityId.cend() && renderKeys.contains(iterator.value()))
            {
                unitHit = true;
                break;
            }
        }

        if (!unitHit)
        {
            continue;
        }

        QSet<RenderEntityKey> unitRenderKeys;
        unitRenderKeys.reserve
        (
            static_cast<qsizetype>(processUnit.orderedMemberEntityIds.size())
        );

        for (cadcam::geometry::EntityId entityId : processUnit.orderedMemberEntityIds)
        {
            const auto iterator = renderKeyByEntityId.constFind(entityId);

            if (iterator == renderKeyByEntityId.cend())
            {
                continue;
            }

            unitRenderKeys.insert(iterator.value());
            expansion.expandedRenderKeys.insert(iterator.value());
            expansion.unassignedRenderKeys.remove(iterator.value());
        }

        if (!unitRenderKeys.isEmpty())
        {
            expansion.processUnitRenderKeyGroups.push_back(std::move(unitRenderKeys));
        }
    }

    return expansion;
}

QVector<CadItem*> CadViewer::selectedEntities() const
{
    QVector<CadItem*> entities;
    CadDocument* scene = m_sceneCoordinator.document();

    if (scene == nullptr || m_selectedRenderKeys.isEmpty())
    {
        return entities;
    }

    entities.reserve(static_cast<qsizetype>(m_selectedRenderKeys.size()));

    for (const std::unique_ptr<CadItem>& entity : scene->m_entities)
    {
        if (entity == nullptr)
        {
            continue;
        }

        const RenderEntityKey renderKey = CadViewerUtils::toRenderEntityKey(entity.get());

        if (m_selectedRenderKeys.contains(renderKey))
        {
            entities.push_back(entity.get());
        }
    }

    return entities;
}

void CadViewer::showSelectionWindowPreview
(
    const QPoint& anchorScreenPos,
    const QPoint& currentScreenPos,
    WindowSelectionTarget target
)
{
    m_selectionWindowPreview.visible = true;
    m_selectionWindowPreview.anchorScreenPos = anchorScreenPos;
    m_selectionWindowPreview.currentScreenPos = currentScreenPos;
    m_selectionWindowPreview.crossingSelection = currentScreenPos.x() < anchorScreenPos.x();
    m_selectionWindowPreview.target = target;
    updateSelectionWindowPreviewCandidates();
    update();
}

void CadViewer::hideSelectionWindowPreview()
{
    const bool hadVisiblePreview = m_selectionWindowPreview.visible;
    m_selectionWindowPreview.visible = false;

    if (m_windowPreviewRenderKeys.isEmpty() && !hadVisiblePreview)
    {
        return;
    }

    m_windowPreviewRenderKeys.clear();
    update();
}

void CadViewer::clearSelection()
{
    setSelectedRenderKey({});
    m_windowPreviewRenderKeys.clear();
    resetOverlappedHandleHoverState();
    update();
}

void CadViewer::setSelectedRenderKey(RenderEntityKey renderKey)
{
    QSet<RenderEntityKey> renderKeys;

    if (renderKey.valid())
    {
        renderKeys.insert(renderKey);
    }

    setSelectedRenderKeys(renderKeys, renderKey);
}

void CadViewer::setSelectedRenderKeys
(
    const QSet<RenderEntityKey>& renderKeys,
    RenderEntityKey preferredRenderKey
)
{
    CadDocument* scene = m_sceneCoordinator.document();
    QSet<RenderEntityKey> filteredRenderKeys;
    RenderEntityKey resolvedPrimaryRenderKey;

    if (scene != nullptr)
    {
        filteredRenderKeys.reserve(renderKeys.size());

        for (const std::unique_ptr<CadItem>& entity : scene->m_entities)
        {
            if (entity == nullptr)
            {
                continue;
            }

            const RenderEntityKey renderKey = CadViewerUtils::toRenderEntityKey(entity.get());
            const bool selected = renderKeys.contains(renderKey);
            entity->m_isSelected = selected;

            if (selected)
            {
                filteredRenderKeys.insert(renderKey);
            }
        }
    }

    if (preferredRenderKey.valid() && filteredRenderKeys.contains(preferredRenderKey))
    {
        resolvedPrimaryRenderKey = preferredRenderKey;
    }
    else if (!filteredRenderKeys.isEmpty() && scene != nullptr)
    {
        for (const std::unique_ptr<CadItem>& entity : scene->m_entities)
        {
            if (entity == nullptr)
            {
                continue;
            }

            const RenderEntityKey renderKey = CadViewerUtils::toRenderEntityKey(entity.get());

            if (filteredRenderKeys.contains(renderKey))
            {
                resolvedPrimaryRenderKey = renderKey;
                break;
            }
        }
    }

    const bool selectionChanged = m_selectedRenderKey != resolvedPrimaryRenderKey
        || m_selectedRenderKeys != filteredRenderKeys;
    m_selectedRenderKey = resolvedPrimaryRenderKey;
    m_selectedRenderKeys = filteredRenderKeys;

    if (selectionChanged)
    {
        invalidateSnapCache();
        resetOverlappedHandleHoverState();
        emit selectedEntityChanged(selectedEntity());
    }
}
