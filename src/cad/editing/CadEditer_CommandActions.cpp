// CadEditer 实现文件
// 实现 CadEditer 模块，对应头文件中声明的主要行为和协作流程。
// 编辑器模块，负责绘图创建、实体修改以及 Undo/Redo 命令栈管理。
#include "platform/pch.h"

#include "cad/editing/CadEditer.h"

#include "cad/document/CadDocument.h"
#include "cad/items/CadItem.h"
#include "cad/editing/DrawStateMachine.h"

#include <QSet>

#include <algorithm>
#include <cmath>
#include <limits>

#include "cad/editing/CadEditerWorkflowInternal.h"

using namespace CadEditerWorkflowInternal;

class ProcessUnitSequenceCommand final : public CadEditer::EditCommand
{
public:
    ProcessUnitSequenceCommand
    (
        cadcam::planning::ProcessUnitSequence before,
        cadcam::planning::ProcessUnitSequence after,
        CadEditer::ProcessUnitSequenceApply apply
    )
        : m_before(std::move(before))
        , m_after(std::move(after))
        , m_apply(std::move(apply))
    {
    }

    bool execute() override
    {
        return m_apply != nullptr && m_apply(m_after);
    }

    bool undo() override
    {
        return m_apply != nullptr && m_apply(m_before);
    }

private:
    cadcam::planning::ProcessUnitSequence m_before;
    cadcam::planning::ProcessUnitSequence m_after;
    CadEditer::ProcessUnitSequenceApply m_apply;
};

class ProcessUnitTraversalCommand final : public CadEditer::EditCommand
{
public:
    ProcessUnitTraversalCommand
    (
        cadcam::planning::ProcessUnitKey key,
        cadcam::process::ProcessUnitTraversalOverride beforeTraversal,
        std::optional<cadcam::process::ProcessUnitTraversalOverride> beforeStored,
        cadcam::process::ProcessUnitTraversalOverride afterTraversal,
        std::optional<cadcam::process::ProcessUnitTraversalOverride> afterStored,
        CadEditer::ProcessUnitTraversalApply apply
    )
        : m_key(std::move(key))
        , m_beforeTraversal(std::move(beforeTraversal))
        , m_beforeStored(std::move(beforeStored))
        , m_afterTraversal(std::move(afterTraversal))
        , m_afterStored(std::move(afterStored))
        , m_apply(std::move(apply))
    {
    }

    bool execute() override
    {
        return m_apply != nullptr
            && m_apply(m_key, m_afterTraversal, m_afterStored);
    }

    bool undo() override
    {
        return m_apply != nullptr
            && m_apply(m_key, m_beforeTraversal, m_beforeStored);
    }

private:
    cadcam::planning::ProcessUnitKey m_key;
    cadcam::process::ProcessUnitTraversalOverride m_beforeTraversal;
    std::optional<cadcam::process::ProcessUnitTraversalOverride> m_beforeStored;
    cadcam::process::ProcessUnitTraversalOverride m_afterTraversal;
    std::optional<cadcam::process::ProcessUnitTraversalOverride> m_afterStored;
    CadEditer::ProcessUnitTraversalApply m_apply;
};

// 添加实体命令：
// 负责把新建原生实体与对应 CadItem 一起插入文档，并支持撤销恢复。
class AddEntityCommand final : public CadEditer::EditCommand
{
public:
    AddEntityCommand(CadDocument* document, std::unique_ptr<DRW_Entity> entity)
        : m_document(document)
        , m_entity(std::move(entity))
    {
    }

    bool execute() override
    {
        // 第一次执行时创建 CadItem，后续 redo 复用已保存对象
        if (m_document == nullptr || m_entity == nullptr)
        {
            return false;
        }

        if (m_item == nullptr)
        {
            m_item = CadDocument::createCadItemForEntity(m_entity.get());
        }

        if (m_item == nullptr)
        {
            return false;
        }

        m_itemPtr = m_item.get();
        return m_document->appendEntity(std::move(m_entity), std::move(m_item)) != nullptr;
    }

    bool undo() override
    {
        // 撤销时把实体和图元从文档中整体取回，以便后续 redo 重新插入
        if (m_document == nullptr || m_itemPtr == nullptr)
        {
            return false;
        }

        auto [entity, item] = m_document->takeEntity(m_itemPtr);

        if (entity == nullptr || item == nullptr)
        {
            return false;
        }

        m_entity = std::move(entity);
        m_item = std::move(item);
        m_itemPtr = m_item.get();
        return true;
    }

private:
    // 目标文档
    CadDocument* m_document = nullptr;

    // 待插入或撤销后保存的原生实体
    std::unique_ptr<DRW_Entity> m_entity;

    // 与原生实体对应的图元对象
    std::unique_ptr<CadItem> m_item;

    // 当前图元裸指针，用于与文档接口协作
    CadItem* m_itemPtr = nullptr;
};

class AddEntitiesCommand final : public CadEditer::EditCommand
{
public:
    AddEntitiesCommand(CadDocument* document, std::vector<std::unique_ptr<DRW_Entity>> entities)
        : m_document(document)
        , m_entities(std::move(entities))
        , m_items(m_entities.size())
        , m_itemPtrs(m_entities.size(), nullptr)
    {
    }

    bool execute() override
    {
        if (m_document == nullptr || m_entities.empty())
        {
            return false;
        }

        for (int index = 0; index < static_cast<int>(m_entities.size()); ++index)
        {
            if (m_entities[index] == nullptr)
            {
                return false;
            }

            if (m_items[index] == nullptr)
            {
                m_items[index] = CadDocument::createCadItemForEntity(m_entities[index].get());
            }

            if (m_items[index] == nullptr)
            {
                return false;
            }

            m_itemPtrs[index] = m_items[index].get();

            if (m_document->appendEntity(std::move(m_entities[index]), std::move(m_items[index])) == nullptr)
            {
                return false;
            }
        }

        return true;
    }

    bool undo() override
    {
        if (m_document == nullptr)
        {
            return false;
        }

        for (int index = static_cast<int>(m_itemPtrs.size()) - 1; index >= 0; --index)
        {
            if (m_itemPtrs[index] == nullptr)
            {
                return false;
            }

            auto [entity, item] = m_document->takeEntity(m_itemPtrs[index]);

            if (entity == nullptr || item == nullptr)
            {
                return false;
            }

            m_entities[index] = std::move(entity);
            m_items[index] = std::move(item);
            m_itemPtrs[index] = m_items[index].get();
        }

        return true;
    }

private:
    CadDocument* m_document = nullptr;
    std::vector<std::unique_ptr<DRW_Entity>> m_entities;
    std::vector<std::unique_ptr<CadItem>> m_items;
    std::vector<CadItem*> m_itemPtrs;
};

class ReplaceEntitiesCommand final : public CadEditer::EditCommand
{
public:
    ReplaceEntitiesCommand
    (
        CadDocument* document,
        cadcam::process::DocumentProcessState* processState,
        const QVector<CadItem*>& sourceItems,
        std::vector<std::unique_ptr<DRW_Entity>> replacementEntities
    )
        : m_document(document)
        , m_processState(processState)
        , m_replacementEntities(std::move(replacementEntities))
        , m_replacementItems(m_replacementEntities.size())
        , m_replacementItemPtrs(m_replacementEntities.size(), nullptr)
    {
        QSet<CadItem*> deduplicated;

        for (CadItem* item : sourceItems)
        {
            if (item == nullptr || deduplicated.contains(item))
            {
                continue;
            }

            deduplicated.insert(item);
            SourceState state;
            state.itemPtr = item;
            if (m_processState != nullptr)
            {
                if (const auto* saved = m_processState->find(item->m_entityId))
                    state.processState = *saved;
            }
            m_sourceStates.push_back(std::move(state));
        }
        if (m_processState != nullptr)
            m_beforeSequence = m_processState->processUnitSequence();
    }

    bool execute() override
    {
        if (m_document == nullptr || m_sourceStates.empty() || m_replacementEntities.empty())
        {
            return false;
        }

        for (SourceState& state : m_sourceStates)
        {
            if (state.itemPtr == nullptr)
            {
                return false;
            }

            auto [entity, item] = m_document->takeEntity(state.itemPtr);

            if (entity == nullptr || item == nullptr)
            {
                return false;
            }

            state.entity = std::move(entity);
            state.item = std::move(item);
            state.itemPtr = state.item.get();
        }

        for (int index = 0; index < static_cast<int>(m_replacementEntities.size()); ++index)
        {
            if (m_replacementEntities[index] == nullptr)
            {
                return false;
            }

            if (m_replacementItems[index] == nullptr)
            {
                m_replacementItems[index] = CadDocument::createCadItemForEntity(m_replacementEntities[index].get());
            }

            if (m_replacementItems[index] == nullptr)
            {
                return false;
            }

            m_replacementItemPtrs[index] = m_replacementItems[index].get();

            if (m_document->appendEntity(std::move(m_replacementEntities[index]), std::move(m_replacementItems[index])) == nullptr)
            {
                return false;
            }
        }

        if (m_processState != nullptr)
        {
            m_processState->beginBatch();
            for (const SourceState& state : m_sourceStates)
                m_processState->erase(state.itemPtr->m_entityId);
            if (m_afterSequence.has_value())
                m_processState->setProcessUnitSequence(m_afterSequence->units);
            m_processState->endBatch();
            if (!m_afterSequence.has_value())
                m_afterSequence = m_processState->processUnitSequence();
        }

        return true;
    }

    bool undo() override
    {
        if (m_document == nullptr)
        {
            return false;
        }

        for (int index = static_cast<int>(m_replacementItemPtrs.size()) - 1; index >= 0; --index)
        {
            if (m_replacementItemPtrs[index] == nullptr)
            {
                return false;
            }

            auto [entity, item] = m_document->takeEntity(m_replacementItemPtrs[index]);

            if (entity == nullptr || item == nullptr)
            {
                return false;
            }

            m_replacementEntities[index] = std::move(entity);
            m_replacementItems[index] = std::move(item);
            m_replacementItemPtrs[index] = m_replacementItems[index].get();
        }

        for (SourceState& state : m_sourceStates)
        {
            if (state.entity == nullptr || state.item == nullptr)
            {
                return false;
            }

            state.itemPtr = state.item.get();

            if (m_document->appendEntity(std::move(state.entity), std::move(state.item)) == nullptr)
            {
                return false;
            }
        }

        if (m_processState != nullptr)
        {
            m_processState->beginBatch();
            for (CadItem* item : m_replacementItemPtrs)
            {
                if (item != nullptr) m_processState->erase(item->m_entityId);
            }
            for (const SourceState& state : m_sourceStates)
            {
                if (state.processState.has_value())
                    m_processState->setState(state.itemPtr->m_entityId, *state.processState);
            }
            if (m_beforeSequence.has_value())
                m_processState->setProcessUnitSequence(m_beforeSequence->units);
            m_processState->endBatch();
        }

        return true;
    }

private:
    struct SourceState
    {
        CadItem* itemPtr = nullptr;
        std::unique_ptr<DRW_Entity> entity;
        std::unique_ptr<CadItem> item;
        std::optional<cadcam::process::EntityProcessState> processState;
    };

    CadDocument* m_document = nullptr;
    cadcam::process::DocumentProcessState* m_processState = nullptr;
    std::vector<SourceState> m_sourceStates;
    std::vector<std::unique_ptr<DRW_Entity>> m_replacementEntities;
    std::vector<std::unique_ptr<CadItem>> m_replacementItems;
    std::vector<CadItem*> m_replacementItemPtrs;
    std::optional<cadcam::planning::ProcessUnitSequence> m_beforeSequence;
    std::optional<cadcam::planning::ProcessUnitSequence> m_afterSequence;
};

// 删除实体命令：
// 执行时从文档中摘出实体，撤销时再插回原位。
class DeleteEntityCommand final : public CadEditer::EditCommand
{
public:
    DeleteEntityCommand(CadDocument* document,
        cadcam::process::DocumentProcessState* processState, CadItem* item)
        : m_document(document)
        , m_processState(processState)
        , m_itemPtr(item)
    {
        if (m_processState != nullptr)
            m_beforeSequence = m_processState->processUnitSequence();
    }

    bool execute() override
    {
        if (m_document == nullptr || m_itemPtr == nullptr)
        {
            return false;
        }

        if (!m_savedState.has_value() && m_processState != nullptr)
        {
            if (const auto* state = m_processState->find(m_itemPtr->m_entityId))
                m_savedState = *state;
        }
        const auto entityId = m_itemPtr->m_entityId;
        auto [entity, item] = m_document->takeEntity(m_itemPtr);

        if (entity == nullptr || item == nullptr)
        {
            return false;
        }

        m_entity = std::move(entity);
        m_item = std::move(item);
        m_itemPtr = m_item.get();
        if (m_processState != nullptr)
        {
            m_processState->beginBatch();
            m_processState->erase(entityId);
            if (m_afterSequence.has_value())
                m_processState->setProcessUnitSequence(m_afterSequence->units);
            m_processState->endBatch();
            if (!m_afterSequence.has_value())
                m_afterSequence = m_processState->processUnitSequence();
        }
        return true;
    }

    bool undo() override
    {
        if (m_document == nullptr || m_entity == nullptr || m_item == nullptr)
        {
            return false;
        }

        m_itemPtr = m_item.get();
        CadItem* restored = m_document->appendEntity(std::move(m_entity), std::move(m_item));
        if (restored == nullptr) return false;
        if (m_processState != nullptr)
        {
            m_processState->beginBatch();
            if (m_savedState.has_value())
                m_processState->setState(restored->m_entityId, *m_savedState);
            if (m_beforeSequence.has_value())
                m_processState->setProcessUnitSequence(m_beforeSequence->units);
            m_processState->endBatch();
        }
        return true;
    }

private:
    // 目标文档
    CadDocument* m_document = nullptr;

    // 被删除后缓存的原生实体
    std::unique_ptr<DRW_Entity> m_entity;

    // 被删除后缓存的图元对象
    std::unique_ptr<CadItem> m_item;

    // 当前图元裸指针
    CadItem* m_itemPtr = nullptr;
    cadcam::process::DocumentProcessState* m_processState = nullptr;
    std::optional<cadcam::process::EntityProcessState> m_savedState;
    std::optional<cadcam::planning::ProcessUnitSequence> m_beforeSequence;
    std::optional<cadcam::planning::ProcessUnitSequence> m_afterSequence;
};

class DeleteEntitiesCommand final : public CadEditer::EditCommand
{
public:
    DeleteEntitiesCommand(CadDocument* document,
        cadcam::process::DocumentProcessState* processState, const QVector<CadItem*>& items)
        : m_document(document)
        , m_processState(processState)
    {
        if (m_processState != nullptr)
            m_beforeSequence = m_processState->processUnitSequence();
        QSet<CadItem*> deduplicated;

        for (CadItem* item : items)
        {
            if (item == nullptr || deduplicated.contains(item))
            {
                continue;
            }

            deduplicated.insert(item);
            ItemState state;
            state.itemPtr = item;
            if (m_processState != nullptr)
            {
                if (const auto* saved = m_processState->find(item->m_entityId))
                    state.processState = *saved;
            }
            m_states.push_back(std::move(state));
        }
    }

    bool execute() override
    {
        if (m_document == nullptr || m_states.empty())
        {
            return false;
        }

        if (m_processState != nullptr) m_processState->beginBatch();
        for (ItemState& state : m_states)
        {
            if (state.itemPtr == nullptr)
            {
                if (m_processState != nullptr) m_processState->endBatch();
                return false;
            }

            auto [entity, item] = m_document->takeEntity(state.itemPtr);

            if (entity == nullptr || item == nullptr)
            {
                if (m_processState != nullptr) m_processState->endBatch();
                return false;
            }

            state.entity = std::move(entity);
            state.item = std::move(item);
            state.itemPtr = state.item.get();
            if (m_processState != nullptr) m_processState->erase(state.itemPtr->m_entityId);
        }
        if (m_processState != nullptr && m_afterSequence.has_value())
            m_processState->setProcessUnitSequence(m_afterSequence->units);
        if (m_processState != nullptr) m_processState->endBatch();
        if (m_processState != nullptr && !m_afterSequence.has_value())
            m_afterSequence = m_processState->processUnitSequence();
        return true;
    }

    bool undo() override
    {
        if (m_document == nullptr || m_states.empty())
        {
            return false;
        }

        if (m_processState != nullptr) m_processState->beginBatch();
        for (ItemState& state : m_states)
        {
            if (state.entity == nullptr || state.item == nullptr)
            {
                if (m_processState != nullptr) m_processState->endBatch();
                return false;
            }

            state.itemPtr = state.item.get();

            CadItem* restored = m_document->appendEntity(std::move(state.entity), std::move(state.item));
            if (restored == nullptr)
            {
                if (m_processState != nullptr) m_processState->endBatch();
                return false;
            }
            state.itemPtr = restored;
            if (m_processState != nullptr && state.processState.has_value())
                m_processState->setState(restored->m_entityId, *state.processState);
        }
        if (m_processState != nullptr && m_beforeSequence.has_value())
            m_processState->setProcessUnitSequence(m_beforeSequence->units);
        if (m_processState != nullptr) m_processState->endBatch();
        return true;
    }

private:
    struct ItemState
    {
        CadItem* itemPtr = nullptr;
        std::unique_ptr<DRW_Entity> entity;
        std::unique_ptr<CadItem> item;
        std::optional<cadcam::process::EntityProcessState> processState;
    };

    CadDocument* m_document = nullptr;
    cadcam::process::DocumentProcessState* m_processState = nullptr;
    std::vector<ItemState> m_states;
    std::optional<cadcam::planning::ProcessUnitSequence> m_beforeSequence;
    std::optional<cadcam::planning::ProcessUnitSequence> m_afterSequence;
};

// 移动实体命令：
// 通过对原生实体做几何平移，再触发文档刷新来实现可撤销移动。
class MoveEntityCommand final : public CadEditer::EditCommand
{
public:
    MoveEntityCommand(CadDocument* document, CadItem* item, const QVector3D& delta)
        : m_document(document)
        , m_item(item)
        , m_delta(delta)
    {
    }

    bool execute() override
    {
        return applyDelta(m_delta);
    }

    bool undo() override
    {
        return applyDelta(-m_delta);
    }

private:
    // 应用一次平移增量；Undo 通过传入相反向量复用同一逻辑
    bool applyDelta(const QVector3D& delta)
    {
        if (m_document == nullptr || m_item == nullptr || !m_document->containsEntity(m_item))
        {
            return false;
        }

        translateEntity(m_item->m_nativeEntity, delta);
        return m_document->refreshEntity(m_item);
    }

private:
    // 目标文档
    CadDocument* m_document = nullptr;

    // 目标实体
    CadItem* m_item = nullptr;

    // 平移增量
    QVector3D m_delta;
};

class MoveEntitiesCommand final : public CadEditer::EditCommand
{
public:
    MoveEntitiesCommand(CadDocument* document, const QVector<CadItem*>& items, const QVector3D& delta)
        : m_document(document)
        , m_items(items)
        , m_delta(delta)
    {
    }

    bool execute() override
    {
        return applyDelta(m_delta);
    }

    bool undo() override
    {
        return applyDelta(-m_delta);
    }

private:
    bool applyDelta(const QVector3D& delta)
    {
        if (m_document == nullptr || m_items.isEmpty())
        {
            return false;
        }

        for (CadItem* item : m_items)
        {
            if (item == nullptr || !m_document->containsEntity(item))
            {
                return false;
            }
        }

        for (CadItem* item : m_items)
        {
            translateEntity(item->m_nativeEntity, delta);

            if (!m_document->refreshEntity(item))
            {
                return false;
            }
        }

        return true;
    }

private:
    CadDocument* m_document = nullptr;
    QVector<CadItem*> m_items;
    QVector3D m_delta;
};

class GripPointEditCommand final : public CadEditer::EditCommand
{
public:
    GripPointEditCommand(CadDocument* document, CadItem* item, int pointIndex, const QVector3D& newPoint)
        : m_document(document)
        , m_item(item)
        , m_pointIndex(pointIndex)
        , m_newPoint(flattenToDrawingPlane(newPoint))
    {
        if (m_item != nullptr)
        {
            m_valid = readEditableControlPoint(m_item, m_pointIndex, m_oldPoint);
        }
    }

    bool execute() override
    {
        return apply(m_newPoint);
    }

    bool undo() override
    {
        return apply(m_oldPoint);
    }

private:
    bool apply(const QVector3D& point)
    {
        if (!m_valid
            || m_document == nullptr
            || m_item == nullptr
            || m_item->m_nativeEntity == nullptr
            || !m_document->containsEntity(m_item))
        {
            return false;
        }

        if (!applyEditableControlPoint(m_item->m_nativeEntity, m_pointIndex, point))
        {
            return false;
        }

        return m_document->refreshEntity(m_item);
    }

private:
    CadDocument* m_document = nullptr;
    CadItem* m_item = nullptr;
    int m_pointIndex = -1;
    QVector3D m_oldPoint;
    QVector3D m_newPoint;
    bool m_valid = false;
};

class CopyEntityCommand final : public CadEditer::EditCommand
{
public:
    CopyEntityCommand(CadDocument* document, CadItem* sourceItem, const QVector3D& delta)
        : m_document(document)
        , m_sourceItem(sourceItem)
        , m_delta(delta)
    {
    }

    bool execute() override
    {
        if (m_document == nullptr || m_sourceItem == nullptr)
        {
            return false;
        }

        if (m_entity == nullptr)
        {
            m_entity = cloneEntity(m_sourceItem->m_nativeEntity);

            if (m_entity == nullptr)
            {
                return false;
            }

            translateEntity(m_entity.get(), m_delta);
        }

        if (m_item == nullptr)
        {
            m_item = CadDocument::createCadItemForEntity(m_entity.get());
        }

        if (m_item == nullptr)
        {
            return false;
        }

        m_itemPtr = m_item.get();
        return m_document->appendEntity(std::move(m_entity), std::move(m_item)) != nullptr;
    }

    bool undo() override
    {
        if (m_document == nullptr || m_itemPtr == nullptr)
        {
            return false;
        }

        auto [entity, item] = m_document->takeEntity(m_itemPtr);

        if (entity == nullptr || item == nullptr)
        {
            return false;
        }

        m_entity = std::move(entity);
        m_item = std::move(item);
        m_itemPtr = m_item.get();
        return true;
    }

private:
    CadDocument* m_document = nullptr;
    CadItem* m_sourceItem = nullptr;
    QVector3D m_delta;
    std::unique_ptr<DRW_Entity> m_entity;
    std::unique_ptr<CadItem> m_item;
    CadItem* m_itemPtr = nullptr;
};

class RotateEntityCommand final : public CadEditer::EditCommand
{
public:
    RotateEntityCommand(CadDocument* document, CadItem* item, const QVector3D& basePoint, double angleDegrees)
        : m_document(document)
        , m_item(item)
        , m_basePoint(basePoint)
        , m_angleDegrees(angleDegrees)
    {
    }

    bool execute() override
    {
        return apply(m_angleDegrees);
    }

    bool undo() override
    {
        return apply(-m_angleDegrees);
    }

private:
    bool apply(double angleDegrees)
    {
        if (m_document == nullptr || m_item == nullptr || !m_document->containsEntity(m_item))
        {
            return false;
        }

        CadEditerWorkflowInternal::rotateEntity(m_item->m_nativeEntity, m_basePoint, angleDegrees);
        return m_document->refreshEntity(m_item);
    }

private:
    CadDocument* m_document = nullptr;
    CadItem* m_item = nullptr;
    QVector3D m_basePoint;
    double m_angleDegrees = 0.0;
};

class RotateEntitiesCommand final : public CadEditer::EditCommand
{
public:
    RotateEntitiesCommand(CadDocument* document, const QVector<CadItem*>& items, const QVector3D& basePoint, double angleDegrees)
        : m_document(document)
        , m_items(items)
        , m_basePoint(basePoint)
        , m_angleDegrees(angleDegrees)
    {
    }

    bool execute() override
    {
        return apply(m_angleDegrees);
    }

    bool undo() override
    {
        return apply(-m_angleDegrees);
    }

private:
    bool apply(double angleDegrees)
    {
        if (m_document == nullptr || m_items.isEmpty())
        {
            return false;
        }

        for (CadItem* item : m_items)
        {
            if (item == nullptr || !m_document->containsEntity(item))
            {
                return false;
            }
        }

        for (CadItem* item : m_items)
        {
            CadEditerWorkflowInternal::rotateEntity(item->m_nativeEntity, m_basePoint, angleDegrees);

            if (!m_document->refreshEntity(item))
            {
                return false;
            }
        }

        return true;
    }

private:
    CadDocument* m_document = nullptr;
    QVector<CadItem*> m_items;
    QVector3D m_basePoint;
    double m_angleDegrees = 0.0;
};

class ScaleEntityCommand final : public CadEditer::EditCommand
{
public:
    ScaleEntityCommand(CadDocument* document, CadItem* item, const QVector3D& basePoint, double scaleFactor)
        : m_document(document)
        , m_item(item)
        , m_basePoint(basePoint)
        , m_scaleFactor(scaleFactor)
    {
    }

    bool execute() override
    {
        return apply(m_scaleFactor);
    }

    bool undo() override
    {
        if (std::abs(m_scaleFactor) <= kGeometryEpsilon)
        {
            return false;
        }

        return apply(1.0 / m_scaleFactor);
    }

private:
    bool apply(double scaleFactor)
    {
        if (m_document == nullptr || m_item == nullptr || !m_document->containsEntity(m_item) || scaleFactor <= kGeometryEpsilon)
        {
            return false;
        }

        CadEditerWorkflowInternal::scaleEntity(m_item->m_nativeEntity, m_basePoint, scaleFactor);
        return m_document->refreshEntity(m_item);
    }

private:
    CadDocument* m_document = nullptr;
    CadItem* m_item = nullptr;
    QVector3D m_basePoint;
    double m_scaleFactor = 1.0;
};

class ScaleEntitiesCommand final : public CadEditer::EditCommand
{
public:
    ScaleEntitiesCommand(CadDocument* document, const QVector<CadItem*>& items, const QVector3D& basePoint, double scaleFactor)
        : m_document(document)
        , m_items(items)
        , m_basePoint(basePoint)
        , m_scaleFactor(scaleFactor)
    {
    }

    bool execute() override
    {
        return apply(m_scaleFactor);
    }

    bool undo() override
    {
        if (std::abs(m_scaleFactor) <= kGeometryEpsilon)
        {
            return false;
        }

        return apply(1.0 / m_scaleFactor);
    }

private:
    bool apply(double scaleFactor)
    {
        if (m_document == nullptr || m_items.isEmpty() || scaleFactor <= kGeometryEpsilon)
        {
            return false;
        }

        for (CadItem* item : m_items)
        {
            if (item == nullptr || !m_document->containsEntity(item))
            {
                return false;
            }
        }

        for (CadItem* item : m_items)
        {
            CadEditerWorkflowInternal::scaleEntity(item->m_nativeEntity, m_basePoint, scaleFactor);

            if (!m_document->refreshEntity(item))
            {
                return false;
            }
        }

        return true;
    }

private:
    CadDocument* m_document = nullptr;
    QVector<CadItem*> m_items;
    QVector3D m_basePoint;
    double m_scaleFactor = 1.0;
};

class ArrayEntityCommand final : public CadEditer::EditCommand
{
public:
    ArrayEntityCommand
    (
        CadDocument* document,
        CadItem* sourceItem,
        int rowCount,
        int columnCount,
        const QVector3D& rowOffset,
        const QVector3D& columnOffset
    )
        : m_document(document)
        , m_sourceItem(sourceItem)
        , m_rowCount(rowCount)
        , m_columnCount(columnCount)
        , m_rowOffset(rowOffset)
        , m_columnOffset(columnOffset)
    {
    }

    bool execute() override
    {
        if (m_document == nullptr || m_sourceItem == nullptr || m_rowCount < 1 || m_columnCount < 1)
        {
            return false;
        }

        if (m_entities.empty())
        {
            for (int row = 0; row < m_rowCount; ++row)
            {
                for (int column = 0; column < m_columnCount; ++column)
                {
                    if (row == 0 && column == 0)
                    {
                        continue;
                    }

                    std::unique_ptr<DRW_Entity> entity = cloneEntity(m_sourceItem->m_nativeEntity);

                    if (entity == nullptr)
                    {
                        return false;
                    }

                    const QVector3D delta = m_rowOffset * static_cast<float>(row) + m_columnOffset * static_cast<float>(column);
                    translateEntity(entity.get(), delta);
                    m_entities.push_back(std::move(entity));
                    m_items.push_back(nullptr);
                    m_itemPtrs.push_back(nullptr);
                }
            }
        }

        for (int index = 0; index < static_cast<int>(m_entities.size()); ++index)
        {
            if (m_entities[index] == nullptr)
            {
                return false;
            }

            if (m_items[index] == nullptr)
            {
                m_items[index] = CadDocument::createCadItemForEntity(m_entities[index].get());
            }

            if (m_items[index] == nullptr)
            {
                return false;
            }

            m_itemPtrs[index] = m_items[index].get();

            if (m_document->appendEntity(std::move(m_entities[index]), std::move(m_items[index])) == nullptr)
            {
                return false;
            }
        }

        return true;
    }

    bool undo() override
    {
        if (m_document == nullptr)
        {
            return false;
        }

        for (int index = static_cast<int>(m_itemPtrs.size()) - 1; index >= 0; --index)
        {
            if (m_itemPtrs[index] == nullptr)
            {
                return false;
            }

            auto [entity, item] = m_document->takeEntity(m_itemPtrs[index]);

            if (entity == nullptr || item == nullptr)
            {
                return false;
            }

            m_entities[index] = std::move(entity);
            m_items[index] = std::move(item);
            m_itemPtrs[index] = m_items[index].get();
        }

        return true;
    }

private:
    CadDocument* m_document = nullptr;
    CadItem* m_sourceItem = nullptr;
    int m_rowCount = 1;
    int m_columnCount = 1;
    QVector3D m_rowOffset;
    QVector3D m_columnOffset;
    std::vector<std::unique_ptr<DRW_Entity>> m_entities;
    std::vector<std::unique_ptr<CadItem>> m_items;
    std::vector<CadItem*> m_itemPtrs;
};

// 修改颜色命令：
// 记录新旧颜色信息，执行与撤销都通过刷新原生实体颜色完成。
class ChangeColorCommand final : public CadEditer::EditCommand
{
public:
    ChangeColorCommand(CadDocument* document, CadItem* item, const QColor& color, int colorIndex)
        : m_document(document)
        , m_item(item)
        , m_newColor(color)
        , m_newColorIndex(colorIndex)
    {
        if (m_item != nullptr && m_item->m_nativeEntity != nullptr)
        {
            m_oldColorIndex = m_item->m_nativeEntity->color;
            m_oldTrueColor = m_item->m_nativeEntity->color24;
        }
    }

    bool execute() override
    {
        return apply(m_newColor, m_newColorIndex);
    }

    bool undo() override
    {
        return apply(m_newColor, m_oldColorIndex, m_oldTrueColor);
    }

private:
    // 按颜色与索引应用颜色，自动决定 true color 或 ACI 方案
    bool apply(const QColor& color, int colorIndex)
    {
        return apply(color, colorIndex, colorIndex >= 0 ? -1 : colorToTrueColor(color));
    }

    bool apply(const QColor& color, int colorIndex, int trueColor)
    {
        if (m_document == nullptr || m_item == nullptr || m_item->m_nativeEntity == nullptr || !m_document->containsEntity(m_item))
        {
            return false;
        }

        if (colorIndex >= 0)
        {
            m_item->m_nativeEntity->color = colorIndex;
            m_item->m_nativeEntity->color24 = -1;
        }
        else
        {
            m_item->m_nativeEntity->color = DRW::ColorByLayer;
            m_item->m_nativeEntity->color24 = trueColor;
        }

        Q_UNUSED(color);
        return m_document->refreshEntity(m_item);
    }

private:
    // 目标文档
    CadDocument* m_document = nullptr;

    // 目标实体
    CadItem* m_item = nullptr;

    // 新颜色
    QColor m_newColor;

    // 新颜色索引
    int m_newColorIndex = -1;

    // 旧颜色索引
    int m_oldColorIndex = DRW::ColorByLayer;

    // 旧 true color
    int m_oldTrueColor = -1;
};

class ChangeLayerCommand final : public CadEditer::EditCommand
{
public:
    ChangeLayerCommand(CadDocument* document, CadItem* item, const QString& layerName)
        : m_document(document)
        , m_item(item)
        , m_newLayerName(layerName.trimmed().isEmpty() ? QStringLiteral("0") : layerName.trimmed())
    {
        if (m_item != nullptr && m_item->m_nativeEntity != nullptr)
        {
            m_oldLayerName = QString::fromUtf8(m_item->m_nativeEntity->layer.c_str());
        }
    }

    bool execute() override
    {
        return apply(m_newLayerName);
    }

    bool undo() override
    {
        return apply(m_oldLayerName);
    }

private:
    bool apply(const QString& layerName)
    {
        if (m_document == nullptr || m_item == nullptr || m_item->m_nativeEntity == nullptr || !m_document->containsEntity(m_item))
        {
            return false;
        }

        const QString normalizedLayerName = layerName.trimmed().isEmpty() ? QStringLiteral("0") : layerName.trimmed();
        m_document->ensureLayerExists(normalizedLayerName);
        m_item->m_nativeEntity->layer = normalizedLayerName.toUtf8().constData();
        return m_document->refreshEntity(m_item);
    }

private:
    CadDocument* m_document = nullptr;
    CadItem* m_item = nullptr;
    QString m_oldLayerName = QStringLiteral("0");
    QString m_newLayerName = QStringLiteral("0");
};

// 删除指定实体
// @param item 待删除实体
// @return 如果删除成功返回 true，否则返回 false
bool CadEditer::deleteEntity(CadItem* item)
{
    if (m_document == nullptr || item == nullptr || !m_document->containsEntity(item))
    {
        return false;
    }

    if (item == m_moveTarget)
    {
        m_moveTarget = nullptr;
    }

    const int moveIndex = m_moveTargets.indexOf(item);

    if (moveIndex >= 0)
    {
        m_moveTargets.removeAt(moveIndex);
    }

    if (item == m_gripTarget)
    {
        m_gripTarget = nullptr;
        m_gripPointIndex = -1;
    }

    return executeCommand(std::make_unique<DeleteEntityCommand>
        (m_document, m_processState, item));
}

bool CadEditer::deleteEntities(const QVector<CadItem*>& items)
{
    if (m_document == nullptr || items.isEmpty())
    {
        return false;
    }

    QVector<CadItem*> validItems;
    QSet<CadItem*> deduplicated;

    for (CadItem* item : items)
    {
        if (item == nullptr || !m_document->containsEntity(item) || deduplicated.contains(item))
        {
            continue;
        }

        deduplicated.insert(item);
        validItems.push_back(item);

        if (item == m_moveTarget)
        {
            m_moveTarget = nullptr;
        }

        m_moveTargets.removeAll(item);

        if (item == m_gripTarget)
        {
            m_gripTarget = nullptr;
            m_gripPointIndex = -1;
        }
    }

    if (validItems.isEmpty())
    {
        return false;
    }

    return executeCommand(std::make_unique<DeleteEntitiesCommand>
        (m_document, m_processState, validItems));
}

bool CadEditer::copyEntity(CadItem* item, const QVector3D& delta)
{
    if (m_document == nullptr || item == nullptr || !m_document->containsEntity(item))
    {
        return false;
    }

    if (delta.lengthSquared() <= kGeometryEpsilon)
    {
        return false;
    }

    return executeCommand(std::make_unique<CopyEntityCommand>(m_document, item, delta));
}

bool CadEditer::copyEntities(const QVector<CadItem*>& items, const QVector3D& delta)
{
    if (m_document == nullptr || items.isEmpty() || delta.lengthSquared() <= kGeometryEpsilon)
    {
        return false;
    }

    QSet<CadItem*> deduplicated;
    std::vector<std::unique_ptr<DRW_Entity>> copiedEntities;

    for (CadItem* item : items)
    {
        if (item == nullptr
            || !m_document->containsEntity(item)
            || item->m_nativeEntity == nullptr
            || deduplicated.contains(item))
        {
            continue;
        }

        std::unique_ptr<DRW_Entity> entity = cloneEntity(item->m_nativeEntity);

        if (entity == nullptr)
        {
            return false;
        }

        translateEntity(entity.get(), delta);
        deduplicated.insert(item);
        copiedEntities.push_back(std::move(entity));
    }

    if (copiedEntities.empty())
    {
        return false;
    }

    return executeCommand(std::make_unique<AddEntitiesCommand>(m_document, std::move(copiedEntities)));
}

bool CadEditer::rotateEntity(CadItem* item, const QVector3D& basePoint, double angleDegrees)
{
    if (m_document == nullptr || item == nullptr || !m_document->containsEntity(item) || std::abs(angleDegrees) <= kGeometryEpsilon)
    {
        return false;
    }

    return executeCommand(std::make_unique<RotateEntityCommand>(m_document, item, basePoint, angleDegrees));
}

bool CadEditer::rotateEntities(const QVector<CadItem*>& items, const QVector3D& basePoint, double angleDegrees)
{
    if (m_document == nullptr || items.isEmpty() || std::abs(angleDegrees) <= kGeometryEpsilon)
    {
        return false;
    }

    QVector<CadItem*> validItems;
    QSet<CadItem*> deduplicated;

    for (CadItem* item : items)
    {
        if (item == nullptr || !m_document->containsEntity(item) || deduplicated.contains(item))
        {
            continue;
        }

        deduplicated.insert(item);
        validItems.push_back(item);
    }

    if (validItems.isEmpty())
    {
        return false;
    }

    return executeCommand(std::make_unique<RotateEntitiesCommand>(m_document, validItems, basePoint, angleDegrees));
}

bool CadEditer::scaleEntity(CadItem* item, const QVector3D& basePoint, double scaleFactor)
{
    if (m_document == nullptr || item == nullptr || !m_document->containsEntity(item) || scaleFactor <= kGeometryEpsilon)
    {
        return false;
    }

    return executeCommand(std::make_unique<ScaleEntityCommand>(m_document, item, basePoint, scaleFactor));
}

bool CadEditer::scaleEntities(const QVector<CadItem*>& items, const QVector3D& basePoint, double scaleFactor)
{
    if (m_document == nullptr || items.isEmpty() || scaleFactor <= kGeometryEpsilon)
    {
        return false;
    }

    QVector<CadItem*> validItems;
    QSet<CadItem*> deduplicated;

    for (CadItem* item : items)
    {
        if (item == nullptr || !m_document->containsEntity(item) || deduplicated.contains(item))
        {
            continue;
        }

        deduplicated.insert(item);
        validItems.push_back(item);
    }

    if (validItems.isEmpty())
    {
        return false;
    }

    return executeCommand(std::make_unique<ScaleEntitiesCommand>(m_document, validItems, basePoint, scaleFactor));
}

bool CadEditer::arrayEntity(CadItem* item, int rowCount, int columnCount, const QVector3D& rowOffset, const QVector3D& columnOffset)
{
    if (m_document == nullptr || item == nullptr || !m_document->containsEntity(item))
    {
        return false;
    }

    if (rowCount < 1 || columnCount < 1 || (rowCount == 1 && columnCount == 1))
    {
        return false;
    }

    return executeCommand
    (
        std::make_unique<ArrayEntityCommand>(m_document, item, rowCount, columnCount, rowOffset, columnOffset)
    );
}

bool CadEditer::rectangularArrayEntities(const QVector<CadItem*>& items, int rowCount, int columnCount, const QVector3D& rowOffset, const QVector3D& columnOffset)
{
    if (m_document == nullptr
        || items.isEmpty()
        || rowCount < 1
        || columnCount < 1
        || (rowCount == 1 && columnCount == 1))
    {
        return false;
    }

    QSet<CadItem*> deduplicated;
    std::vector<std::unique_ptr<DRW_Entity>> arrayEntities;

    for (CadItem* item : items)
    {
        if (item == nullptr
            || !m_document->containsEntity(item)
            || item->m_nativeEntity == nullptr
            || deduplicated.contains(item))
        {
            continue;
        }

        deduplicated.insert(item);

        for (int row = 0; row < rowCount; ++row)
        {
            for (int column = 0; column < columnCount; ++column)
            {
                if (row == 0 && column == 0)
                {
                    continue;
                }

                std::unique_ptr<DRW_Entity> entity = cloneEntity(item->m_nativeEntity);

                if (entity == nullptr)
                {
                    return false;
                }

                translateEntity(entity.get(), rowOffset * static_cast<float>(row) + columnOffset * static_cast<float>(column));
                arrayEntities.push_back(std::move(entity));
            }
        }
    }

    if (arrayEntities.empty())
    {
        return false;
    }

    return executeCommand(std::make_unique<AddEntitiesCommand>(m_document, std::move(arrayEntities)));
}

bool CadEditer::mirrorEntities
(
    const QVector<CadItem*>& items,
    const QVector3D& firstPoint,
    const QVector3D& secondPoint,
    bool eraseSource
)
{
    if (m_document == nullptr || items.isEmpty() || pointsNear(firstPoint, secondPoint))
    {
        return false;
    }

    QVector<CadItem*> validItems;
    QSet<CadItem*> deduplicated;
    std::vector<std::unique_ptr<DRW_Entity>> mirroredEntities;

    for (CadItem* item : items)
    {
        if (item == nullptr || !m_document->containsEntity(item) || item->m_nativeEntity == nullptr || deduplicated.contains(item))
        {
            continue;
        }

        std::unique_ptr<DRW_Entity> mirrored = cloneEntity(item->m_nativeEntity);

        if (mirrored == nullptr || !mirrorEntityGeometry(mirrored.get(), firstPoint, secondPoint))
        {
            return false;
        }

        deduplicated.insert(item);
        validItems.push_back(item);
        mirroredEntities.push_back(std::move(mirrored));
    }

    if (validItems.isEmpty())
    {
        return false;
    }

    if (eraseSource)
    {
        return executeCommand(std::make_unique<ReplaceEntitiesCommand>
            (m_document, m_processState, validItems, std::move(mirroredEntities)));
    }

    return executeCommand(std::make_unique<AddEntitiesCommand>(m_document, std::move(mirroredEntities)));
}

bool CadEditer::polarArrayEntities
(
    const QVector<CadItem*>& items,
    const QVector3D& center,
    int itemCount,
    double totalAngleDegrees,
    bool rotateItems
)
{
    if (m_document == nullptr || items.isEmpty() || itemCount < 2 || std::abs(totalAngleDegrees) <= kGeometryEpsilon)
    {
        return false;
    }

    QVector<CadItem*> validItems;
    QSet<CadItem*> deduplicated;
    std::vector<std::unique_ptr<DRW_Entity>> arrayEntities;
    const double useFullCircleSpacing = std::abs(std::abs(totalAngleDegrees) - 360.0) <= 1.0e-6 ? static_cast<double>(itemCount) : static_cast<double>(itemCount - 1);
    const double angleStep = totalAngleDegrees / useFullCircleSpacing;

    for (CadItem* item : items)
    {
        if (item == nullptr || !m_document->containsEntity(item) || item->m_nativeEntity == nullptr || deduplicated.contains(item))
        {
            continue;
        }

        deduplicated.insert(item);
        validItems.push_back(item);

        for (int index = 1; index < itemCount; ++index)
        {
            const double angle = angleStep * static_cast<double>(index);
            std::unique_ptr<DRW_Entity> entity = cloneEntity(item->m_nativeEntity);

            if (entity == nullptr)
            {
                return false;
            }

            if (rotateItems)
            {
                CadEditerWorkflowInternal::rotateEntity(entity.get(), center, angle);
            }
            else
            {
                const QVector3D itemCenter = itemGeometryCenter(item);
                const QVector3D rotatedCenter = rotatePlanarPoint(itemCenter, center, angle * kPi / 180.0);
                translateEntity(entity.get(), rotatedCenter - itemCenter);
            }

            arrayEntities.push_back(std::move(entity));
        }
    }

    if (validItems.isEmpty() || arrayEntities.empty())
    {
        return false;
    }

    return executeCommand(std::make_unique<AddEntitiesCommand>(m_document, std::move(arrayEntities)));
}

bool CadEditer::offsetEntity(CadItem* item, double distance)
{
    if (m_document == nullptr || item == nullptr || !m_document->containsEntity(item))
    {
        return false;
    }

    std::unique_ptr<DRW_Entity> offset = createOffsetEntity(item, distance);

    if (offset == nullptr)
    {
        return false;
    }

    return executeCommand(std::make_unique<AddEntityCommand>(m_document, std::move(offset)));
}

bool CadEditer::trimEntity(CadItem* boundaryItem, CadItem* targetItem, bool trimStart)
{
    if (m_document == nullptr
        || boundaryItem == nullptr
        || targetItem == nullptr
        || boundaryItem == targetItem
        || !m_document->containsEntity(boundaryItem)
        || !m_document->containsEntity(targetItem))
    {
        return false;
    }

    std::unique_ptr<DRW_Entity> trimmed = cloneEntity(targetItem->m_nativeEntity);

    if (trimmed == nullptr || !trimOrExtendLineEntity(trimmed.get(), boundaryItem->m_nativeEntity, trimStart, true))
    {
        return false;
    }

    std::vector<std::unique_ptr<DRW_Entity>> replacements;
    replacements.push_back(std::move(trimmed));
    return executeCommand(std::make_unique<ReplaceEntitiesCommand>
        (m_document, m_processState, QVector<CadItem*>{ targetItem }, std::move(replacements)));
}

bool CadEditer::extendEntity(CadItem* boundaryItem, CadItem* targetItem, bool extendStart)
{
    if (m_document == nullptr
        || boundaryItem == nullptr
        || targetItem == nullptr
        || boundaryItem == targetItem
        || !m_document->containsEntity(boundaryItem)
        || !m_document->containsEntity(targetItem))
    {
        return false;
    }

    std::unique_ptr<DRW_Entity> extended = cloneEntity(targetItem->m_nativeEntity);

    if (extended == nullptr || !trimOrExtendLineEntity(extended.get(), boundaryItem->m_nativeEntity, extendStart, false))
    {
        return false;
    }

    std::vector<std::unique_ptr<DRW_Entity>> replacements;
    replacements.push_back(std::move(extended));
    return executeCommand(std::make_unique<ReplaceEntitiesCommand>
        (m_document, m_processState, QVector<CadItem*>{ targetItem }, std::move(replacements)));
}

bool CadEditer::joinEntities(const QVector<CadItem*>& items)
{
    if (m_document == nullptr || items.size() < 2)
    {
        return false;
    }

    QVector<CadItem*> validItems;
    QSet<CadItem*> deduplicated;

    for (CadItem* item : items)
    {
        if (item == nullptr || !m_document->containsEntity(item) || deduplicated.contains(item))
        {
            continue;
        }

        deduplicated.insert(item);
        validItems.push_back(item);
    }

    if (validItems.size() < 2)
    {
        return false;
    }

    std::unique_ptr<DRW_Entity> joinedEntity;

    if (!buildJoinedPolylineEntity(validItems, joinedEntity))
    {
        return false;
    }

    std::vector<std::unique_ptr<DRW_Entity>> replacements;
    replacements.push_back(std::move(joinedEntity));
    return executeCommand(std::make_unique<ReplaceEntitiesCommand>
        (m_document, m_processState, validItems, std::move(replacements)));
}

bool CadEditer::filletEntities(CadItem* firstItem, CadItem* secondItem, double radius)
{
    if (m_document == nullptr
        || firstItem == nullptr
        || secondItem == nullptr
        || firstItem == secondItem
        || !m_document->containsEntity(firstItem)
        || !m_document->containsEntity(secondItem))
    {
        return false;
    }

    std::vector<std::unique_ptr<DRW_Entity>> replacements;

    if (!buildFilletReplacementEntities(firstItem, secondItem, radius, replacements))
    {
        return false;
    }

    return executeCommand
    (
        std::make_unique<ReplaceEntitiesCommand>
        (m_document, m_processState, QVector<CadItem*>{ firstItem, secondItem }, std::move(replacements))
    );
}

bool CadEditer::chamferEntities(CadItem* firstItem, CadItem* secondItem, double firstDistance, double secondDistance)
{
    if (m_document == nullptr
        || firstItem == nullptr
        || secondItem == nullptr
        || firstItem == secondItem
        || !m_document->containsEntity(firstItem)
        || !m_document->containsEntity(secondItem))
    {
        return false;
    }

    std::vector<std::unique_ptr<DRW_Entity>> replacements;

    if (!buildChamferReplacementEntities(firstItem, secondItem, firstDistance, secondDistance, replacements))
    {
        return false;
    }

    return executeCommand
    (
        std::make_unique<ReplaceEntitiesCommand>
        (m_document, m_processState, QVector<CadItem*>{ firstItem, secondItem }, std::move(replacements))
    );
}

// 修改指定实体颜色
// @param item 目标实体
// @param color 新颜色
// @param colorIndex 可选 ACI 颜色索引，小于 0 时使用 true color
// @return 如果修改成功返回 true，否则返回 false
bool CadEditer::changeEntityColor(CadItem* item, const QColor& color, int colorIndex)
{
    if (m_document == nullptr || item == nullptr || !m_document->containsEntity(item) || !color.isValid())
    {
        return false;
    }

    return executeCommand(std::make_unique<ChangeColorCommand>(m_document, item, color, colorIndex));
}

bool CadEditer::changeEntityLayer(CadItem* item, const QString& layerName)
{
    if (m_document == nullptr || item == nullptr || !m_document->containsEntity(item))
    {
        return false;
    }

    const QString normalizedLayerName = layerName.trimmed().isEmpty() ? QStringLiteral("0") : layerName.trimmed();
    return executeCommand(std::make_unique<ChangeLayerCommand>(m_document, item, normalizedLayerName));
}

// 切换指定实体的反向加工标记
// @param item 目标实体
// @return 如果切换成功返回 true，否则返回 false
// 设置指定实体的加工顺序
// @param item 目标实体
// @param processOrder 新的加工顺序
// @return 如果设置成功返回 true，否则返回 false
// 批量更新实体的加工顺序、反向加工状态与闭合图元起刀缝点
// @param updates 目标实体的加工状态更新数组
// @return 如果批量更新成功返回 true，否则返回 false
// 处理移动编辑命令
bool CadEditer::handleMoveEditing
(
    const DrawStateMachine& previousState,
    DrawStateMachine& currentState,
    const QVector3D& worldPos
)
{
    // 目标失效时立刻退出 Move 模式，避免悬空编辑状态
    if (m_document == nullptr)
    {
        currentState.commandPoints.clear();
        currentState.editType = EditType::None;
        currentState.moveSubMode = MoveEditSubMode::Idle;
        m_moveTarget = nullptr;
        m_moveTargets.clear();
        return false;
    }

    QVector<CadItem*> validTargets;
    validTargets.reserve(m_moveTargets.size());

    for (CadItem* item : m_moveTargets)
    {
        if (item != nullptr && m_document->containsEntity(item))
        {
            validTargets.append(item);
        }
    }

    if (validTargets.isEmpty())
    {
        currentState.commandPoints.clear();
        currentState.editType = EditType::None;
        currentState.moveSubMode = MoveEditSubMode::Idle;
        m_moveTarget = nullptr;
        m_moveTargets.clear();
        return false;
    }

    m_moveTargets = validTargets;
    m_moveTarget = m_moveTargets.front();

    if (previousState.moveSubMode == MoveEditSubMode::AwaitBasePoint)
    {
        // 第一次点击记录基点
        currentState.commandPoints = { worldPos };
        return true;
    }

    if (previousState.moveSubMode == MoveEditSubMode::AwaitTargetPoint && !currentState.commandPoints.isEmpty())
    {
        // 第二次点击确定目标点，并以两点差值作为移动增量
        const QVector3D basePoint = currentState.commandPoints.front();
        const QVector3D delta = worldPos - basePoint;

        if (delta.lengthSquared() > kGeometryEpsilon)
        {
            if (!executeCommand(std::make_unique<MoveEntitiesCommand>(m_document, m_moveTargets, delta)))
            {
                return false;
            }
        }

        // 移动完成后清空编辑状态
        currentState.commandPoints.clear();
        currentState.editType = EditType::None;
        currentState.moveSubMode = MoveEditSubMode::Idle;
        m_moveTarget = nullptr;
        m_moveTargets.clear();
        return true;
    }

    return false;
}

bool CadEditer::handleGripEditing
(
    const DrawStateMachine& previousState,
    DrawStateMachine& currentState,
    const QVector3D& worldPos
)
{
    if (m_gripTarget == nullptr
        || m_document == nullptr
        || !m_document->containsEntity(m_gripTarget)
        || m_gripPointIndex < 0)
    {
        currentState.commandPoints.clear();
        currentState.editType = EditType::None;
        currentState.gripSubMode = GripEditSubMode::Idle;
        currentState.gripPointIndex = -1;
        m_gripTarget = nullptr;
        m_gripPointIndex = -1;
        return false;
    }

    if (previousState.gripSubMode == GripEditSubMode::AwaitTargetPoint && !currentState.commandPoints.isEmpty())
    {
        const QVector3D basePoint = currentState.commandPoints.front();
        const QVector3D targetPoint = flattenToDrawingPlane(worldPos);

        if ((targetPoint - basePoint).lengthSquared() > kGeometryEpsilon)
        {
            if (!executeCommand(std::make_unique<GripPointEditCommand>(m_document, m_gripTarget, m_gripPointIndex, targetPoint)))
            {
                return false;
            }
        }

        currentState.commandPoints.clear();
        currentState.editType = EditType::None;
        currentState.gripSubMode = GripEditSubMode::Idle;
        currentState.gripPointIndex = -1;
        m_gripTarget = nullptr;
        m_gripPointIndex = -1;
        return true;
    }

    return false;
}

// 将完整加工单元序列作为一个可撤销命令应用。
bool CadEditer::changeProcessUnitSequence
(
    const cadcam::planning::ProcessUnitSequence& before,
    const cadcam::planning::ProcessUnitSequence& after,
    ProcessUnitSequenceApply apply
)
{
    if (before.units == after.units || apply == nullptr)
    {
        return false;
    }

    return executeCommand(std::make_unique<ProcessUnitSequenceCommand>
        (before, after, std::move(apply)));
}

// 将加工单元整组遍历反向作为一个可撤销命令应用。
bool CadEditer::changeProcessUnitTraversal
(
    const cadcam::planning::ProcessUnitKey& key,
    const cadcam::process::ProcessUnitTraversalOverride& beforeTraversal,
    const std::optional<cadcam::process::ProcessUnitTraversalOverride>& beforeStored,
    const cadcam::process::ProcessUnitTraversalOverride& afterTraversal,
    const std::optional<cadcam::process::ProcessUnitTraversalOverride>& afterStored,
    ProcessUnitTraversalApply apply
)
{
    if (!cadcam::planning::validProcessUnitKey(key)
        || beforeTraversal == afterTraversal || apply == nullptr) return false;

    return executeCommand(std::make_unique<ProcessUnitTraversalCommand>
    (
        key,
        beforeTraversal,
        beforeStored,
        afterTraversal,
        afterStored,
        std::move(apply)
    ));
}

// 执行命令并压入 Undo 栈
// @param command 待执行的命令对象
// @return 如果执行成功返回 true，否则返回 false
bool CadEditer::executeCommand(std::unique_ptr<EditCommand> command)
{
    // 一旦产生新命令，Redo 栈就失效
    if (command == nullptr)
    {
        return false;
    }

    auto contentBatch = m_document->beginContentChangeBatch();
    if (!command->execute())
    {
        return false;
    }

    m_redoStack.clear();
    m_undoStack.push_back(std::move(command));
    return true;
}

// 向文档追加新实体
// @param entity 待追加的原生 DXF 实体
// @return 如果追加成功返回 true，否则返回 false
bool CadEditer::addEntity(std::unique_ptr<DRW_Entity> entity)
{
    if (entity == nullptr)
    {
        return false;
    }

    return executeCommand(std::make_unique<AddEntityCommand>(m_document, std::move(entity)));
}

