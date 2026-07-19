#include "application/process/DocumentProcessState.h"

#include <cmath>
#include <set>

namespace cadcam::process
{
    std::uint64_t DocumentProcessState::revision() const { return m_revision; }

    const EntityProcessState* DocumentProcessState::find(geometry::EntityId entityId) const
    {
        const auto found = m_states.find(entityId);
        return found == m_states.end() ? nullptr : &found->second;
    }

    EntityProcessState DocumentProcessState::stateOrDefault(geometry::EntityId entityId) const
    {
        const EntityProcessState* state = find(entityId);
        return state != nullptr ? *state : EntityProcessState{};
    }

    bool DocumentProcessState::setDirection(geometry::EntityId id, DirectionPreference value)
    {
        EntityProcessState state = stateOrDefault(id);
        state.overrideData.direction = value;
        return store(id, state);
    }

    bool DocumentProcessState::setStartParameter(geometry::EntityId id, std::optional<double> value)
    {
        if (value.has_value() && !std::isfinite(*value)) return false;
        EntityProcessState state = stateOrDefault(id);
        state.overrideData.startParameter = value;
        return store(id, state);
    }

    bool DocumentProcessState::setProcessEnabled(geometry::EntityId id, bool value)
    {
        EntityProcessState state = stateOrDefault(id);
        state.overrideData.processEnabled = value;
        return store(id, state);
    }

    bool DocumentProcessState::setBoundary
    (geometry::EntityId id, planning::BoundaryRole role, int pairId)
    {
        if ((role == planning::BoundaryRole::None && pairId != -1)
            || (role != planning::BoundaryRole::None && pairId < 0)) return false;
        EntityProcessState state = stateOrDefault(id);
        state.overrideData.boundaryRole = role;
        state.overrideData.boundaryPairId = pairId;
        return store(id, state);
    }

    bool DocumentProcessState::setAutomaticInternalExclusion(geometry::EntityId id, bool value)
    {
        EntityProcessState state = stateOrDefault(id);
        state.analysis.automaticInternalExclusion = value;
        return store(id, state);
    }

    bool DocumentProcessState::automaticInternalExclusion(geometry::EntityId id) const
    {
        return stateOrDefault(id).analysis.automaticInternalExclusion;
    }

    bool DocumentProcessState::setManualInternalExclusionOverride
    (geometry::EntityId id, std::optional<bool> value)
    {
        EntityProcessState state = stateOrDefault(id);
        state.overrideData.manualInternalExclusionOverride = value;
        return store(id, state);
    }

    bool DocumentProcessState::clearManualInternalExclusionOverride(geometry::EntityId id)
    {
        return setManualInternalExclusionOverride(id, std::nullopt);
    }

    std::optional<bool> DocumentProcessState::manualInternalExclusionOverride
        (geometry::EntityId id) const
    {
        return stateOrDefault(id).overrideData.manualInternalExclusionOverride;
    }

    bool DocumentProcessState::effectiveInternalExclusion(geometry::EntityId id) const
    {
        return stateOrDefault(id).effectiveInternalExclusion();
    }

    bool DocumentProcessState::setManualProcessOrder
    (geometry::EntityId id, std::optional<int> value)
    {
        if (value.has_value() && *value < 0) return false;
        EntityProcessState state = stateOrDefault(id);
        state.overrideData.manualProcessOrder = value;
        return store(id, state);
    }

    bool DocumentProcessState::clearManualProcessOrder(geometry::EntityId id)
    {
        return setManualProcessOrder(id, std::nullopt);
    }

    bool DocumentProcessState::clearAllManualProcessOrders()
    {
        bool changed = false;
        beginBatch();
        for (auto it = m_states.begin(); it != m_states.end();)
        {
            if (!it->second.overrideData.manualProcessOrder.has_value())
            {
                ++it;
                continue;
            }
            EntityProcessState state = it->second;
            state.overrideData.manualProcessOrder.reset();
            const geometry::EntityId id = it->first;
            ++it;
            changed = store(id, state) || changed;
        }
        endBatch();
        return changed;
    }

    std::optional<int> DocumentProcessState::manualProcessOrder(geometry::EntityId id) const
    {
        return stateOrDefault(id).overrideData.manualProcessOrder;
    }

    bool DocumentProcessState::setInternalGeometryExcluded(geometry::EntityId id, bool value)
    {
        return setAutomaticInternalExclusion(id, value);
    }

    bool DocumentProcessState::setState(geometry::EntityId id, const EntityProcessState& state)
    {
        if ((state.overrideData.boundaryRole == planning::BoundaryRole::None
                && state.overrideData.boundaryPairId != -1)
            || (state.overrideData.boundaryRole != planning::BoundaryRole::None
                && state.overrideData.boundaryPairId < 0)
            || (state.overrideData.startParameter.has_value()
                && !std::isfinite(*state.overrideData.startParameter))
            || (state.overrideData.manualProcessOrder.has_value()
                && *state.overrideData.manualProcessOrder < 0)) return false;
        return store(id, state);
    }

    bool DocumentProcessState::erase(geometry::EntityId id)
    {
        if (id == 0 || m_states.erase(id) == 0U) return false;
        markChanged();
        return true;
    }

    void DocumentProcessState::retainOnly(const std::vector<geometry::EntityId>& ids)
    {
        const std::set<geometry::EntityId> valid(ids.begin(), ids.end());
        beginBatch();
        for (auto it = m_states.begin(); it != m_states.end();)
        {
            if (valid.count(it->first) != 0U) { ++it; continue; }
            it = m_states.erase(it);
            markChanged();
        }
        endBatch();
    }

    void DocumentProcessState::clear()
    {
        if (m_states.empty()) return;
        m_states.clear();
        markChanged();
    }

    void DocumentProcessState::beginBatch() { ++m_batchDepth; }

    void DocumentProcessState::endBatch()
    {
        if (m_batchDepth <= 0) return;
        --m_batchDepth;
        if (m_batchDepth == 0 && m_batchChanged)
        {
            ++m_revision;
            m_batchChanged = false;
        }
    }

    bool DocumentProcessState::store(geometry::EntityId id, const EntityProcessState& state)
    {
        if (id == 0) return false;
        const EntityProcessState old = stateOrDefault(id);
        if (old == state) return false;
        if (isDefault(state)) m_states.erase(id);
        else m_states[id] = state;
        markChanged();
        return true;
    }

    void DocumentProcessState::markChanged()
    {
        if (m_batchDepth > 0) m_batchChanged = true;
        else ++m_revision;
    }

    bool DocumentProcessState::isDefault(const EntityProcessState& state)
    {
        return state == EntityProcessState{};
    }
}
