#include "application/process/DocumentProcessState.h"

#include <algorithm>
#include <cmath>
#include <set>

namespace cadcam::process
{
    namespace
    {
        bool validUnitKey(const planning::ProcessUnitKey& key)
        {
            if (key.memberEntityIds.empty()) return false;
            for (std::size_t index = 0; index < key.memberEntityIds.size(); ++index)
            {
                if (key.memberEntityIds[index] == 0U
                    || (index > 0U
                        && key.memberEntityIds[index - 1U] >= key.memberEntityIds[index]))
                {
                    return false;
                }
            }
            return true;
        }

        bool validUnitTraversal
        (
            const planning::ProcessUnitKey& key,
            const ProcessUnitTraversalOverride& traversal
        )
        {
            if (!validUnitKey(key)
                || traversal.members.size() != key.memberEntityIds.size()) return false;

            std::vector<geometry::EntityId> memberIds;
            memberIds.reserve(traversal.members.size());
            for (const ProcessUnitMemberTraversal& member : traversal.members)
            {
                if (member.entityId == 0U
                    || (member.startParameter.has_value()
                        && !std::isfinite(*member.startParameter))) return false;
                memberIds.push_back(member.entityId);
            }
            std::sort(memberIds.begin(), memberIds.end());
            return memberIds == key.memberEntityIds
                && std::adjacent_find(memberIds.begin(), memberIds.end()) == memberIds.end();
        }
    }

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

    const planning::ProcessUnitSequence& DocumentProcessState::processUnitSequence() const
    {
        return m_processUnitSequence;
    }

    bool DocumentProcessState::setProcessUnitSequence
    (const std::vector<planning::ProcessUnitKey>& units)
    {
        std::set<std::vector<geometry::EntityId>> uniqueKeys;
        for (const planning::ProcessUnitKey& key : units)
        {
            if (!validUnitKey(key) || !uniqueKeys.insert(key.memberEntityIds).second) return false;
        }
        if (m_processUnitSequence.units == units) return false;
        m_processUnitSequence.units = units;
        ++m_processUnitSequence.revision;
        markChanged();
        return true;
    }

    bool DocumentProcessState::clearProcessUnitSequence()
    {
        if (m_processUnitSequence.units.empty()) return false;
        m_processUnitSequence.units.clear();
        ++m_processUnitSequence.revision;
        markChanged();
        return true;
    }

    const ProcessUnitTraversalOverride* DocumentProcessState::findProcessUnitTraversalOverride
        (const planning::ProcessUnitKey& key) const
    {
        const auto found = m_processUnitTraversalOverrides.find(key.memberEntityIds);
        return found == m_processUnitTraversalOverrides.end() ? nullptr : &found->second;
    }

    bool DocumentProcessState::setProcessUnitTraversalOverride
    (
        const planning::ProcessUnitKey& key,
        const std::optional<ProcessUnitTraversalOverride>& traversal
    )
    {
        if (!validUnitKey(key)
            || (traversal.has_value() && !validUnitTraversal(key, *traversal))) return false;

        const auto found = m_processUnitTraversalOverrides.find(key.memberEntityIds);
        if (!traversal.has_value())
        {
            if (found == m_processUnitTraversalOverrides.end()) return false;
            m_processUnitTraversalOverrides.erase(found);
        }
        else
        {
            if (found != m_processUnitTraversalOverrides.end()
                && found->second == *traversal) return false;
            m_processUnitTraversalOverrides[key.memberEntityIds] = *traversal;
        }
        markChanged();
        return true;
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
                && !std::isfinite(*state.overrideData.startParameter))) return false;
        return store(id, state);
    }

    bool DocumentProcessState::erase(geometry::EntityId id)
    {
        if (id == 0) return false;
        bool changed = m_states.erase(id) != 0U;
        const auto newEnd = std::remove_if
        (
            m_processUnitSequence.units.begin(),
            m_processUnitSequence.units.end(),
            [id](const planning::ProcessUnitKey& key)
            {
                return std::binary_search
                    (key.memberEntityIds.begin(), key.memberEntityIds.end(), id);
            }
        );
        if (newEnd != m_processUnitSequence.units.end())
        {
            m_processUnitSequence.units.erase(newEnd, m_processUnitSequence.units.end());
            ++m_processUnitSequence.revision;
            changed = true;
        }
        for (auto it = m_processUnitTraversalOverrides.begin();
            it != m_processUnitTraversalOverrides.end();)
        {
            if (std::binary_search(it->first.begin(), it->first.end(), id))
            {
                it = m_processUnitTraversalOverrides.erase(it);
                changed = true;
            }
            else
            {
                ++it;
            }
        }
        if (!changed) return false;
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
        const auto newEnd = std::remove_if
        (
            m_processUnitSequence.units.begin(),
            m_processUnitSequence.units.end(),
            [&valid](const planning::ProcessUnitKey& key)
            {
                return std::any_of
                (
                    key.memberEntityIds.begin(), key.memberEntityIds.end(),
                    [&valid](geometry::EntityId id) { return valid.count(id) == 0U; }
                );
            }
        );
        if (newEnd != m_processUnitSequence.units.end())
        {
            m_processUnitSequence.units.erase(newEnd, m_processUnitSequence.units.end());
            ++m_processUnitSequence.revision;
            markChanged();
        }
        for (auto it = m_processUnitTraversalOverrides.begin();
            it != m_processUnitTraversalOverrides.end();)
        {
            const bool invalid = std::any_of
            (
                it->first.begin(), it->first.end(),
                [&valid](geometry::EntityId id) { return valid.count(id) == 0U; }
            );
            if (invalid)
            {
                it = m_processUnitTraversalOverrides.erase(it);
                markChanged();
            }
            else
            {
                ++it;
            }
        }
        endBatch();
    }

    void DocumentProcessState::clear()
    {
        if (m_states.empty() && m_processUnitSequence.units.empty()
            && m_processUnitTraversalOverrides.empty()) return;
        m_states.clear();
        m_processUnitTraversalOverrides.clear();
        if (!m_processUnitSequence.units.empty())
        {
            m_processUnitSequence.units.clear();
            ++m_processUnitSequence.revision;
        }
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
