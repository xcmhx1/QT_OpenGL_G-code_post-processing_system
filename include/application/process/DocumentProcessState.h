#pragma once

#include "core/planning/ProcessPlan.h"

#include <cstdint>
#include <map>
#include <optional>
#include <vector>

namespace cadcam::process
{
    struct ProcessUnitMemberTraversal
    {
        geometry::EntityId entityId = 0;
        bool reverse = false;
        std::optional<double> startParameter;

        bool operator==(const ProcessUnitMemberTraversal& other) const
        {
            return entityId == other.entityId
                && reverse == other.reverse
                && startParameter == other.startParameter;
        }

        bool operator!=(const ProcessUnitMemberTraversal& other) const
        {
            return !(*this == other);
        }
    };

    struct ProcessUnitTraversalOverride
    {
        std::vector<ProcessUnitMemberTraversal> members;

        bool operator==(const ProcessUnitTraversalOverride& other) const
        {
            return members == other.members;
        }


        bool operator!=(const ProcessUnitTraversalOverride& other) const
        {
            return !(*this == other);
        }
    };

    struct ProcessOverride
    {
        bool processEnabled = true;
        DirectionPreference direction = DirectionPreference::Auto;
        std::optional<double> startParameter;
        planning::BoundaryRole boundaryRole = planning::BoundaryRole::None;
        int boundaryPairId = -1;

        bool operator==(const ProcessOverride& other) const
        {
            return processEnabled == other.processEnabled
                && direction == other.direction
                && startParameter == other.startParameter
                && boundaryRole == other.boundaryRole
                && boundaryPairId == other.boundaryPairId;
        }
    };

    struct EntityProcessState
    {
        ProcessOverride overrideData;

        bool operator==(const EntityProcessState& other) const
        {
            return overrideData == other.overrideData;
        }
    };

    class DocumentProcessState
    {
    public:
        std::uint64_t revision() const;
        const EntityProcessState* find(geometry::EntityId entityId) const;
        EntityProcessState stateOrDefault(geometry::EntityId entityId) const;

        bool setDirection(geometry::EntityId entityId, DirectionPreference direction);
        bool setStartParameter(geometry::EntityId entityId, std::optional<double> parameter);
        bool setProcessEnabled(geometry::EntityId entityId, bool enabled);
        bool setBoundary(geometry::EntityId entityId, planning::BoundaryRole role, int pairId);
        const planning::ProcessUnitSequence& processUnitSequence() const;
        bool setProcessUnitSequence(const std::vector<planning::ProcessUnitKey>& units);
        bool clearProcessUnitSequence();
        const ProcessUnitTraversalOverride* findProcessUnitTraversalOverride
            (const planning::ProcessUnitKey& key) const;
        bool setProcessUnitTraversalOverride
        (
            const planning::ProcessUnitKey& key,
            const std::optional<ProcessUnitTraversalOverride>& traversal
        );

        bool setState(geometry::EntityId entityId, const EntityProcessState& state);
        bool erase(geometry::EntityId entityId);
        void retainOnly(const std::vector<geometry::EntityId>& validEntityIds);
        void clear();

        void beginBatch();
        void endBatch();

    private:
        bool store(geometry::EntityId entityId, const EntityProcessState& state);
        void markChanged();
        static bool isDefault(const EntityProcessState& state);

        std::uint64_t m_revision = 1;
        std::map<geometry::EntityId, EntityProcessState> m_states;
        planning::ProcessUnitSequence m_processUnitSequence;
        std::map<std::vector<geometry::EntityId>, ProcessUnitTraversalOverride>
            m_processUnitTraversalOverrides;
        int m_batchDepth = 0;
        bool m_batchChanged = false;
    };
}
