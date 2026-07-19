#pragma once

#include "core/planning/ProcessPlan.h"

#include <cstdint>
#include <map>
#include <optional>
#include <vector>

namespace cadcam::process
{
    struct ProcessOverride
    {
        bool processEnabled = true;
        DirectionPreference direction = DirectionPreference::Auto;
        std::optional<double> startParameter;
        std::optional<bool> manualInternalExclusionOverride;
        planning::BoundaryRole boundaryRole = planning::BoundaryRole::None;
        int boundaryPairId = -1;

        bool operator==(const ProcessOverride& other) const
        {
            return processEnabled == other.processEnabled
                && direction == other.direction
                && startParameter == other.startParameter
                && manualInternalExclusionOverride == other.manualInternalExclusionOverride
                && boundaryRole == other.boundaryRole
                && boundaryPairId == other.boundaryPairId;
        }
    };

    struct ProcessAnalysisState
    {
        bool automaticInternalExclusion = false;

        bool operator==(const ProcessAnalysisState& other) const
        {
            return automaticInternalExclusion == other.automaticInternalExclusion;
        }
    };

    struct EntityProcessState
    {
        ProcessOverride overrideData;
        ProcessAnalysisState analysis;

        bool effectiveInternalExclusion() const
        {
            return overrideData.manualInternalExclusionOverride.value_or
                (analysis.automaticInternalExclusion);
        }

        bool operator==(const EntityProcessState& other) const
        {
            return overrideData == other.overrideData && analysis == other.analysis;
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
        bool setAutomaticInternalExclusion(geometry::EntityId entityId, bool excluded);
        bool automaticInternalExclusion(geometry::EntityId entityId) const;
        bool setManualInternalExclusionOverride
            (geometry::EntityId entityId, std::optional<bool> excluded);
        bool clearManualInternalExclusionOverride(geometry::EntityId entityId);
        std::optional<bool> manualInternalExclusionOverride
            (geometry::EntityId entityId) const;
        bool effectiveInternalExclusion(geometry::EntityId entityId) const;
        const planning::ProcessUnitSequence& processUnitSequence() const;
        bool setProcessUnitSequence(const std::vector<planning::ProcessUnitKey>& units);
        bool clearProcessUnitSequence();

        // Compatibility wrapper: legacy callers set the automatic analysis result.
        bool setInternalGeometryExcluded(geometry::EntityId entityId, bool excluded);
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
        int m_batchDepth = 0;
        bool m_batchChanged = false;
    };
}
