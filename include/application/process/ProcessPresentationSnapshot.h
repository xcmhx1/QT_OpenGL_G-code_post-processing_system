#pragma once

#include "core/diagnostics/OperationContext.h"
#include "core/diagnostics/OperationResult.h"
#include "core/planning/ProcessPlan.h"

#include <optional>
#include <vector>

namespace cadcam::process
{
    struct ProcessPresentationEntry
    {
        geometry::EntityId entityId = 0;
        int processOrder = -1;
        int continuousGroupId = -1;
        bool reverse = false;
        std::optional<double> startParameter;
        bool excluded = false;
        std::optional<planning::ProcessExclusionReason> exclusionReason;
    };

    struct ProcessPresentationSnapshot
    {
        std::uint64_t contentRevision = 0;
        std::uint64_t processStateRevision = 0;
        planning::ProcessPlanMode mode = planning::ProcessPlanMode::Planar3Axis;
        std::vector<ProcessPresentationEntry> entries;

        const ProcessPresentationEntry* find(geometry::EntityId entityId) const;
        static OperationResult<ProcessPresentationSnapshot> build
        (const planning::ProcessPlan& plan, const OperationContext& context);
    };
}
