#pragma once

#include "core/diagnostics/OperationResult.h"
#include "core/planning/ProcessPlan.h"

namespace cadcam::planning
{
    class SingleClosedEntryRefiner
    {
    public:
        static OperationReport refine
        (
            ProcessPlan& plan,
            const ProcessPlanningInput& input,
            const ProcessPlanningPolicy& policy,
            const OperationContext& context
        );
    };
}
