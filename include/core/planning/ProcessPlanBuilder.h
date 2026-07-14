#pragma once

#include "core/diagnostics/OperationContext.h"
#include "core/diagnostics/OperationResult.h"
#include "core/planning/ProcessPlan.h"

namespace cadcam::planning
{
    class ProcessPlanBuilder
    {
    public:
        static OperationResult<ProcessPlan> build
        (
            const ProcessPlanningInput& input,
            const ProcessPlanningPolicy& policy,
            const OperationContext& context
        );
    };
}
