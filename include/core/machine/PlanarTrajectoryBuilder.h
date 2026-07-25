#pragma once

#include "core/diagnostics/OperationContext.h"
#include "core/diagnostics/OperationResult.h"
#include "core/machine/MachineTrajectory.h"

namespace cadcam::machine
{
    class PlanarTrajectoryBuilder
    {
    public:
        static OperationResult<PlanarTrajectory> build
        (
            const std::vector<PlanarTrajectoryEntityInput>& entities,
            const ToolClearancePolicy& clearance,
            const OperationContext& context
        );
    };
}
