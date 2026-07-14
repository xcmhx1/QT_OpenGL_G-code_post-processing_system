#pragma once

#include "core/diagnostics/OperationContext.h"
#include "core/diagnostics/OperationResult.h"
#include "core/machine/MachineTrajectory.h"

namespace cadcam::machine
{
    class RotaryKinematics
    {
    public:
        static OperationResult<std::vector<MachinePose4D>> transform
        (
            const geometry::Path3D& path,
            const RotaryMachinePolicy& policy,
            const std::optional<machining::TubeSectionModel>& section,
            const OperationContext& context
        );
    };
}
