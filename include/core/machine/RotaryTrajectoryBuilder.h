#pragma once

#include "application/tasks/TaskContext.h"
#include "core/machine/MachineTrajectory.h"

namespace cadcam::machine
{
    class RotaryTrajectoryBuilder
    {
    public:
        static OperationResult<MachineTrajectory> build
        (
            const RotaryTrajectoryInput& input,
            const RotaryMachinePolicy& policy,
            const TaskContext& context
        );
    };
}
