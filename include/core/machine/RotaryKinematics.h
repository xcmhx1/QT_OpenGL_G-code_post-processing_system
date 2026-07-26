#pragma once

#include "core/diagnostics/OperationContext.h"
#include "core/diagnostics/OperationResult.h"
#include "core/machine/MachineTrajectory.h"

namespace cadcam::machine
{
    struct RotaryKinematicsResult
    {
        std::vector<MachinePose4D> poses;
        RotarySurfaceSummary surface;
    };

    class RotaryKinematics
    {
    public:
        static geometry::Vector3d sourceRetractPose
        (
            const geometry::Vector3d& cutEnd,
            double outwardDistance,
            const std::optional<machining::TubeSectionModel>& section,
            double tubeCenterY,
            double tubeCenterZ,
            double tolerance
        );

        static OperationResult<RotaryKinematicsResult> transform
        (
            const geometry::Path3D& path,
            const RotaryMachinePolicy& policy,
            const std::optional<machining::TubeSectionModel>& section,
            const OperationContext& context
        );
    };
}
