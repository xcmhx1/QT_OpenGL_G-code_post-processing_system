#pragma once

#include "core/diagnostics/OperationContext.h"
#include "core/diagnostics/OperationResult.h"
#include "core/machine/MachineTrajectory.h"

namespace cadcam::machine
{
    struct RotarySurfaceOverride
    {
        RotarySurfaceRegion region = RotarySurfaceRegion::Unknown;
        std::optional<geometry::Vector2d> cornerCenter;
    };

    struct RotaryKinematicsResult
    {
        std::vector<MachinePose4D> poses;
        RotarySurfaceSummary surface;
    };

    class RotaryKinematics
    {
    public:
        static geometry::Vector3d sourceLocalClearancePose
        (
            const geometry::Vector3d& cutEnd,
            double outwardDistance,
            const std::optional<machining::TubeSectionModel>& section,
            double tubeCenterY,
            double tubeCenterZ,
            double tolerance
        );

        static double sectionMaximumCollisionRadius
        (
            const machining::TubeSectionModel& section,
            double tubeCenterY,
            double tubeCenterZ
        );

        static double rotationSafeMachineZ
        (
            double tubeCenterZ,
            double maximumCollisionRadius,
            double rotationSafetyClearance
        );

        static OperationResult<std::vector<RotarySurfaceOverride>>
            classifyNoSectionUnitSurfaces
        (
            const std::vector<const geometry::Path3D*>& paths,
            bool closed,
            const RotaryMachinePolicy& policy,
            const OperationContext& context
        );

        static geometry::Vector3d sourceTransferAnchor
        (
            const geometry::Vector3d& cutEnd,
            const std::optional<machining::TubeZone16>& previousOwnerZone,
            const std::optional<machining::TubeZone16>& nextOwnerZone,
            const ToolTransferPolicy& transfer,
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
            const std::optional<RotarySurfaceOverride>& surfaceOverride,
            const OperationContext& context
        );
    };
}
