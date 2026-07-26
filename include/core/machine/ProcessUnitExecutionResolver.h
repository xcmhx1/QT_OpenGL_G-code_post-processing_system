#pragma once

#include "core/machine/RotaryKinematics.h"
#include "core/planning/ProcessPlan.h"

#include <optional>
#include <vector>

namespace cadcam::machine
{
    struct ProcessUnitExecutionSource
    {
        geometry::EntityId entityId = 0;
        std::size_t sourceIndex = 0U;
        geometry::SourceGeometryKind sourceKind =
            geometry::SourceGeometryKind::Unknown;
        const geometry::SourceEntity* sourceEntity = nullptr;
        geometry::SamplingPolicy samplingPolicy;
    };

    struct ProcessUnitExecutionPath
    {
        geometry::EntityId entityId = 0;
        std::size_t sourceIndex = 0U;
        geometry::SourceGeometryKind sourceKind =
            geometry::SourceGeometryKind::Unknown;
        int sourceProcessOrder = -1;
        int fragmentOrder = -1;
        int processGroupId = -1;
        int processUnitIndex = -1;
        geometry::Path3D path;
    };

    struct ProcessUnitOvercutTarget
    {
        MachinePose4D pose;
        geometry::Vector3d sourcePosition;
    };

    struct ProcessUnitExecutionResult
    {
        std::vector<ProcessUnitExecutionPath> paths;
        std::vector<std::vector<MachinePose4D>> posesByPath;
        std::vector<RotarySurfaceSummary> surfaceSummaries;
        MachinePose4D cutStart;
        MachinePose4D finalCutPose;
        geometry::Vector3d sourceStart;
        geometry::Vector3d finalSourcePosition;
        std::optional<MachinePose4D> closurePose;
        std::vector<ProcessUnitOvercutTarget> overcutTargets;
        bool overcutLimited = false;
    };

    class ProcessUnitExecutionResolver
    {
    public:
        static OperationResult<std::vector<ProcessUnitExecutionPath>>
            compilePaths
            (
                const planning::ProcessUnit& unit,
                int processUnitIndex,
                const std::vector<planning::ProcessAssignment>& assignments,
                const std::vector<planning::ProcessPathFragment>& fragments,
                const std::vector<ProcessUnitExecutionSource>& sources,
                const OperationContext& context
            );

        static OperationResult<ProcessUnitExecutionResult> resolve
            (
                const std::vector<ProcessUnitExecutionPath>& paths,
                bool closed,
                const RotaryMachinePolicy& policy,
                const std::optional<machining::TubeSectionModel>& section,
                const std::optional<MachinePose4D>& previousPose,
                const OperationContext& context
            );
    };
}
