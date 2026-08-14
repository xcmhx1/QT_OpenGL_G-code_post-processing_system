#pragma once

#include "infrastructure/config/MachiningProcessConfig.h"
#include "application/tasks/TaskContext.h"
#include "application/process/DocumentProcessState.h"
#include "core/machine/MachineTrajectory.h"

class CadDocument;

class MachineTrajectoryService
{
public:
    OperationResult<cadcam::machine::MachineTrajectory> buildRotaryTrajectory
    (
        CadDocument& document,
        const cadcam::process::DocumentProcessState& processState,
        const cadcam::planning::ProcessPlan& plan,
        const std::optional<cadcam::machining::TubeSectionModel>& tubeSection,
        const MachiningProcessConfig& processConfig,
        const TaskContext& context,
        const std::optional<cadcam::geometry::Vector2d>& explicitTubeCenter = std::nullopt
    ) const;
};
