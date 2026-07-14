#pragma once

#include "GProfile.h"
#include "application/tasks/TaskContext.h"
#include "core/machine/MachineTrajectory.h"

class CadDocument;

class MachineTrajectoryService
{
public:
    OperationResult<cadcam::machine::MachineTrajectory> buildRotaryTrajectory
    (
        CadDocument& document,
        const cadcam::planning::ProcessPlan& plan,
        const std::optional<cadcam::machining::TubeSectionModel>& tubeSection,
        const GProfileRotaryAxisConfig& config,
        const TaskContext& context,
        const std::optional<cadcam::geometry::Vector2d>& explicitTubeCenter = std::nullopt
    ) const;
};
