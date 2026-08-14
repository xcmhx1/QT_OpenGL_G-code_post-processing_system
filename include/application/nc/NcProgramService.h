#pragma once

#include "infrastructure/config/MachiningProcessConfig.h"
#include "application/tasks/TaskContext.h"
#include "core/machining/TubeSection.h"
#include "core/nc/NcProgram.h"
#include "core/planning/ProcessPlan.h"
#include "application/process/DocumentProcessState.h"

class CadDocument;

class NcProgramService
{
public:
    OperationResult<cadcam::nc::NcProgram> buildPlanarProgram
    (
        CadDocument& document,
        const cadcam::process::DocumentProcessState& processState,
        const cadcam::planning::ProcessPlan& processPlan,
        const MachiningProcessConfig& processConfig,
        const OperationContext& context
    ) const;

    OperationResult<cadcam::nc::NcProgram> buildRotaryProgram
    (
        CadDocument& document,
        const cadcam::process::DocumentProcessState& processState,
        const cadcam::planning::ProcessPlan& processPlan,
        const std::optional<cadcam::machining::TubeSectionModel>& tubeSection,
        const MachiningProcessConfig& processConfig,
        const OperationContext& context,
        const std::optional<cadcam::geometry::Vector2d>& explicitTubeCenter = std::nullopt
    ) const;
};
