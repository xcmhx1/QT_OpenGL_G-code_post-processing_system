#pragma once

#include "core/diagnostics/OperationContext.h"
#include "core/diagnostics/OperationResult.h"
#include "core/machine/MachineTrajectory.h"
#include "core/machining/TubeSection.h"
#include "core/nc/NcProgram.h"

namespace cadcam::nc
{
    class NcProgramBuilder
    {
    public:
        static OperationResult<NcProgram> buildRotary
        (
            const machine::MachineTrajectory& trajectory,
            const std::vector<NcEntityMetadata>& metadata,
            const OperationContext& context,
            const std::optional<machining::TubeSectionModel>& tubeSection = std::nullopt
        );
    };
}
