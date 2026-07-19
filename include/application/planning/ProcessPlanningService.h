#pragma once

#include "core/diagnostics/OperationContext.h"
#include "core/diagnostics/OperationResult.h"
#include "core/planning/ProcessPlan.h"
#include "core/planning/PlanarProcessPlanBuilder.h"
#include "application/process/DocumentProcessState.h"

#include <optional>

class CadDocument;

class ProcessPlanningService
{
public:
    OperationResult<cadcam::planning::ProcessPlan> buildPlanarPlan
    (
        CadDocument& document,
        const cadcam::process::DocumentProcessState& processState,
        const cadcam::planning::PlanarProcessPlanningPolicy& policy,
        const OperationContext& context
    ) const;

    OperationResult<cadcam::planning::ProcessPlan> buildRotaryPlan
    (
        CadDocument& document,
        const cadcam::process::DocumentProcessState& processState,
        const std::optional<cadcam::machining::TubeSectionModel>& tubeSection,
        const cadcam::planning::ProcessPlanningPolicy& policy,
        const OperationContext& context
    ) const;

    OperationResult<cadcam::planning::ProcessPlan> reorderPlanByUnitSequence
    (
        const cadcam::planning::ProcessPlan& plan,
        const cadcam::planning::ProcessUnitSequence& sequence,
        const OperationContext& context
    ) const;

};
