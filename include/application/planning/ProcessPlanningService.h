#pragma once

#include "core/diagnostics/OperationContext.h"
#include "core/diagnostics/OperationResult.h"
#include "core/planning/ProcessPlan.h"

#include <optional>

class CadDocument;

class ProcessPlanningService
{
public:
    OperationResult<cadcam::planning::ProcessPlan> build
    (
        CadDocument& document,
        const std::optional<cadcam::machining::TubeSectionModel>& tubeSection,
        const cadcam::planning::ProcessPlanningPolicy& policy,
        const OperationContext& context
    ) const;

    OperationReport apply
    (
        CadDocument& document,
        const cadcam::planning::ProcessPlan& plan,
        const OperationContext& context
    ) const;
};
