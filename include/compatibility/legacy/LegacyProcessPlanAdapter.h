#pragma once

#include "core/diagnostics/OperationContext.h"
#include "core/diagnostics/OperationResult.h"
#include "core/planning/ProcessPlan.h"

#include <optional>

class CadDocument;

class LegacyProcessPlanAdapter
{
public:
    OperationResult<cadcam::planning::ProcessPlanningInput> capture
    (
        CadDocument& document,
        const std::optional<cadcam::machining::TubeSectionModel>& tubeSection,
        double connectionTolerance,
        cadcam::topology::PathTopology& topologyStorage,
        const OperationContext& context
    ) const;

    OperationReport apply
    (
        CadDocument& document,
        const cadcam::planning::ProcessPlan& plan,
        const OperationContext& context
    ) const;
};
