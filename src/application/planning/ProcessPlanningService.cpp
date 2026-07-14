#include "application/planning/ProcessPlanningService.h"

#include "CadDocument.h"
#include "compatibility/legacy/LegacyProcessPlanAdapter.h"
#include "core/planning/ProcessPlanBuilder.h"

OperationResult<cadcam::planning::ProcessPlan> ProcessPlanningService::build
(
    CadDocument& document,
    const std::optional<cadcam::machining::TubeSectionModel>& tubeSection,
    const cadcam::planning::ProcessPlanningPolicy& policy,
    const OperationContext& context
) const
{
    cadcam::topology::PathTopology topology;
    LegacyProcessPlanAdapter adapter;
    auto capture = adapter.capture
        (document, tubeSection, policy.connectionTolerance, topology, context);
    if (!capture.succeeded() || !capture.value.has_value())
    {
        OperationResult<cadcam::planning::ProcessPlan> result;
        result.status = capture.status;
        result.mergeDiagnostics(capture.diagnostics);
        return result;
    }
    auto result = cadcam::planning::ProcessPlanBuilder::build
        (*capture.value, policy, context);
    result.mergeDiagnostics(capture.diagnostics);
    return result;
}

OperationReport ProcessPlanningService::apply
(
    CadDocument& document,
    const cadcam::planning::ProcessPlan& plan,
    const OperationContext& context
) const
{
    return LegacyProcessPlanAdapter().apply(document, plan, context);
}
