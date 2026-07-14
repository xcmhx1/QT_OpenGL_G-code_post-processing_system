#include "application/planning/ProcessPlanningService.h"

#include "CadDocument.h"
#include "compatibility/legacy/DocumentProcessPlanningAdapter.h"
#include "compatibility/legacy/LegacyProcessPlanAdapter.h"
#include "core/planning/PlanarProcessPlanBuilder.h"
#include "core/planning/ProcessPlanBuilder.h"

OperationResult<cadcam::planning::ProcessPlan> ProcessPlanningService::buildPlanarPlan
(
    CadDocument& document,
    const cadcam::planning::PlanarProcessPlanningPolicy& policy,
    const OperationContext& context
) const
{
    OperationResult<cadcam::planning::ProcessPlan> result;
    DocumentProcessPlanningAdapter adapter;
    auto capture = adapter.capturePlanar(document, context);
    result.mergeDiagnostics(capture);
    if (!capture.succeeded() || !capture.value.has_value())
    {
        result.status = capture.status;
        return result;
    }
    auto plan = cadcam::planning::PlanarProcessPlanBuilder::build
        (*capture.value, policy, context);
    result.mergeDiagnostics(plan);
    if (!plan.succeeded() || !plan.value.has_value())
    {
        result.status = plan.status;
        return result;
    }
    if (document.contentRevision() != plan.value->contentRevision)
    {
        result.status = OperationStatus::Conflict;
        Diagnostic diagnostic;
        diagnostic.code = DiagnosticCode::ProcessPlanningRevisionMismatch;
        diagnostic.severity = DiagnosticSeverity::Error;
        diagnostic.component = QStringLiteral("ProcessPlanningService");
        diagnostic.operation = context.operationName;
        diagnostic.stage = QStringLiteral("validate-planar-plan-revision");
        diagnostic.userMessage = QStringLiteral("文档在三轴加工计划构建期间已变更。");
        diagnostic.correlationId = context.correlationId;
        result.addDiagnostic(diagnostic);
        return result;
    }
    OperationReport applied = LegacyProcessPlanAdapter().apply(document, *plan.value, context);
    result.mergeDiagnostics(applied);
    if (!applied.succeeded())
    {
        result.status = applied.status;
        return result;
    }
    result.status = plan.status;
    result.value = std::move(*plan.value);
    return result;
}

OperationResult<cadcam::planning::ProcessPlan> ProcessPlanningService::buildRotaryPlan
(
    CadDocument& document,
    const std::optional<cadcam::machining::TubeSectionModel>& tubeSection,
    const cadcam::planning::ProcessPlanningPolicy& policy,
    const OperationContext& context
) const
{
    cadcam::topology::PathTopology topology;
    DocumentProcessPlanningAdapter adapter;
    auto capture = adapter.captureRotary
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
