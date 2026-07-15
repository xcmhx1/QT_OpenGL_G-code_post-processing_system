#include "application/planning/ProcessPlanningService.h"

#include "cad/document/CadDocument.h"
#include "application/planning/DocumentProcessPlanningAdapter.h"
#include "core/planning/PlanarProcessPlanBuilder.h"
#include "core/planning/ProcessPlanBuilder.h"

OperationResult<cadcam::planning::ProcessPlan> ProcessPlanningService::buildPlanarPlan
(
    CadDocument& document,
    const cadcam::process::DocumentProcessState& processState,
    const cadcam::planning::PlanarProcessPlanningPolicy& policy,
    const OperationContext& context
) const
{
    OperationResult<cadcam::planning::ProcessPlan> result;
    DocumentProcessPlanningAdapter adapter;
    auto capture = adapter.capturePlanar(document, processState, context);
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
    if (document.contentRevision() != plan.value->contentRevision
        || processState.revision() != plan.value->processStateRevision)
    {
        result.status = OperationStatus::Conflict;
        Diagnostic diagnostic;
        diagnostic.code = DiagnosticCode::ProcessStateRevisionMismatch;
        diagnostic.severity = DiagnosticSeverity::Error;
        diagnostic.component = QStringLiteral("ProcessPlanningService");
        diagnostic.operation = context.operationName;
        diagnostic.stage = QStringLiteral("validate-planar-plan-revision");
        diagnostic.userMessage = QStringLiteral("文档或加工状态在三轴加工计划构建期间已变更。");
        diagnostic.correlationId = context.correlationId;
        result.addDiagnostic(diagnostic);
        return result;
    }
    result.status = plan.status;
    result.value = std::move(*plan.value);
    return result;
}

OperationResult<cadcam::planning::ProcessPlan> ProcessPlanningService::buildRotaryPlan
(
    CadDocument& document,
    const cadcam::process::DocumentProcessState& processState,
    const std::optional<cadcam::machining::TubeSectionModel>& tubeSection,
    const cadcam::planning::ProcessPlanningPolicy& policy,
    const OperationContext& context
) const
{
    cadcam::topology::PathTopology topology;
    DocumentProcessPlanningAdapter adapter;
    auto capture = adapter.captureRotary
        (document, processState, tubeSection, policy.connectionTolerance, topology, context);
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
    if (result.succeeded() && result.value.has_value()
        && (document.contentRevision() != result.value->contentRevision
            || processState.revision() != result.value->processStateRevision))
    {
        result.status = OperationStatus::Conflict;
        result.value.reset();
        Diagnostic diagnostic;
        diagnostic.code = DiagnosticCode::ProcessStateRevisionMismatch;
        diagnostic.severity = DiagnosticSeverity::Error;
        diagnostic.component = QStringLiteral("ProcessPlanningService");
        diagnostic.operation = context.operationName;
        diagnostic.stage = QStringLiteral("validate-rotary-plan-revision");
        diagnostic.userMessage = QStringLiteral("文档或加工状态在四轴加工计划构建期间已变更。");
        diagnostic.correlationId = context.correlationId;
        result.addDiagnostic(diagnostic);
    }
    return result;
}
