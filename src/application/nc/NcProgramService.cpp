#include "application/nc/NcProgramService.h"

#include "cad/document/CadDocument.h"
#include "application/machine/MachineTrajectoryService.h"
#include "compatibility/legacy/DocumentNcMetadataAdapter.h"
#include "compatibility/legacy/DocumentPlanarNcInputAdapter.h"
#include "core/nc/NcProgramBuilder.h"
#include "core/nc/PlanarNcProgramBuilder.h"

OperationResult<cadcam::nc::NcProgram> NcProgramService::buildPlanarProgram
(
    CadDocument& document,
    const cadcam::process::DocumentProcessState& processState,
    const cadcam::planning::ProcessPlan& processPlan,
    const OperationContext& context
) const
{
    OperationResult<cadcam::nc::NcProgram> result;
    if (processPlan.mode != cadcam::planning::ProcessPlanMode::Planar3Axis)
    {
        result.status = OperationStatus::Conflict;
        Diagnostic diagnostic;
        diagnostic.code = DiagnosticCode::ProcessPlanModeMismatch;
        diagnostic.severity = DiagnosticSeverity::Error;
        diagnostic.component = QStringLiteral("NcProgramService");
        diagnostic.operation = context.operationName;
        diagnostic.stage = QStringLiteral("validate-planar-plan-mode");
        diagnostic.userMessage = QStringLiteral("三轴 NC 只能使用三轴加工计划。");
        diagnostic.correlationId = context.correlationId;
        result.addDiagnostic(diagnostic);
        return result;
    }
    auto capture = DocumentPlanarNcInputAdapter::capture(document, processPlan, context);
    result.mergeDiagnostics(capture);
    if (!capture.succeeded() || !capture.value.has_value())
    {
        result.status = capture.status;
        return result;
    }

    if (capture.value->contentRevision != document.contentRevision()
        || capture.value->contentRevision != processPlan.contentRevision
        || processPlan.processStateRevision != processState.revision())
    {
        result.status = OperationStatus::Conflict;
        Diagnostic diagnostic;
        diagnostic.code = DiagnosticCode::PlanarNcRevisionMismatch;
        diagnostic.severity = DiagnosticSeverity::Error;
        diagnostic.component = QStringLiteral("NcProgramService");
        diagnostic.operation = context.operationName;
        diagnostic.stage = QStringLiteral("validate-planar-revision");
        diagnostic.userMessage = QStringLiteral("三轴 NC 输入与当前文档版本不一致。");
        diagnostic.correlationId = context.correlationId;
        result.addDiagnostic(diagnostic);
        return result;
    }

    cadcam::nc::PlanarNcBuildPolicy policy;
    auto program = cadcam::nc::PlanarNcProgramBuilder::build
        (capture.value->contentRevision, capture.value->entities, policy, context,
            processPlan.processStateRevision);
    result.mergeDiagnostics(program);
    if (!program.succeeded() || !program.value.has_value())
    {
        result.status = program.status;
        return result;
    }
    if (document.contentRevision() != capture.value->contentRevision
        || processState.revision() != processPlan.processStateRevision)
    {
        result.status = OperationStatus::Conflict;
        Diagnostic diagnostic;
        diagnostic.code = DiagnosticCode::PlanarNcRevisionMismatch;
        diagnostic.severity = DiagnosticSeverity::Error;
        diagnostic.component = QStringLiteral("NcProgramService");
        diagnostic.operation = context.operationName;
        diagnostic.stage = QStringLiteral("validate-planar-result");
        diagnostic.userMessage = QStringLiteral("文档在三轴 NC 程序构建期间已变更。");
        diagnostic.correlationId = context.correlationId;
        result.addDiagnostic(diagnostic);
        return result;
    }

    result.status = capture.status == OperationStatus::PartialSuccess
        || program.status == OperationStatus::PartialSuccess
        ? OperationStatus::PartialSuccess : OperationStatus::Success;
    result.value = std::move(*program.value);
    return result;
}

OperationResult<cadcam::nc::NcProgram> NcProgramService::buildRotaryProgram
(
    CadDocument& document,
    const cadcam::process::DocumentProcessState& processState,
    const cadcam::planning::ProcessPlan& processPlan,
    const std::optional<cadcam::machining::TubeSectionModel>& tubeSection,
    const GProfileRotaryAxisConfig& rotaryConfig,
    const OperationContext& context,
    const std::optional<cadcam::geometry::Vector2d>& explicitTubeCenter
) const
{
    OperationResult<cadcam::nc::NcProgram> result;
    TaskContext taskContext;
    taskContext.operationContext = context;
    MachineTrajectoryService trajectoryService;
    auto trajectory = trajectoryService.buildRotaryTrajectory
    (
        document,
        processState,
        processPlan,
        tubeSection,
        rotaryConfig,
        taskContext,
        explicitTubeCenter
    );
    result.mergeDiagnostics(trajectory);
    if (!trajectory.succeeded() || !trajectory.value.has_value())
    {
        result.status = trajectory.status;
        return result;
    }

    auto metadata = DocumentNcMetadataAdapter::capture(document, processPlan, context);
    result.mergeDiagnostics(metadata);
    if (!metadata.succeeded() || !metadata.value.has_value())
    {
        result.status = metadata.status;
        return result;
    }
    if (document.contentRevision() != processPlan.contentRevision
        || processState.revision() != processPlan.processStateRevision
        || trajectory.value->contentRevision != processPlan.contentRevision
        || trajectory.value->processStateRevision != processPlan.processStateRevision)
    {
        result.status = OperationStatus::Conflict;
        Diagnostic diagnostic;
        diagnostic.code = DiagnosticCode::NcProgramRevisionMismatch;
        diagnostic.severity = DiagnosticSeverity::Error;
        diagnostic.component = QStringLiteral("NcProgramService");
        diagnostic.operation = context.operationName;
        diagnostic.stage = QStringLiteral("validate-revision");
        diagnostic.userMessage = QStringLiteral("轨迹、元数据、加工计划或文档版本不一致。");
        diagnostic.correlationId = context.correlationId;
        diagnostic.context.insert(QStringLiteral("contentRevision"),
            QVariant::fromValue<qulonglong>(document.contentRevision()));
        result.addDiagnostic(diagnostic);
        return result;
    }

    auto program = cadcam::nc::NcProgramBuilder::buildRotary
        (*trajectory.value, *metadata.value, context, tubeSection);
    result.mergeDiagnostics(program);
    result.status = program.status;
    result.value = std::move(program.value);
    return result;
}
