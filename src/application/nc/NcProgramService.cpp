#include "application/nc/NcProgramService.h"

#include "CadDocument.h"
#include "application/machine/MachineTrajectoryService.h"
#include "compatibility/legacy/DocumentNcMetadataAdapter.h"
#include "core/nc/NcProgramBuilder.h"

OperationResult<cadcam::nc::NcProgram> NcProgramService::buildRotaryProgram
(
    CadDocument& document,
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
        || trajectory.value->contentRevision != processPlan.contentRevision)
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
