#include "application/machine/MachineTrajectoryService.h"

#include "cad/document/CadDocument.h"
#include "application/geometry/DocumentGeometrySnapshotBuilder.h"
#include "core/geometry/GeometryCompiler.h"
#include "core/machine/RotaryTrajectoryBuilder.h"
#include "drw_entities.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>

namespace
{
    cadcam::geometry::SamplingPolicy productionSamplingPolicy(int dxfType)
    {
        cadcam::geometry::SamplingPolicy policy;
        policy.chordTolerance = 0.0;
        switch (static_cast<DRW::ETYPE>(dxfType))
        {
        case DRW::ETYPE::CIRCLE:
            policy.minimumSegments = 128;
            policy.fullTurnSegments = 128;
            break;
        case DRW::ETYPE::ARC:
            policy.minimumSegments = 8;
            policy.maximumAngularStep = 5.0 * 6.28318530717958647692 / 360.0;
            policy.fullTurnSegments = 128;
            break;
        case DRW::ETYPE::ELLIPSE:
            policy.minimumSegments = 16;
            policy.fullTurnSegments = 128;
            break;
        case DRW::ETYPE::POLYLINE:
        case DRW::ETYPE::LWPOLYLINE:
            policy.minimumSegments = 1;
            policy.minimumBulgeSegments = 4;
            policy.fullTurnSegments = 128;
            break;
        default:
            policy.minimumSegments = 1;
            break;
        }
        return policy;
    }

    Diagnostic serviceDiagnostic
    (
        DiagnosticCode code,
        const QString& message,
        const OperationContext& context,
        cadcam::geometry::EntityId entityId = 0
    )
    {
        Diagnostic diagnostic;
        diagnostic.code = code;
        diagnostic.severity = DiagnosticSeverity::Error;
        diagnostic.component = QStringLiteral("MachineTrajectoryService");
        diagnostic.operation = context.operationName;
        diagnostic.stage = QStringLiteral("build-rotary-trajectory");
        diagnostic.userMessage = message;
        diagnostic.correlationId = context.correlationId;
        if (entityId != 0) diagnostic.entityId = entityId;
        return diagnostic;
    }
}

OperationResult<cadcam::machine::MachineTrajectory> MachineTrajectoryService::buildRotaryTrajectory
(
    CadDocument& document,
    const cadcam::process::DocumentProcessState& processState,
    const cadcam::planning::ProcessPlan& plan,
    const std::optional<cadcam::machining::TubeSectionModel>& tubeSection,
    const GProfileRotaryAxisConfig& config,
    const TaskContext& taskContext,
    const std::optional<cadcam::geometry::Vector2d>& explicitTubeCenter
) const
{
    using namespace cadcam;
    OperationResult<machine::MachineTrajectory> result;
    if (plan.mode != planning::ProcessPlanMode::Rotary4Axis)
    {
        result.status = OperationStatus::Conflict;
        result.addDiagnostic(serviceDiagnostic(DiagnosticCode::ProcessPlanModeMismatch,
            QStringLiteral("四轴轨迹只能使用四轴加工计划。"),
            taskContext.operationContext));
        return result;
    }
    DocumentGeometrySnapshotBuilder snapshotBuilder;
    auto captured = snapshotBuilder.capture(document, taskContext.operationContext);
    result.mergeDiagnostics(captured);
    if (!captured.succeeded() || !captured.value.has_value())
    {
        result.status = OperationStatus::Failed;
        return result;
    }
    if (captured.value->contentRevision != plan.contentRevision
        || plan.contentRevision != document.contentRevision()
        || plan.processStateRevision != processState.revision()
        || (tubeSection.has_value() && tubeSection->contentRevision != 0
            && tubeSection->contentRevision != plan.contentRevision))
    {
        result.status = OperationStatus::Conflict;
        result.addDiagnostic(serviceDiagnostic(DiagnosticCode::MachineTrajectoryRevisionMismatch,
            QStringLiteral("加工计划、截面模型或文档版本不一致，四轴导出已拒绝。"),
            taskContext.operationContext));
        return result;
    }

    std::vector<planning::ProcessAssignment> assignments = plan.assignments;
    std::sort(assignments.begin(), assignments.end(), [](const auto& left, const auto& right)
    {
        return left.processOrder < right.processOrder;
    });
    if (assignments.empty())
    {
        result.status = OperationStatus::InvalidInput;
        result.addDiagnostic(serviceDiagnostic(DiagnosticCode::MachineTrajectoryInputInvalid,
            QStringLiteral("当前四轴加工计划为空。"), taskContext.operationContext));
        return result;
    }

    std::set<geometry::EntityId> plannedIds;
    for (const auto& assignment : assignments) plannedIds.insert(assignment.entityId);
    std::set<geometry::EntityId> excludedIds;
    for (const auto& exclusion : plan.exclusions) excludedIds.insert(exclusion.entityId);
    for (const auto& entry : captured.value->entries)
    {
        const bool processable = entry.sourceKind != geometry::SourceGeometryKind::Point
            && entry.sourceKind != geometry::SourceGeometryKind::Unknown
            && entry.sourceEntity.has_value();
        if (processable && excludedIds.count(entry.attributes.entityId) == 0U
            && plannedIds.count(entry.attributes.entityId) == 0U)
        {
            result.status = OperationStatus::Conflict;
            result.addDiagnostic(serviceDiagnostic(DiagnosticCode::MachineTrajectoryEntityMissing,
                QStringLiteral("四轴加工计划未覆盖全部可加工图元。"),
                taskContext.operationContext, entry.attributes.entityId));
            return result;
        }
    }

    std::map<geometry::EntityId, const GeometrySourceEntry*> entries;
    for (const auto& entry : captured.value->entries) entries[entry.attributes.entityId] = &entry;
    std::map<int, const planning::ProcessGroup*> groups;
    for (const auto& group : plan.groups) groups[group.groupId] = &group;

    machine::RotaryTrajectoryInput input;
    input.contentRevision = plan.contentRevision;
    input.processStateRevision = plan.processStateRevision;
    input.processGroups = plan.groups;
    input.tubeSection = tubeSection;
    geometry::GeometryCompiler compiler;
    input.entities.reserve(assignments.size());
    for (std::size_t index = 0; index < assignments.size(); ++index)
    {
        const auto& assignment = assignments[index];
        const auto found = entries.find(assignment.entityId);
        if (assignment.processOrder != static_cast<int>(index) || found == entries.end()
            || found->second == nullptr || !found->second->sourceEntity.has_value())
        {
            result.status = OperationStatus::Failed;
            if (found != entries.end() && found->second != nullptr)
                result.addDiagnostic(serviceDiagnostic(DiagnosticCode::EmptyPath,
                    QStringLiteral("图元加工路径为空。"), taskContext.operationContext, assignment.entityId));
            result.addDiagnostic(serviceDiagnostic(DiagnosticCode::MachineTrajectoryEntityMissing,
                QStringLiteral("加工计划中的图元在几何快照中不存在。"),
                taskContext.operationContext, assignment.entityId));
            return result;
        }
        geometry::PathCompileOptions options;
        options.reverse = assignment.reverse;
        options.startParameter = assignment.startParameter;
        auto compiled = compiler.compile
        (
            *found->second->sourceEntity,
            productionSamplingPolicy(found->second->attributes.originalDxfType),
            options,
            taskContext.operationContext
        );
        result.mergeDiagnostics(compiled);
        if (!compiled.succeeded() || !compiled.value.has_value())
        {
            result.status = OperationStatus::Failed;
            result.addDiagnostic(serviceDiagnostic(DiagnosticCode::EmptyPath,
                QStringLiteral("图元加工路径为空。"),
                taskContext.operationContext, assignment.entityId));
            result.addDiagnostic(serviceDiagnostic(DiagnosticCode::MachineTrajectoryGeometryCompileFailed,
                QStringLiteral("加工计划中的图元无法编译为四轴源路径。"),
                taskContext.operationContext, assignment.entityId));
            return result;
        }

        machine::TrajectoryEntityInput entity;
        entity.entityId = assignment.entityId;
        entity.sourceIndex = found->second->sourceIndex;
        entity.sourceKind = found->second->sourceKind;
        entity.processOrder = assignment.processOrder;
        entity.processGroupId = assignment.continuousGroupId;
        entity.path = std::move(*compiled.value);
        entity.closed = entity.path.closed;
        const auto group = groups.find(entity.processGroupId);
        if (group != groups.end() && group->second != nullptr)
        {
            entity.closed = group->second->closed;
            entity.firstInGroup = index == 0
                || assignments[index - 1].continuousGroupId != entity.processGroupId;
            entity.lastInGroup = index + 1 == assignments.size()
                || assignments[index + 1].continuousGroupId != entity.processGroupId;
        }
        else
        {
            entity.firstInGroup = true;
            entity.lastInGroup = true;
        }
        input.entities.push_back(std::move(entity));
    }

    machine::RotaryMachinePolicy policy;
    policy.rotaryAxisY = config.centerY;
    policy.rotaryAxisZ = config.centerZ;
    policy.invertAAxisDirection = config.invertAAxisDirection;
    policy.aAxisOffsetDegrees = config.aAxisOffsetDegrees;
    policy.keepContinuousAngle = config.keepContinuousAngle;
    policy.useInitialMachinePoint = config.useInitialMachinePoint;
    policy.initialMachinePoint = { config.initialMachineX, config.initialMachineY, config.initialMachineZ, 0.0 };
    policy.useSafeZBeforeRapid = config.useSafeZBeforeRapid;
    policy.safeRadialClearance = config.safeZ;
    policy.machiningPlaneZOffset = config.machiningPlaneZOffset;
    policy.overcutDistance = config.overcutDistance;
    if (tubeSection.has_value() && !tubeSection->geometry.boundary.empty())
    {
        policy.tubeCenterY = tubeSection->geometry.centerY;
        policy.tubeCenterZ = tubeSection->geometry.centerZ;
    }
    else if (explicitTubeCenter.has_value())
    {
        policy.tubeCenterY = explicitTubeCenter->x;
        policy.tubeCenterZ = explicitTubeCenter->y;
    }
    else
    {
        double minimumY = input.entities.front().path.vertices.front().position.y;
        double maximumY = minimumY;
        double minimumZ = input.entities.front().path.vertices.front().position.z;
        double maximumZ = minimumZ;
        for (const auto& entity : input.entities)
            for (const auto& vertex : entity.path.vertices)
            {
                minimumY = std::min(minimumY, vertex.position.y);
                maximumY = std::max(maximumY, vertex.position.y);
                minimumZ = std::min(minimumZ, vertex.position.z);
                maximumZ = std::max(maximumZ, vertex.position.z);
            }
        policy.tubeCenterY = (minimumY + maximumY) * 0.5;
        policy.tubeCenterZ = (minimumZ + maximumZ) * 0.5;
    }
    if (!std::isfinite(policy.tubeCenterY) || !std::isfinite(policy.tubeCenterZ)
        || !std::isfinite(policy.rotaryAxisY) || !std::isfinite(policy.rotaryAxisZ))
    {
        result.status = OperationStatus::InvalidInput;
        result.addDiagnostic(serviceDiagnostic(DiagnosticCode::RotaryCenterInvalid,
            QStringLiteral("方管中心或旋转轴中心无效。"), taskContext.operationContext));
        return result;
    }

    auto built = machine::RotaryTrajectoryBuilder::build(input, policy, taskContext);
    result.mergeDiagnostics(built);
    result.status = built.status;
    result.value = std::move(built.value);
    return result;
}
