#include "application/machine/MachineTrajectoryService.h"

#include "cad/document/CadDocument.h"
#include "application/geometry/DocumentGeometrySnapshotBuilder.h"
#include "core/machine/ProcessUnitExecutionResolver.h"
#include "core/machine/RotaryTrajectoryBuilder.h"
#include "core/machining/TubeSectionProjector.h"
#include "drw_entities.h"

#include <QDebug>

#include <algorithm>
#include <cmath>
#include <map>
#include <set>

namespace
{
    QString surfaceRegionName(cadcam::machine::RotarySurfaceRegion region)
    {
        using cadcam::machine::RotarySurfaceRegion;
        switch (region)
        {
        case RotarySurfaceRegion::Top: return QStringLiteral("Top");
        case RotarySurfaceRegion::Right: return QStringLiteral("Right");
        case RotarySurfaceRegion::Bottom: return QStringLiteral("Bottom");
        case RotarySurfaceRegion::Left: return QStringLiteral("Left");
        case RotarySurfaceRegion::Corner: return QStringLiteral("Corner");
        case RotarySurfaceRegion::Radial: return QStringLiteral("Radial");
        case RotarySurfaceRegion::Unknown: return QStringLiteral("Unknown");
        }
        return QStringLiteral("Unknown");
    }

    QString sourceKindName(cadcam::geometry::SourceGeometryKind kind)
    {
        using cadcam::geometry::SourceGeometryKind;
        switch (kind)
        {
        case SourceGeometryKind::Point: return QStringLiteral("Point");
        case SourceGeometryKind::Line: return QStringLiteral("Line");
        case SourceGeometryKind::Arc: return QStringLiteral("Arc");
        case SourceGeometryKind::Circle: return QStringLiteral("Circle");
        case SourceGeometryKind::Ellipse: return QStringLiteral("Ellipse");
        case SourceGeometryKind::Polyline: return QStringLiteral("Polyline");
        case SourceGeometryKind::Spline: return QStringLiteral("Spline");
        case SourceGeometryKind::Unknown: return QStringLiteral("Unknown");
        }
        return QStringLiteral("Unknown");
    }

    void logSurfaceSummaries(const cadcam::machine::MachineTrajectory& trajectory)
    {
        for (const auto& summary : trajectory.surfaceSummaries)
        {
            qInfo().noquote()
                << QStringLiteral("[RotaryKinematics][Surface] entityId=%1 processGroupId=%2 sourceKind=%3 pointCount=%4 ySpan=%5 zSpan=%6 classification=%7 surfaceTolerance=%8 rawAStart=%9 rawAEnd=%10 alignedAStart=%11 alignedAEnd=%12")
                    .arg(summary.entityId)
                    .arg(summary.processGroupId)
                    .arg(sourceKindName(summary.sourceKind))
                    .arg(summary.pointCount)
                    .arg(summary.ySpan, 0, 'g', 15)
                    .arg(summary.zSpan, 0, 'g', 15)
                    .arg(surfaceRegionName(summary.classification))
                    .arg(summary.surfaceTolerance, 0, 'g', 15)
                    .arg(summary.rawAStart, 0, 'g', 15)
                    .arg(summary.rawAEnd, 0, 'g', 15)
                    .arg(summary.alignedAStart, 0, 'g', 15)
                    .arg(summary.alignedAEnd, 0, 'g', 15);
        }
    }

    QString transferKindName(cadcam::machine::TransferMotionKind kind)
    {
        using cadcam::machine::TransferMotionKind;
        switch (kind)
        {
        case TransferMotionKind::InitialApproach:
            return QStringLiteral("InitialApproach");
        case TransferMotionKind::SameZoneSurfaceTransfer:
            return QStringLiteral("SameZoneSurfaceTransfer");
        case TransferMotionKind::SameZoneClearanceTransfer:
            return QStringLiteral("SameZoneClearanceTransfer");
        case TransferMotionKind::CrossZoneRotaryTransfer:
            return QStringLiteral("CrossZoneRotaryTransfer");
        }
        return QStringLiteral("Unknown");
    }

    QString ownerZoneName
    (
        const std::optional<cadcam::machining::TubeZone16>& zone
    )
    {
        return zone.has_value()
            ? cadcam::machining::tubeZoneName(*zone)
            : QStringLiteral("None");
    }

    QString poseText(const cadcam::machine::MachinePose4D& pose)
    {
        return QStringLiteral("(%1,%2,%3,%4)")
            .arg(pose.x, 0, 'g', 15)
            .arg(pose.y, 0, 'g', 15)
            .arg(pose.z, 0, 'g', 15)
            .arg(pose.aDegrees, 0, 'g', 15);
    }

    QString sourcePointText(const cadcam::geometry::Vector3d& point)
    {
        return QStringLiteral("(%1,%2,%3)")
            .arg(point.x, 0, 'g', 15)
            .arg(point.y, 0, 'g', 15)
            .arg(point.z, 0, 'g', 15);
    }

    void logTransferSummaries
    (
        const cadcam::machine::MachineTrajectory& trajectory
    )
    {
        using cadcam::machine::TransferMotionKind;
        for (const auto& summary : trajectory.transferSummaries)
        {
            QString message = QStringLiteral(
                "[MachineTrajectory][Transfer] fromProcessUnit=%1 "
                "toProcessUnit=%2 fromOwnerZone=%3 toOwnerZone=%4 "
                "kind=%5 previousCutEnd=%6 nextCutStart=%7 deltaA=%8 "
                "rotationSafetyClearance=%9 sameZoneTransferClearance=%10 "
                "rotationSafeMachineZ=%11 coordinated=%12 segmentCount=%13 "
                "plannedFinalApproachOrigin=%14 "
                "actualFinalApproachOrigin=%15 previewMatched=%16")
                .arg(summary.fromProcessUnit)
                .arg(summary.toProcessUnit)
                .arg(ownerZoneName(summary.fromOwnerZone))
                .arg(ownerZoneName(summary.toOwnerZone))
                .arg(transferKindName(summary.kind))
                .arg(poseText(summary.previousCutEnd))
                .arg(poseText(summary.nextCutStart))
                .arg(summary.deltaA, 0, 'g', 15)
                .arg(summary.rotationSafetyClearance, 0, 'g', 15)
                .arg(summary.sameZoneTransferClearance, 0, 'g', 15)
                .arg(summary.rotationSafeMachineZ, 0, 'g', 15)
                .arg(summary.coordinated
                    ? QStringLiteral("true") : QStringLiteral("false"))
                .arg(summary.segmentCount)
                .arg(summary.hasPlannedPreview
                    ? poseText(summary.plannedFinalApproachOrigin)
                    : QStringLiteral("None"))
                .arg(poseText(summary.actualFinalApproachOrigin))
                .arg(summary.hasPlannedPreview
                    ? (summary.previewMatched
                        ? QStringLiteral("true")
                        : QStringLiteral("false"))
                    : QStringLiteral("not-planned"));
            if (summary.hasPlannedPreview)
            {
                message += QStringLiteral(
                    " plannedPreviousCutEnd=%1 actualPreviousCutEnd=%2 "
                    "plannedPreviousSourceEnd=%3 actualPreviousSourceEnd=%4 "
                    "poseDelta=%5 sourceDelta=%6")
                    .arg(poseText(summary.plannedPreviousCutEnd))
                    .arg(poseText(summary.actualPreviousCutEnd))
                    .arg(sourcePointText
                        (summary.plannedPreviousSourceEnd))
                    .arg(sourcePointText
                        (summary.actualPreviousSourceEnd))
                    .arg(summary.poseDelta, 0, 'g', 15)
                    .arg(summary.sourceDelta, 0, 'g', 15);
            }
            if (summary.kind
                == TransferMotionKind::CrossZoneRotaryTransfer
                || summary.kind == TransferMotionKind::InitialApproach)
            {
                message += QStringLiteral(
                    " departureTarget=%1 rotaryTransferTarget=%2 "
                    "approachTarget=%3")
                    .arg(poseText(summary.departureTarget))
                    .arg(poseText(summary.rotaryTransferTarget))
                    .arg(poseText(summary.approachTarget));
            }
            else if (summary.kind
                == TransferMotionKind::SameZoneSurfaceTransfer)
            {
                message += QStringLiteral(" surfaceTransfer=%1")
                    .arg(summary.surfaceTransfer
                        ? QStringLiteral("true") : QStringLiteral("false"));
            }
            else if (summary.kind
                == TransferMotionKind::SameZoneClearanceTransfer)
            {
                message += QStringLiteral(
                    " departureTarget=%1 arrivalClearanceTarget=%2 "
                    "approachTarget=%3")
                    .arg(poseText(summary.departureTarget))
                    .arg(poseText(summary.rotaryTransferTarget))
                    .arg(poseText(summary.approachTarget));
            }
            qInfo().noquote() << message;
        }
    }

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
    const GProfileToolTransferConfig& transferConfig,
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

    machine::RotaryTrajectoryInput input;
    input.contentRevision = plan.contentRevision;
    input.processStateRevision = plan.processStateRevision;
    input.processGroups = plan.groups;
    input.tubeSection = tubeSection;

    std::vector<machine::ProcessUnitExecutionSource> executionSources;
    executionSources.reserve(captured.value->entries.size());
    for (const GeometrySourceEntry& entry : captured.value->entries)
    {
        if (!entry.sourceEntity.has_value()) continue;
        executionSources.push_back
        ({
            entry.attributes.entityId,
            entry.sourceIndex,
            entry.sourceKind,
            &*entry.sourceEntity,
            productionSamplingPolicy(entry.attributes.originalDxfType)
        });
    }

    input.entities.reserve(assignments.size() + plan.plannedFragments.size());
    for (std::size_t unitIndex = 0U;
        unitIndex < plan.processUnits.size(); ++unitIndex)
    {
        const planning::ProcessUnit& unit = plan.processUnits[unitIndex];
        auto compiled = machine::ProcessUnitExecutionResolver::compilePaths
        (
            unit,
            static_cast<int>(unitIndex),
            assignments,
            plan.plannedFragments,
            executionSources,
            taskContext.operationContext
        );
        result.mergeDiagnostics(compiled);
        if (!compiled.succeeded() || !compiled.value.has_value())
        {
            result.status = compiled.status;
            return result;
        }

        const planning::ProcessAssignment* firstAssignment = nullptr;
        for (const planning::ProcessAssignment& assignment : assignments)
        {
            if (assignment.processUnitIndex != static_cast<int>(unitIndex))
            {
                continue;
            }
            if (firstAssignment == nullptr
                || assignment.processOrder < firstAssignment->processOrder)
            {
                firstAssignment = &assignment;
            }
        }
        if (firstAssignment == nullptr)
        {
            result.status = OperationStatus::Failed;
            result.addDiagnostic(serviceDiagnostic
            (
                DiagnosticCode::MachineTrajectoryEntityMissing,
                QStringLiteral("加工单元缺少加工分配。"),
                taskContext.operationContext
            ));
            return result;
        }

        for (std::size_t pathIndex = 0U;
            pathIndex < compiled.value->size(); ++pathIndex)
        {
            machine::ProcessUnitExecutionPath& path =
                (*compiled.value)[pathIndex];
            machine::TrajectoryEntityInput entity;
            entity.entityId = path.entityId;
            entity.sourceIndex = path.sourceIndex;
            entity.sourceKind = path.sourceKind;
            entity.processOrder = static_cast<int>(input.entities.size());
            entity.sourceProcessOrder = path.sourceProcessOrder;
            entity.fragmentOrder = path.fragmentOrder;
            entity.processGroupId = path.processGroupId;
            entity.processUnitIndex = path.processUnitIndex;
            entity.ownerZone = unit.ownerZone;
            entity.closed = unit.closed;
            if (pathIndex == 0U)
            {
                entity.plannedIncomingTransfer =
                    firstAssignment->plannedIncomingTransfer;
            }
            entity.path = std::move(path.path);
            input.entities.push_back(std::move(entity));
        }
    }
    for (std::size_t index = 0; index < input.entities.size(); ++index)
    {
        auto& entity = input.entities[index];
        entity.firstInGroup = index == 0U
            || input.entities[index - 1U].processUnitIndex
                != entity.processUnitIndex;
        entity.lastInGroup = index + 1U == input.entities.size()
            || input.entities[index + 1U].processUnitIndex
                != entity.processUnitIndex;
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
    policy.transfer.rotationSafetyClearance =
        transferConfig.rotationSafetyClearance;
    policy.transfer.sameZoneTransferClearance =
        transferConfig.sameZoneTransferClearance;
    policy.transfer.coordinatedTransferEnabled =
        transferConfig.coordinatedTransferEnabled;
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
    policy.surfaceClassificationTolerance = std::max(policy.numericalEpsilon, 1.0e-6);
    if (tubeSection.has_value() && !tubeSection->geometry.boundary.empty())
    {
        double minimumY = tubeSection->geometry.boundary.front().x;
        double maximumY = minimumY;
        double minimumZ = tubeSection->geometry.boundary.front().y;
        double maximumZ = minimumZ;
        for (const auto& point : tubeSection->geometry.boundary)
        {
            minimumY = std::min(minimumY, point.x);
            maximumY = std::max(maximumY, point.x);
            minimumZ = std::min(minimumZ, point.y);
            maximumZ = std::max(maximumZ, point.y);
        }
        const double maximumDimension = std::max(maximumY - minimumY, maximumZ - minimumZ);
        policy.surfaceClassificationTolerance = std::max
            (policy.surfaceClassificationTolerance, maximumDimension * 1.0e-8);
    }
    if (!std::isfinite(policy.tubeCenterY) || !std::isfinite(policy.tubeCenterZ)
        || !std::isfinite(policy.rotaryAxisY) || !std::isfinite(policy.rotaryAxisZ))
    {
        result.status = OperationStatus::InvalidInput;
        result.addDiagnostic(serviceDiagnostic(DiagnosticCode::RotaryCenterInvalid,
            QStringLiteral("方管中心或旋转轴中心无效。"), taskContext.operationContext));
        return result;
    }

    qInfo().noquote()
        << QStringLiteral("[MachineTrajectory][TransferPolicy] mode=Rotary4Axis rotationSafetyClearance=%1 sameZoneTransferClearance=%2 coordinated=%3")
            .arg(policy.transfer.rotationSafetyClearance, 0, 'g', 15)
            .arg(policy.transfer.sameZoneTransferClearance, 0, 'g', 15)
            .arg(policy.transfer.coordinatedTransferEnabled
                ? QStringLiteral("true") : QStringLiteral("false"));
    auto built = machine::RotaryTrajectoryBuilder::build(input, policy, taskContext);
    result.mergeDiagnostics(built);
    result.status = built.status;
    result.value = std::move(built.value);
    if (result.value.has_value())
    {
        logSurfaceSummaries(*result.value);
        logTransferSummaries(*result.value);
    }
    return result;
}
