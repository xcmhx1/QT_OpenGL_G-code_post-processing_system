#include "core/machine/RotaryTrajectoryBuilder.h"

#include "core/machine/RotaryKinematics.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>

namespace cadcam::machine
{
    namespace
    {
        double sourceDistance(const geometry::Vector3d& left, const geometry::Vector3d& right)
        {
            return std::sqrt
            (
                (left.x - right.x) * (left.x - right.x)
                + (left.y - right.y) * (left.y - right.y)
                + (left.z - right.z) * (left.z - right.z)
            );
        }

        double machinePositionDistance(const MachinePose4D& left, const MachinePose4D& right)
        {
            return std::sqrt
            (
                (left.x - right.x) * (left.x - right.x)
                + (left.y - right.y) * (left.y - right.y)
                + (left.z - right.z) * (left.z - right.z)
            );
        }

        Diagnostic diagnostic
        (
            DiagnosticCode code,
            DiagnosticSeverity severity,
            const QString& message,
            const OperationContext& context,
            const TrajectoryEntityInput* entity = nullptr
        )
        {
            Diagnostic value;
            value.code = code;
            value.severity = severity;
            value.component = QStringLiteral("RotaryTrajectoryBuilder");
            value.operation = context.operationName;
            value.stage = QStringLiteral("build-machine-trajectory");
            value.userMessage = message;
            value.correlationId = context.correlationId;
            if (entity != nullptr)
            {
                value.entityId = entity->entityId;
                value.groupId = entity->processGroupId;
                value.context.insert(QStringLiteral("sourceIndex"), static_cast<qulonglong>(entity->sourceIndex));
                value.context.insert(QStringLiteral("processOrder"), entity->processOrder);
                value.context.insert(QStringLiteral("processGroupId"), entity->processGroupId);
            }
            return value;
        }

        MachineMove move
        (
            MachineMoveKind kind,
            const MachinePose4D& pose,
            const TrajectoryEntityInput& entity,
            TransferMotionKind transferKind =
                TransferMotionKind::InitialApproach,
            TransferMotionPhase transferPhase = TransferMotionPhase::None
        )
        {
            return
            {
                kind,
                pose,
                transferKind,
                transferPhase,
                entity.entityId,
                entity.processGroupId
            };
        }

        MachinePose4D interpolate
        (
            const MachinePose4D& start,
            const MachinePose4D& end,
            double factor
        );

        bool transferIsSafe
        (
            const MachinePose4D& start,
            const MachinePose4D& targetCutStart,
            const std::vector<MachineMove>& moves,
            const TransferMotionSummary& summary,
            double tolerance
        )
        {
            MachinePose4D current = start;
            int surfaceCount = 0;
            bool hasDeparture = false;
            bool hasRotary = false;
            bool hasApproach = false;
            for (const MachineMove& value : moves)
            {
                if (value.transferKind != summary.kind) return false;
                const bool aChanges = std::abs
                    (value.target.aDegrees - current.aDegrees) > tolerance;
                if (value.transferPhase == TransferMotionPhase::SurfaceTransfer)
                    ++surfaceCount;
                if (value.transferPhase
                    == TransferMotionPhase::CoordinatedDeparture)
                    hasDeparture = true;
                if (value.transferPhase
                    == TransferMotionPhase::SafeRotaryTransfer)
                    hasRotary = true;
                if (value.transferPhase
                    == TransferMotionPhase::CoordinatedApproach)
                    hasApproach = true;

                if (aChanges)
                {
                    if (value.transferPhase
                            != TransferMotionPhase::SafeRotaryTransfer
                        || current.z < summary.rotationSafeMachineZ - tolerance
                        || value.target.z
                            < summary.rotationSafeMachineZ - tolerance)
                    {
                        return false;
                    }
                }
                if (value.transferPhase
                        == TransferMotionPhase::CoordinatedApproach
                    && std::abs(current.aDegrees
                        - targetCutStart.aDegrees) > tolerance)
                {
                    return false;
                }
                current = value.target;
            }

            if (machinePositionDistance(current, targetCutStart) > tolerance
                || std::abs(current.aDegrees - targetCutStart.aDegrees)
                    > tolerance)
            {
                return false;
            }
            if (summary.kind == TransferMotionKind::SameZoneSurfaceTransfer)
            {
                return surfaceCount <= 1
                    && !hasDeparture
                    && !hasRotary
                    && !hasApproach
                    && std::abs(current.aDegrees - start.aDegrees) <= tolerance;
            }
            if (summary.kind == TransferMotionKind::SameZoneClearanceTransfer)
            {
                const bool departureClearanceReached =
                    summary.departureTarget.z
                    >= start.z + summary.sameZoneTransferClearance - tolerance;
                const bool arrivalClearanceReached =
                    summary.rotaryTransferTarget.z
                    >= targetCutStart.z
                        + summary.sameZoneTransferClearance - tolerance;
                return !hasRotary
                    && hasApproach
                    && departureClearanceReached
                    && arrivalClearanceReached
                    && std::abs(current.aDegrees - start.aDegrees) <= tolerance;
            }
            const bool rotationExpected =
                std::abs(targetCutStart.aDegrees - start.aDegrees)
                    > tolerance;
            return hasApproach
                && !moves.empty()
                && (!rotationExpected || hasRotary)
                && (hasDeparture || start.z
                    >= summary.rotationSafeMachineZ - tolerance);
        }

        RotaryTransferRequest transferRequest
        (
            const MachinePose4D& previousCutEnd,
            const MachinePose4D& nextCutStart,
            const geometry::Vector3d& previousSourceEnd,
            const geometry::Vector3d& nextSourceStart,
            const TrajectoryEntityInput* previousEntity,
            const TrajectoryEntityInput& nextEntity,
            const RotaryMachinePolicy& policy,
            const std::optional<machining::TubeSectionModel>& section,
            double rotationSafeMachineZ
        )
        {
            RotaryTransferRequest request;
            request.previousCutEnd = previousCutEnd;
            request.nextCutStart = nextCutStart;
            request.previousSourceEnd = previousSourceEnd;
            request.nextSourceStart = nextSourceStart;
            request.previousProcessUnitIndex = previousEntity != nullptr
                ? previousEntity->processUnitIndex : -1;
            request.nextProcessUnitIndex = nextEntity.processUnitIndex;
            request.previousOwnerZone = previousEntity != nullptr
                ? previousEntity->ownerZone : std::nullopt;
            request.nextOwnerZone = nextEntity.ownerZone;
            request.policy = policy.transfer;
            request.tubeSection = &section;
            request.tubeCenterY = policy.tubeCenterY;
            request.tubeCenterZ = policy.tubeCenterZ;
            request.rotationSafeMachineZ = rotationSafeMachineZ;
            request.numericalEpsilon = policy.numericalEpsilon;
            return request;
        }

        bool poseMatches
        (
            const MachinePose4D& actual,
            const planning::PlannedMachinePose4D& planned,
            double tolerance
        )
        {
            return machinePositionDistance(actual,
                    { planned.x, planned.y, planned.z, planned.aDegrees })
                    <= tolerance
                && std::abs(actual.aDegrees - planned.aDegrees) <= tolerance;
        }

        bool plannedKindMatches
        (
            TransferMotionKind actual,
            planning::PlannedTransferMotionKind planned
        )
        {
            switch (actual)
            {
            case TransferMotionKind::InitialApproach:
                return planned
                    == planning::PlannedTransferMotionKind::InitialApproach;
            case TransferMotionKind::SameZoneSurfaceTransfer:
                return planned == planning::PlannedTransferMotionKind::
                    SameZoneSurfaceTransfer;
            case TransferMotionKind::SameZoneClearanceTransfer:
                return planned == planning::PlannedTransferMotionKind::
                    SameZoneClearanceTransfer;
            case TransferMotionKind::CrossZoneRotaryTransfer:
                return planned == planning::PlannedTransferMotionKind::
                    CrossZoneRotaryTransfer;
            }
            return false;
        }

        bool plannedPhaseMatches
        (
            TransferMotionPhase actual,
            planning::PlannedTransferMotionPhase planned
        )
        {
            switch (actual)
            {
            case TransferMotionPhase::SurfaceTransfer:
                return planned
                    == planning::PlannedTransferMotionPhase::SurfaceTransfer;
            case TransferMotionPhase::CoordinatedDeparture:
                return planned == planning::PlannedTransferMotionPhase::
                    CoordinatedDeparture;
            case TransferMotionPhase::SafeRotaryTransfer:
                return planned == planning::PlannedTransferMotionPhase::
                    SafeRotaryTransfer;
            case TransferMotionPhase::CoordinatedApproach:
                return planned == planning::PlannedTransferMotionPhase::
                    CoordinatedApproach;
            case TransferMotionPhase::None:
                return false;
            }
            return false;
        }

        bool previewMatchesPlan
        (
            const RotaryTransferPreview& preview,
            const planning::PlannedTransferSignature& planned,
            double tolerance
        )
        {
            if (!plannedKindMatches(preview.kind, planned.kind)
                || !poseMatches(preview.finalApproachOrigin,
                    planned.finalApproachOrigin, tolerance)
                || !poseMatches(preview.cutStart, planned.cutStart, tolerance)
                || preview.targets.size() != planned.targets.size()
                || preview.phases.size() != planned.phases.size())
            {
                return false;
            }
            for (std::size_t index = 0; index < preview.targets.size(); ++index)
            {
                if (!poseMatches(preview.targets[index],
                        planned.targets[index], tolerance)
                    || !plannedPhaseMatches(preview.phases[index],
                        planned.phases[index]))
                {
                    return false;
                }
            }
            return true;
        }

        TransferMotionSummary appendTransferPreview
        (
            std::vector<MachineMove>& moves,
            const RotaryTransferRequest& request,
            const RotaryTransferPreview& preview,
            const TrajectoryEntityInput& nextEntity
        )
        {
            for (std::size_t index = 0; index < preview.targets.size(); ++index)
            {
                moves.push_back(move(MachineMoveKind::Rapid,
                    preview.targets[index], nextEntity, preview.kind,
                    preview.phases[index]));
            }

            TransferMotionSummary summary;
            summary.fromProcessUnit = request.previousProcessUnitIndex;
            summary.toProcessUnit = request.nextProcessUnitIndex;
            summary.fromOwnerZone = request.previousOwnerZone;
            summary.toOwnerZone = request.nextOwnerZone;
            summary.kind = preview.kind;
            summary.previousCutEnd = request.previousCutEnd;
            summary.nextCutStart = request.nextCutStart;
            summary.actualFinalApproachOrigin =
                preview.finalApproachOrigin;
            summary.deltaA = request.nextCutStart.aDegrees
                - request.previousCutEnd.aDegrees;
            summary.rotationSafetyClearance =
                request.policy.rotationSafetyClearance;
            summary.sameZoneTransferClearance =
                request.policy.sameZoneTransferClearance;
            summary.rotationSafeMachineZ =
                request.rotationSafeMachineZ;
            summary.coordinated =
                request.policy.coordinatedTransferEnabled;
            summary.surfaceTransfer = preview.kind
                == TransferMotionKind::SameZoneSurfaceTransfer;
            summary.segmentCount = static_cast<int>(preview.targets.size());
            for (std::size_t index = 0; index < preview.targets.size(); ++index)
            {
                switch (preview.phases[index])
                {
                case TransferMotionPhase::CoordinatedDeparture:
                    summary.departureTarget = preview.targets[index];
                    break;
                case TransferMotionPhase::SafeRotaryTransfer:
                    summary.rotaryTransferTarget = preview.targets[index];
                    break;
                case TransferMotionPhase::CoordinatedApproach:
                case TransferMotionPhase::SurfaceTransfer:
                    summary.approachTarget = preview.targets[index];
                    break;
                case TransferMotionPhase::None:
                    break;
                }
            }
            if (nextEntity.plannedIncomingTransfer.has_value())
            {
                summary.hasPlannedPreview = true;
                const auto& planned =
                    *nextEntity.plannedIncomingTransfer;
                summary.plannedFinalApproachOrigin =
                {
                    planned.finalApproachOrigin.x,
                    planned.finalApproachOrigin.y,
                    planned.finalApproachOrigin.z,
                    planned.finalApproachOrigin.aDegrees
                };
                summary.previewMatched = previewMatchesPlan
                    (preview, planned, request.numericalEpsilon);
            }
            return summary;
        }

        MachinePose4D interpolate(const MachinePose4D& start, const MachinePose4D& end, double factor)
        {
            return
            {
                start.x + (end.x - start.x) * factor,
                start.y + (end.y - start.y) * factor,
                start.z + (end.z - start.z) * factor,
                start.aDegrees + (end.aDegrees - start.aDegrees) * factor
            };
        }

        bool finitePose(const MachinePose4D& pose)
        {
            return std::isfinite(pose.x) && std::isfinite(pose.y)
                && std::isfinite(pose.z) && std::isfinite(pose.aDegrees);
        }

        MachinePose4D finalPose(const EntityTrajectory& entity, const MachinePose4D& fallback)
        {
            if (!entity.overcutMoves.empty()) return entity.overcutMoves.back().target;
            if (!entity.cuttingMoves.empty()) return entity.cuttingMoves.back().target;
            if (!entity.approachMoves.empty()) return entity.approachMoves.back().target;
            return fallback;
        }
    }

    OperationResult<MachineTrajectory> RotaryTrajectoryBuilder::build
    (
        const RotaryTrajectoryInput& input,
        const RotaryMachinePolicy& policy,
        const TaskContext& taskContext
    )
    {
        OperationResult<MachineTrajectory> result;
        if (input.entities.empty() || input.contentRevision == 0
            || !std::isfinite(policy.rotaryAxisY) || !std::isfinite(policy.rotaryAxisZ)
            || !std::isfinite(policy.tubeCenterY) || !std::isfinite(policy.tubeCenterZ)
            || !std::isfinite(policy.transfer.rotationSafetyClearance)
            || policy.transfer.rotationSafetyClearance <= 0.0
            || !std::isfinite(policy.transfer.sameZoneTransferClearance)
            || policy.transfer.sameZoneTransferClearance < 0.0
            || !std::isfinite(policy.continuousConnectionTolerance)
            || policy.continuousConnectionTolerance <= 0.0
            || !std::isfinite(policy.numericalEpsilon) || policy.numericalEpsilon <= 0.0
            || !std::isfinite(policy.surfaceClassificationTolerance)
            || policy.surfaceClassificationTolerance <= 0.0)
        {
            result.status = OperationStatus::InvalidInput;
            result.addDiagnostic(diagnostic(DiagnosticCode::MachineTrajectoryInputInvalid,
                DiagnosticSeverity::Error, QStringLiteral("四轴轨迹输入为空或版本无效。"), taskContext.operationContext));
            return result;
        }

        std::set<geometry::EntityId> wholeEntityIds;
        std::set<std::pair<geometry::EntityId, int>> fragmentIds;
        std::set<geometry::EntityId> fragmentedEntityIds;
        for (std::size_t index = 0; index < input.entities.size(); ++index)
        {
            const auto& entity = input.entities[index];
            if (entity.processOrder != static_cast<int>(index) || entity.entityId == 0
                || entity.sourceProcessOrder < 0
                || entity.processUnitIndex < 0
                || entity.path.vertices.empty())
            {
                result.status = OperationStatus::InvalidInput;
                result.addDiagnostic(diagnostic(DiagnosticCode::MachineTrajectoryInputInvalid,
                    DiagnosticSeverity::Error, QStringLiteral("四轴轨迹的加工顺序、图元标识或路径无效。"),
                    taskContext.operationContext, &entity));
                return result;
            }
            bool identityValid = false;
            if (entity.fragmentOrder >= 0)
            {
                identityValid =
                    wholeEntityIds.count(entity.entityId) == 0U
                    && fragmentIds.emplace(entity.entityId,
                        entity.fragmentOrder).second;
                fragmentedEntityIds.insert(entity.entityId);
            }
            else
            {
                identityValid =
                    fragmentedEntityIds.count(entity.entityId) == 0U
                    && wholeEntityIds.insert(entity.entityId).second;
            }
            if (!identityValid)
            {
                result.status = OperationStatus::InvalidInput;
                result.addDiagnostic(diagnostic
                (
                    DiagnosticCode::MachineTrajectoryInputInvalid,
                    DiagnosticSeverity::Error,
                    QStringLiteral("四轴轨迹包含重复的完整图元或片段执行标识。"),
                    taskContext.operationContext, &entity
                ));
                return result;
            }
        }

        std::map<int, const planning::ProcessGroup*> groupDefinitions;
        for (const auto& group : input.processGroups) groupDefinitions[group.groupId] = &group;
        std::map<int, std::vector<geometry::EntityId>> actualGroupMembers;
        for (const auto& entity : input.entities)
            if (entity.processGroupId >= 0) actualGroupMembers[entity.processGroupId].push_back(entity.entityId);
        for (const auto& [groupId, entityIds] : actualGroupMembers)
        {
            const auto found = groupDefinitions.find(groupId);
            const std::set<geometry::EntityId> actualIds(entityIds.begin(), entityIds.end());
            const std::set<geometry::EntityId> plannedIds = found != groupDefinitions.end()
                && found->second != nullptr
                ? std::set<geometry::EntityId>
                    (found->second->entityIds.begin(), found->second->entityIds.end())
                : std::set<geometry::EntityId>{};
            std::size_t firstIndex = input.entities.size();
            std::size_t lastIndex = 0;
            for (std::size_t index = 0; index < input.entities.size(); ++index)
                if (input.entities[index].processGroupId == groupId)
                {
                    firstIndex = std::min(firstIndex, index);
                    lastIndex = index;
                }
            const bool contiguous = firstIndex < input.entities.size()
                && lastIndex - firstIndex + 1U == entityIds.size();
            if (found == groupDefinitions.end() || found->second == nullptr
                || actualIds != plannedIds || !contiguous)
            {
                result.status = OperationStatus::Conflict;
                result.addDiagnostic(diagnostic(DiagnosticCode::MachineTrajectoryInvariantViolation,
                    DiagnosticSeverity::Error, QStringLiteral("连续加工组的成员或顺序与加工计划不一致。"),
                    taskContext.operationContext));
                return result;
            }
        }

        MachineTrajectory trajectory;
        trajectory.contentRevision = input.contentRevision;
        trajectory.processStateRevision = input.processStateRevision;
        trajectory.rotaryContext.rotaryAxisY = policy.rotaryAxisY;
        trajectory.rotaryContext.rotaryAxisZ = policy.rotaryAxisZ;
        trajectory.rotaryContext.tubeCenterY = policy.tubeCenterY;
        trajectory.rotaryContext.tubeCenterZ = policy.tubeCenterZ;
        if (input.tubeSection.has_value() && !input.tubeSection->geometry.boundary.empty())
        {
            const auto& boundary = input.tubeSection->geometry.boundary;
            trajectory.rotaryContext.hasSectionBounds = true;
            trajectory.rotaryContext.sectionMinimumY = trajectory.rotaryContext.sectionMaximumY = boundary.front().x;
            trajectory.rotaryContext.sectionMinimumZ = trajectory.rotaryContext.sectionMaximumZ = boundary.front().y;
            for (const auto& point : boundary)
            {
                trajectory.rotaryContext.sectionMinimumY = std::min(trajectory.rotaryContext.sectionMinimumY, point.x);
                trajectory.rotaryContext.sectionMaximumY = std::max(trajectory.rotaryContext.sectionMaximumY, point.x);
                trajectory.rotaryContext.sectionMinimumZ = std::min(trajectory.rotaryContext.sectionMinimumZ, point.y);
                trajectory.rotaryContext.sectionMaximumZ = std::max(trajectory.rotaryContext.sectionMaximumZ, point.y);
            }
        }
        else
        {
            const auto& first = input.entities.front().path.vertices.front().position;
            trajectory.rotaryContext.hasSectionBounds = true;
            trajectory.rotaryContext.sectionMinimumY = trajectory.rotaryContext.sectionMaximumY = first.y;
            trajectory.rotaryContext.sectionMinimumZ = trajectory.rotaryContext.sectionMaximumZ = first.z;
            for (const auto& entity : input.entities)
                for (const auto& vertex : entity.path.vertices)
                {
                    trajectory.rotaryContext.sectionMinimumY = std::min(trajectory.rotaryContext.sectionMinimumY, vertex.position.y);
                    trajectory.rotaryContext.sectionMaximumY = std::max(trajectory.rotaryContext.sectionMaximumY, vertex.position.y);
                    trajectory.rotaryContext.sectionMinimumZ = std::min(trajectory.rotaryContext.sectionMinimumZ, vertex.position.z);
                    trajectory.rotaryContext.sectionMaximumZ = std::max(trajectory.rotaryContext.sectionMaximumZ, vertex.position.z);
                }
        }
        if (input.tubeSection.has_value()
            && !input.tubeSection->geometry.boundary.empty())
        {
            trajectory.rotaryContext.maximumCollisionRadius =
                RotaryKinematics::sectionMaximumCollisionRadius
                (
                    *input.tubeSection,
                    policy.tubeCenterY,
                    policy.tubeCenterZ
                );
        }
        else
        {
            for (const auto& entity : input.entities)
                for (const auto& vertex : entity.path.vertices)
                    trajectory.rotaryContext.maximumCollisionRadius = std::max
                    (
                        trajectory.rotaryContext.maximumCollisionRadius,
                        std::hypot(vertex.position.y - policy.tubeCenterY,
                                   vertex.position.z - policy.tubeCenterZ)
                    );
        }
        trajectory.rotaryContext.safeMachineZ =
            RotaryKinematics::rotationSafeMachineZ
            (
                policy.tubeCenterZ,
                trajectory.rotaryContext.maximumCollisionRadius,
                policy.transfer.rotationSafetyClearance
            );

        std::vector<std::vector<MachinePose4D>> posesByEntity;
        posesByEntity.reserve(input.entities.size());
        std::vector<RotarySurfaceSummary> surfaceSummaries;
        surfaceSummaries.reserve(input.entities.size());
        for (const auto& entity : input.entities)
        {
            auto transformed = RotaryKinematics::transform
                (entity.path, policy, input.tubeSection, taskContext.operationContext);
            result.mergeDiagnostics(transformed);
            if (!transformed.succeeded() || !transformed.value.has_value())
            {
                result.status = OperationStatus::Failed;
                return result;
            }
            transformed.value->surface.processGroupId = entity.processGroupId;
            posesByEntity.push_back(std::move(transformed.value->poses));
            surfaceSummaries.push_back(std::move(transformed.value->surface));
        }

        trajectory.entities.reserve(input.entities.size());
        bool hasPrevious = false;
        MachinePose4D previousPose;
        geometry::Vector3d previousSourcePose;
        std::size_t groupStartIndex = 0U;
        for (std::size_t index = 0; index < input.entities.size(); ++index)
        {
            const auto& inputEntity = input.entities[index];
            auto& poses = posesByEntity[index];
            const bool sameUnit = index > 0
                && input.entities[index - 1].processUnitIndex
                    == inputEntity.processUnitIndex;
            const bool sameGroup = index > 0 && inputEntity.processGroupId >= 0
                && input.entities[index - 1].processGroupId == inputEntity.processGroupId;
            if (!sameGroup) groupStartIndex = index;
            if (policy.keepContinuousAngle && hasPrevious)
            {
                double offset = 0.0;
                while (poses.front().aDegrees + offset - previousPose.aDegrees > 180.0) offset -= 360.0;
                while (poses.front().aDegrees + offset - previousPose.aDegrees < -180.0) offset += 360.0;
                for (auto& pose : poses) pose.aDegrees += offset;
            }
            surfaceSummaries[index].alignedAStart = poses.front().aDegrees;
            surfaceSummaries[index].alignedAEnd = poses.back().aDegrees;
            const double connectionDistance = index > 0
                ? sourceDistance(input.entities[index - 1].path.vertices.back().position,
                                 inputEntity.path.vertices.front().position)
                : 0.0;
            if (sameUnit
                && connectionDistance > policy.continuousConnectionTolerance)
            {
                result.status = OperationStatus::Failed;
                Diagnostic value = diagnostic(DiagnosticCode::MachineTrajectoryContinuityFailure,
                    DiagnosticSeverity::Error, QStringLiteral("连续加工组内的相邻路径没有连接。"),
                    taskContext.operationContext, &inputEntity);
                value.context.insert(QStringLiteral("connectionDistance"), connectionDistance);
                result.addDiagnostic(value);
                return result;
            }
            if (sameUnit)
            {
                const MachinePose4D& previousEnd = posesByEntity[index - 1U].back();
                const MachinePose4D& nextStart = poses.front();
                const double machineDistance = machinePositionDistance(previousEnd, nextStart);
                const double angleDifference = std::abs(nextStart.aDegrees - previousEnd.aDegrees);
                if (machineDistance > policy.continuousConnectionTolerance
                    || angleDifference > 180.0 + policy.numericalEpsilon)
                {
                    result.status = OperationStatus::Failed;
                    Diagnostic value = diagnostic
                    (
                        DiagnosticCode::MachineTrajectoryContinuityFailure,
                        DiagnosticSeverity::Error,
                        QStringLiteral("连续加工组的源路径相连，但机床轨迹端点不连续。"),
                        taskContext.operationContext,
                        &inputEntity
                    );
                    value.context.insert(QStringLiteral("previousEntityId"),
                        QVariant::fromValue<qulonglong>(input.entities[index - 1U].entityId));
                    value.context.insert(QStringLiteral("nextEntityId"),
                        QVariant::fromValue<qulonglong>(inputEntity.entityId));
                    value.context.insert(QStringLiteral("sourceConnectionDistance"), connectionDistance);
                    value.context.insert(QStringLiteral("machineConnectionDistance"), machineDistance);
                    value.context.insert(QStringLiteral("previousA"), previousEnd.aDegrees);
                    value.context.insert(QStringLiteral("nextA"), nextStart.aDegrees);
                    result.addDiagnostic(value);
                    return result;
                }
            }

            EntityTrajectory entity;
            entity.entityId = inputEntity.entityId;
            entity.sourceKind = inputEntity.sourceKind;
            entity.sourceIndex = inputEntity.sourceIndex;
            entity.processOrder = inputEntity.processOrder;
            entity.sourceProcessOrder = inputEntity.sourceProcessOrder;
            entity.fragmentOrder = inputEntity.fragmentOrder;
            entity.processGroupId = inputEntity.processGroupId;
            entity.closed = inputEntity.closed;
            entity.continuousFromPrevious = sameUnit;
            entity.firstInGroup = inputEntity.firstInGroup;
            entity.lastInGroup = inputEntity.lastInGroup;
            for (const auto& vertex : inputEntity.path.vertices) entity.sourcePath.push_back(vertex.position);

            const MachinePose4D& first = poses.front();
            if (!hasPrevious)
            {
                const MachinePose4D initial = policy.useInitialMachinePoint
                    ? policy.initialMachinePoint
                    : MachinePose4D{ first.x, first.y, 0.0, 0.0 };
                const RotaryTransferRequest request = transferRequest
                    (initial, first,
                        inputEntity.path.vertices.front().position,
                        inputEntity.path.vertices.front().position,
                        nullptr, inputEntity, policy, input.tubeSection,
                        trajectory.rotaryContext.safeMachineZ);
                auto transfer = RotaryTransferPlanner::preview
                    (request, taskContext.operationContext);
                result.mergeDiagnostics(transfer);
                if (!transfer.succeeded() || !transfer.value.has_value())
                {
                    result.status = OperationStatus::Failed;
                    return result;
                }
                const TransferMotionSummary summary =
                    appendTransferPreview(entity.approachMoves, request,
                        *transfer.value, inputEntity);
                if (summary.hasPlannedPreview && !summary.previewMatched)
                {
                    result.status = OperationStatus::Conflict;
                    result.addDiagnostic(diagnostic
                    (
                        DiagnosticCode::
                            MachineTrajectoryTransferPreviewMismatch,
                        DiagnosticSeverity::Error,
                        QStringLiteral("实际首次接近轨迹与加工计划预览不一致。"),
                        taskContext.operationContext,
                        &inputEntity
                    ));
                    return result;
                }
                if (!transferIsSafe(initial, first, entity.approachMoves,
                    summary, policy.numericalEpsilon))
                {
                    result.status = OperationStatus::Failed;
                    result.addDiagnostic(diagnostic
                    (
                        DiagnosticCode::
                            MachineTrajectoryTransferSafetyViolation,
                        DiagnosticSeverity::Error,
                        QStringLiteral("首次接近轨迹未满足旋转安全约束。"),
                        taskContext.operationContext,
                        &inputEntity
                    ));
                    return result;
                }
                trajectory.transferSummaries.push_back(summary);
            }
            else if (!sameUnit)
            {
                const RotaryTransferRequest request = transferRequest
                    (previousPose, first,
                        previousSourcePose,
                        inputEntity.path.vertices.front().position,
                        &input.entities[index - 1U], inputEntity, policy,
                        input.tubeSection,
                        trajectory.rotaryContext.safeMachineZ);
                auto transfer = RotaryTransferPlanner::preview
                    (request, taskContext.operationContext);
                result.mergeDiagnostics(transfer);
                if (!transfer.succeeded() || !transfer.value.has_value())
                {
                    result.status = OperationStatus::Failed;
                    return result;
                }
                const TransferMotionSummary summary =
                    appendTransferPreview(entity.approachMoves, request,
                        *transfer.value, inputEntity);
                if (summary.hasPlannedPreview && !summary.previewMatched)
                {
                    result.status = OperationStatus::Conflict;
                    Diagnostic mismatch = diagnostic
                    (
                        DiagnosticCode::
                            MachineTrajectoryTransferPreviewMismatch,
                        DiagnosticSeverity::Error,
                        QStringLiteral("实际加工单元转移与加工计划预览不一致。"),
                        taskContext.operationContext,
                        &inputEntity
                    );
                    mismatch.context.insert(QStringLiteral("fromProcessUnit"),
                        summary.fromProcessUnit);
                    mismatch.context.insert(QStringLiteral("toProcessUnit"),
                        summary.toProcessUnit);
                    result.addDiagnostic(mismatch);
                    return result;
                }
                if (!transferIsSafe(previousPose, first,
                    entity.approachMoves, summary,
                    policy.numericalEpsilon))
                {
                    result.status = OperationStatus::Failed;
                    result.addDiagnostic(diagnostic
                    (
                        DiagnosticCode::
                            MachineTrajectoryTransferSafetyViolation,
                        DiagnosticSeverity::Error,
                        QStringLiteral("加工单元间转移未满足旋转安全约束。"),
                        taskContext.operationContext,
                        &inputEntity
                    ));
                    return result;
                }
                trajectory.transferSummaries.push_back(summary);
            }
            else if (connectionDistance > policy.numericalEpsilon)
                entity.cuttingMoves.push_back(move(MachineMoveKind::CuttingConnection, first, inputEntity));

            for (std::size_t pointIndex = 1; pointIndex < poses.size(); ++pointIndex)
                entity.cuttingMoves.push_back(move(MachineMoveKind::Cutting, poses[pointIndex], inputEntity));

            trajectory.entities.push_back(std::move(entity));
            geometry::Vector3d entityFinalSource =
                inputEntity.path.vertices.back().position;
            const bool groupEnds = index + 1U == input.entities.size()
                || input.entities[index + 1U].processGroupId != inputEntity.processGroupId;
            const auto groupDefinition = groupDefinitions.find(inputEntity.processGroupId);
            if (groupEnds && groupDefinition != groupDefinitions.end()
                && groupDefinition->second != nullptr && groupDefinition->second->closed)
            {
                std::vector<geometry::Vector3d> groupSourcePath;
                std::vector<MachinePose4D> groupMachinePath;
                for (std::size_t entityIndex = groupStartIndex; entityIndex <= index; ++entityIndex)
                {
                    const auto& path = input.entities[entityIndex].path.vertices;
                    const auto& groupPoses = posesByEntity[entityIndex];
                    for (std::size_t pointIndex = 0; pointIndex < path.size(); ++pointIndex)
                    {
                        if (!groupSourcePath.empty() && pointIndex == 0
                            && sourceDistance(groupSourcePath.back(), path.front().position)
                                <= policy.numericalEpsilon) continue;
                        groupSourcePath.push_back(path[pointIndex].position);
                        groupMachinePath.push_back(groupPoses[pointIndex]);
                    }
                }
                if (groupSourcePath.size() < 2U || groupSourcePath.size() != groupMachinePath.size())
                {
                    result.status = OperationStatus::Failed;
                    result.addDiagnostic(diagnostic(DiagnosticCode::MachineTrajectoryOvercutFailed,
                        DiagnosticSeverity::Error, QStringLiteral("闭合加工组的源路径无效。"),
                        taskContext.operationContext, &inputEntity));
                    return result;
                }

                const MachinePose4D loopStart = groupMachinePath.front();
                const MachinePose4D loopEnd = groupMachinePath.back();
                entityFinalSource = groupSourcePath.front();
                MachinePose4D alignedLoopStart = loopStart;
                if (policy.keepContinuousAngle)
                {
                    while (alignedLoopStart.aDegrees - loopEnd.aDegrees > 180.0)
                        alignedLoopStart.aDegrees -= 360.0;
                    while (alignedLoopStart.aDegrees - loopEnd.aDegrees < -180.0)
                        alignedLoopStart.aDegrees += 360.0;
                }
                if (loopEnd.x != alignedLoopStart.x || loopEnd.y != alignedLoopStart.y
                    || loopEnd.z != alignedLoopStart.z
                    || loopEnd.aDegrees != alignedLoopStart.aDegrees)
                    trajectory.entities.back().cuttingMoves.push_back
                        (move(MachineMoveKind::Cutting, alignedLoopStart, inputEntity));

                if (policy.overcutDistance > policy.numericalEpsilon)
                {
                    double totalLength = 0.0;
                    for (std::size_t pointIndex = 1; pointIndex < groupSourcePath.size(); ++pointIndex)
                        totalLength += sourceDistance
                            (groupSourcePath[pointIndex - 1], groupSourcePath[pointIndex]);
                    totalLength += sourceDistance(groupSourcePath.back(), groupSourcePath.front());
                    double remaining = std::min(policy.overcutDistance, totalLength);
                    const double angleOffset = alignedLoopStart.aDegrees - loopStart.aDegrees;
                    for (std::size_t pointIndex = 1;
                        pointIndex < groupSourcePath.size() && remaining > policy.numericalEpsilon;
                        ++pointIndex)
                    {
                        const double length = sourceDistance
                            (groupSourcePath[pointIndex - 1], groupSourcePath[pointIndex]);
                        if (length <= policy.numericalEpsilon) continue;
                        const double used = std::min(remaining, length);
                        MachinePose4D start = groupMachinePath[pointIndex - 1];
                        MachinePose4D end = groupMachinePath[pointIndex];
                        start.aDegrees += angleOffset;
                        end.aDegrees += angleOffset;
                        trajectory.entities.back().overcutMoves.push_back
                            (move(MachineMoveKind::Overcut,
                                interpolate(start, end, used / length), inputEntity));
                        entityFinalSource =
                            geometry::Vector3d
                            {
                                groupSourcePath[pointIndex - 1U].x
                                    + (groupSourcePath[pointIndex].x
                                        - groupSourcePath[pointIndex - 1U].x)
                                        * used / length,
                                groupSourcePath[pointIndex - 1U].y
                                    + (groupSourcePath[pointIndex].y
                                        - groupSourcePath[pointIndex - 1U].y)
                                        * used / length,
                                groupSourcePath[pointIndex - 1U].z
                                    + (groupSourcePath[pointIndex].z
                                        - groupSourcePath[pointIndex - 1U].z)
                                        * used / length
                            };
                        remaining -= used;
                    }
                    if (remaining > policy.numericalEpsilon)
                    {
                        const double closureLength = sourceDistance
                            (groupSourcePath.back(), groupSourcePath.front());
                        if (closureLength > policy.numericalEpsilon)
                        {
                            MachinePose4D closureStart = groupMachinePath.back();
                            closureStart.aDegrees += angleOffset;
                            const double used = std::min(remaining, closureLength);
                            trajectory.entities.back().overcutMoves.push_back
                                (move(MachineMoveKind::Overcut,
                                    interpolate(closureStart, alignedLoopStart, used / closureLength),
                                    inputEntity));
                            entityFinalSource =
                            {
                                groupSourcePath.back().x
                                    + (groupSourcePath.front().x
                                        - groupSourcePath.back().x)
                                        * used / closureLength,
                                groupSourcePath.back().y
                                    + (groupSourcePath.front().y
                                        - groupSourcePath.back().y)
                                        * used / closureLength,
                                groupSourcePath.back().z
                                    + (groupSourcePath.front().z
                                        - groupSourcePath.back().z)
                                        * used / closureLength
                            };
                            remaining -= used;
                        }
                    }
                    if (policy.overcutDistance > totalLength + policy.numericalEpsilon)
                        result.addDiagnostic(diagnostic(DiagnosticCode::MachineTrajectoryOvercutFailed,
                            DiagnosticSeverity::Warning, QStringLiteral("过切距离超过闭环总长，已限制为一圈。"),
                            taskContext.operationContext, &inputEntity));
                }
            }

            previousPose = finalPose(trajectory.entities.back(), poses.back());
            previousSourcePose = entityFinalSource;
            hasPrevious = true;
        }

        trajectory.surfaceSummaries = std::move(surfaceSummaries);

        for (const auto& entity : trajectory.entities)
        {
            const auto validMoves = [](const std::vector<MachineMove>& moves)
            {
                return std::all_of(moves.cbegin(), moves.cend(), [](const MachineMove& value)
                {
                    return finitePose(value.target);
                });
            };
            if (!validMoves(entity.approachMoves) || !validMoves(entity.cuttingMoves)
                || !validMoves(entity.overcutMoves))
            {
                result.status = OperationStatus::Failed;
                result.value.reset();
                result.addDiagnostic(diagnostic(DiagnosticCode::MachineTrajectoryInvariantViolation,
                    DiagnosticSeverity::Error, QStringLiteral("四轴轨迹包含非有限机床坐标。"),
                    taskContext.operationContext));
                return result;
            }
        }

        result.status = result.diagnostics.isEmpty() ? OperationStatus::Success : OperationStatus::PartialSuccess;
        result.value = std::move(trajectory);
        return result;
    }
}
