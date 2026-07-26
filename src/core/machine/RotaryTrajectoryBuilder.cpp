#include "core/machine/RotaryTrajectoryBuilder.h"

#include "core/machine/ProcessUnitExecutionResolver.h"
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
            const RotaryTransferRequest& request,
            const planning::PlannedTransferSignature& planned,
            double tolerance
        )
        {
            if (!plannedKindMatches(preview.kind, planned.kind)
                || !poseMatches(request.previousCutEnd,
                    planned.previousCutEnd, tolerance)
                || sourceDistance(request.previousSourceEnd,
                    planned.previousSourceEnd) > tolerance
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
            summary.actualPreviousCutEnd = request.previousCutEnd;
            summary.actualPreviousSourceEnd = request.previousSourceEnd;
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
                summary.plannedPreviousCutEnd =
                {
                    planned.previousCutEnd.x,
                    planned.previousCutEnd.y,
                    planned.previousCutEnd.z,
                    planned.previousCutEnd.aDegrees
                };
                summary.plannedPreviousSourceEnd =
                    planned.previousSourceEnd;
                summary.poseDelta = std::max
                (
                    machinePositionDistance
                        (summary.plannedPreviousCutEnd,
                            summary.actualPreviousCutEnd),
                    std::abs(summary.plannedPreviousCutEnd.aDegrees
                        - summary.actualPreviousCutEnd.aDegrees)
                );
                summary.sourceDelta = sourceDistance
                    (summary.plannedPreviousSourceEnd,
                        summary.actualPreviousSourceEnd);
                summary.previewMatched = previewMatchesPlan
                    (preview, request, planned,
                        request.numericalEpsilon);
            }
            return summary;
        }

        bool finitePose(const MachinePose4D& pose)
        {
            return std::isfinite(pose.x) && std::isfinite(pose.y)
                && std::isfinite(pose.z) && std::isfinite(pose.aDegrees);
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

        trajectory.entities.reserve(input.entities.size());
        bool hasPrevious = false;
        MachinePose4D previousPose;
        geometry::Vector3d previousSourcePose;
        const TrajectoryEntityInput* previousEntity = nullptr;
        std::size_t unitBegin = 0U;
        while (unitBegin < input.entities.size())
        {
            const int processUnitIndex =
                input.entities[unitBegin].processUnitIndex;
            std::size_t unitEnd = unitBegin + 1U;
            while (unitEnd < input.entities.size()
                && input.entities[unitEnd].processUnitIndex
                    == processUnitIndex)
            {
                ++unitEnd;
            }

            std::vector<ProcessUnitExecutionPath> executionPaths;
            executionPaths.reserve(unitEnd - unitBegin);
            for (std::size_t index = unitBegin; index < unitEnd; ++index)
            {
                const TrajectoryEntityInput& entity = input.entities[index];
                executionPaths.push_back
                ({
                    entity.entityId,
                    entity.sourceIndex,
                    entity.sourceKind,
                    entity.sourceProcessOrder,
                    entity.fragmentOrder,
                    entity.processGroupId,
                    entity.processUnitIndex,
                    entity.path
                });
            }

            auto resolved = ProcessUnitExecutionResolver::resolve
            (
                executionPaths,
                input.entities[unitBegin].closed,
                policy,
                input.tubeSection,
                hasPrevious
                    ? std::optional<MachinePose4D>(previousPose)
                    : std::nullopt,
                taskContext.operationContext
            );
            result.mergeDiagnostics(resolved);
            if (!resolved.succeeded() || !resolved.value.has_value())
            {
                result.status = resolved.status;
                return result;
            }

            ProcessUnitExecutionResult& execution = *resolved.value;
            trajectory.surfaceSummaries.insert
            (
                trajectory.surfaceSummaries.end(),
                execution.surfaceSummaries.begin(),
                execution.surfaceSummaries.end()
            );

            for (std::size_t localIndex = 0U;
                localIndex < execution.paths.size(); ++localIndex)
            {
                const std::size_t inputIndex = unitBegin + localIndex;
                const TrajectoryEntityInput& inputEntity =
                    input.entities[inputIndex];
                const std::vector<MachinePose4D>& poses =
                    execution.posesByPath[localIndex];

                EntityTrajectory entity;
                entity.entityId = inputEntity.entityId;
                entity.sourceKind = inputEntity.sourceKind;
                entity.sourceIndex = inputEntity.sourceIndex;
                entity.processOrder = inputEntity.processOrder;
                entity.sourceProcessOrder =
                    inputEntity.sourceProcessOrder;
                entity.fragmentOrder = inputEntity.fragmentOrder;
                entity.processGroupId = inputEntity.processGroupId;
                entity.closed = inputEntity.closed;
                entity.continuousFromPrevious = localIndex > 0U;
                entity.firstInGroup = inputEntity.firstInGroup;
                entity.lastInGroup = inputEntity.lastInGroup;
                for (const auto& vertex : inputEntity.path.vertices)
                    entity.sourcePath.push_back(vertex.position);

                if (localIndex == 0U)
                {
                    const MachinePose4D initial =
                        policy.useInitialMachinePoint
                        ? policy.initialMachinePoint
                        : MachinePose4D
                            {
                                execution.cutStart.x,
                                execution.cutStart.y,
                                0.0,
                                0.0
                            };
                    const MachinePose4D& transferStart =
                        hasPrevious ? previousPose : initial;
                    const RotaryTransferRequest request = transferRequest
                    (
                        transferStart,
                        execution.cutStart,
                        hasPrevious
                            ? previousSourcePose
                            : execution.sourceStart,
                        execution.sourceStart,
                        hasPrevious ? previousEntity : nullptr,
                        inputEntity,
                        policy,
                        input.tubeSection,
                        trajectory.rotaryContext.safeMachineZ
                    );
                    auto transfer = RotaryTransferPlanner::preview
                        (request, taskContext.operationContext);
                    result.mergeDiagnostics(transfer);
                    if (!transfer.succeeded()
                        || !transfer.value.has_value())
                    {
                        result.status = transfer.status;
                        return result;
                    }
                    const TransferMotionSummary summary =
                        appendTransferPreview
                        (
                            entity.approachMoves,
                            request,
                            *transfer.value,
                            inputEntity
                        );
                    if (summary.hasPlannedPreview
                        && !summary.previewMatched)
                    {
                        result.status = OperationStatus::Conflict;
                        Diagnostic mismatch = diagnostic
                        (
                            DiagnosticCode::
                                MachineTrajectoryTransferPreviewMismatch,
                            DiagnosticSeverity::Error,
                            hasPrevious
                                ? QStringLiteral("实际加工单元转移与加工计划预览不一致。")
                                : QStringLiteral("实际首次接近轨迹与加工计划预览不一致。"),
                            taskContext.operationContext,
                            &inputEntity
                        );
                        mismatch.context.insert
                            (QStringLiteral("fromProcessUnit"),
                                summary.fromProcessUnit);
                        mismatch.context.insert
                            (QStringLiteral("toProcessUnit"),
                                summary.toProcessUnit);
                        mismatch.context.insert
                            (QStringLiteral("plannedPreviousCutEnd"),
                                QStringLiteral("(%1,%2,%3,%4)")
                                    .arg(summary.plannedPreviousCutEnd.x,
                                        0, 'g', 15)
                                    .arg(summary.plannedPreviousCutEnd.y,
                                        0, 'g', 15)
                                    .arg(summary.plannedPreviousCutEnd.z,
                                        0, 'g', 15)
                                    .arg(summary.plannedPreviousCutEnd
                                        .aDegrees, 0, 'g', 15));
                        mismatch.context.insert
                            (QStringLiteral("actualPreviousCutEnd"),
                                QStringLiteral("(%1,%2,%3,%4)")
                                    .arg(summary.actualPreviousCutEnd.x,
                                        0, 'g', 15)
                                    .arg(summary.actualPreviousCutEnd.y,
                                        0, 'g', 15)
                                    .arg(summary.actualPreviousCutEnd.z,
                                        0, 'g', 15)
                                    .arg(summary.actualPreviousCutEnd
                                        .aDegrees, 0, 'g', 15));
                        mismatch.context.insert
                            (QStringLiteral("plannedPreviousSourceEnd"),
                                QStringLiteral("(%1,%2,%3)")
                                    .arg(summary.plannedPreviousSourceEnd.x,
                                        0, 'g', 15)
                                    .arg(summary.plannedPreviousSourceEnd.y,
                                        0, 'g', 15)
                                    .arg(summary.plannedPreviousSourceEnd.z,
                                        0, 'g', 15));
                        mismatch.context.insert
                            (QStringLiteral("actualPreviousSourceEnd"),
                                QStringLiteral("(%1,%2,%3)")
                                    .arg(summary.actualPreviousSourceEnd.x,
                                        0, 'g', 15)
                                    .arg(summary.actualPreviousSourceEnd.y,
                                        0, 'g', 15)
                                    .arg(summary.actualPreviousSourceEnd.z,
                                        0, 'g', 15));
                        mismatch.context.insert
                            (QStringLiteral("poseDelta"),
                                summary.poseDelta);
                        mismatch.context.insert
                            (QStringLiteral("sourceDelta"),
                                summary.sourceDelta);
                        mismatch.context.insert
                            (QStringLiteral("plannedFinalApproachOrigin"),
                                QStringLiteral("(%1,%2,%3,%4)")
                                    .arg(summary
                                        .plannedFinalApproachOrigin.x,
                                        0, 'g', 15)
                                    .arg(summary
                                        .plannedFinalApproachOrigin.y,
                                        0, 'g', 15)
                                    .arg(summary
                                        .plannedFinalApproachOrigin.z,
                                        0, 'g', 15)
                                    .arg(summary
                                        .plannedFinalApproachOrigin
                                        .aDegrees, 0, 'g', 15));
                        mismatch.context.insert
                            (QStringLiteral("actualFinalApproachOrigin"),
                                QStringLiteral("(%1,%2,%3,%4)")
                                    .arg(summary
                                        .actualFinalApproachOrigin.x,
                                        0, 'g', 15)
                                    .arg(summary
                                        .actualFinalApproachOrigin.y,
                                        0, 'g', 15)
                                    .arg(summary
                                        .actualFinalApproachOrigin.z,
                                        0, 'g', 15)
                                    .arg(summary
                                        .actualFinalApproachOrigin
                                        .aDegrees, 0, 'g', 15));
                        result.addDiagnostic(mismatch);
                        return result;
                    }
                    if (!transferIsSafe
                        (
                            transferStart,
                            execution.cutStart,
                            entity.approachMoves,
                            summary,
                            policy.numericalEpsilon
                        ))
                    {
                        result.status = OperationStatus::Failed;
                        result.addDiagnostic(diagnostic
                        (
                            DiagnosticCode::
                                MachineTrajectoryTransferSafetyViolation,
                            DiagnosticSeverity::Error,
                            hasPrevious
                                ? QStringLiteral("加工单元间转移未满足旋转安全约束。")
                                : QStringLiteral("首次接近轨迹未满足旋转安全约束。"),
                            taskContext.operationContext,
                            &inputEntity
                        ));
                        return result;
                    }
                    trajectory.transferSummaries.push_back(summary);
                }
                else
                {
                    const double connectionDistance = sourceDistance
                    (
                        input.entities[inputIndex - 1U]
                            .path.vertices.back().position,
                        inputEntity.path.vertices.front().position
                    );
                    if (connectionDistance > policy.numericalEpsilon)
                    {
                        entity.cuttingMoves.push_back
                            (move(MachineMoveKind::CuttingConnection,
                                poses.front(), inputEntity));
                    }
                }

                for (std::size_t pointIndex = 1U;
                    pointIndex < poses.size(); ++pointIndex)
                {
                    entity.cuttingMoves.push_back
                        (move(MachineMoveKind::Cutting,
                            poses[pointIndex], inputEntity));
                }

                if (localIndex + 1U == execution.paths.size())
                {
                    if (execution.closurePose.has_value())
                    {
                        entity.cuttingMoves.push_back
                            (move(MachineMoveKind::Cutting,
                                *execution.closurePose, inputEntity));
                    }
                    for (const ProcessUnitOvercutTarget& target :
                        execution.overcutTargets)
                    {
                        entity.overcutMoves.push_back
                            (move(MachineMoveKind::Overcut,
                                target.pose, inputEntity));
                    }
                    if (execution.overcutLimited)
                    {
                        result.addDiagnostic(diagnostic
                        (
                            DiagnosticCode::MachineTrajectoryOvercutFailed,
                            DiagnosticSeverity::Warning,
                            QStringLiteral("过切距离超过闭环总长，已限制为一圈。"),
                            taskContext.operationContext,
                            &inputEntity
                        ));
                    }
                }

                trajectory.entities.push_back(std::move(entity));
            }

            previousPose = execution.finalCutPose;
            previousSourcePose = execution.finalSourcePosition;
            hasPrevious = true;
            previousEntity = &input.entities[unitEnd - 1U];
            unitBegin = unitEnd;
        }

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
