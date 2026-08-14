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

        TransferMotionKind machineTransferKind
        (
            planning::PlannedTransferMotionKind kind
        )
        {
            switch (kind)
            {
            case planning::PlannedTransferMotionKind::InitialApproach:
                return TransferMotionKind::InitialApproach;
            case planning::PlannedTransferMotionKind::SameZoneSurfaceTransfer:
                return TransferMotionKind::SameZoneSurfaceTransfer;
            case planning::PlannedTransferMotionKind::SameZoneClearanceTransfer:
                return TransferMotionKind::SameZoneClearanceTransfer;
            case planning::PlannedTransferMotionKind::CrossZoneRotaryTransfer:
                return TransferMotionKind::CrossZoneRotaryTransfer;
            }
            return TransferMotionKind::InitialApproach;
        }

        TransferMotionPhase machineTransferPhase
        (
            planning::PlannedTransferMotionPhase phase
        )
        {
            switch (phase)
            {
            case planning::PlannedTransferMotionPhase::SurfaceTransfer:
                return TransferMotionPhase::SurfaceTransfer;
            case planning::PlannedTransferMotionPhase::CoordinatedDeparture:
                return TransferMotionPhase::CoordinatedDeparture;
            case planning::PlannedTransferMotionPhase::SafeRotaryTransfer:
                return TransferMotionPhase::SafeRotaryTransfer;
            case planning::PlannedTransferMotionPhase::CoordinatedApproach:
                return TransferMotionPhase::CoordinatedApproach;
            }
            return TransferMotionPhase::None;
        }

        RotaryTransferPreview previewFromSignature
        (
            const planning::PlannedTransferSignature& planned
        )
        {
            RotaryTransferPreview preview;
            preview.kind = machineTransferKind(planned.kind);
            preview.finalApproachOrigin =
            {
                planned.finalApproachOrigin.x,
                planned.finalApproachOrigin.y,
                planned.finalApproachOrigin.z,
                planned.finalApproachOrigin.aDegrees
            };
            preview.cutStart =
            {
                planned.cutStart.x,
                planned.cutStart.y,
                planned.cutStart.z,
                planned.cutStart.aDegrees
            };
            preview.targets.reserve(planned.targets.size());
            for (const planning::PlannedMachinePose4D& target : planned.targets)
            {
                preview.targets.push_back
                ({ target.x, target.y, target.z, target.aDegrees });
            }
            preview.phases.reserve(planned.phases.size());
            for (const planning::PlannedTransferMotionPhase phase : planned.phases)
            {
                preview.phases.push_back(machineTransferPhase(phase));
            }
            return preview;
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
                    summary.approachTarget = preview.targets[index];
                    break;
                case TransferMotionPhase::SurfaceTransfer:
                    if (preview.kind
                        == TransferMotionKind::SameZoneClearanceTransfer)
                    {
                        summary.rotaryTransferTarget =
                            preview.targets[index];
                    }
                    else
                    {
                        summary.approachTarget = preview.targets[index];
                    }
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

    namespace
    {
        // 轨迹构建上下文：各工艺阶段共享的窄接口状态。
        struct RotaryTrajectoryBuildContext
        {
            const RotaryTrajectoryInput& input;
            const RotaryMachinePolicy& policy;
            const TaskContext& taskContext;
            OperationResult<MachineTrajectory>& result;
            MachineTrajectory trajectory;

            struct ResolvedUnit
            {
                std::size_t begin = 0U;
                std::size_t end = 0U;
                ProcessUnitExecutionResult execution;
            };
            std::vector<ResolvedUnit> resolvedUnits;
        };

        // 工艺阶段：对上下文做一次变换。返回 false 表示失败并已写入 ctx.result，
        // 构建立即终止。新增加工工艺时实现新阶段并注册到 defaultRotaryStages()。
        class RotaryTrajectoryStage
        {
        public:
            virtual ~RotaryTrajectoryStage() = default;
            virtual QString stageName() const = 0;
            virtual bool apply(RotaryTrajectoryBuildContext& ctx) = 0;
        };

        class ValidateTrajectoryInputStage final : public RotaryTrajectoryStage
        {
        public:
            QString stageName() const override
            {
                return QStringLiteral("ValidateTrajectoryInput");
            }

            bool apply(RotaryTrajectoryBuildContext& ctx) override
            {
                OperationResult<MachineTrajectory>& result = ctx.result;
                const RotaryTrajectoryInput& input = ctx.input;
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
                            ctx.taskContext.operationContext, &entity));
                        return false;
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
                            ctx.taskContext.operationContext, &entity
                        ));
                        return false;
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
                            ctx.taskContext.operationContext));
                        return false;
                    }
                }
                return true;
            }
        };

        class BuildRotaryContextStage final : public RotaryTrajectoryStage
        {
        public:
            QString stageName() const override
            {
                return QStringLiteral("BuildRotaryContext");
            }

            bool apply(RotaryTrajectoryBuildContext& ctx) override
            {
                MachineTrajectory& trajectory = ctx.trajectory;
                const RotaryMachinePolicy& policy = ctx.policy;
                const RotaryTrajectoryInput& input = ctx.input;
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
                for (const TrajectoryEntityInput& inputEntity : input.entities)
                {
                    EntityTrajectory entity;
                    entity.entityId = inputEntity.entityId;
                    entity.sourceKind = inputEntity.sourceKind;
                    entity.sourceIndex = inputEntity.sourceIndex;
                    entity.processOrder = inputEntity.processOrder;
                    entity.sourceProcessOrder = inputEntity.sourceProcessOrder;
                    entity.fragmentOrder = inputEntity.fragmentOrder;
                    entity.processGroupId = inputEntity.processGroupId;
                    entity.processUnitIndex = inputEntity.processUnitIndex;
                    entity.closed = inputEntity.closed;
                    entity.firstInGroup = inputEntity.firstInGroup;
                    entity.lastInGroup = inputEntity.lastInGroup;
                    for (const auto& vertex : inputEntity.path.vertices)
                        entity.sourcePath.push_back(vertex.position);
                    trajectory.entities.push_back(std::move(entity));
                }
                return true;
            }
        };

        class ResolveUnitStage final : public RotaryTrajectoryStage
        {
        public:
            QString stageName() const override
            {
                return QStringLiteral("ResolveUnit");
            }

            bool apply(RotaryTrajectoryBuildContext& ctx) override
            {
                MachinePose4D previousPose;
                bool hasPrevious = false;
                std::size_t unitBegin = 0U;
                while (unitBegin < ctx.input.entities.size())
                {
                    const int processUnitIndex =
                        ctx.input.entities[unitBegin].processUnitIndex;
                    std::size_t unitEnd = unitBegin + 1U;
                    while (unitEnd < ctx.input.entities.size()
                        && ctx.input.entities[unitEnd].processUnitIndex
                            == processUnitIndex)
                    {
                        ++unitEnd;
                    }

                    std::vector<ProcessUnitExecutionPath> executionPaths;
                    executionPaths.reserve(unitEnd - unitBegin);
                    for (std::size_t index = unitBegin; index < unitEnd; ++index)
                    {
                        const TrajectoryEntityInput& entity = ctx.input.entities[index];
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
                        ctx.input.entities[unitBegin].closed,
                        ctx.policy,
                        ctx.input.tubeSection,
                        hasPrevious
                            ? std::optional<MachinePose4D>(previousPose)
                            : std::nullopt,
                        ctx.taskContext.operationContext
                    );
                    ctx.result.mergeDiagnostics(resolved);
                    if (!resolved.succeeded() || !resolved.value.has_value())
                    {
                        ctx.result.status = resolved.status;
                        return false;
                    }

                    RotaryTrajectoryBuildContext::ResolvedUnit unit;
                    unit.begin = unitBegin;
                    unit.end = unitEnd;
                    unit.execution = std::move(*resolved.value);
                    ctx.trajectory.surfaceSummaries.insert
                    (
                        ctx.trajectory.surfaceSummaries.end(),
                        unit.execution.surfaceSummaries.begin(),
                        unit.execution.surfaceSummaries.end()
                    );
                    previousPose = unit.execution.finalCutPose;
                    hasPrevious = true;
                    ctx.resolvedUnits.push_back(std::move(unit));
                    unitBegin = unitEnd;
                }
                return true;
            }
        };

        class TransferStage final : public RotaryTrajectoryStage
        {
        public:
            QString stageName() const override
            {
                return QStringLiteral("Transfer");
            }

            bool apply(RotaryTrajectoryBuildContext& ctx) override
            {
                for (std::size_t unitOrder = 0U;
                    unitOrder < ctx.resolvedUnits.size(); ++unitOrder)
                {
                    const auto& unit = ctx.resolvedUnits[unitOrder];
                    const std::size_t inputIndex = unit.begin;
                    const TrajectoryEntityInput& inputEntity =
                        ctx.input.entities[inputIndex];
                    const ProcessUnitExecutionResult& execution =
                        unit.execution;
                    const bool hasPrevious = unitOrder > 0U;
                    const MachinePose4D previousPose = hasPrevious
                        ? ctx.resolvedUnits[unitOrder - 1U].execution.finalCutPose
                        : MachinePose4D{};
                    const geometry::Vector3d previousSourcePose =
                        hasPrevious
                        ? ctx.resolvedUnits[unitOrder - 1U]
                            .execution.finalSourcePosition
                        : geometry::Vector3d{};
                    const TrajectoryEntityInput* previousEntity =
                        hasPrevious
                        ? &ctx.input.entities
                            [ctx.resolvedUnits[unitOrder - 1U].end - 1U]
                        : nullptr;

                    const MachinePose4D initial =
                        ctx.policy.useInitialMachinePoint
                        ? ctx.policy.initialMachinePoint
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
                        previousEntity,
                        inputEntity,
                        ctx.policy,
                        ctx.input.tubeSection,
                        ctx.trajectory.rotaryContext.safeMachineZ
                    );
                    TransferMotionSummary summary;
                    if (inputEntity.plannedIncomingTransfer.has_value())
                    {
                        // 单一实现：直接消费计划中保存的转移签名，不再重新计算。
                        const auto& planned =
                            *inputEntity.plannedIncomingTransfer;
                        if (!poseMatches(transferStart,
                                planned.previousCutEnd,
                                ctx.policy.numericalEpsilon)
                            || sourceDistance(previousSourcePose,
                                planned.previousSourceEnd)
                                > ctx.policy.numericalEpsilon
                            || !poseMatches(execution.cutStart,
                                planned.cutStart,
                                ctx.policy.numericalEpsilon))
                        {
                            ctx.result.status = OperationStatus::Conflict;
                            ctx.result.addDiagnostic(diagnostic
                            (
                                DiagnosticCode::
                                    MachineTrajectoryTransferPreviewMismatch,
                                DiagnosticSeverity::Error,
                                hasPrevious
                                    ? QStringLiteral("实际前序终点与加工计划记录的转移起点不一致。")
                                    : QStringLiteral("实际首次接近起点与加工计划记录不一致。"),
                                ctx.taskContext.operationContext,
                                &inputEntity
                            ));
                            return false;
                        }
                        summary = appendTransferPreview
                        (
                            ctx.trajectory.entities[inputIndex].approachMoves,
                            request,
                            previewFromSignature(planned),
                            inputEntity
                        );
                    }
                    else
                    {
                        auto transfer = RotaryTransferPlanner::preview
                            (request, ctx.taskContext.operationContext);
                        ctx.result.mergeDiagnostics(transfer);
                        if (!transfer.succeeded()
                            || !transfer.value.has_value())
                        {
                            ctx.result.status = transfer.status;
                            return false;
                        }
                        summary = appendTransferPreview
                        (
                            ctx.trajectory.entities[inputIndex].approachMoves,
                            request,
                            *transfer.value,
                            inputEntity
                        );
                    }
                    if (summary.hasPlannedPreview
                        && !summary.previewMatched)
                    {
                        ctx.result.status = OperationStatus::Conflict;
                        Diagnostic mismatch = diagnostic
                        (
                            DiagnosticCode::
                                MachineTrajectoryTransferPreviewMismatch,
                            DiagnosticSeverity::Error,
                            hasPrevious
                                ? QStringLiteral("实际加工单元转移与加工计划预览不一致。")
                                : QStringLiteral("实际首次接近轨迹与加工计划预览不一致。"),
                            ctx.taskContext.operationContext,
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
                        ctx.result.addDiagnostic(mismatch);
                        return false;
                    }
                    if (!transferIsSafe
                        (
                            transferStart,
                            execution.cutStart,
                            ctx.trajectory.entities[inputIndex].approachMoves,
                            summary,
                            ctx.policy.numericalEpsilon
                        ))
                    {
                        ctx.result.status = OperationStatus::Failed;
                        Diagnostic safetyFailure = diagnostic
                        (
                            DiagnosticCode::
                                MachineTrajectoryTransferSafetyViolation,
                            DiagnosticSeverity::Error,
                            hasPrevious
                                ? QStringLiteral("加工单元间转移未满足旋转安全约束。")
                                : QStringLiteral("首次接近轨迹未满足旋转安全约束。"),
                            ctx.taskContext.operationContext,
                            &inputEntity
                        );
                        safetyFailure.context.insert
                            (QStringLiteral("fromProcessUnit"),
                                summary.fromProcessUnit);
                        safetyFailure.context.insert
                            (QStringLiteral("toProcessUnit"),
                                summary.toProcessUnit);
                        safetyFailure.context.insert
                            (QStringLiteral("transferKind"),
                                static_cast<int>(summary.kind));
                        safetyFailure.context.insert
                            (QStringLiteral("startZ"), transferStart.z);
                        safetyFailure.context.insert
                            (QStringLiteral("targetCutStartZ"),
                                execution.cutStart.z);
                        safetyFailure.context.insert
                            (QStringLiteral("departureTargetZ"),
                                summary.departureTarget.z);
                        safetyFailure.context.insert
                            (QStringLiteral("arrivalClearanceTargetZ"),
                                summary.rotaryTransferTarget.z);
                        safetyFailure.context.insert
                            (QStringLiteral("rotationSafeMachineZ"),
                                summary.rotationSafeMachineZ);
                        safetyFailure.context.insert
                            (QStringLiteral("sameZoneTransferClearance"),
                                summary.sameZoneTransferClearance);
                        safetyFailure.context.insert
                            (QStringLiteral("segmentCount"),
                                summary.segmentCount);
                        ctx.result.addDiagnostic(safetyFailure);
                        return false;
                    }
                    ctx.trajectory.transferSummaries.push_back(summary);
                }
                return true;
            }
        };

        class CuttingStage final : public RotaryTrajectoryStage
        {
        public:
            QString stageName() const override
            {
                return QStringLiteral("Cutting");
            }

            bool apply(RotaryTrajectoryBuildContext& ctx) override
            {
                for (const auto& unit : ctx.resolvedUnits)
                {
                    const ProcessUnitExecutionResult& execution =
                        unit.execution;
                    for (std::size_t localIndex = 0U;
                        localIndex < execution.paths.size(); ++localIndex)
                    {
                        const std::size_t inputIndex =
                            unit.begin + localIndex;
                        const TrajectoryEntityInput& inputEntity =
                            ctx.input.entities[inputIndex];
                        EntityTrajectory& entity =
                            ctx.trajectory.entities[inputIndex];
                        entity.continuousFromPrevious = localIndex > 0U;
                        const std::vector<MachinePose4D>& poses =
                            execution.posesByPath[localIndex];

                        if (localIndex > 0U)
                        {
                            const double connectionDistance = sourceDistance
                            (
                                ctx.input.entities[inputIndex - 1U]
                                    .path.vertices.back().position,
                                inputEntity.path.vertices.front().position
                            );
                            if (connectionDistance > ctx.policy.numericalEpsilon)
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
                                ctx.result.addDiagnostic(diagnostic
                                (
                                    DiagnosticCode::MachineTrajectoryOvercutFailed,
                                    DiagnosticSeverity::Warning,
                                    QStringLiteral("过切距离超过闭环总长，已限制为一圈。"),
                                    ctx.taskContext.operationContext,
                                    &inputEntity
                                ));
                            }
                        }
                    }
                }
                return true;
            }
        };

        class FinalizeStage final : public RotaryTrajectoryStage
        {
        public:
            QString stageName() const override
            {
                return QStringLiteral("Finalize");
            }

            bool apply(RotaryTrajectoryBuildContext& ctx) override
            {
                for (const auto& entity : ctx.trajectory.entities)
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
                        ctx.result.status = OperationStatus::Failed;
                        ctx.result.value.reset();
                        ctx.result.addDiagnostic(diagnostic(DiagnosticCode::MachineTrajectoryInvariantViolation,
                            DiagnosticSeverity::Error, QStringLiteral("四轴轨迹包含非有限机床坐标。"),
                            ctx.taskContext.operationContext));
                        return false;
                    }
                }
                ctx.result.status = ctx.result.diagnostics.isEmpty() ? OperationStatus::Success : OperationStatus::PartialSuccess;
                ctx.result.value = std::move(ctx.trajectory);
                return true;
            }
        };

        std::vector<std::unique_ptr<RotaryTrajectoryStage>> defaultRotaryStages()
        {
            std::vector<std::unique_ptr<RotaryTrajectoryStage>> stages;
            stages.push_back(std::make_unique<ValidateTrajectoryInputStage>());
            stages.push_back(std::make_unique<BuildRotaryContextStage>());
            stages.push_back(std::make_unique<ResolveUnitStage>());
            stages.push_back(std::make_unique<TransferStage>());
            stages.push_back(std::make_unique<CuttingStage>());
            stages.push_back(std::make_unique<FinalizeStage>());
            return stages;
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

        RotaryTrajectoryBuildContext ctx{ input, policy, taskContext, result, {} };
        for (const auto& stage : defaultRotaryStages())
        {
            if (!stage->apply(ctx))
            {
                return result;
            }
        }
        return result;
    }
}
