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

        double poseDistance(const MachinePose4D& left, const MachinePose4D& right)
        {
            return std::sqrt
            (
                (left.x - right.x) * (left.x - right.x)
                + (left.y - right.y) * (left.y - right.y)
                + (left.z - right.z) * (left.z - right.z)
                + (left.aDegrees - right.aDegrees) * (left.aDegrees - right.aDegrees)
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

        MachineMove move(MachineMoveKind kind, const MachinePose4D& pose, const TrajectoryEntityInput& entity)
        {
            return { kind, pose, entity.entityId, entity.processGroupId };
        }

        MachinePose4D safePose(const MachinePose4D& pose, double safeZ)
        {
            MachinePose4D result = pose;
            result.z = std::max(result.z, safeZ);
            return result;
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
            || !std::isfinite(policy.safeRadialClearance) || policy.safeRadialClearance < 0.0
            || !std::isfinite(policy.continuousConnectionTolerance)
            || policy.continuousConnectionTolerance <= 0.0
            || !std::isfinite(policy.numericalEpsilon) || policy.numericalEpsilon <= 0.0)
        {
            result.status = OperationStatus::InvalidInput;
            result.addDiagnostic(diagnostic(DiagnosticCode::MachineTrajectoryInputInvalid,
                DiagnosticSeverity::Error, QStringLiteral("四轴轨迹输入为空或版本无效。"), taskContext.operationContext));
            return result;
        }

        std::set<geometry::EntityId> uniqueIds;
        for (std::size_t index = 0; index < input.entities.size(); ++index)
        {
            const auto& entity = input.entities[index];
            if (entity.processOrder != static_cast<int>(index) || entity.entityId == 0
                || !uniqueIds.insert(entity.entityId).second || entity.path.vertices.empty())
            {
                result.status = OperationStatus::InvalidInput;
                result.addDiagnostic(diagnostic(DiagnosticCode::MachineTrajectoryInputInvalid,
                    DiagnosticSeverity::Error, QStringLiteral("四轴轨迹的加工顺序、图元标识或路径无效。"),
                    taskContext.operationContext, &entity));
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
        for (const auto& entity : input.entities)
            for (const auto& vertex : entity.path.vertices)
                trajectory.rotaryContext.maximumCollisionRadius = std::max
                (
                    trajectory.rotaryContext.maximumCollisionRadius,
                    std::hypot(vertex.position.y - policy.tubeCenterY,
                               vertex.position.z - policy.tubeCenterZ)
                );
        trajectory.rotaryContext.safeMachineZ = policy.tubeCenterZ
            + trajectory.rotaryContext.maximumCollisionRadius + policy.safeRadialClearance;

        std::vector<std::vector<MachinePose4D>> posesByEntity;
        posesByEntity.reserve(input.entities.size());
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
            if (policy.keepContinuousAngle && !posesByEntity.empty())
            {
                const double previous = posesByEntity.back().back().aDegrees;
                double offset = 0.0;
                while (transformed.value->front().aDegrees + offset - previous > 180.0) offset -= 360.0;
                while (transformed.value->front().aDegrees + offset - previous < -180.0) offset += 360.0;
                for (auto& pose : *transformed.value) pose.aDegrees += offset;
            }
            posesByEntity.push_back(std::move(*transformed.value));
        }

        trajectory.entities.reserve(input.entities.size());
        bool hasPrevious = false;
        MachinePose4D previousPose;
        for (std::size_t index = 0; index < input.entities.size(); ++index)
        {
            const auto& inputEntity = input.entities[index];
            const auto& poses = posesByEntity[index];
            const bool sameGroup = index > 0 && inputEntity.processGroupId >= 0
                && input.entities[index - 1].processGroupId == inputEntity.processGroupId;
            const double connectionDistance = index > 0
                ? sourceDistance(input.entities[index - 1].path.vertices.back().position,
                                 inputEntity.path.vertices.front().position)
                : 0.0;
            if (sameGroup && connectionDistance > policy.continuousConnectionTolerance)
            {
                result.status = OperationStatus::Failed;
                Diagnostic value = diagnostic(DiagnosticCode::MachineTrajectoryContinuityFailure,
                    DiagnosticSeverity::Error, QStringLiteral("连续加工组内的相邻路径没有连接。"),
                    taskContext.operationContext, &inputEntity);
                value.context.insert(QStringLiteral("connectionDistance"), connectionDistance);
                result.addDiagnostic(value);
                return result;
            }

            EntityTrajectory entity;
            entity.entityId = inputEntity.entityId;
            entity.sourceKind = inputEntity.sourceKind;
            entity.sourceIndex = inputEntity.sourceIndex;
            entity.processOrder = inputEntity.processOrder;
            entity.processGroupId = inputEntity.processGroupId;
            entity.closed = inputEntity.closed;
            entity.continuousFromPrevious = sameGroup;
            entity.firstInGroup = inputEntity.firstInGroup;
            entity.lastInGroup = inputEntity.lastInGroup;
            for (const auto& vertex : inputEntity.path.vertices) entity.sourcePath.push_back(vertex.position);

            const MachinePose4D& first = poses.front();
            if (!hasPrevious)
            {
                if (policy.useInitialMachinePoint)
                {
                    MachinePose4D initial = policy.initialMachinePoint;
                    initial.aDegrees = first.aDegrees;
                    if (poseDistance(initial, first) > policy.numericalEpsilon)
                        entity.approachMoves.push_back(move(MachineMoveKind::Rapid, initial, inputEntity));
                }
                if (policy.useSafeZBeforeRapid && trajectory.rotaryContext.safeMachineZ > first.z + policy.numericalEpsilon)
                {
                    const MachinePose4D safe = safePose(first, trajectory.rotaryContext.safeMachineZ);
                    entity.approachMoves.push_back(move(MachineMoveKind::Rapid, safe, inputEntity));
                    if (poseDistance(safe, first) > policy.numericalEpsilon)
                        entity.approachMoves.push_back(move(MachineMoveKind::Rapid, first, inputEntity));
                }
                else entity.approachMoves.push_back(move(MachineMoveKind::Rapid, first, inputEntity));
            }
            else if (!sameGroup)
            {
                if (policy.useSafeZBeforeRapid
                    && trajectory.rotaryContext.safeMachineZ > std::min(previousPose.z, first.z) + policy.numericalEpsilon)
                {
                    const MachinePose4D departure = safePose(previousPose, trajectory.rotaryContext.safeMachineZ);
                    const MachinePose4D approach = safePose(first, trajectory.rotaryContext.safeMachineZ);
                    if (poseDistance(previousPose, departure) > policy.numericalEpsilon)
                        entity.approachMoves.push_back(move(MachineMoveKind::Rapid, departure, inputEntity));
                    if (poseDistance(departure, approach) > policy.numericalEpsilon)
                        entity.approachMoves.push_back(move(MachineMoveKind::Rapid, approach, inputEntity));
                    if (poseDistance(approach, first) > policy.numericalEpsilon)
                        entity.approachMoves.push_back(move(MachineMoveKind::Rapid, first, inputEntity));
                }
                else entity.approachMoves.push_back(move(MachineMoveKind::Rapid, first, inputEntity));
            }
            else if (connectionDistance > policy.numericalEpsilon)
                entity.cuttingMoves.push_back(move(MachineMoveKind::CuttingConnection, first, inputEntity));

            for (std::size_t pointIndex = 1; pointIndex < poses.size(); ++pointIndex)
                entity.cuttingMoves.push_back(move(MachineMoveKind::Cutting, poses[pointIndex], inputEntity));

            trajectory.entities.push_back(std::move(entity));
            previousPose = poses.back();
            hasPrevious = true;
        }

        for (const auto& [groupId, group] : groupDefinitions)
        {
            if (group == nullptr || !group->closed || policy.overcutDistance <= policy.numericalEpsilon) continue;
            std::vector<std::size_t> indices;
            for (std::size_t index = 0; index < input.entities.size(); ++index)
                if (input.entities[index].processGroupId == groupId) indices.push_back(index);
            if (indices.empty()) continue;

            const std::size_t lastIndex = indices.back();
            std::vector<geometry::Vector3d> groupSourcePath;
            std::vector<MachinePose4D> groupMachinePath;
            for (const std::size_t entityIndex : indices)
            {
                const auto& path = input.entities[entityIndex].path.vertices;
                const auto& poses = posesByEntity[entityIndex];
                for (std::size_t pointIndex = 0; pointIndex < path.size(); ++pointIndex)
                {
                    if (!groupSourcePath.empty() && pointIndex == 0
                        && sourceDistance(groupSourcePath.back(), path.front().position)
                            <= policy.numericalEpsilon) continue;
                    groupSourcePath.push_back(path[pointIndex].position);
                    groupMachinePath.push_back(poses[pointIndex]);
                }
            }
            if (groupSourcePath.size() < 2U || groupSourcePath.size() != groupMachinePath.size())
            {
                result.status = OperationStatus::Failed;
                result.addDiagnostic(diagnostic(DiagnosticCode::MachineTrajectoryOvercutFailed,
                    DiagnosticSeverity::Error, QStringLiteral("闭合加工组的过切源路径无效。"),
                    taskContext.operationContext, &input.entities[lastIndex]));
                return result;
            }

            const MachinePose4D loopStart = groupMachinePath.front();
            const MachinePose4D loopEnd = groupMachinePath.back();
            MachinePose4D alignedLoopStart = loopStart;
            if (policy.keepContinuousAngle)
            {
                while (alignedLoopStart.aDegrees - loopEnd.aDegrees > 180.0)
                    alignedLoopStart.aDegrees -= 360.0;
                while (alignedLoopStart.aDegrees - loopEnd.aDegrees < -180.0)
                    alignedLoopStart.aDegrees += 360.0;
            }
            if (poseDistance(loopEnd, alignedLoopStart) > policy.numericalEpsilon)
                trajectory.entities[lastIndex].cuttingMoves.push_back
                    (move(MachineMoveKind::Cutting, alignedLoopStart, input.entities[lastIndex]));

            double totalLength = 0.0;
            for (std::size_t pointIndex = 1; pointIndex < groupSourcePath.size(); ++pointIndex)
                totalLength += sourceDistance(groupSourcePath[pointIndex - 1], groupSourcePath[pointIndex]);
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
                trajectory.entities[lastIndex].overcutMoves.push_back
                    (move(MachineMoveKind::Overcut,
                        interpolate(start, end, used / length), input.entities[lastIndex]));
                remaining -= used;
            }
            if (remaining > policy.numericalEpsilon)
            {
                const double closureLength = sourceDistance(groupSourcePath.back(), groupSourcePath.front());
                if (closureLength > policy.numericalEpsilon)
                {
                    MachinePose4D closureStart = groupMachinePath.back();
                    closureStart.aDegrees += angleOffset;
                    const double used = std::min(remaining, closureLength);
                    trajectory.entities[lastIndex].overcutMoves.push_back
                        (move(MachineMoveKind::Overcut,
                            interpolate(closureStart, alignedLoopStart, used / closureLength),
                            input.entities[lastIndex]));
                    remaining -= used;
                }
            }
            if (policy.overcutDistance > totalLength + policy.numericalEpsilon)
                result.addDiagnostic(diagnostic(DiagnosticCode::MachineTrajectoryOvercutFailed,
                    DiagnosticSeverity::Warning, QStringLiteral("过切距离超过闭环总长，已限制为一圈。"),
                    taskContext.operationContext, &input.entities[lastIndex]));
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
