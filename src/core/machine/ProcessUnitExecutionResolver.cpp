#include "core/machine/ProcessUnitExecutionResolver.h"

#include "core/geometry/GeometryCompiler.h"

#include <QVariant>

#include <algorithm>
#include <cmath>
#include <map>
#include <set>

namespace cadcam::machine
{
    namespace
    {
        double distance
        (
            const geometry::Vector3d& left,
            const geometry::Vector3d& right
        )
        {
            return std::sqrt
            (
                (left.x - right.x) * (left.x - right.x)
                + (left.y - right.y) * (left.y - right.y)
                + (left.z - right.z) * (left.z - right.z)
            );
        }

        MachinePose4D interpolate
        (
            const MachinePose4D& start,
            const MachinePose4D& end,
            double factor
        )
        {
            return
            {
                start.x + (end.x - start.x) * factor,
                start.y + (end.y - start.y) * factor,
                start.z + (end.z - start.z) * factor,
                start.aDegrees + (end.aDegrees - start.aDegrees) * factor
            };
        }

        geometry::Vector3d interpolate
        (
            const geometry::Vector3d& start,
            const geometry::Vector3d& end,
            double factor
        )
        {
            return
            {
                start.x + (end.x - start.x) * factor,
                start.y + (end.y - start.y) * factor,
                start.z + (end.z - start.z) * factor
            };
        }

        geometry::PathVertex3D interpolateVertex
        (
            const geometry::PathVertex3D& start,
            const geometry::PathVertex3D& end,
            double parameter
        )
        {
            const double denominator =
                end.sourceParameter - start.sourceParameter;
            const double factor = std::abs(denominator) <= 1.0e-12
                ? 0.0
                : std::clamp
                    ((parameter - start.sourceParameter) / denominator,
                        0.0, 1.0);
            return
            {
                interpolate(start.position, end.position, factor),
                parameter
            };
        }

        struct PathParameterLocation
        {
            std::size_t segmentIndex = 0U;
            geometry::PathVertex3D vertex;
        };

        std::optional<PathParameterLocation> locatePathParameter
        (
            const std::vector<geometry::PathVertex3D>& vertices,
            double parameter
        )
        {
            if (vertices.size() < 2U || !std::isfinite(parameter))
                return std::nullopt;
            constexpr double parameterEpsilon = 1.0e-10;
            for (std::size_t index = 1U; index < vertices.size(); ++index)
            {
                const double minimum = std::min
                    (vertices[index - 1U].sourceParameter,
                        vertices[index].sourceParameter)
                    - parameterEpsilon;
                const double maximum = std::max
                    (vertices[index - 1U].sourceParameter,
                        vertices[index].sourceParameter)
                    + parameterEpsilon;
                if (parameter < minimum || parameter > maximum) continue;
                return PathParameterLocation
                {
                    index - 1U,
                    interpolateVertex
                        (vertices[index - 1U], vertices[index], parameter)
                };
            }
            return std::nullopt;
        }

        std::optional<geometry::Path3D> slicePath
        (
            const geometry::Path3D& source,
            const planning::ProcessPathFragment& fragment
        )
        {
            std::vector<geometry::PathVertex3D> vertices = source.vertices;
            if (fragment.reverse)
                std::reverse(vertices.begin(), vertices.end());
            const auto begin = locatePathParameter
                (vertices, fragment.sourceParameterBegin);
            const auto end = locatePathParameter
                (vertices, fragment.sourceParameterEnd);
            if (!begin.has_value() || !end.has_value()
                || begin->segmentIndex > end->segmentIndex)
            {
                return std::nullopt;
            }

            geometry::Path3D result = source;
            result.vertices.clear();
            result.closed = false;
            result.vertices.push_back(begin->vertex);
            for (std::size_t index = begin->segmentIndex + 1U;
                index <= end->segmentIndex && index < vertices.size();
                ++index)
            {
                if (distance(result.vertices.back().position,
                        vertices[index].position) > 1.0e-12)
                {
                    result.vertices.push_back(vertices[index]);
                }
            }
            if (distance(result.vertices.back().position,
                    end->vertex.position) > 1.0e-12)
            {
                result.vertices.push_back(end->vertex);
            }
            else
            {
                result.vertices.back() = end->vertex;
            }
            if (result.vertices.size() < 2U
                || distance(result.vertices.front().position,
                    result.vertices.back().position) <= 1.0e-12)
            {
                return std::nullopt;
            }
            return result;
        }

        Diagnostic resolverDiagnostic
        (
            DiagnosticCode code,
            const QString& message,
            const OperationContext& context,
            geometry::EntityId entityId = 0
        )
        {
            Diagnostic diagnostic;
            diagnostic.code = code;
            diagnostic.severity = DiagnosticSeverity::Error;
            diagnostic.component =
                QStringLiteral("ProcessUnitExecutionResolver");
            diagnostic.operation = context.operationName;
            diagnostic.stage =
                QStringLiteral("resolve-process-unit-execution");
            diagnostic.userMessage = message;
            diagnostic.correlationId = context.correlationId;
            if (entityId != 0U) diagnostic.entityId = entityId;
            return diagnostic;
        }

        bool sameUnitMembers
        (
            std::vector<geometry::EntityId> actual,
            std::vector<geometry::EntityId> expected
        )
        {
            std::sort(actual.begin(), actual.end());
            actual.erase(std::unique(actual.begin(), actual.end()),
                actual.end());
            std::sort(expected.begin(), expected.end());
            expected.erase(std::unique(expected.begin(), expected.end()),
                expected.end());
            return actual == expected;
        }

        void alignPoses
        (
            std::vector<MachinePose4D>& poses,
            const std::optional<MachinePose4D>& previous,
            bool keepContinuousAngle
        )
        {
            if (!keepContinuousAngle || !previous.has_value()
                || poses.empty())
            {
                return;
            }
            double offset = 0.0;
            while (poses.front().aDegrees + offset
                - previous->aDegrees > 180.0)
            {
                offset -= 360.0;
            }
            while (poses.front().aDegrees + offset
                - previous->aDegrees < -180.0)
            {
                offset += 360.0;
            }
            for (MachinePose4D& pose : poses) pose.aDegrees += offset;
        }

        QString surfaceName(RotarySurfaceRegion region)
        {
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
    }

    OperationResult<std::vector<ProcessUnitExecutionPath>>
        ProcessUnitExecutionResolver::compilePaths
        (
            const planning::ProcessUnit& unit,
            int processUnitIndex,
            const std::vector<planning::ProcessAssignment>& assignments,
            const std::vector<planning::ProcessPathFragment>& fragments,
            const std::vector<ProcessUnitExecutionSource>& sources,
            const OperationContext& context
        )
    {
        OperationResult<std::vector<ProcessUnitExecutionPath>> result;
        std::map<geometry::EntityId, const ProcessUnitExecutionSource*>
            sourceById;
        for (const ProcessUnitExecutionSource& source : sources)
        {
            if (source.entityId == 0U || source.sourceEntity == nullptr
                || !sourceById.emplace(source.entityId, &source).second)
            {
                result.status = OperationStatus::InvalidInput;
                result.addDiagnostic(resolverDiagnostic
                (
                    DiagnosticCode::MachineTrajectoryEntityMissing,
                    QStringLiteral("加工单元源几何无效或编号重复。"),
                    context,
                    source.entityId
                ));
                return result;
            }
        }

        std::vector<const planning::ProcessAssignment*> unitAssignments;
        std::map<geometry::EntityId,
            const planning::ProcessAssignment*> assignmentById;
        for (const planning::ProcessAssignment& assignment : assignments)
        {
            if (assignment.processUnitIndex != processUnitIndex) continue;
            unitAssignments.push_back(&assignment);
            assignmentById.emplace(assignment.entityId, &assignment);
        }
        std::sort(unitAssignments.begin(), unitAssignments.end(),
            [](const auto* left, const auto* right)
            {
                return left->processOrder < right->processOrder;
            });
        std::vector<geometry::EntityId> assignmentIds;
        for (const auto* assignment : unitAssignments)
            assignmentIds.push_back(assignment->entityId);
        if (unitAssignments.empty()
            || !sameUnitMembers(assignmentIds, unit.key.memberEntityIds))
        {
            result.status = OperationStatus::InvalidInput;
            result.addDiagnostic(resolverDiagnostic
            (
                DiagnosticCode::ProcessPlanningInvariantViolation,
                QStringLiteral("加工单元成员与加工分配不一致。"),
                context
            ));
            return result;
        }

        std::vector<const planning::ProcessPathFragment*> unitFragments;
        for (const planning::ProcessPathFragment& fragment : fragments)
        {
            if (fragment.processUnitIndex == processUnitIndex)
                unitFragments.push_back(&fragment);
        }
        std::sort(unitFragments.begin(), unitFragments.end(),
            [](const auto* left, const auto* right)
            {
                return left->fragmentOrder < right->fragmentOrder;
            });

        geometry::GeometryCompiler compiler;
        std::vector<ProcessUnitExecutionPath> paths;
        if (!unitFragments.empty())
        {
            std::map<geometry::EntityId, geometry::Path3D> canonicalPaths;
            for (std::size_t index = 0U; index < unitFragments.size(); ++index)
            {
                const planning::ProcessPathFragment& fragment =
                    *unitFragments[index];
                const auto assignment = assignmentById.find(fragment.entityId);
                const auto source = sourceById.find(fragment.entityId);
                if (fragment.fragmentOrder != static_cast<int>(index)
                    || assignment == assignmentById.end()
                    || source == sourceById.end())
                {
                    result.status = OperationStatus::InvalidInput;
                    result.addDiagnostic(resolverDiagnostic
                    (
                        DiagnosticCode::MachineTrajectoryInvalidPath,
                        QStringLiteral("加工单元片段顺序或源图元无效。"),
                        context,
                        fragment.entityId
                    ));
                    return result;
                }
                auto canonical = canonicalPaths.find(fragment.entityId);
                if (canonical == canonicalPaths.end())
                {
                    geometry::PathCompileOptions options;
                    auto compiled = compiler.compile
                    (
                        *source->second->sourceEntity,
                        source->second->samplingPolicy,
                        options,
                        context
                    );
                    result.mergeDiagnostics(compiled);
                    if (!compiled.succeeded()
                        || !compiled.value.has_value())
                    {
                        result.status = compiled.status;
                        return result;
                    }
                    canonical = canonicalPaths.emplace
                        (fragment.entityId,
                            std::move(*compiled.value)).first;
                }
                const auto sliced = slicePath(canonical->second, fragment);
                if (!sliced.has_value())
                {
                    result.status = OperationStatus::Failed;
                    result.addDiagnostic(resolverDiagnostic
                    (
                        DiagnosticCode::MachineTrajectoryInvalidPath,
                        QStringLiteral("计划片段无法映射到完整源路径。"),
                        context,
                        fragment.entityId
                    ));
                    return result;
                }
                paths.push_back
                ({
                    fragment.entityId,
                    source->second->sourceIndex,
                    source->second->sourceKind,
                    assignment->second->processOrder,
                    fragment.fragmentOrder,
                    assignment->second->continuousGroupId,
                    processUnitIndex,
                    std::move(*sliced)
                });
            }
        }
        else
        {
            for (const planning::ProcessAssignment* assignment :
                unitAssignments)
            {
                const auto source = sourceById.find(assignment->entityId);
                if (source == sourceById.end())
                {
                    result.status = OperationStatus::InvalidInput;
                    result.addDiagnostic(resolverDiagnostic
                    (
                        DiagnosticCode::MachineTrajectoryEntityMissing,
                        QStringLiteral("加工分配缺少源几何。"),
                        context,
                        assignment->entityId
                    ));
                    return result;
                }
                geometry::PathCompileOptions options;
                options.reverse = assignment->reverse;
                options.startParameter = assignment->startParameter;
                auto compiled = compiler.compile
                (
                    *source->second->sourceEntity,
                    source->second->samplingPolicy,
                    options,
                    context
                );
                result.mergeDiagnostics(compiled);
                if (!compiled.succeeded() || !compiled.value.has_value())
                {
                    result.status = compiled.status;
                    return result;
                }
                paths.push_back
                ({
                    assignment->entityId,
                    source->second->sourceIndex,
                    source->second->sourceKind,
                    assignment->processOrder,
                    -1,
                    assignment->continuousGroupId,
                    processUnitIndex,
                    std::move(*compiled.value)
                });
            }
        }

        result.status = OperationStatus::Success;
        result.value = std::move(paths);
        return result;
    }

    OperationResult<ProcessUnitExecutionResult>
        ProcessUnitExecutionResolver::resolve
        (
            const std::vector<ProcessUnitExecutionPath>& paths,
            bool closed,
            const RotaryMachinePolicy& policy,
            const std::optional<machining::TubeSectionModel>& section,
            const std::optional<MachinePose4D>& previousPose,
            const OperationContext& context
        )
    {
        OperationResult<ProcessUnitExecutionResult> result;
        if (paths.empty())
        {
            result.status = OperationStatus::InvalidInput;
            result.addDiagnostic(resolverDiagnostic
            (
                DiagnosticCode::MachineTrajectoryInvalidPath,
                QStringLiteral("加工单元没有实际执行路径。"),
                context
            ));
            return result;
        }

        ProcessUnitExecutionResult execution;
        execution.paths = paths;
        execution.posesByPath.reserve(paths.size());
        execution.surfaceSummaries.reserve(paths.size());
        std::optional<MachinePose4D> alignmentPose = previousPose;
        std::vector<geometry::Vector3d> sourcePath;
        std::vector<MachinePose4D> machinePath;
        for (std::size_t pathIndex = 0U;
            pathIndex < paths.size(); ++pathIndex)
        {
            const ProcessUnitExecutionPath& path = paths[pathIndex];
            auto transformed = RotaryKinematics::transform
                (path.path, policy, section, context);
            result.mergeDiagnostics(transformed);
            if (!transformed.succeeded() || !transformed.value.has_value()
                || transformed.value->poses.size()
                    != path.path.vertices.size())
            {
                result.status = transformed.succeeded()
                    ? OperationStatus::InternalError : transformed.status;
                return result;
            }
            std::vector<MachinePose4D> poses =
                std::move(transformed.value->poses);
            alignPoses(poses, alignmentPose, policy.keepContinuousAngle);
            if (pathIndex > 0U)
            {
                const ProcessUnitExecutionPath& previousPath =
                    paths[pathIndex - 1U];
                const double sourceConnectionDistance = distance
                (
                    previousPath.path.vertices.back().position,
                    path.path.vertices.front().position
                );
                const double machineConnectionDistance = distance
                (
                    {
                        execution.posesByPath.back().back().x,
                        execution.posesByPath.back().back().y,
                        execution.posesByPath.back().back().z
                    },
                    { poses.front().x, poses.front().y, poses.front().z }
                );
                const double angleDifference = std::abs
                    (poses.front().aDegrees
                        - execution.posesByPath.back().back().aDegrees);
                if (sourceConnectionDistance
                        > policy.continuousConnectionTolerance
                    || machineConnectionDistance
                        > policy.continuousConnectionTolerance
                    || angleDifference
                        > 180.0 + policy.numericalEpsilon)
                {
                    result.status = OperationStatus::Failed;
                    Diagnostic diagnostic = resolverDiagnostic
                    (
                        DiagnosticCode::MachineTrajectoryContinuityFailure,
                        QStringLiteral("连续加工单元的实际执行路径不连续。"),
                        context,
                        path.entityId
                    );
                    diagnostic.context.insert
                    (
                        QStringLiteral("previousEntityId"),
                        QVariant::fromValue<qulonglong>
                            (previousPath.entityId)
                    );
                    diagnostic.context.insert
                    (
                        QStringLiteral("nextEntityId"),
                        QVariant::fromValue<qulonglong>(path.entityId)
                    );
                    diagnostic.context.insert
                        (QStringLiteral("sourceConnectionDistance"),
                            sourceConnectionDistance);
                    diagnostic.context.insert
                        (QStringLiteral("machineConnectionDistance"),
                            machineConnectionDistance);
                    diagnostic.context.insert
                        (QStringLiteral("previousA"),
                            execution.posesByPath.back().back().aDegrees);
                    diagnostic.context.insert
                        (QStringLiteral("nextA"), poses.front().aDegrees);
                    diagnostic.context.insert
                        (QStringLiteral("processUnitIndex"),
                            path.processUnitIndex);
                    diagnostic.context.insert
                        (QStringLiteral("previousSurface"),
                            surfaceName(execution.surfaceSummaries.back()
                                .classification));
                    diagnostic.context.insert
                        (QStringLiteral("nextSurface"),
                            surfaceName(transformed.value->surface
                                .classification));
                    diagnostic.context.insert
                    (
                        QStringLiteral("failure"),
                        sourceConnectionDistance
                            > policy.continuousConnectionTolerance
                            ? QStringLiteral("SourceGap")
                            : machineConnectionDistance
                                > policy.continuousConnectionTolerance
                                ? QStringLiteral("MachineGap")
                                : QStringLiteral("AngleJump")
                    );
                    result.addDiagnostic(diagnostic);
                    return result;
                }
            }
            transformed.value->surface.processGroupId = path.processGroupId;
            transformed.value->surface.alignedAStart =
                poses.front().aDegrees;
            transformed.value->surface.alignedAEnd =
                poses.back().aDegrees;
            for (std::size_t index = 0U;
                index < path.path.vertices.size(); ++index)
            {
                if (!sourcePath.empty() && index == 0U
                    && distance(sourcePath.back(),
                        path.path.vertices.front().position)
                        <= policy.numericalEpsilon)
                {
                    continue;
                }
                sourcePath.push_back(path.path.vertices[index].position);
                machinePath.push_back(poses[index]);
            }
            alignmentPose = poses.back();
            execution.posesByPath.push_back(std::move(poses));
            execution.surfaceSummaries.push_back
                (std::move(transformed.value->surface));
        }

        if (sourcePath.size() < 2U
            || sourcePath.size() != machinePath.size())
        {
            result.status = OperationStatus::Failed;
            result.addDiagnostic(resolverDiagnostic
            (
                DiagnosticCode::MachineTrajectoryInvalidPath,
                QStringLiteral("加工单元实际执行路径无效。"),
                context,
                paths.front().entityId
            ));
            return result;
        }

        execution.cutStart = machinePath.front();
        execution.finalCutPose = machinePath.back();
        execution.sourceStart = sourcePath.front();
        execution.finalSourcePosition = sourcePath.back();
        if (closed)
        {
            const MachinePose4D loopStart = machinePath.front();
            const MachinePose4D loopEnd = machinePath.back();
            MachinePose4D alignedLoopStart = loopStart;
            if (policy.keepContinuousAngle)
            {
                while (alignedLoopStart.aDegrees
                    - loopEnd.aDegrees > 180.0)
                {
                    alignedLoopStart.aDegrees -= 360.0;
                }
                while (alignedLoopStart.aDegrees
                    - loopEnd.aDegrees < -180.0)
                {
                    alignedLoopStart.aDegrees += 360.0;
                }
            }
            execution.finalCutPose = alignedLoopStart;
            execution.finalSourcePosition = sourcePath.front();
            if (loopEnd.x != alignedLoopStart.x
                || loopEnd.y != alignedLoopStart.y
                || loopEnd.z != alignedLoopStart.z
                || loopEnd.aDegrees != alignedLoopStart.aDegrees)
            {
                execution.closurePose = alignedLoopStart;
            }

            double totalLength = 0.0;
            for (std::size_t index = 1U; index < sourcePath.size(); ++index)
                totalLength += distance(sourcePath[index - 1U],
                    sourcePath[index]);
            totalLength += distance(sourcePath.back(), sourcePath.front());
            double remaining = std::min(policy.overcutDistance, totalLength);
            const double angleOffset =
                alignedLoopStart.aDegrees - loopStart.aDegrees;
            for (std::size_t index = 1U;
                index < sourcePath.size()
                    && remaining > policy.numericalEpsilon;
                ++index)
            {
                const double length =
                    distance(sourcePath[index - 1U], sourcePath[index]);
                if (length <= policy.numericalEpsilon) continue;
                const double used = std::min(remaining, length);
                MachinePose4D start = machinePath[index - 1U];
                MachinePose4D end = machinePath[index];
                start.aDegrees += angleOffset;
                end.aDegrees += angleOffset;
                ProcessUnitOvercutTarget target;
                target.pose = interpolate(start, end, used / length);
                target.sourcePosition = interpolate
                    (sourcePath[index - 1U], sourcePath[index],
                        used / length);
                execution.overcutTargets.push_back(target);
                execution.finalCutPose = target.pose;
                execution.finalSourcePosition = target.sourcePosition;
                remaining -= used;
            }
            if (remaining > policy.numericalEpsilon)
            {
                const double closureLength =
                    distance(sourcePath.back(), sourcePath.front());
                if (closureLength > policy.numericalEpsilon)
                {
                    MachinePose4D closureStart = machinePath.back();
                    closureStart.aDegrees += angleOffset;
                    const double used = std::min(remaining, closureLength);
                    ProcessUnitOvercutTarget target;
                    target.pose = interpolate(closureStart,
                        alignedLoopStart, used / closureLength);
                    target.sourcePosition = interpolate(sourcePath.back(),
                        sourcePath.front(), used / closureLength);
                    execution.overcutTargets.push_back(target);
                    execution.finalCutPose = target.pose;
                    execution.finalSourcePosition = target.sourcePosition;
                }
            }
            execution.overcutLimited =
                policy.overcutDistance
                    > totalLength + policy.numericalEpsilon;
        }

        result.status = result.diagnostics.isEmpty()
            ? OperationStatus::Success : OperationStatus::PartialSuccess;
        result.value = std::move(execution);
        return result;
    }
}
