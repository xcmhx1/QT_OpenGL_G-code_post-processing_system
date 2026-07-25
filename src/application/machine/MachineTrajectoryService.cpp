#include "application/machine/MachineTrajectoryService.h"

#include "cad/document/CadDocument.h"
#include "application/geometry/DocumentGeometrySnapshotBuilder.h"
#include "core/geometry/GeometryCompiler.h"
#include "core/machine/RotaryTrajectoryBuilder.h"
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

    double pathDistance
    (
        const cadcam::geometry::Vector3d& left,
        const cadcam::geometry::Vector3d& right
    )
    {
        return std::sqrt
        (
            (left.x - right.x) * (left.x - right.x)
            + (left.y - right.y) * (left.y - right.y)
            + (left.z - right.z) * (left.z - right.z)
        );
    }

    cadcam::geometry::PathVertex3D interpolatePathVertex
    (
        const cadcam::geometry::PathVertex3D& start,
        const cadcam::geometry::PathVertex3D& end,
        double parameter
    )
    {
        const double denominator =
            end.sourceParameter - start.sourceParameter;
        const double factor = std::abs(denominator) <= 1.0e-12
            ? 0.0 : std::clamp
                ((parameter - start.sourceParameter) / denominator,
                    0.0, 1.0);
        return
        {
            {
                start.position.x
                    + (end.position.x - start.position.x) * factor,
                start.position.y
                    + (end.position.y - start.position.y) * factor,
                start.position.z
                    + (end.position.z - start.position.z) * factor
            },
            parameter
        };
    }

    struct PathParameterLocation
    {
        std::size_t segmentIndex = 0U;
        cadcam::geometry::PathVertex3D vertex;
    };

    std::optional<PathParameterLocation> locatePathParameter
    (
        const std::vector<cadcam::geometry::PathVertex3D>& vertices,
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
                interpolatePathVertex
                    (vertices[index - 1U], vertices[index], parameter)
            };
        }
        return std::nullopt;
    }

    std::optional<cadcam::geometry::Path3D> slicePath
    (
        const cadcam::geometry::Path3D& source,
        const cadcam::planning::ProcessPathFragment& fragment
    )
    {
        std::vector<cadcam::geometry::PathVertex3D> vertices =
            source.vertices;
        if (fragment.reverse) std::reverse(vertices.begin(), vertices.end());
        const auto begin = locatePathParameter
            (vertices, fragment.sourceParameterBegin);
        const auto end = locatePathParameter
            (vertices, fragment.sourceParameterEnd);
        if (!begin.has_value() || !end.has_value()
            || begin->segmentIndex > end->segmentIndex)
        {
            return std::nullopt;
        }

        cadcam::geometry::Path3D result = source;
        result.vertices.clear();
        result.closed = false;
        result.vertices.push_back(begin->vertex);
        for (std::size_t index = begin->segmentIndex + 1U;
            index <= end->segmentIndex && index < vertices.size(); ++index)
        {
            if (pathDistance(result.vertices.back().position,
                vertices[index].position) > 1.0e-12)
            {
                result.vertices.push_back(vertices[index]);
            }
        }
        if (pathDistance(result.vertices.back().position,
            end->vertex.position) > 1.0e-12)
        {
            result.vertices.push_back(end->vertex);
        }
        else
        {
            result.vertices.back() = end->vertex;
        }
        if (result.vertices.size() < 2U
            || pathDistance(result.vertices.front().position,
                result.vertices.back().position) <= 1.0e-12)
        {
            return std::nullopt;
        }
        return result;
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
    const GProfileToolClearanceConfig& clearanceConfig,
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
    std::map<geometry::EntityId, const planning::ProcessAssignment*>
        assignmentsById;
    for (const auto& assignment : assignments)
        assignmentsById.emplace(assignment.entityId, &assignment);
    std::map<int, std::vector<const planning::ProcessPathFragment*>>
        fragmentsByUnit;
    for (const auto& fragment : plan.plannedFragments)
        fragmentsByUnit[fragment.processUnitIndex].push_back(&fragment);
    for (auto& [unitIndex, fragments] : fragmentsByUnit)
    {
        std::sort(fragments.begin(), fragments.end(),
            [](const auto* left, const auto* right)
            {
                return left->fragmentOrder < right->fragmentOrder;
            });
    }
    std::map<geometry::EntityId, geometry::Path3D> canonicalPaths;
    std::set<int> emittedFragmentUnits;
    input.entities.reserve(assignments.size() + plan.plannedFragments.size());
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

        const auto fragmentedUnit = fragmentsByUnit.find
            (assignment.processUnitIndex);
        if (fragmentedUnit != fragmentsByUnit.end())
        {
            if (!emittedFragmentUnits.insert
                (assignment.processUnitIndex).second)
            {
                continue;
            }
            for (const planning::ProcessPathFragment* fragment :
                fragmentedUnit->second)
            {
                if (fragment == nullptr) continue;
                const auto sourceAssignment = assignmentsById.find
                    (fragment->entityId);
                const auto sourceEntry = entries.find
                    (fragment->entityId);
                if (sourceAssignment == assignmentsById.end()
                    || sourceEntry == entries.end()
                    || sourceEntry->second == nullptr
                    || !sourceEntry->second->sourceEntity.has_value())
                {
                    result.status = OperationStatus::Failed;
                    result.addDiagnostic(serviceDiagnostic
                    (
                        DiagnosticCode::MachineTrajectoryEntityMissing,
                        QStringLiteral("计划片段对应的图元在几何快照中不存在。"),
                        taskContext.operationContext,
                        fragment->entityId
                    ));
                    return result;
                }

                auto canonical = canonicalPaths.find(fragment->entityId);
                if (canonical == canonicalPaths.end())
                {
                    geometry::PathCompileOptions options;
                    auto compiled = compiler.compile
                    (
                        *sourceEntry->second->sourceEntity,
                        productionSamplingPolicy
                            (sourceEntry->second->attributes.originalDxfType),
                        options,
                        taskContext.operationContext
                    );
                    result.mergeDiagnostics(compiled);
                    if (!compiled.succeeded()
                        || !compiled.value.has_value())
                    {
                        result.status = OperationStatus::Failed;
                        result.addDiagnostic(serviceDiagnostic
                        (
                            DiagnosticCode::MachineTrajectoryGeometryCompileFailed,
                            QStringLiteral("计划片段的源图元无法编译为四轴路径。"),
                            taskContext.operationContext,
                            fragment->entityId
                        ));
                        return result;
                    }
                    canonical = canonicalPaths.emplace
                        (fragment->entityId,
                            std::move(*compiled.value)).first;
                }
                const auto path = slicePath
                    (canonical->second, *fragment);
                if (!path.has_value())
                {
                    result.status = OperationStatus::Failed;
                    result.addDiagnostic(serviceDiagnostic
                    (
                        DiagnosticCode::MachineTrajectoryInvalidPath,
                        QStringLiteral("计划片段无法映射到完整源路径。"),
                        taskContext.operationContext,
                        fragment->entityId
                    ));
                    return result;
                }

                machine::TrajectoryEntityInput entity;
                entity.entityId = fragment->entityId;
                entity.sourceIndex = sourceEntry->second->sourceIndex;
                entity.sourceKind = sourceEntry->second->sourceKind;
                entity.processOrder =
                    static_cast<int>(input.entities.size());
                entity.sourceProcessOrder =
                    sourceAssignment->second->processOrder;
                entity.fragmentOrder = fragment->fragmentOrder;
                entity.processGroupId =
                    sourceAssignment->second->continuousGroupId;
                entity.path = *path;
                const auto group = groups.find(entity.processGroupId);
                entity.closed = group != groups.end()
                    && group->second != nullptr
                    ? group->second->closed : false;
                input.entities.push_back(std::move(entity));
            }
            continue;
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
        entity.processOrder = static_cast<int>(input.entities.size());
        entity.sourceProcessOrder = assignment.processOrder;
        entity.processGroupId = assignment.continuousGroupId;
        entity.path = std::move(*compiled.value);
        entity.closed = entity.path.closed;
        const auto group = groups.find(entity.processGroupId);
        if (group != groups.end() && group->second != nullptr)
        {
            entity.closed = group->second->closed;
        }
        input.entities.push_back(std::move(entity));
    }
    for (std::size_t index = 0; index < input.entities.size(); ++index)
    {
        auto& entity = input.entities[index];
        entity.firstInGroup = entity.processGroupId < 0 || index == 0U
            || input.entities[index - 1U].processGroupId
                != entity.processGroupId;
        entity.lastInGroup = entity.processGroupId < 0
            || index + 1U == input.entities.size()
            || input.entities[index + 1U].processGroupId
                != entity.processGroupId;
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
    policy.clearance.retractClearance =
        clearanceConfig.retractClearance;
    policy.clearance.approachClearance =
        clearanceConfig.approachClearance;
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
        << QStringLiteral("[MachineTrajectory][Clearance] mode=Rotary4Axis retractClearance=%1 approachClearance=%2")
            .arg(policy.clearance.retractClearance, 0, 'g', 15)
            .arg(policy.clearance.approachClearance, 0, 'g', 15);
    auto built = machine::RotaryTrajectoryBuilder::build(input, policy, taskContext);
    result.mergeDiagnostics(built);
    result.status = built.status;
    result.value = std::move(built.value);
    if (result.value.has_value()) logSurfaceSummaries(*result.value);
    return result;
}
