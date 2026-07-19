#include "application/planning/DocumentProcessPlanningAdapter.h"

#include "cad/document/CadDocument.h"
#include "application/geometry/DocumentGeometrySnapshotBuilder.h"
#include "application/tasks/TaskContext.h"
#include "core/geometry/GeometryCompiler.h"
#include "drw_entities.h"

#include <QThread>

#include <cmath>

namespace
{
    using cadcam::geometry::EntityId;
    using cadcam::geometry::SourceGeometryKind;

    Diagnostic captureDiagnostic
    (
        DiagnosticCode code,
        const QString& message,
        const OperationContext& context,
        std::uint64_t contentRevision,
        std::uint64_t processStateRevision,
        EntityId entityId = 0
    )
    {
        Diagnostic diagnostic;
        diagnostic.code = code;
        diagnostic.severity = DiagnosticSeverity::Error;
        diagnostic.component = QStringLiteral("DocumentProcessPlanningAdapter");
        diagnostic.operation = context.operationName;
        diagnostic.stage = QStringLiteral("capture-planning-input");
        diagnostic.userMessage = message;
        diagnostic.correlationId = context.correlationId;
        diagnostic.context =
        {
            { QStringLiteral("contentRevision"), QVariant::fromValue<qulonglong>(contentRevision) },
            { QStringLiteral("processStateRevision"), QVariant::fromValue<qulonglong>(processStateRevision) },
            { QStringLiteral("entityId"), QVariant::fromValue<qulonglong>(entityId) }
        };
        if (entityId != 0U) diagnostic.entityId = entityId;
        return diagnostic;
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
            policy.minimumSegments = 16;
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

    bool supportsProcessPath(SourceGeometryKind kind)
    {
        return kind == SourceGeometryKind::Line
            || kind == SourceGeometryKind::Arc
            || kind == SourceGeometryKind::Circle
            || kind == SourceGeometryKind::Ellipse
            || kind == SourceGeometryKind::Polyline
            || kind == SourceGeometryKind::Spline;
    }

    bool hasUsablePath(const cadcam::geometry::Path3D& path, double minimumEdgeLength)
    {
        if (path.vertices.size() < 2U) return false;
        for (std::size_t index = 1; index < path.vertices.size(); ++index)
        {
            const auto& left = path.vertices[index - 1U].position;
            const auto& right = path.vertices[index].position;
            const double dx = right.x - left.x;
            const double dy = right.y - left.y;
            const double dz = right.z - left.z;
            if (std::sqrt(dx * dx + dy * dy + dz * dz) > minimumEdgeLength) return true;
        }
        return false;
    }

    std::optional<double> effectiveStartParameter
    (
        const std::optional<double>& requestedStartParameter,
        SourceGeometryKind sourceKind,
        bool closed
    )
    {
        if (requestedStartParameter.has_value()) return requestedStartParameter;
        if (closed && (sourceKind == SourceGeometryKind::Circle
            || sourceKind == SourceGeometryKind::Ellipse))
        {
            return 1.57079632679489661923;
        }
        return std::nullopt;
    }
}

OperationResult<cadcam::planning::PlanarProcessPlanningInput>
DocumentProcessPlanningAdapter::capturePlanar
(
    CadDocument& document,
    const cadcam::process::DocumentProcessState& processState,
    cadcam::planning::ProcessSortIntent sortIntent,
    const OperationContext& context
) const
{
    using namespace cadcam;
    OperationResult<planning::PlanarProcessPlanningInput> result;
    const std::uint64_t contentRevision = document.contentRevision();
    const std::uint64_t stateRevision = processState.revision();
    if (QThread::currentThread() != document.thread() || contentRevision == 0U)
    {
        result.status = OperationStatus::InvalidInput;
        result.addDiagnostic(captureDiagnostic(DiagnosticCode::PlanarPlanningInputInvalid,
            QStringLiteral("三轴加工计划输入必须在文档线程捕获。"), context,
            contentRevision, stateRevision));
        return result;
    }

    DocumentGeometrySnapshotBuilder snapshotBuilder;
    auto snapshot = snapshotBuilder.capture(document, context);
    result.mergeDiagnostics(snapshot);
    if (!snapshot.succeeded() || !snapshot.value.has_value())
    {
        result.status = snapshot.status;
        return result;
    }

    planning::PlanarProcessPlanningInput input;
    input.contentRevision = contentRevision;
    input.processStateRevision = stateRevision;
    input.entities.reserve(snapshot.value->entries.size());
    geometry::GeometryCompiler compiler;
    const bool rebuildSequence =
        sortIntent == planning::ProcessSortIntent::RebuildSequence;
    for (const GeometrySourceEntry& entry : snapshot.value->entries)
    {
        planning::PlanarPlanningEntity entity;
        entity.entityId = entry.attributes.entityId;
        entity.sourceIndex = entry.sourceIndex;
        entity.sourceKind = entry.sourceKind;
        entity.visible = entry.attributes.visible;
        const process::EntityProcessState state = processState.stateOrDefault(entity.entityId);
        entity.processEnabled = state.overrideData.processEnabled;
        entity.excludedAsInternalGeometry = state.effectiveInternalExclusion();
        entity.directionPreference = rebuildSequence
            ? process::DirectionPreference::Auto : state.overrideData.direction;
        entity.startParameter = rebuildSequence
            ? std::nullopt : state.overrideData.startParameter;
        if (entry.sourceEntity.has_value())
        {
            entity.sourceEntity = *entry.sourceEntity;
            geometry::PathCompileOptions options;
            options.startParameter = entity.startParameter;
            auto path = compiler.compile(entity.sourceEntity,
                productionSamplingPolicy(entry.attributes.originalDxfType), options, context);
            if (path.succeeded() && path.value.has_value())
            {
                entity.path = std::move(*path.value);
                entity.startParameter = effectiveStartParameter
                    (entity.startParameter, entity.sourceKind, entity.path.closed);
            }
            else result.mergeDiagnostics(path);
        }
        else
        {
            entity.sourceEntity.id = entity.entityId;
            entity.sourceEntity.kind = entity.sourceKind;
        }
        input.entities.push_back(std::move(entity));
    }

    if (document.contentRevision() != contentRevision || processState.revision() != stateRevision)
    {
        result.status = OperationStatus::Conflict;
        result.addDiagnostic(captureDiagnostic(DiagnosticCode::ProcessStateRevisionMismatch,
            QStringLiteral("文档或加工状态在三轴计划输入捕获期间已变更。"), context,
            contentRevision, stateRevision));
        return result;
    }
    result.status = OperationStatus::Success;
    result.value = std::move(input);
    return result;
}

OperationResult<cadcam::planning::ProcessPlanningInput>
DocumentProcessPlanningAdapter::captureRotary
(
    CadDocument& document,
    const cadcam::process::DocumentProcessState& processState,
    const std::optional<cadcam::machining::TubeSectionModel>& tubeSection,
    cadcam::planning::ProcessSortIntent sortIntent,
    double connectionTolerance,
    cadcam::topology::PathTopology& topologyStorage,
    const OperationContext& context
) const
{
    using namespace cadcam;
    OperationResult<planning::ProcessPlanningInput> result;
    const std::uint64_t contentRevision = document.contentRevision();
    const std::uint64_t stateRevision = processState.revision();
    if (QThread::currentThread() != document.thread() || contentRevision == 0U)
    {
        result.status = OperationStatus::InvalidInput;
        result.addDiagnostic(captureDiagnostic(DiagnosticCode::ProcessPlanningInputInvalid,
            QStringLiteral("四轴加工计划输入必须在文档线程捕获。"), context,
            contentRevision, stateRevision));
        return result;
    }

    DocumentGeometrySnapshotBuilder snapshotBuilder;
    auto snapshot = snapshotBuilder.capture(document, context);
    result.mergeDiagnostics(snapshot);
    if (!snapshot.succeeded() || !snapshot.value.has_value())
    {
        result.status = snapshot.status;
        return result;
    }

    const topology::PathTopologyTolerance topologyTolerance =
        topology::PathTopologyTolerance::fromConnectionTolerance(connectionTolerance);
    planning::ProcessPlanningInput input;
    input.contentRevision = contentRevision;
    input.processStateRevision = stateRevision;
    input.tubeSection = tubeSection;
    if (tubeSection.has_value())
    {
        input.tubeSectionCenter = geometry::Vector2d
            { tubeSection->geometry.centerY, tubeSection->geometry.centerZ };
    }
    input.topologyInput.contentRevision = contentRevision;
    input.entities.reserve(snapshot.value->entries.size());
    geometry::GeometryCompiler compiler;
    const bool rebuildSequence =
        sortIntent == planning::ProcessSortIntent::RebuildSequence;
    for (const GeometrySourceEntry& entry : snapshot.value->entries)
    {
        planning::PlanningEntity entity;
        entity.entityId = entry.attributes.entityId;
        entity.sourceIndex = entry.sourceIndex;
        entity.sourceKind = entry.sourceKind;
        entity.visible = entry.attributes.visible;
        const process::EntityProcessState state = processState.stateOrDefault(entity.entityId);
        entity.processEnabled = state.overrideData.processEnabled;
        entity.excludedAsInternalGeometry = state.effectiveInternalExclusion();
        entity.boundaryRole = state.overrideData.boundaryRole;
        entity.boundaryPairId = state.overrideData.boundaryPairId;
        entity.directionPreference = rebuildSequence
            ? process::DirectionPreference::Auto : state.overrideData.direction;
        entity.startParameter = rebuildSequence
            ? std::nullopt : state.overrideData.startParameter;

        if (entry.sourceEntity.has_value())
        {
            geometry::PathCompileOptions options;
            options.startParameter = entity.startParameter;
            geometry::SamplingPolicy pathPolicy =
                productionSamplingPolicy(entry.attributes.originalDxfType);
            auto path = compiler.compile(*entry.sourceEntity, pathPolicy, options, context);
            if (path.succeeded() && path.value.has_value())
            {
                entity.path = std::move(*path.value);
                entity.startParameter = effectiveStartParameter
                    (entity.startParameter, entity.sourceKind, entity.path.closed);
                if (supportsProcessPath(entity.sourceKind)
                    && hasUsablePath(entity.path, topologyTolerance.minimumEdgeLength))
                {
                    auto topologyPath = compiler.compile
                        (*entry.sourceEntity, pathPolicy, options, context);
                    topology::TopologyPathRecord record;
                    record.sourceIndex = entity.sourceIndex;
                    record.entityId = entity.entityId;
                    record.sourceKind = entity.sourceKind;
                    record.semanticallyClosed = entity.path.closed;
                    const geometry::Path3D& sourcePath =
                        topologyPath.succeeded() && topologyPath.value.has_value()
                            ? *topologyPath.value
                            : entity.path;
                    record.points.reserve(sourcePath.vertices.size());
                    for (const auto& vertex : sourcePath.vertices)
                        record.points.push_back(vertex.position);
                    input.topologyInput.records.push_back(std::move(record));
                }
            }
            else result.mergeDiagnostics(path);
        }
        input.entities.push_back(std::move(entity));
    }

    TaskContext taskContext;
    taskContext.operationContext = context;
    topology::PathTopologyBuilder topologyBuilder;
    auto topologyResult = topologyBuilder.build(input.topologyInput, topologyTolerance, taskContext);
    result.mergeDiagnostics(topologyResult);
    if (!topologyResult.succeeded() || !topologyResult.value.has_value())
    {
        result.status = topologyResult.status;
        return result;
    }
    topologyStorage = std::move(*topologyResult.value);
    input.topology = &topologyStorage;

    if (document.contentRevision() != contentRevision || processState.revision() != stateRevision)
    {
        result.status = OperationStatus::Conflict;
        result.addDiagnostic(captureDiagnostic(DiagnosticCode::ProcessStateRevisionMismatch,
            QStringLiteral("文档或加工状态在四轴计划输入捕获期间已变更。"), context,
            contentRevision, stateRevision));
        return result;
    }
    result.status = OperationStatus::Success;
    result.value = std::move(input);
    return result;
}
