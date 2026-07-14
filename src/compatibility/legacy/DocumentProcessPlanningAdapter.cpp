#include "compatibility/legacy/DocumentProcessPlanningAdapter.h"

#include "CadDocument.h"
#include "CadItem.h"
#include "application/geometry/DocumentGeometrySnapshotBuilder.h"
#include "compatibility/legacy/LegacyProcessPlanAdapter.h"
#include "core/geometry/GeometryCompiler.h"
#include "drw_entities.h"

#include <QThread>

namespace
{
    Diagnostic captureDiagnostic
    (
        DiagnosticCode code,
        DiagnosticSeverity severity,
        const QString& message,
        const OperationContext& context,
        std::uint64_t revision,
        cadcam::geometry::EntityId entityId = 0,
        std::size_t sourceIndex = 0
    )
    {
        Diagnostic diagnostic;
        diagnostic.code = code;
        diagnostic.severity = severity;
        diagnostic.component = QStringLiteral("DocumentProcessPlanningAdapter");
        diagnostic.operation = context.operationName;
        diagnostic.stage = QStringLiteral("capture-planar-planning-input");
        diagnostic.userMessage = message;
        diagnostic.correlationId = context.correlationId;
        diagnostic.context =
        {
            { QStringLiteral("contentRevision"), QVariant::fromValue<qulonglong>(revision) },
            { QStringLiteral("planMode"), QStringLiteral("Planar3Axis") },
            { QStringLiteral("entityId"), QVariant::fromValue<qulonglong>(entityId) },
            { QStringLiteral("sourceIndex"), QVariant::fromValue<qulonglong>(sourceIndex) }
        };
        if (entityId != 0U) diagnostic.entityId = entityId;
        return diagnostic;
    }

    cadcam::geometry::SamplingPolicy productionSamplingPolicy(int dxfType)
    {
        cadcam::geometry::SamplingPolicy policy;
        policy.chordTolerance = 0.0;
        policy.singlePrecisionEvaluation = true;
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
}

OperationResult<cadcam::planning::PlanarProcessPlanningInput>
DocumentProcessPlanningAdapter::capturePlanar
(
    CadDocument& document,
    const OperationContext& context
) const
{
    using namespace cadcam;
    OperationResult<planning::PlanarProcessPlanningInput> result;
    const std::uint64_t revision = document.contentRevision();
    if (QThread::currentThread() != document.thread() || revision == 0U)
    {
        result.status = OperationStatus::InvalidInput;
        result.addDiagnostic(captureDiagnostic(DiagnosticCode::PlanarPlanningInputInvalid,
            DiagnosticSeverity::Error,
            QStringLiteral("三轴加工计划输入必须在文档线程捕获。"),
            context, revision));
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
    if (snapshot.value->contentRevision != revision || document.contentRevision() != revision)
    {
        result.status = OperationStatus::Conflict;
        result.addDiagnostic(captureDiagnostic(DiagnosticCode::ProcessPlanningRevisionMismatch,
            DiagnosticSeverity::Error,
            QStringLiteral("文档在三轴加工计划输入捕获期间已变更。"),
            context, revision));
        return result;
    }

    planning::PlanarProcessPlanningInput input;
    input.contentRevision = revision;
    input.entities.reserve(snapshot.value->entries.size());
    geometry::GeometryCompiler compiler;
    for (const GeometrySourceEntry& entry : snapshot.value->entries)
    {
        planning::PlanarPlanningEntity entity;
        entity.entityId = entry.attributes.entityId;
        entity.sourceIndex = entry.sourceIndex;
        entity.sourceKind = entry.sourceKind;
        entity.visible = entry.attributes.visible;

        CadItem* item = entry.sourceIndex < document.m_entities.size()
            ? document.m_entities[entry.sourceIndex].get() : nullptr;
        entity.processEnabled = item != nullptr && item->m_nativeEntity != nullptr;
        entity.excludedAsInternalGeometry = item != nullptr
            && item->m_excludedAsInternalGeometry;
        entity.currentReversePreference = item != nullptr && item->m_isReverse;
        if (item != nullptr && item->m_hasCustomProcessStart)
            entity.customStartParameter = item->m_processStartParameter;

        if (entry.sourceEntity.has_value())
        {
            entity.sourceEntity = *entry.sourceEntity;
            geometry::PathCompileOptions options;
            auto path = compiler.compile(entity.sourceEntity,
                productionSamplingPolicy(entry.attributes.originalDxfType), options, context);
            if (path.succeeded() && path.value.has_value())
                entity.path = std::move(*path.value);
            else
                result.mergeDiagnostics(path);
        }
        else
        {
            entity.sourceEntity.id = entity.entityId;
            entity.sourceEntity.kind = entity.sourceKind;
        }
        input.entities.push_back(std::move(entity));
    }

    if (document.contentRevision() != revision)
    {
        result.status = OperationStatus::Conflict;
        result.addDiagnostic(captureDiagnostic(DiagnosticCode::ProcessPlanningRevisionMismatch,
            DiagnosticSeverity::Error,
            QStringLiteral("文档在三轴加工计划输入捕获期间已变更。"),
            context, revision));
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
    const std::optional<cadcam::machining::TubeSectionModel>& tubeSection,
    double connectionTolerance,
    cadcam::topology::PathTopology& topologyStorage,
    const OperationContext& context
) const
{
    return LegacyProcessPlanAdapter{}.capture
        (document, tubeSection, connectionTolerance, topologyStorage, context);
}
