#include "application/geometry/DocumentGeometrySnapshotBuilder.h"

#include "cad/document/CadDocument.h"
#include "cad/items/CadItem.h"
#include "infrastructure/dxf/DxfGeometryAdapter.h"

#include <QThread>

namespace
{
    cadcam::geometry::SourceGeometryKind sourceKindForType(DRW::ETYPE type)
    {
        using cadcam::geometry::SourceGeometryKind;
        switch (type)
        {
        case DRW::ETYPE::LINE: return SourceGeometryKind::Line;
        case DRW::ETYPE::ARC: return SourceGeometryKind::Arc;
        case DRW::ETYPE::CIRCLE: return SourceGeometryKind::Circle;
        case DRW::ETYPE::ELLIPSE: return SourceGeometryKind::Ellipse;
        case DRW::ETYPE::POLYLINE:
        case DRW::ETYPE::LWPOLYLINE: return SourceGeometryKind::Polyline;
        case DRW::ETYPE::SPLINE: return SourceGeometryKind::Spline;
        case DRW::ETYPE::POINT: return SourceGeometryKind::Point;
        default: return SourceGeometryKind::Unknown;
        }
    }

    Diagnostic captureDiagnostic
    (
        const OperationContext& context,
        DiagnosticCode code,
        const QString& detail
    )
    {
        Diagnostic diagnostic;
        diagnostic.code = code;
        diagnostic.severity = DiagnosticSeverity::Error;
        diagnostic.component = QStringLiteral("DocumentGeometrySnapshotBuilder");
        diagnostic.operation = QStringLiteral("CaptureGeometrySourceSnapshot");
        diagnostic.stage = QStringLiteral("CaptureDocumentValues");
        diagnostic.userMessage = QStringLiteral("无法建立文档几何源快照。");
        diagnostic.technicalDetail = detail;
        diagnostic.correlationId = context.correlationId;
        return diagnostic;
    }

    void enrichDiagnostics
    (
        QVector<Diagnostic>& diagnostics,
        const GeometrySourceEntry& entry
    )
    {
        const QString kindName = QString::fromLatin1
            (cadcam::geometry::sourceGeometryKindName(entry.sourceKind));
        for (Diagnostic& diagnostic : diagnostics)
        {
            diagnostic.entityId = entry.attributes.entityId;
            diagnostic.context.insert
                (QStringLiteral("sourceIndex"), static_cast<qulonglong>(entry.sourceIndex));
            diagnostic.context.insert(QStringLiteral("sourceKind"), kindName);
        }
    }
}

OperationResult<GeometrySourceSnapshot> DocumentGeometrySnapshotBuilder::capture
(
    const CadDocument& document,
    const OperationContext& context
) const
{
    OperationResult<GeometrySourceSnapshot> result;
    if (QThread::currentThread() != document.thread())
    {
        result.status = OperationStatus::Failed;
        result.addDiagnostic(captureDiagnostic
        (
            context,
            DiagnosticCode::InternalInvariantViolation,
            QStringLiteral("capture() must run on the CadDocument owning thread")
        ));
        return result;
    }
    if (document.contentRevision() == 0U)
    {
        result.status = OperationStatus::Failed;
        result.addDiagnostic(captureDiagnostic
        (
            context,
            DiagnosticCode::InternalInvariantViolation,
            QStringLiteral("document content revision is zero")
        ));
        return result;
    }

    GeometrySourceSnapshot snapshot;
    snapshot.contentRevision = document.contentRevision();
    snapshot.entries.reserve(document.m_entities.size());
    bool hasFailure = false;

    for (std::size_t index = 0; index < document.m_entities.size(); ++index)
    {
        const std::unique_ptr<CadItem>& item = document.m_entities[index];
        GeometrySourceEntry entry;
        entry.sourceIndex = index;
        if (item == nullptr || item->m_nativeEntity == nullptr)
        {
            entry.status = OperationStatus::InvalidInput;
            entry.diagnostics.push_back(captureDiagnostic
            (
                context,
                DiagnosticCode::GeometryAdapterFailure,
                QStringLiteral("document entry has no CadItem or native entity")
            ));
            enrichDiagnostics(entry.diagnostics, entry);
            snapshot.entries.push_back(std::move(entry));
            hasFailure = true;
            continue;
        }

        entry.attributes.entityId = item->m_entityId;
        entry.attributes.originalDxfType = static_cast<int>(item->m_type);
        entry.attributes.layer = QString::fromUtf8(item->m_nativeEntity->layer.c_str());
        entry.attributes.color = item->m_color;
        entry.attributes.visible = item->m_nativeEntity->visible;
        entry.sourceKind = sourceKindForType(item->m_type);

        OperationResult<cadcam::geometry::SourceEntity> adapted =
            DxfGeometryAdapter::convert
            (
                item->m_entityId,
                *item->m_nativeEntity,
                context
            );
        entry.status = adapted.status;
        entry.diagnostics = adapted.diagnostics;
        if (adapted.value.has_value())
        {
            entry.sourceKind = adapted.value->kind;
            entry.sourceEntity = std::move(*adapted.value);
        }
        if (!adapted.succeeded() || !entry.sourceEntity.has_value())
        {
            hasFailure = true;
        }
        enrichDiagnostics(entry.diagnostics, entry);
        snapshot.entries.push_back(std::move(entry));
    }

    result.status = hasFailure ? OperationStatus::PartialSuccess : OperationStatus::Success;
    result.value = std::move(snapshot);
    for (const GeometrySourceEntry& entry : result.value->entries)
    {
        result.mergeDiagnostics(entry.diagnostics);
    }
    return result;
}
