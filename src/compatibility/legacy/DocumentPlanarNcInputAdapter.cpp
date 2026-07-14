#include "compatibility/legacy/DocumentPlanarNcInputAdapter.h"

#include "CadDocument.h"
#include "CadItem.h"
#include "application/geometry/DocumentGeometrySnapshotBuilder.h"
#include "compatibility/legacy/DocumentNcMetadataAdapter.h"

#include <QThread>

#include <algorithm>
#include <map>
#include <set>

namespace
{
    Diagnostic planarDiagnostic
    (
        DiagnosticCode code,
        DiagnosticSeverity severity,
        const QString& message,
        const OperationContext& context,
        std::uint64_t revision,
        cadcam::geometry::EntityId entityId = 0
    )
    {
        Diagnostic diagnostic;
        diagnostic.code = code;
        diagnostic.severity = severity;
        diagnostic.component = QStringLiteral("DocumentPlanarNcInputAdapter");
        diagnostic.operation = context.operationName;
        diagnostic.stage = QStringLiteral("capture-planar-input");
        diagnostic.userMessage = message;
        diagnostic.correlationId = context.correlationId;
        diagnostic.context.insert(QStringLiteral("contentRevision"),
            QVariant::fromValue<qulonglong>(revision));
        if (entityId != 0) diagnostic.entityId = entityId;
        return diagnostic;
    }

    bool processable(cadcam::geometry::SourceGeometryKind kind)
    {
        using Kind = cadcam::geometry::SourceGeometryKind;
        return kind == Kind::Line || kind == Kind::Arc || kind == Kind::Circle
            || kind == Kind::Ellipse || kind == Kind::Polyline || kind == Kind::Spline;
    }
}

OperationResult<PlanarNcCapture> DocumentPlanarNcInputAdapter::capture
(
    CadDocument& document,
    const OperationContext& context
)
{
    OperationResult<PlanarNcCapture> result;
    const std::uint64_t revision = document.contentRevision();
    if (document.thread() != QThread::currentThread())
    {
        result.status = OperationStatus::Conflict;
        result.addDiagnostic(planarDiagnostic(DiagnosticCode::PlanarNcInputInvalid,
            DiagnosticSeverity::Error,
            QStringLiteral("三轴 NC 输入只能在文档线程中捕获。"), context, revision));
        return result;
    }

    DocumentGeometrySnapshotBuilder snapshotBuilder;
    auto snapshotResult = snapshotBuilder.capture(document, context);
    result.mergeDiagnostics(snapshotResult);
    if (!snapshotResult.succeeded() || !snapshotResult.value.has_value())
    {
        result.status = snapshotResult.status;
        return result;
    }
    if (revision == 0 || snapshotResult.value->contentRevision != revision
        || document.contentRevision() != revision)
    {
        result.status = OperationStatus::Conflict;
        result.addDiagnostic(planarDiagnostic(DiagnosticCode::PlanarNcRevisionMismatch,
            DiagnosticSeverity::Error,
            QStringLiteral("文档在三轴 NC 输入捕获期间已变更。"), context, revision));
        return result;
    }

    std::map<cadcam::geometry::EntityId, const GeometrySourceEntry*> sources;
    for (const GeometrySourceEntry& entry : snapshotResult.value->entries)
    {
        if (entry.attributes.entityId != 0)
            sources.emplace(entry.attributes.entityId, &entry);
    }

    std::vector<std::pair<CadItem*, std::size_t>> ordered;
    std::set<int> processOrders;
    std::set<cadcam::geometry::EntityId> entityIds;
    bool skippedInvalid = false;
    for (std::size_t index = 0; index < document.m_entities.size(); ++index)
    {
        CadItem* item = document.m_entities[index].get();
        if (item == nullptr || item->m_excludedFromProcessing) continue;
        const auto found = sources.find(item->m_entityId);
        const GeometrySourceEntry* source = found == sources.end() ? nullptr : found->second;
        if (source == nullptr || !processable(source->sourceKind)) continue;
        if (!source->sourceEntity.has_value())
        {
            skippedInvalid = true;
            result.addDiagnostic(planarDiagnostic(DiagnosticCode::PlanarNcEntityMissing,
                DiagnosticSeverity::Warning,
                QStringLiteral("图元的几何快照无效，已跳过。"), context, revision, item->m_entityId));
            continue;
        }
        if (item->m_processOrder < 0 || !processOrders.insert(item->m_processOrder).second
            || item->m_entityId == 0 || !entityIds.insert(item->m_entityId).second)
        {
            result.status = OperationStatus::InvalidInput;
            result.addDiagnostic(planarDiagnostic(DiagnosticCode::PlanarNcInputInvalid,
                DiagnosticSeverity::Error,
                QStringLiteral("三轴加工顺序缺失、重复，或图元编号无效。"), context,
                revision, item->m_entityId));
            return result;
        }
        ordered.emplace_back(item, index);
    }

    std::sort(ordered.begin(), ordered.end(), [](const auto& left, const auto& right)
    {
        return left.first->m_processOrder < right.first->m_processOrder;
    });

    PlanarNcCapture capture;
    capture.contentRevision = revision;
    capture.entities.reserve(ordered.size());
    for (std::size_t order = 0; order < ordered.size(); ++order)
    {
        CadItem& item = *ordered[order].first;
        const GeometrySourceEntry* source = sources.at(item.m_entityId);
        auto metadata = DocumentNcMetadataAdapter::captureEntity
        (
            item,
            ordered[order].second,
            static_cast<int>(order),
            item.m_processContinuousGroupId,
            context
        );
        result.mergeDiagnostics(metadata);
        if (!metadata.succeeded() || !metadata.value.has_value())
        {
            result.status = metadata.status;
            return result;
        }

        cadcam::nc::PlanarNcEntityInput input;
        input.sourceEntity = *source->sourceEntity;
        input.metadata = std::move(*metadata.value);
        input.reverse = item.m_isReverse;
        if (item.m_hasCustomProcessStart)
            input.startParameter = item.m_processStartParameter;
        capture.entities.push_back(std::move(input));
    }

    if (document.contentRevision() != revision)
    {
        result.status = OperationStatus::Conflict;
        result.addDiagnostic(planarDiagnostic(DiagnosticCode::PlanarNcRevisionMismatch,
            DiagnosticSeverity::Error,
            QStringLiteral("文档在三轴 NC 输入捕获期间已变更。"), context, revision));
        return result;
    }
    if (capture.entities.empty())
    {
        result.status = OperationStatus::InvalidInput;
        result.addDiagnostic(planarDiagnostic(DiagnosticCode::PlanarNcInputInvalid,
            DiagnosticSeverity::Error,
            QStringLiteral("文档中没有已排序的可加工图元。"), context, revision));
        return result;
    }

    result.status = skippedInvalid ? OperationStatus::PartialSuccess : OperationStatus::Success;
    result.value = std::move(capture);
    return result;
}
