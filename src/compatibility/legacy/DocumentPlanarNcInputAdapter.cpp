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
        const QString& message,
        const OperationContext& context,
        const cadcam::planning::ProcessPlan& plan,
        cadcam::geometry::EntityId entityId = 0
    )
    {
        Diagnostic diagnostic;
        diagnostic.code = code;
        diagnostic.severity = DiagnosticSeverity::Error;
        diagnostic.component = QStringLiteral("DocumentPlanarNcInputAdapter");
        diagnostic.operation = context.operationName;
        diagnostic.stage = QStringLiteral("capture-planar-input");
        diagnostic.userMessage = message;
        diagnostic.correlationId = context.correlationId;
        diagnostic.context =
        {
            { QStringLiteral("contentRevision"), QVariant::fromValue<qulonglong>(plan.contentRevision) },
            { QStringLiteral("planMode"), plan.mode == cadcam::planning::ProcessPlanMode::Planar3Axis
                ? QStringLiteral("Planar3Axis") : QStringLiteral("Rotary4Axis") },
            { QStringLiteral("assignmentCount"), static_cast<qulonglong>(plan.assignments.size()) },
            { QStringLiteral("excludedCount"), static_cast<qulonglong>(plan.exclusions.size()) }
        };
        if (entityId != 0U) diagnostic.entityId = entityId;
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
    const cadcam::planning::ProcessPlan& plan,
    const OperationContext& context
)
{
    OperationResult<PlanarNcCapture> result;
    if (document.thread() != QThread::currentThread())
    {
        result.status = OperationStatus::Conflict;
        result.addDiagnostic(planarDiagnostic(DiagnosticCode::PlanarNcInputInvalid,
            QStringLiteral("三轴 NC 输入只能在文档线程中捕获。"), context, plan));
        return result;
    }
    if (plan.mode != cadcam::planning::ProcessPlanMode::Planar3Axis)
    {
        result.status = OperationStatus::Conflict;
        result.addDiagnostic(planarDiagnostic(DiagnosticCode::ProcessPlanModeMismatch,
            QStringLiteral("三轴 NC 只能使用三轴加工计划。"), context, plan));
        return result;
    }
    if (plan.contentRevision == 0U || document.contentRevision() != plan.contentRevision)
    {
        result.status = OperationStatus::Conflict;
        result.addDiagnostic(planarDiagnostic(DiagnosticCode::PlanarNcRevisionMismatch,
            QStringLiteral("三轴加工计划与当前文档版本不一致。"), context, plan));
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

    std::map<cadcam::geometry::EntityId, const GeometrySourceEntry*> sources;
    std::map<cadcam::geometry::EntityId, std::pair<CadItem*, std::size_t>> items;
    for (const GeometrySourceEntry& entry : snapshot.value->entries)
    {
        if (entry.attributes.entityId == 0U
            || !sources.emplace(entry.attributes.entityId, &entry).second)
        {
            result.status = OperationStatus::InvalidInput;
            result.addDiagnostic(planarDiagnostic(DiagnosticCode::PlanarNcInputInvalid,
                QStringLiteral("文档几何快照包含无效或重复图元编号。"), context, plan,
                entry.attributes.entityId));
            return result;
        }
        CadItem* item = entry.sourceIndex < document.m_entities.size()
            ? document.m_entities[entry.sourceIndex].get() : nullptr;
        if (item == nullptr || item->m_entityId != entry.attributes.entityId)
        {
            result.status = OperationStatus::Conflict;
            result.addDiagnostic(planarDiagnostic(DiagnosticCode::PlanarNcEntityMissing,
                QStringLiteral("加工计划图元已不在当前文档中。"), context, plan,
                entry.attributes.entityId));
            return result;
        }
        items.emplace(entry.attributes.entityId, std::make_pair(item, entry.sourceIndex));
    }

    std::set<cadcam::geometry::EntityId> referenced;
    for (const auto& exclusion : plan.exclusions)
    {
        if (items.find(exclusion.entityId) == items.end()
            || !referenced.insert(exclusion.entityId).second)
        {
            result.status = OperationStatus::Conflict;
            result.addDiagnostic(planarDiagnostic(DiagnosticCode::PlanarNcEntityMissing,
                QStringLiteral("三轴计划排除项引用了缺失或重复图元。"), context, plan,
                exclusion.entityId));
            return result;
        }
    }

    std::vector<cadcam::planning::ProcessAssignment> assignments = plan.assignments;
    std::sort(assignments.begin(), assignments.end(), [](const auto& left, const auto& right)
    {
        return left.processOrder < right.processOrder;
    });
    PlanarNcCapture capture;
    capture.contentRevision = plan.contentRevision;
    capture.entities.reserve(assignments.size());
    for (std::size_t order = 0; order < assignments.size(); ++order)
    {
        const auto& assignment = assignments[order];
        const auto itemFound = items.find(assignment.entityId);
        const auto sourceFound = sources.find(assignment.entityId);
        if (assignment.processOrder != static_cast<int>(order)
            || assignment.continuousGroupId != -1
            || itemFound == items.end() || sourceFound == sources.end()
            || !referenced.insert(assignment.entityId).second
            || !sourceFound->second->sourceEntity.has_value()
            || !processable(sourceFound->second->sourceKind))
        {
            result.status = OperationStatus::Conflict;
            result.addDiagnostic(planarDiagnostic(DiagnosticCode::PlanarNcInputInvalid,
                QStringLiteral("三轴计划的顺序、分组或图元几何无效。"), context, plan,
                assignment.entityId));
            return result;
        }

        auto metadata = DocumentNcMetadataAdapter::captureEntity
        (
            *itemFound->second.first,
            itemFound->second.second,
            assignment.processOrder,
            assignment.continuousGroupId,
            context
        );
        result.mergeDiagnostics(metadata);
        if (!metadata.succeeded() || !metadata.value.has_value())
        {
            result.status = metadata.status;
            return result;
        }
        cadcam::nc::PlanarNcEntityInput input;
        input.sourceEntity = *sourceFound->second->sourceEntity;
        input.metadata = std::move(*metadata.value);
        input.reverse = assignment.reverse;
        input.startParameter = assignment.startParameter;
        capture.entities.push_back(std::move(input));
    }

    if (referenced.size() != items.size() || document.contentRevision() != plan.contentRevision)
    {
        result.status = OperationStatus::Conflict;
        result.addDiagnostic(planarDiagnostic(DiagnosticCode::PlanarNcRevisionMismatch,
            QStringLiteral("三轴计划未完整覆盖当前文档，或文档已变更。"), context, plan));
        return result;
    }
    if (capture.entities.empty())
    {
        result.status = OperationStatus::InvalidInput;
        result.addDiagnostic(planarDiagnostic(DiagnosticCode::PlanarNcInputInvalid,
            QStringLiteral("三轴加工计划中没有可生成 NC 的图元。"), context, plan));
        return result;
    }
    result.status = OperationStatus::Success;
    result.value = std::move(capture);
    return result;
}
