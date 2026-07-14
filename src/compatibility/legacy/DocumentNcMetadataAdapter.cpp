#include "compatibility/legacy/DocumentNcMetadataAdapter.h"

#include "CadDocument.h"
#include "CadItem.h"
#include "GProfile.h"
#include "drw_entities.h"

#include <QColor>
#include <QThread>

#include <algorithm>
#include <map>

namespace
{
    Diagnostic adapterDiagnostic
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
        diagnostic.component = QStringLiteral("DocumentNcMetadataAdapter");
        diagnostic.operation = context.operationName;
        diagnostic.stage = QStringLiteral("capture-document-metadata");
        diagnostic.userMessage = message;
        diagnostic.correlationId = context.correlationId;
        if (entityId != 0) diagnostic.entityId = entityId;
        return diagnostic;
    }

    cadcam::geometry::SourceGeometryKind sourceKind(DRW::ETYPE type)
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

    QString entityTypeKey(DRW::ETYPE type)
    {
        switch (type)
        {
        case DRW::ETYPE::LINE: return QStringLiteral("LINE");
        case DRW::ETYPE::ARC: return QStringLiteral("ARC");
        case DRW::ETYPE::CIRCLE: return QStringLiteral("CIRCLE");
        case DRW::ETYPE::ELLIPSE: return QStringLiteral("ELLIPSE");
        case DRW::ETYPE::POLYLINE: return QStringLiteral("POLYLINE");
        case DRW::ETYPE::LWPOLYLINE: return QStringLiteral("LWPOLYLINE");
        case DRW::ETYPE::SPLINE: return QStringLiteral("SPLINE");
        case DRW::ETYPE::POINT: return QStringLiteral("POINT");
        default: return QString();
        }
    }

    QString colorKey(const DRW_Entity& entity)
    {
        if (entity.color24 >= 0)
        {
            return GProfile::colorKeyFromColor(QColor::fromRgb
            (
                (entity.color24 >> 16) & 0xFF,
                (entity.color24 >> 8) & 0xFF,
                entity.color24 & 0xFF
            ));
        }
        if (entity.color == DRW::ColorByLayer) return QStringLiteral("BYLAYER");
        if (entity.color == DRW::ColorByBlock) return QStringLiteral("BYBLOCK");
        return GProfile::colorKeyFromAci(entity.color);
    }
}

OperationResult<std::vector<cadcam::nc::NcEntityMetadata>> DocumentNcMetadataAdapter::capture
(
    CadDocument& document,
    const cadcam::planning::ProcessPlan& plan,
    const OperationContext& context
)
{
    OperationResult<std::vector<cadcam::nc::NcEntityMetadata>> result;
    if (document.thread() != QThread::currentThread())
    {
        result.status = OperationStatus::Conflict;
        result.addDiagnostic(adapterDiagnostic(DiagnosticCode::NcProgramInputInvalid,
            QStringLiteral("NC 图元元数据只能在文档线程中捕获。"), context));
        return result;
    }
    if (document.contentRevision() == 0 || document.contentRevision() != plan.contentRevision)
    {
        result.status = OperationStatus::Conflict;
        result.addDiagnostic(adapterDiagnostic(DiagnosticCode::NcProgramRevisionMismatch,
            QStringLiteral("文档与加工计划版本不一致，无法捕获 NC 图元元数据。"), context));
        return result;
    }

    std::map<cadcam::geometry::EntityId, std::pair<CadItem*, std::size_t>> documentItems;
    for (std::size_t index = 0; index < document.m_entities.size(); ++index)
    {
        CadItem* item = document.m_entities[index].get();
        if (item == nullptr) continue;
        if (item->m_entityId == 0 || !documentItems.emplace(item->m_entityId, std::make_pair(item, index)).second)
        {
            result.status = OperationStatus::InvalidInput;
            result.addDiagnostic(adapterDiagnostic(DiagnosticCode::NcProgramDuplicateEntity,
                QStringLiteral("文档包含零编号或重复编号图元。"), context, item->m_entityId));
            return result;
        }
    }

    std::vector<cadcam::planning::ProcessAssignment> assignments = plan.assignments;
    std::sort(assignments.begin(), assignments.end(), [](const auto& left, const auto& right)
    {
        return left.processOrder < right.processOrder;
    });
    std::vector<cadcam::nc::NcEntityMetadata> metadata;
    metadata.reserve(assignments.size());
    std::map<cadcam::geometry::EntityId, bool> usedIds;
    for (std::size_t index = 0; index < assignments.size(); ++index)
    {
        const auto& assignment = assignments[index];
        const auto found = documentItems.find(assignment.entityId);
        if (assignment.entityId == 0 || !usedIds.emplace(assignment.entityId, true).second
            || assignment.processOrder != static_cast<int>(index))
        {
            result.status = OperationStatus::InvalidInput;
            result.addDiagnostic(adapterDiagnostic(DiagnosticCode::NcProgramDuplicateEntity,
                QStringLiteral("加工计划包含零编号、重复图元或不连续加工顺序。"), context,
                assignment.entityId));
            return result;
        }
        if (found == documentItems.end() || found->second.first == nullptr
            || found->second.first->m_nativeEntity == nullptr)
        {
            result.status = OperationStatus::Failed;
            result.addDiagnostic(adapterDiagnostic(DiagnosticCode::NcProgramEntityMissing,
                QStringLiteral("加工计划中的图元已不在当前文档中。"), context, assignment.entityId));
            return result;
        }

        CadItem* item = found->second.first;
        cadcam::nc::NcEntityMetadata entry;
        entry.entityId = assignment.entityId;
        entry.sourceKind = sourceKind(item->m_type);
        entry.sourceIndex = found->second.second;
        entry.processOrder = assignment.processOrder;
        entry.processGroupId = assignment.continuousGroupId;
        entry.entityTypeKey = entityTypeKey(item->m_type).toStdString();
        entry.layerKey = GProfile::normalizeLayerKey
            (QString::fromUtf8(item->m_nativeEntity->layer.c_str())).toStdString();
        entry.colorKey = colorKey(*item->m_nativeEntity).toStdString();
        if (entry.sourceKind == cadcam::geometry::SourceGeometryKind::Unknown
            || entry.entityTypeKey.empty())
        {
            result.status = OperationStatus::NotSupported;
            result.addDiagnostic(adapterDiagnostic(DiagnosticCode::NcProgramInputInvalid,
                QStringLiteral("加工计划包含不支持的图元类型。"), context, assignment.entityId));
            return result;
        }
        metadata.push_back(std::move(entry));
    }

    if (metadata.empty())
    {
        result.status = OperationStatus::InvalidInput;
        result.addDiagnostic(adapterDiagnostic(DiagnosticCode::NcProgramInputInvalid,
            QStringLiteral("加工计划中没有可捕获的 NC 图元元数据。"), context));
        return result;
    }
    result.status = OperationStatus::Success;
    result.value = std::move(metadata);
    return result;
}
