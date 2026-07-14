#include "compatibility/legacy/LegacyCadItemTopologyAdapter.h"

#include "CadEllipseGeometry.h"
#include "CadItem.h"

#include <QVector3D>

#include <cmath>
#include <set>

namespace
{
    using cadcam::geometry::EntityId;
    using cadcam::geometry::SourceGeometryKind;
    using cadcam::geometry::Vector3d;
    using cadcam::topology::PathTopologyTolerance;
    using cadcam::topology::TopologyInput;
    using cadcam::topology::TopologyPathRecord;

    Diagnostic adapterDiagnostic
    (
        const OperationContext& context,
        DiagnosticCode code,
        const QString& detail,
        std::size_t sourceIndex,
        EntityId entityId,
        int dxfType,
        std::size_t pointCount,
        const PathTopologyTolerance& tolerance
    )
    {
        Diagnostic diagnostic;
        diagnostic.code = code;
        diagnostic.severity = DiagnosticSeverity::Error;
        diagnostic.component = QStringLiteral("LegacyCadItemTopologyAdapter");
        diagnostic.operation = QStringLiteral("AdaptCadItemsToTopologyInput");
        diagnostic.stage = QStringLiteral("RebuildLegacyProcessPath");
        diagnostic.userMessage = QStringLiteral("部分图元无法建立拓扑输入。");
        diagnostic.technicalDetail = detail;
        diagnostic.correlationId = context.correlationId;
        if (entityId != 0U)
        {
            diagnostic.entityId = entityId;
        }
        diagnostic.context.insert
            (QStringLiteral("entityId"), static_cast<qulonglong>(entityId));
        diagnostic.context.insert
            (QStringLiteral("sourceIndex"), static_cast<qulonglong>(sourceIndex));
        diagnostic.context.insert(QStringLiteral("dxfType"), dxfType);
        diagnostic.context.insert
            (QStringLiteral("pointCount"), static_cast<qulonglong>(pointCount));
        diagnostic.context.insert(QStringLiteral("nodeSnap"), tolerance.nodeSnap);
        diagnostic.context.insert(QStringLiteral("closure"), tolerance.closure);
        diagnostic.context.insert
            (QStringLiteral("intersectionTolerance"), tolerance.intersection);
        diagnostic.context.insert
            (QStringLiteral("minimumEdgeLength"), tolerance.minimumEdgeLength);
        return diagnostic;
    }

    SourceGeometryKind sourceKind(DRW::ETYPE type)
    {
        switch (type)
        {
        case DRW::LINE: return SourceGeometryKind::Line;
        case DRW::ARC: return SourceGeometryKind::Arc;
        case DRW::CIRCLE: return SourceGeometryKind::Circle;
        case DRW::ELLIPSE: return SourceGeometryKind::Ellipse;
        case DRW::POLYLINE:
        case DRW::LWPOLYLINE: return SourceGeometryKind::Polyline;
        case DRW::SPLINE: return SourceGeometryKind::Spline;
        case DRW::POINT: return SourceGeometryKind::Point;
        default: return SourceGeometryKind::Unknown;
        }
    }

    bool semanticallyClosed(const CadItem& item)
    {
        if (item.m_nativeEntity == nullptr)
        {
            return false;
        }
        switch (item.m_type)
        {
        case DRW::CIRCLE:
            return true;
        case DRW::ELLIPSE:
        {
            const auto* ellipse = static_cast<const DRW_Ellipse*>(item.m_nativeEntity);
            return CadEllipseGeometryUtils::isFullEllipseParameterRange
                (ellipse->staparam, ellipse->endparam);
        }
        case DRW::LWPOLYLINE:
            return (static_cast<const DRW_LWPolyline*>(item.m_nativeEntity)->flags & 1) != 0;
        case DRW::POLYLINE:
            return (static_cast<const DRW_Polyline*>(item.m_nativeEntity)->flags & 1) != 0;
        case DRW::SPLINE:
            return (static_cast<const DRW_Spline*>(item.m_nativeEntity)->flags & 3) != 0;
        default:
            return false;
        }
    }

    double legacyDistance(const QVector3D& left, const QVector3D& right)
    {
        return static_cast<double>((left - right).length());
    }

    Vector3d toCore(const QVector3D& point)
    {
        return
        {
            static_cast<double>(point.x()),
            static_cast<double>(point.y()),
            static_cast<double>(point.z())
        };
    }
}

OperationResult<cadcam::topology::TopologyInput>
LegacyCadItemTopologyAdapter::convert
(
    const QVector<CadItem*>& items,
    const cadcam::topology::PathTopologyTolerance& tolerance,
    const OperationContext& context
) const
{
    OperationResult<TopologyInput> result;
    TopologyInput input;
    input.contentRevision = 1U;
    input.records.reserve(static_cast<std::size_t>(items.size()));
    std::set<EntityId> entityIds;

    for (int index = 0; index < items.size(); ++index)
    {
        CadItem* item = items[index];
        const std::size_t sourceIndex = static_cast<std::size_t>(index);
        if (item == nullptr)
        {
            input.diagnostics.push_back(adapterDiagnostic
            (
                context, DiagnosticCode::LegacyTopologyAdapterFailure,
                QStringLiteral("CadItem pointer is null"), sourceIndex, 0U, -1, 0U, tolerance
            ));
            continue;
        }
        if (item->m_entityId == 0U)
        {
            input.diagnostics.push_back(adapterDiagnostic
            (
                context, DiagnosticCode::LegacyTopologyAdapterFailure,
                QStringLiteral("CadItem has zero EntityId"), sourceIndex, 0U,
                static_cast<int>(item->m_type), 0U, tolerance
            ));
            continue;
        }
        if (!entityIds.insert(item->m_entityId).second)
        {
            input.diagnostics.push_back(adapterDiagnostic
            (
                context, DiagnosticCode::DuplicateTopologyEntityId,
                QStringLiteral("CadItem EntityId is duplicated"), sourceIndex, item->m_entityId,
                static_cast<int>(item->m_type), 0U, tolerance
            ));
            continue;
        }

        item->rebuildRawPathPoints3D();
        const std::vector<RawPathPoint3D>& rawPath = item->rawPathPoints3D();
        std::vector<QVector3D> cleanPath;
        cleanPath.reserve(rawPath.size() + 1U);
        for (const RawPathPoint3D& rawPoint : rawPath)
        {
            const QVector3D point
            (
                static_cast<float>(rawPoint.x),
                static_cast<float>(rawPoint.y),
                static_cast<float>(rawPoint.z)
            );
            if (cleanPath.empty()
                || legacyDistance(cleanPath.back(), point) > tolerance.minimumEdgeLength)
            {
                cleanPath.push_back(point);
            }
        }
        if (cleanPath.size() < 2U)
        {
            input.diagnostics.push_back(adapterDiagnostic
            (
                context, DiagnosticCode::LegacyTopologyAdapterFailure,
                QStringLiteral("CadItem process path contains fewer than two distinct points"),
                sourceIndex, item->m_entityId, static_cast<int>(item->m_type),
                cleanPath.size(), tolerance
            ));
            continue;
        }

        TopologyPathRecord record;
        record.sourceIndex = sourceIndex;
        record.entityId = item->m_entityId;
        record.sourceKind = sourceKind(item->m_type);
        record.semanticallyClosed = semanticallyClosed(*item);
        if (record.semanticallyClosed && cleanPath.size() >= 3U
            && legacyDistance(cleanPath.front(), cleanPath.back()) > tolerance.minimumEdgeLength)
        {
            cleanPath.push_back(cleanPath.front());
        }
        record.points.reserve(cleanPath.size());
        for (const QVector3D& point : cleanPath)
        {
            record.points.push_back(toCore(point));
        }
        input.records.push_back(std::move(record));
    }

    if (!input.diagnostics.isEmpty())
    {
        result.status = OperationStatus::InvalidInput;
        result.diagnostics = input.diagnostics;
        return result;
    }
    result.status = OperationStatus::Success;
    result.value = std::move(input);
    return result;
}
