#include "application/topology/GeometrySnapshotTopologyAdapter.h"

#include <algorithm>
#include <cmath>
#include <set>

namespace
{
    double distance3D
    (
        const cadcam::geometry::Vector3d& left,
        const cadcam::geometry::Vector3d& right
    )
    {
        const double dx = left.x - right.x;
        const double dy = left.y - right.y;
        const double dz = left.z - right.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    Diagnostic adapterDiagnostic
    (
        const OperationContext& context,
        DiagnosticCode code,
        DiagnosticSeverity severity,
        const QString& detail,
        std::size_t recordCount,
        const cadcam::topology::PathTopologyTolerance& tolerance
    )
    {
        Diagnostic diagnostic;
        diagnostic.code = code;
        diagnostic.severity = severity;
        diagnostic.component = QStringLiteral("GeometrySnapshotTopologyAdapter");
        diagnostic.operation = QStringLiteral("AdaptGeometrySnapshotTopology");
        diagnostic.stage = QStringLiteral("CopyPathRecords");
        diagnostic.userMessage = code == DiagnosticCode::TopologyPathUnavailable
            ? QStringLiteral("部分图元没有可用的拓扑路径。")
            : QStringLiteral("几何快照无法转换为拓扑输入。");
        diagnostic.technicalDetail = detail;
        diagnostic.correlationId = context.correlationId;
        diagnostic.context.insert(QStringLiteral("entityId"), static_cast<qulonglong>(0U));
        diagnostic.context.insert(QStringLiteral("sourceIndex"), static_cast<qulonglong>(0U));
        diagnostic.context.insert
            (QStringLiteral("recordCount"), static_cast<qulonglong>(recordCount));
        diagnostic.context.insert(QStringLiteral("nodeCount"), static_cast<qulonglong>(0U));
        diagnostic.context.insert(QStringLiteral("edgeCount"), static_cast<qulonglong>(0U));
        diagnostic.context.insert(QStringLiteral("componentCount"), 0);
        diagnostic.context.insert(QStringLiteral("openNodeCount"), 0);
        diagnostic.context.insert(QStringLiteral("branchNodeCount"), 0);
        diagnostic.context.insert(QStringLiteral("closureGap"), 0.0);
        diagnostic.context.insert(QStringLiteral("nodeSnap"), tolerance.nodeSnap);
        diagnostic.context.insert
            (QStringLiteral("intersectionTolerance"), tolerance.intersection);
        return diagnostic;
    }
}

OperationResult<cadcam::topology::TopologyInput>
GeometrySnapshotTopologyAdapter::convert
(
    const GeometrySnapshot& snapshot,
    const std::vector<cadcam::geometry::EntityId>& subset,
    const cadcam::topology::PathTopologyTolerance& tolerance,
    const OperationContext& context
) const
{
    using cadcam::geometry::EntityId;
    using cadcam::topology::TopologyInput;
    using cadcam::topology::TopologyPathRecord;

    OperationResult<TopologyInput> result;
    if (snapshot.contentRevision == 0U
        || tolerance.minimumEdgeLength < 0.0
        || !std::isfinite(tolerance.minimumEdgeLength))
    {
        result.status = OperationStatus::InvalidInput;
        result.addDiagnostic(adapterDiagnostic
        (
            context,
            DiagnosticCode::TopologyInputInvalid,
            DiagnosticSeverity::Error,
            QStringLiteral("snapshot revision or minimum edge length is invalid"),
            0U,
            tolerance
        ));
        return result;
    }

    const std::set<EntityId> requested(subset.begin(), subset.end());
    std::set<EntityId> found;
    TopologyInput input;
    input.contentRevision = snapshot.contentRevision;
    bool hasFailure = false;

    std::vector<const GeometrySnapshotEntry*> orderedEntries;
    orderedEntries.reserve(snapshot.entries.size());
    for (const GeometrySnapshotEntry& entry : snapshot.entries)
    {
        orderedEntries.push_back(&entry);
    }
    std::stable_sort
    (
        orderedEntries.begin(), orderedEntries.end(),
        [](const GeometrySnapshotEntry* left, const GeometrySnapshotEntry* right)
        {
            return left->sourceIndex < right->sourceIndex;
        }
    );

    for (const GeometrySnapshotEntry* entry : orderedEntries)
    {
        if (!requested.empty() && requested.count(entry->attributes.entityId) == 0U)
        {
            continue;
        }
        found.insert(entry->attributes.entityId);

        if (!entry->path.has_value()
            || (entry->status != OperationStatus::Success
                && entry->status != OperationStatus::PartialSuccess))
        {
            input.diagnostics += entry->diagnostics;
            Diagnostic diagnostic = adapterDiagnostic
            (
                context,
                DiagnosticCode::TopologyPathUnavailable,
                DiagnosticSeverity::Warning,
                QStringLiteral("snapshot entry has no successful Path3D"),
                input.records.size(),
                tolerance
            );
            diagnostic.entityId = entry->attributes.entityId;
            diagnostic.context.insert(QStringLiteral("entityId"),
                static_cast<qulonglong>(entry->attributes.entityId));
            diagnostic.context.insert
                (QStringLiteral("sourceIndex"), static_cast<qulonglong>(entry->sourceIndex));
            input.diagnostics.push_back(diagnostic);
            hasFailure = true;
            continue;
        }

        TopologyPathRecord record;
        record.sourceIndex = entry->sourceIndex;
        record.entityId = entry->attributes.entityId;
        record.sourceKind = entry->sourceKind;
        record.semanticallyClosed = entry->path->closed;
        record.points.reserve(entry->path->vertices.size() + (record.semanticallyClosed ? 1U : 0U));
        for (const cadcam::geometry::PathVertex3D& vertex : entry->path->vertices)
        {
            if (record.points.empty()
                || distance3D(record.points.back(), vertex.position) > tolerance.minimumEdgeLength)
            {
                record.points.push_back(vertex.position);
            }
        }
        if (record.semanticallyClosed && record.points.size() >= 3U
            && distance3D(record.points.front(), record.points.back()) > tolerance.minimumEdgeLength)
        {
            record.points.push_back(record.points.front());
        }
        input.records.push_back(std::move(record));
    }

    for (EntityId entityId : requested)
    {
        if (found.count(entityId) != 0U)
        {
            continue;
        }
        Diagnostic diagnostic = adapterDiagnostic
        (
            context,
            DiagnosticCode::TopologyPathUnavailable,
            DiagnosticSeverity::Warning,
            QStringLiteral("requested EntityId is absent from snapshot"),
            input.records.size(),
            tolerance
        );
        diagnostic.entityId = entityId;
        diagnostic.context.insert
            (QStringLiteral("entityId"), static_cast<qulonglong>(entityId));
        input.diagnostics.push_back(diagnostic);
        hasFailure = true;
    }

    result.status = hasFailure ? OperationStatus::PartialSuccess : OperationStatus::Success;
    result.diagnostics = input.diagnostics;
    result.value = std::move(input);
    return result;
}
