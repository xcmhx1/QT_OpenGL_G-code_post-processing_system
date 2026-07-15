#include "compatibility/legacy/GeometrySnapshotParityVerifier.h"

#include "cad/document/CadDocument.h"
#include "cad/items/CadItem.h"
#include "compatibility/legacy/LegacyCadItemPathBridge.h"

#include <QThread>

#include <algorithm>
#include <cmath>

namespace
{
    cadcam::geometry::Vector3d quantized
    (const cadcam::geometry::Vector3d& point)
    {
        return
        {
            static_cast<double>(static_cast<float>(point.x)),
            static_cast<double>(static_cast<float>(point.y)),
            static_cast<double>(static_cast<float>(point.z))
        };
    }

    cadcam::geometry::Vector3d quantized(const RawPathPoint3D& point)
    {
        return quantized(cadcam::geometry::Vector3d{ point.x, point.y, point.z });
    }

    double distance
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

    double pathLength
    (
        const std::vector<cadcam::geometry::Vector3d>& points,
        bool closed
    )
    {
        double length = 0.0;
        for (std::size_t index = 1U; index < points.size(); ++index)
        {
            length += distance(points[index - 1U], points[index]);
        }
        if (closed && points.size() > 1U)
        {
            length += distance(points.back(), points.front());
        }
        return length;
    }

    Diagnostic parityDiagnostic
    (
        const OperationContext& context,
        const GeometrySnapshotParityEntry& entry,
        const QString& detail
    )
    {
        Diagnostic diagnostic;
        diagnostic.code = DiagnosticCode::OutputVerificationFailure;
        diagnostic.severity = DiagnosticSeverity::Warning;
        diagnostic.component = QStringLiteral("GeometrySnapshotParityVerifier");
        diagnostic.operation = QStringLiteral("VerifyGeometrySnapshotParity");
        diagnostic.stage = QStringLiteral("CompareCanonicalPaths");
        diagnostic.userMessage = QStringLiteral("几何快照与旧兼容路径不一致。");
        diagnostic.technicalDetail = detail;
        diagnostic.correlationId = context.correlationId;
        diagnostic.entityId = entry.entityId;
        diagnostic.context.insert
            (QStringLiteral("sourceIndex"), static_cast<qulonglong>(entry.sourceIndex));
        return diagnostic;
    }
}

OperationResult<GeometrySnapshotParityReport> GeometrySnapshotParityVerifier::verify
(
    const CadDocument& document,
    const GeometrySnapshot& snapshot,
    const OperationContext& context
) const
{
    OperationResult<GeometrySnapshotParityReport> result;
    if (QThread::currentThread() != document.thread())
    {
        result.status = OperationStatus::Failed;
        GeometrySnapshotParityEntry entry;
        result.addDiagnostic(parityDiagnostic
        (
            context,
            entry,
            QStringLiteral("parity verification must run on the document thread")
        ));
        return result;
    }

    GeometrySnapshotParityReport report;
    report.equivalent = true;
    report.entries.reserve(snapshot.entries.size());
    for (const GeometrySnapshotEntry& snapshotEntry : snapshot.entries)
    {
        GeometrySnapshotParityEntry entry;
        entry.sourceIndex = snapshotEntry.sourceIndex;
        entry.entityId = snapshotEntry.attributes.entityId;
        const auto itemIterator = std::find_if
        (
            document.m_entities.cbegin(),
            document.m_entities.cend(),
            [&entry](const std::unique_ptr<CadItem>& item)
            {
                return item != nullptr && item->m_entityId == entry.entityId;
            }
        );
        if (itemIterator == document.m_entities.cend() || !snapshotEntry.path.has_value())
        {
            report.equivalent = false;
            report.diagnostics.push_back(parityDiagnostic
            (
                context,
                entry,
                QStringLiteral("document item or snapshot path is unavailable")
            ));
            report.entries.push_back(entry);
            continue;
        }

        cadcam::geometry::PathCompileOptions options;
        options.reverse = false;
        options.startParameter = std::nullopt;
        const OperationResult<cadcam::geometry::Path3D> legacy =
            LegacyCadItemPathBridge::compile
            (
                **itemIterator,
                snapshot.samplingPolicy,
                options,
                context
            );
        if (!legacy.succeeded() || !legacy.value.has_value())
        {
            report.equivalent = false;
            report.diagnostics += legacy.diagnostics;
            report.diagnostics.push_back(parityDiagnostic
            (
                context,
                entry,
                QStringLiteral("legacy compatibility bridge returned no path")
            ));
            report.entries.push_back(entry);
            continue;
        }

        std::vector<RawPathPoint3D> legacyRaw;
        LegacyCadItemPathBridge::copyToLegacyRawPath(*legacy.value, legacyRaw);
        if (legacy.value->closed && legacyRaw.size() > 1U
            && distance(quantized(legacyRaw.front()), quantized(legacyRaw.back())) <= 1.0e-9)
        {
            legacyRaw.pop_back();
        }

        std::vector<cadcam::geometry::Vector3d> legacyPoints;
        legacyPoints.reserve(legacyRaw.size());
        for (const RawPathPoint3D& point : legacyRaw)
        {
            legacyPoints.push_back(quantized(point));
        }
        std::vector<cadcam::geometry::Vector3d> snapshotPoints;
        snapshotPoints.reserve(snapshotEntry.path->vertices.size());
        for (const cadcam::geometry::PathVertex3D& point : snapshotEntry.path->vertices)
        {
            snapshotPoints.push_back(quantized(point.position));
        }

        entry.legacyPointCount = legacyPoints.size();
        entry.snapshotPointCount = snapshotPoints.size();
        entry.closedMatches = legacy.value->closed == snapshotEntry.path->closed;
        entry.legacyPathLength = pathLength(legacyPoints, legacy.value->closed);
        entry.snapshotPathLength = pathLength(snapshotPoints, snapshotEntry.path->closed);
        const std::size_t comparedCount = std::min
            (legacyPoints.size(), snapshotPoints.size());
        for (std::size_t index = 0; index < comparedCount; ++index)
        {
            const double pointDistance = distance(legacyPoints[index], snapshotPoints[index]);
            entry.maximumPointDistance = std::max
                (entry.maximumPointDistance, pointDistance);
            if (pointDistance > 1.0e-6 && entry.firstDifferentIndex < 0)
            {
                entry.firstDifferentIndex = static_cast<int>(index);
            }
        }
        if (legacyPoints.size() != snapshotPoints.size()
            && entry.firstDifferentIndex < 0)
        {
            entry.firstDifferentIndex = static_cast<int>(comparedCount);
        }
        entry.equivalent = entry.entityId == snapshotEntry.path->sourceEntityId
            && legacy.value->sourceKind == snapshotEntry.path->sourceKind
            && entry.closedMatches
            && entry.legacyPointCount == entry.snapshotPointCount
            && entry.firstDifferentIndex < 0
            && std::abs(entry.legacyPathLength - entry.snapshotPathLength) <= 1.0e-6;
        report.maximumPointDistance = std::max
            (report.maximumPointDistance, entry.maximumPointDistance);
        if (!entry.equivalent)
        {
            report.equivalent = false;
            report.diagnostics.push_back(parityDiagnostic
            (
                context,
                entry,
                QStringLiteral("entity type, closure, point sequence or path length differs")
            ));
        }
        report.entries.push_back(entry);
    }

    report.diagnostics.squeeze();
    result.status = report.equivalent
        ? OperationStatus::Success : OperationStatus::PartialSuccess;
    result.diagnostics = report.diagnostics;
    result.value = std::move(report);
    return result;
}
