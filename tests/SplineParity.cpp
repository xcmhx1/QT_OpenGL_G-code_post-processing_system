#include "SplineParity.h"

#include "CadPolylineItem.h"
#include "CadSplineConverter.h"
#include "compatibility/legacy/LegacyCadItemPathBridge.h"
#include "core/geometry/GeometryCompiler.h"
#include "infrastructure/dxf/DxfGeometryAdapter.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

namespace
{
    cadcam::geometry::Vector3d quantize(const cadcam::geometry::Vector3d& point)
    {
        return
        {
            static_cast<double>(static_cast<float>(point.x)),
            static_cast<double>(static_cast<float>(point.y)),
            static_cast<double>(static_cast<float>(point.z))
        };
    }

    cadcam::geometry::Vector3d quantize(const RawPathPoint3D& point)
    {
        return quantize(cadcam::geometry::Vector3d{ point.x, point.y, point.z });
    }

    double pointDistance
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

    OperationContext parityContext(const QString& operation)
    {
        return { QStringLiteral("spline-parity"), operation };
    }
}

SplineParityReport compareSplineWithLegacy(const DRW_Spline& spline)
{
    using namespace cadcam::geometry;

    SplineParityReport report;
    std::unique_ptr<DRW_Polyline> legacyPolyline = convertSplineToPolyline(&spline);
    if (!legacyPolyline)
    {
        Diagnostic diagnostic;
        diagnostic.code = DiagnosticCode::InvalidSpline;
        diagnostic.severity = DiagnosticSeverity::Error;
        diagnostic.component = QStringLiteral("SplineParity");
        diagnostic.operation = QStringLiteral("CompareSpline");
        diagnostic.stage = QStringLiteral("LegacyConversion");
        diagnostic.userMessage = QStringLiteral("旧样条转换未生成多段线。");
        diagnostic.technicalDetail = QStringLiteral("convertSplineToPolyline returned nullptr");
        report.diagnostics.push_back(diagnostic);
        return report;
    }

    CadPolylineItem legacyItem(legacyPolyline.get());
    legacyItem.m_entityId = 1;
    const OperationResult<Path3D> legacyPathResult = LegacyCadItemPathBridge::compile
    (
        legacyItem,
        LegacyCadItemPathBridge::legacySamplingPolicy(legacyItem),
        {},
        parityContext(QStringLiteral("compile-legacy-polyline"))
    );
    report.diagnostics += legacyPathResult.diagnostics;
    if (!legacyPathResult.succeeded() || !legacyPathResult.value.has_value())
    {
        return report;
    }
    std::vector<RawPathPoint3D> legacyRaw;
    LegacyCadItemPathBridge::copyToLegacyRawPath(*legacyPathResult.value, legacyRaw);

    const OperationResult<SourceEntity> source = DxfGeometryAdapter::convert
    (
        2,
        spline,
        parityContext(QStringLiteral("adapt-core-spline"))
    );
    report.diagnostics += source.diagnostics;
    if (!source.succeeded() || !source.value.has_value())
    {
        return report;
    }

    GeometryCompiler compiler;
    SamplingPolicy samplingPolicy;
    const OperationResult<Path3D> corePathResult = compiler.compile
    (
        *source.value,
        samplingPolicy,
        {},
        parityContext(QStringLiteral("compile-core-spline"))
    );
    report.diagnostics += corePathResult.diagnostics;
    if (!corePathResult.succeeded() || !corePathResult.value.has_value())
    {
        return report;
    }

    std::vector<Vector3d> legacyPoints;
    legacyPoints.reserve(legacyRaw.size());
    for (const RawPathPoint3D& point : legacyRaw)
    {
        legacyPoints.push_back(quantize(point));
    }
    std::vector<Vector3d> corePoints;
    corePoints.reserve
        (corePathResult.value->vertices.size() + (corePathResult.value->closed ? 1U : 0U));
    for (const PathVertex3D& vertex : corePathResult.value->vertices)
    {
        corePoints.push_back(quantize(vertex.position));
    }
    if (corePathResult.value->closed && !corePoints.empty())
    {
        corePoints.push_back(corePoints.front());
    }

    report.legacyPointCount = legacyPoints.size();
    report.corePointCount = corePoints.size();
    const std::size_t comparedCount = std::min(legacyPoints.size(), corePoints.size());
    for (std::size_t index = 0; index < comparedCount; ++index)
    {
        const double difference = pointDistance(legacyPoints[index], corePoints[index]);
        report.maximumPointDistance = std::max(report.maximumPointDistance, difference);
        if (difference > 0.0 && report.firstDifferentIndex < 0)
        {
            report.firstDifferentIndex = static_cast<int>(index);
        }
    }
    if (legacyPoints.size() != corePoints.size() && report.firstDifferentIndex < 0)
    {
        report.firstDifferentIndex = static_cast<int>(comparedCount);
    }

    report.equivalent = legacyPoints.size() == corePoints.size()
        && legacyPathResult.value->closed == corePathResult.value->closed
        && report.firstDifferentIndex < 0;
    return report;
}
