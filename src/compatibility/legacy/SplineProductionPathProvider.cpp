#include "compatibility/legacy/SplineProductionPathProvider.h"

#include "CadSplineConverter.h"
#include "core/geometry/NurbsCurveEvaluator.h"
#include "infrastructure/dxf/DxfGeometryAdapter.h"

#include <libdxfrw.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
    using namespace cadcam::geometry;

    double squaredDistance(const Vector3d& left, const Vector3d& right)
    {
        const double dx = left.x - right.x;
        const double dy = left.y - right.y;
        const double dz = left.z - right.z;
        return dx * dx + dy * dy + dz * dz;
    }

    double pointToChordDistance
    (
        const Vector3d& point,
        const Vector3d& start,
        const Vector3d& end
    )
    {
        const Vector3d segment
            { end.x - start.x, end.y - start.y, end.z - start.z };
        const double lengthSquared = segment.x * segment.x
            + segment.y * segment.y + segment.z * segment.z;
        if (lengthSquared <= (std::numeric_limits<double>::epsilon)())
        {
            return std::sqrt(squaredDistance(point, start));
        }
        const Vector3d offset
            { point.x - start.x, point.y - start.y, point.z - start.z };
        const double factor = std::clamp
        (
            (offset.x * segment.x + offset.y * segment.y + offset.z * segment.z)
                / lengthSquared,
            0.0,
            1.0
        );
        const Vector3d projection
        {
            start.x + segment.x * factor,
            start.y + segment.y * factor,
            start.z + segment.z * factor
        };
        return std::sqrt(squaredDistance(point, projection));
    }

    Diagnostic productionDiagnostic
    (
        EntityId entityId,
        const OperationContext& context,
        DiagnosticCode code,
        DiagnosticSeverity severity,
        const QString& detail
    )
    {
        Diagnostic diagnostic;
        diagnostic.code = code;
        diagnostic.severity = severity;
        diagnostic.component = QStringLiteral("SplineProductionPathProvider");
        diagnostic.operation = QStringLiteral("BuildSplineProductionPath");
        diagnostic.stage = QStringLiteral("CompileOrFallback");
        diagnostic.userMessage = code == DiagnosticCode::SplineLegacyFallbackUsed
            ? QStringLiteral("样条曲线已使用旧版兼容路径。")
            : QStringLiteral("样条曲线生产路径校验失败。");
        diagnostic.technicalDetail = detail;
        diagnostic.correlationId = context.correlationId;
        diagnostic.entityId = entityId;
        diagnostic.context.insert
            (QStringLiteral("entityId"), static_cast<qulonglong>(entityId));
        return diagnostic;
    }

    bool validateCompiledPath
    (
        const SourceEntity& source,
        const Path3D& path,
        const PathCompileOptions& options,
        const OperationContext& context,
        QString& detail
    )
    {
        const SplineGeometry& spline = std::get<SplineGeometry>(source.geometry);
        const bool expectedClosed = spline.closed || spline.periodic;
        if (path.closed != expectedClosed || path.vertices.size() < 2U
            || !std::isfinite(path.samplingTolerance)
            || path.samplingTolerance <= 0.0)
        {
            detail = QStringLiteral("closed state, point count or sampling tolerance is invalid");
            return false;
        }

        NurbsCurveEvaluator evaluator;
        const double expectedStartParameter = options.reverse
            ? spline.parameterEnd : spline.parameterStart;
        const double expectedEndParameter = options.reverse
            ? spline.parameterStart : spline.parameterEnd;
        const OperationResult<Vector3d> expectedStart =
            evaluator.evaluate(spline, expectedStartParameter, context);
        const OperationResult<Vector3d> expectedEnd =
            evaluator.evaluate(spline, expectedEndParameter, context);
        if (!expectedStart.succeeded() || !expectedEnd.succeeded()
            || squaredDistance(path.vertices.front().position, *expectedStart.value)
                > path.samplingTolerance * path.samplingTolerance)
        {
            detail = QStringLiteral("compiled path start point differs from the exact NURBS endpoint");
            return false;
        }
        if (!path.closed
            && squaredDistance(path.vertices.back().position, *expectedEnd.value)
                > path.samplingTolerance * path.samplingTolerance)
        {
            detail = QStringLiteral("compiled path end point differs from the exact NURBS endpoint");
            return false;
        }

        const double allowedError = path.samplingTolerance * 1.01 + 1.0e-12;
        for (std::size_t index = 1U; index < path.vertices.size(); ++index)
        {
            const PathVertex3D& left = path.vertices[index - 1U];
            const PathVertex3D& right = path.vertices[index];
            const double middleParameter =
                (left.sourceParameter + right.sourceParameter) * 0.5;
            const OperationResult<Vector3d> middle =
                evaluator.evaluate(spline, middleParameter, context);
            if (!middle.succeeded() || !middle.value.has_value()
                || pointToChordDistance
                    (*middle.value, left.position, right.position) > allowedError)
            {
                detail = QStringLiteral("compiled path exceeds the configured chord tolerance");
                return false;
            }
        }
        return true;
    }

    OperationResult<Path3D> buildLegacyFallback
    (
        EntityId entityId,
        const DRW_Spline& spline,
        const PathCompileOptions& options,
        const OperationContext& context
    )
    {
        OperationResult<Path3D> result;
        std::unique_ptr<DRW_Polyline> polyline = convertSplineToPolyline(&spline);
        if (!polyline)
        {
            result.status = OperationStatus::Failed;
            result.addDiagnostic(productionDiagnostic
            (
                entityId, context, DiagnosticCode::SplineProductionCompileFailure,
                DiagnosticSeverity::Error,
                QStringLiteral("legacy converter did not produce a polyline")
            ));
            return result;
        }

        OperationResult<SourceEntity> source = DxfGeometryAdapter::convert
            (entityId, *polyline, context);
        if (!source.succeeded() || !source.value.has_value())
        {
            result.status = source.status;
            result.mergeDiagnostics(source);
            return result;
        }

        SamplingPolicy legacyPolicy;
        legacyPolicy.chordTolerance = 0.0;
        legacyPolicy.singlePrecisionEvaluation = true;
        legacyPolicy.minimumSegments = 1;
        legacyPolicy.minimumBulgeSegments = 4;
        legacyPolicy.fullTurnSegments = 128;
        PathCompileOptions legacyOptions;
        legacyOptions.reverse = options.reverse;
        GeometryCompiler compiler;
        result = compiler.compile(*source.value, legacyPolicy, legacyOptions, context);
        if (!result.succeeded() || !result.value.has_value())
        {
            return result;
        }
        result.value->sourceKind = SourceGeometryKind::Spline;
        result.status = OperationStatus::PartialSuccess;
        result.addDiagnostic(productionDiagnostic
        (
            entityId, context, DiagnosticCode::SplineLegacyFallbackUsed,
            DiagnosticSeverity::Warning,
            QStringLiteral("DxfGeometryAdapter or GeometryCompiler path was unavailable")
        ));
        return result;
    }
}

OperationResult<cadcam::geometry::Path3D> SplineProductionPathProvider::build
(
    cadcam::geometry::EntityId entityId,
    const DRW_Spline& spline,
    const cadcam::geometry::SamplingPolicy& policy,
    const cadcam::geometry::PathCompileOptions& options,
    const OperationContext& context
)
{
    using namespace cadcam::geometry;

    // CadItem constructors build their display cache before CadDocument assigns
    // the persistent entity id. Geometry Core still requires a non-zero source id.
    const EntityId compileEntityId = entityId == 0U ? 1U : entityId;

    OperationResult<SourceEntity> source = DxfGeometryAdapter::convert
        (compileEntityId, spline, context);
    if (source.succeeded() && source.value.has_value())
    {
        GeometryCompiler compiler;
        OperationResult<Path3D> compiled =
            compiler.compile(*source.value, policy, options, context);
        compiled.mergeDiagnostics(source);
        if (compiled.succeeded() && compiled.value.has_value())
        {
            const SplineGeometry& geometry =
                std::get<SplineGeometry>(source.value->geometry);
            const bool exactDataValid = geometry.degree >= 1
                && geometry.controlPoints.size()
                    > static_cast<std::size_t>(geometry.degree)
                && geometry.knots.size() >= geometry.controlPoints.size()
                    + static_cast<std::size_t>(geometry.degree) + 1U;
            QString validationDetail;
            if (!exactDataValid
                || validateCompiledPath
                    (*source.value, *compiled.value, options, context, validationDetail))
            {
                return compiled;
            }
            compiled.addDiagnostic(productionDiagnostic
            (
                entityId, context, DiagnosticCode::SplineLegacyParityMismatch,
                DiagnosticSeverity::Warning, validationDetail
            ));
        }
    }

    OperationResult<Path3D> fallback =
        buildLegacyFallback(compileEntityId, spline, options, context);
    if (fallback.succeeded())
    {
        fallback.addDiagnostic(productionDiagnostic
        (
            entityId, context, DiagnosticCode::SplineProductionCompileFailure,
            DiagnosticSeverity::Warning,
            QStringLiteral("new spline production path failed validation")
        ));
    }
    return fallback;
}
