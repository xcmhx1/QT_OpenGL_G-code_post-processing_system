#include "core/geometry/GeometryCompiler.h"
#include "core/geometry/LocalCoordinateFrame.h"
#include "core/geometry/NurbsCurveEvaluator.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <type_traits>

namespace cadcam::geometry
{
    namespace
    {
        constexpr double kHalfPi = 1.57079632679489661923;
        constexpr double kTwoPi = 6.28318530717958647692;
        constexpr double kGeometryTolerance = 1.0e-12;
        constexpr double kPolylineContinuityTolerance = 1.0e-5;

        double squaredLength(const Vector3d& vector)
        {
            return vector.x * vector.x + vector.y * vector.y + vector.z * vector.z;
        }

        Vector3d addScaled
        (
            const Vector3d& origin,
            const Vector3d& axisU,
            double scaleU,
            const Vector3d& axisV,
            double scaleV
        )
        {
            const Vector3d localOffset
            {
                axisU.x * scaleU + axisV.x * scaleV,
                axisU.y * scaleU + axisV.y * scaleV,
                axisU.z * scaleU + axisV.z * scaleV
            };
            return
            {
                origin.x + localOffset.x,
                origin.y + localOffset.y,
                origin.z + localOffset.z
            };
        }

        bool isFinite(double value)
        {
            return std::isfinite(value);
        }

        bool isFinite(const Vector3d& value)
        {
            return isFinite(value.x) && isFinite(value.y) && isFinite(value.z);
        }

        double squaredDistance(const Vector3d& left, const Vector3d& right)
        {
            return squaredLength
            ({
                left.x - right.x,
                left.y - right.y,
                left.z - right.z
            });
        }

        Vector3d arcPoint(const ArcGeometry& arc, double parameter)
        {
            return addScaled
            (
                arc.center,
                arc.axisU,
                std::cos(parameter) * arc.radius,
                arc.axisV,
                std::sin(parameter) * arc.radius
            );
        }

        Vector3d primitiveStart(const PolylinePrimitive& primitive)
        {
            return std::visit
            (
                [](const auto& geometry)
                {
                    using Geometry = std::decay_t<decltype(geometry)>;
                    if constexpr (std::is_same_v<Geometry, LineGeometry>)
                    {
                        return geometry.start;
                    }
                    else
                    {
                        return arcPoint(geometry, geometry.startParameter);
                    }
                },
                primitive
            );
        }

        Vector3d primitiveEnd(const PolylinePrimitive& primitive)
        {
            return std::visit
            (
                [](const auto& geometry)
                {
                    using Geometry = std::decay_t<decltype(geometry)>;
                    if constexpr (std::is_same_v<Geometry, LineGeometry>)
                    {
                        return geometry.end;
                    }
                    else
                    {
                        return arcPoint(geometry, geometry.endParameter);
                    }
                },
                primitive
            );
        }

        bool validPolylineGeometry(const PolylineGeometry& polyline)
        {
            if (polyline.sourceVertexCount < 2U || polyline.segments.empty())
            {
                return false;
            }
            const std::size_t expectedSegmentCount = polyline.closed
                ? polyline.sourceVertexCount
                : polyline.sourceVertexCount - 1U;
            if (polyline.segments.size() != expectedSegmentCount)
            {
                return false;
            }

            for (std::size_t index = 0; index < polyline.segments.size(); ++index)
            {
                bool primitiveValid = true;
                std::visit
                (
                    [&](const auto& geometry)
                    {
                        using Geometry = std::decay_t<decltype(geometry)>;
                        if constexpr (std::is_same_v<Geometry, LineGeometry>)
                        {
                            primitiveValid = isFinite(geometry.start)
                                && isFinite(geometry.end)
                                && squaredDistance(geometry.start, geometry.end)
                                    > kGeometryTolerance * kGeometryTolerance;
                        }
                        else
                        {
                            primitiveValid = isFinite(geometry.center)
                                && isFinite(geometry.axisU)
                                && isFinite(geometry.axisV)
                                && isFinite(geometry.radius)
                                && isFinite(geometry.startParameter)
                                && isFinite(geometry.endParameter)
                                && geometry.radius > kGeometryTolerance
                                && squaredLength(geometry.axisU)
                                    > kGeometryTolerance * kGeometryTolerance
                                && squaredLength(geometry.axisV)
                                    > kGeometryTolerance * kGeometryTolerance
                                && std::abs(geometry.endParameter - geometry.startParameter)
                                    > kGeometryTolerance;
                        }
                    },
                    polyline.segments[index]
                );
                if (!primitiveValid)
                {
                    return false;
                }

                if (index > 0U
                    && squaredDistance
                    (
                        primitiveEnd(polyline.segments[index - 1U]),
                        primitiveStart(polyline.segments[index])
                    ) > kPolylineContinuityTolerance * kPolylineContinuityTolerance)
                {
                    return false;
                }
            }

            return !polyline.closed
                || squaredDistance
                (
                    primitiveEnd(polyline.segments.back()),
                    primitiveStart(polyline.segments.front())
                ) <= kPolylineContinuityTolerance * kPolylineContinuityTolerance;
        }

        Diagnostic makeCompileDiagnostic
        (
            const SourceEntity& source,
            const OperationContext& context,
            DiagnosticCode code,
            const QString& stage,
            const QString& userMessage,
            const QString& technicalDetail,
            const QVariantMap& diagnosticContext = QVariantMap()
        )
        {
            Diagnostic diagnostic;
            diagnostic.code = code;
            diagnostic.severity = DiagnosticSeverity::Error;
            diagnostic.component = QStringLiteral("GeometryCompiler");
            diagnostic.operation = QStringLiteral("CompilePath3D");
            diagnostic.stage = stage;
            diagnostic.userMessage = userMessage;
            diagnostic.technicalDetail = technicalDetail;
            diagnostic.correlationId = context.correlationId;
            diagnostic.entityId = source.id;
            diagnostic.context = diagnosticContext;
            diagnostic.context.insert
            (
                QStringLiteral("sourceKind"),
                QString::fromLatin1(sourceGeometryKindName(source.kind))
            );
            return diagnostic;
        }

        QVariantMap splineCompileContext
        (
            const SourceEntity& source,
            const SplineGeometry& spline,
            int subdivisionDepth,
            std::size_t generatedPointCount
        )
        {
            return
            {
                { QStringLiteral("entityId"), static_cast<qulonglong>(source.id) },
                { QStringLiteral("degree"), spline.degree },
                { QStringLiteral("controlPointCount"),
                    static_cast<qulonglong>(spline.controlPoints.size()) },
                { QStringLiteral("knotCount"), static_cast<qulonglong>(spline.knots.size()) },
                { QStringLiteral("weightCount"), static_cast<qulonglong>(spline.weights.size()) },
                { QStringLiteral("fitPointCount"),
                    static_cast<qulonglong>(spline.fitPoints.size()) },
                { QStringLiteral("parameterStart"), spline.parameterStart },
                { QStringLiteral("parameterEnd"), spline.parameterEnd },
                { QStringLiteral("subdivisionDepth"), subdivisionDepth },
                { QStringLiteral("generatedPointCount"),
                    static_cast<qulonglong>(generatedPointCount) },
                { QStringLiteral("closed"), spline.closed },
                { QStringLiteral("periodic"), spline.periodic },
                { QStringLiteral("rational"), spline.rational }
            };
        }

        Diagnostic makeSplineCompileDiagnostic
        (
            const SourceEntity& source,
            const SplineGeometry& spline,
            const OperationContext& context,
            DiagnosticCode code,
            DiagnosticSeverity severity,
            const QString& detail,
            int subdivisionDepth,
            std::size_t generatedPointCount
        )
        {
            Diagnostic diagnostic = makeCompileDiagnostic
            (
                source,
                context,
                code,
                QStringLiteral("CompileSpline"),
                code == DiagnosticCode::SplineFitPointFallbackUsed
                    ? QStringLiteral("样条曲线已使用拟合点降级路径。")
                    : QStringLiteral("样条曲线无法生成有效路径。"),
                detail,
                splineCompileContext
                (source, spline, subdivisionDepth, generatedPointCount)
            );
            diagnostic.severity = severity;
            return diagnostic;
        }

        Vector3d subtract(const Vector3d& left, const Vector3d& right)
        {
            return { left.x - right.x, left.y - right.y, left.z - right.z };
        }

        Vector3d add(const Vector3d& left, const Vector3d& right)
        {
            return { left.x + right.x, left.y + right.y, left.z + right.z };
        }

        Vector3d multiply(const Vector3d& point, double factor)
        {
            return { point.x * factor, point.y * factor, point.z * factor };
        }

        double distance(const Vector3d& left, const Vector3d& right)
        {
            return std::sqrt(squaredDistance(left, right));
        }

        double pointToSegmentDistance
        (
            const Vector3d& point,
            const Vector3d& start,
            const Vector3d& end
        )
        {
            const Vector3d segment = subtract(end, start);
            const double segmentLengthSquared = squaredLength(segment);
            if (segmentLengthSquared <= (std::numeric_limits<double>::epsilon)())
            {
                return distance(point, start);
            }
            const Vector3d offset = subtract(point, start);
            const double projection = offset.x * segment.x
                + offset.y * segment.y
                + offset.z * segment.z;
            const double factor = std::clamp(projection / segmentLengthSquared, 0.0, 1.0);
            return distance(point, add(start, multiply(segment, factor)));
        }

        bool appendDistinctSplinePoint
        (
            std::vector<PathVertex3D>& points,
            const Vector3d& point,
            double parameter,
            double tolerance,
            int maximumPoints
        )
        {
            if (!isFinite(point))
            {
                return false;
            }
            if (!points.empty() && distance(points.back().position, point) <= tolerance)
            {
                return true;
            }
            if (points.size() >= static_cast<std::size_t>(maximumPoints))
            {
                return false;
            }
            points.push_back({ point, parameter });
            return true;
        }

        struct SplineSubdivisionState
        {
            bool evaluationFailed = false;
            bool subdivisionLimitReached = false;
            bool pointLimitReached = false;
            int deepestSubdivision = 0;
            QVector<Diagnostic> diagnostics;
        };

        void tessellateSplineSpan
        (
            const NurbsCurveEvaluator& evaluator,
            const SplineGeometry& spline,
            const SplineSamplingPolicy& policy,
            const OperationContext& context,
            double startParameter,
            double endParameter,
            const Vector3d& startPoint,
            const Vector3d& endPoint,
            double tolerance,
            double maximumSegmentLength,
            int depth,
            std::vector<PathVertex3D>& points,
            SplineSubdivisionState& state
        )
        {
            if (state.evaluationFailed || state.subdivisionLimitReached || state.pointLimitReached)
            {
                return;
            }
            state.deepestSubdivision = std::max(state.deepestSubdivision, depth);
            if (points.size() >= static_cast<std::size_t>(policy.maximumPoints))
            {
                state.pointLimitReached = true;
                return;
            }

            const double middleParameter = (startParameter + endParameter) * 0.5;
            const double firstQuarterParameter = (startParameter + middleParameter) * 0.5;
            const double thirdQuarterParameter = (middleParameter + endParameter) * 0.5;
            const OperationResult<Vector3d> middle =
                evaluator.evaluate(spline, middleParameter, context);
            const OperationResult<Vector3d> firstQuarter =
                evaluator.evaluate(spline, firstQuarterParameter, context);
            const OperationResult<Vector3d> thirdQuarter =
                evaluator.evaluate(spline, thirdQuarterParameter, context);
            if (!middle.succeeded() || !middle.value.has_value()
                || !firstQuarter.succeeded() || !firstQuarter.value.has_value()
                || !thirdQuarter.succeeded() || !thirdQuarter.value.has_value())
            {
                state.evaluationFailed = true;
                state.diagnostics += middle.diagnostics;
                state.diagnostics += firstQuarter.diagnostics;
                state.diagnostics += thirdQuarter.diagnostics;
                return;
            }

            const double deviation = std::max
            ({
                pointToSegmentDistance(*middle.value, startPoint, endPoint),
                pointToSegmentDistance(*firstQuarter.value, startPoint, endPoint),
                pointToSegmentDistance(*thirdQuarter.value, startPoint, endPoint)
            });
            const bool preciseEnough = deviation <= tolerance
                && distance(startPoint, endPoint) <= maximumSegmentLength;
            if (preciseEnough)
            {
                if (!appendDistinctSplinePoint
                    (points, endPoint, endParameter, tolerance * 0.01, policy.maximumPoints))
                {
                    state.pointLimitReached = true;
                }
                return;
            }
            if (depth >= policy.maximumSubdivisionDepth)
            {
                state.subdivisionLimitReached = true;
                return;
            }

            tessellateSplineSpan
            (
                evaluator, spline, policy, context,
                startParameter, middleParameter, startPoint, *middle.value,
                tolerance, maximumSegmentLength, depth + 1, points, state
            );
            tessellateSplineSpan
            (
                evaluator, spline, policy, context,
                middleParameter, endParameter, *middle.value, endPoint,
                tolerance, maximumSegmentLength, depth + 1, points, state
            );
        }

        Vector3d catmullRom
        (
            const Vector3d& p0,
            const Vector3d& p1,
            const Vector3d& p2,
            const Vector3d& p3,
            double parameter
        )
        {
            const double parameter2 = parameter * parameter;
            const double parameter3 = parameter2 * parameter;
            return
            {
                0.5 * ((2.0 * p1.x) + (-p0.x + p2.x) * parameter
                    + (2.0 * p0.x - 5.0 * p1.x + 4.0 * p2.x - p3.x) * parameter2
                    + (-p0.x + 3.0 * p1.x - 3.0 * p2.x + p3.x) * parameter3),
                0.5 * ((2.0 * p1.y) + (-p0.y + p2.y) * parameter
                    + (2.0 * p0.y - 5.0 * p1.y + 4.0 * p2.y - p3.y) * parameter2
                    + (-p0.y + 3.0 * p1.y - 3.0 * p2.y + p3.y) * parameter3),
                0.5 * ((2.0 * p1.z) + (-p0.z + p2.z) * parameter
                    + (2.0 * p0.z - 5.0 * p1.z + 4.0 * p2.z - p3.z) * parameter2
                    + (-p0.z + 3.0 * p1.z - 3.0 * p2.z + p3.z) * parameter3)
            };
        }

        OperationResult<Path3D> compileSplineFitFallback
        (
            const SourceEntity& source,
            const SplineGeometry& spline,
            const SamplingPolicy& samplingPolicy,
            const PathCompileOptions& options,
            const OperationContext& context
        )
        {
            OperationResult<Path3D> result;
            const SplineSamplingPolicy& policy = samplingPolicy.spline;
            std::vector<Vector3d> fitPoints;
            for (const Vector3d& point : spline.fitPoints)
            {
                if (isFinite(point))
                {
                    fitPoints.push_back(point);
                }
            }
            if (!policy.allowFitPointFallback || fitPoints.size() < 2U)
            {
                result.status = OperationStatus::InvalidInput;
                result.addDiagnostic(makeSplineCompileDiagnostic
                (
                    source, spline, context, DiagnosticCode::InvalidSpline,
                    DiagnosticSeverity::Error,
                    QStringLiteral("exact NURBS data and fit-point fallback are unavailable"),
                    0, 0U
                ));
                return result;
            }

            const LocalFrame3d frame{ stableBoundsCenter(fitPoints) };
            for (Vector3d& point : fitPoints)
            {
                point = frame.toLocal(point);
            }

            Path3D path;
            path.sourceEntityId = source.id;
            path.sourceKind = SourceGeometryKind::Spline;
            path.closed = spline.closed || spline.periodic;
            path.samplingTolerance = policy.minimumTolerance;
            const std::size_t spanCount = path.closed
                ? fitPoints.size()
                : fitPoints.size() - 1U;
            auto pointAt = [&](long long index) -> const Vector3d&
                {
                    if (path.closed)
                    {
                        const long long count = static_cast<long long>(fitPoints.size());
                        return fitPoints[static_cast<std::size_t>((index % count + count) % count)];
                    }
                    const long long clamped = std::clamp<long long>
                    (index, 0, static_cast<long long>(fitPoints.size()) - 1);
                    return fitPoints[static_cast<std::size_t>(clamped)];
                };

            appendDistinctSplinePoint
            (path.vertices, fitPoints.front(), 0.0, policy.minimumTolerance * 0.01,
                policy.maximumPoints);
            for (std::size_t span = 0; span < spanCount; ++span)
            {
                for (int sample = 1; sample <= policy.fitFallbackSamplesPerSpan; ++sample)
                {
                    const double localParameter = static_cast<double>(sample)
                        / static_cast<double>(policy.fitFallbackSamplesPerSpan);
                    if (!appendDistinctSplinePoint
                        (
                            path.vertices,
                            catmullRom
                            (
                                pointAt(static_cast<long long>(span) - 1),
                                pointAt(static_cast<long long>(span)),
                                pointAt(static_cast<long long>(span) + 1),
                                pointAt(static_cast<long long>(span) + 2),
                                localParameter
                            ),
                            static_cast<double>(span) + localParameter,
                            policy.minimumTolerance * 0.01,
                            policy.maximumPoints
                        ))
                    {
                        result.status = OperationStatus::Failed;
                        result.addDiagnostic(makeSplineCompileDiagnostic
                        (
                            source, spline, context,
                            DiagnosticCode::SplinePointLimitExceeded,
                            DiagnosticSeverity::Error,
                            QStringLiteral("fit-point fallback exceeded maximumPoints"),
                            0, path.vertices.size()
                        ));
                        return result;
                    }
                }
            }

            if (path.closed && path.vertices.size() > 2U
                && distance(path.vertices.front().position, path.vertices.back().position)
                    <= policy.minimumTolerance)
            {
                path.vertices.pop_back();
            }
            if (options.reverse)
            {
                std::reverse(path.vertices.begin(), path.vertices.end());
            }
            for (PathVertex3D& vertex : path.vertices)
            {
                vertex.position = frame.toWorld(vertex.position);
            }

            OperationReport validation = validatePath3D(path, context);
            if (!validation.succeeded())
            {
                result.status = validation.status;
                result.mergeDiagnostics(validation);
                return result;
            }
            result.status = OperationStatus::PartialSuccess;
            result.value = std::move(path);
            result.addDiagnostic(makeSplineCompileDiagnostic
            (
                source, spline, context,
                DiagnosticCode::SplineFitPointFallbackUsed,
                DiagnosticSeverity::Warning,
                QStringLiteral("Catmull-Rom interpolation was used for valid fit points"),
                0, result.value->vertices.size()
            ));
            return result;
        }

        OperationResult<Path3D> compileSplineGeometry
        (
            const SourceEntity& source,
            const SplineGeometry& spline,
            const SamplingPolicy& samplingPolicy,
            const PathCompileOptions& options,
            const OperationContext& context
        )
        {
            const SplineSamplingPolicy& policy = samplingPolicy.spline;
            NurbsCurveEvaluator evaluator;
            const OperationResult<Vector3d> domainStart =
                evaluator.evaluate(spline, spline.parameterStart, context);
            if (!domainStart.succeeded() || !domainStart.value.has_value())
            {
                if (policy.allowFitPointFallback)
                {
                    return compileSplineFitFallback
                        (source, spline, samplingPolicy, options, context);
                }
                OperationResult<Path3D> failed;
                failed.status = domainStart.status;
                failed.mergeDiagnostics(domainStart);
                for (Diagnostic& diagnostic : failed.diagnostics)
                {
                    diagnostic.entityId = source.id;
                    diagnostic.context.insert
                        (QStringLiteral("entityId"), static_cast<qulonglong>(source.id));
                }
                return failed;
            }

            Vector3d minimum = spline.controlPoints.front();
            Vector3d maximum = minimum;
            for (const Vector3d& point : spline.controlPoints)
            {
                minimum.x = std::min(minimum.x, point.x);
                minimum.y = std::min(minimum.y, point.y);
                minimum.z = std::min(minimum.z, point.z);
                maximum.x = std::max(maximum.x, point.x);
                maximum.y = std::max(maximum.y, point.y);
                maximum.z = std::max(maximum.z, point.z);
            }
            const double diagonal = distance(minimum, maximum);
            if (!isFinite(diagonal)
                || diagonal <= (std::numeric_limits<double>::epsilon)())
            {
                return compileSplineFitFallback
                    (source, spline, samplingPolicy, options, context);
            }

            const double tolerance = std::max
            (policy.minimumTolerance, diagonal * policy.relativeChordTolerance);
            const double maximumSegmentLength = std::max
            (tolerance * 32.0, diagonal * policy.relativeMaximumSegmentLength);
            Path3D path;
            path.sourceEntityId = source.id;
            path.sourceKind = SourceGeometryKind::Spline;
            path.closed = spline.closed || spline.periodic;
            path.samplingTolerance = tolerance;
            SplineSubdivisionState state;

            const std::size_t controlCount = spline.controlPoints.size();
            for (std::size_t knotIndex = static_cast<std::size_t>(spline.degree);
                knotIndex < controlCount; ++knotIndex)
            {
                const double startParameter = spline.knots[knotIndex];
                const double endParameter = spline.knots[knotIndex + 1U];
                if (endParameter - startParameter <= policy.knotSpanTolerance)
                {
                    continue;
                }
                const OperationResult<Vector3d> start =
                    evaluator.evaluate(spline, startParameter, context);
                const OperationResult<Vector3d> end =
                    evaluator.evaluate(spline, endParameter, context);
                if (!start.succeeded() || !start.value.has_value()
                    || !end.succeeded() || !end.value.has_value())
                {
                    state.evaluationFailed = true;
                    state.diagnostics += start.diagnostics;
                    state.diagnostics += end.diagnostics;
                    break;
                }
                if (!appendDistinctSplinePoint
                    (path.vertices, *start.value, startParameter, tolerance * 0.01,
                        policy.maximumPoints))
                {
                    state.pointLimitReached = true;
                    break;
                }
                tessellateSplineSpan
                (
                    evaluator, spline, policy, context,
                    startParameter, endParameter, *start.value, *end.value,
                    tolerance, maximumSegmentLength, 0, path.vertices, state
                );
                if (state.evaluationFailed
                    || state.subdivisionLimitReached
                    || state.pointLimitReached)
                {
                    break;
                }
            }

            OperationResult<Path3D> result;
            if (state.evaluationFailed)
            {
                result.status = OperationStatus::Failed;
                result.mergeDiagnostics(state.diagnostics);
                for (Diagnostic& diagnostic : result.diagnostics)
                {
                    diagnostic.entityId = source.id;
                    diagnostic.context.insert
                        (QStringLiteral("entityId"), static_cast<qulonglong>(source.id));
                }
                result.addDiagnostic(makeSplineCompileDiagnostic
                (
                    source, spline, context, DiagnosticCode::SplineEvaluationFailure,
                    DiagnosticSeverity::Error,
                    QStringLiteral("NURBS evaluation failed during adaptive subdivision"),
                    state.deepestSubdivision, path.vertices.size()
                ));
                return result;
            }
            if (state.subdivisionLimitReached)
            {
                result.status = OperationStatus::Failed;
                result.addDiagnostic(makeSplineCompileDiagnostic
                (
                    source, spline, context, DiagnosticCode::SplineSubdivisionLimit,
                    DiagnosticSeverity::Error,
                    QStringLiteral("adaptive subdivision reached maximumSubdivisionDepth"),
                    state.deepestSubdivision, path.vertices.size()
                ));
                return result;
            }
            if (state.pointLimitReached)
            {
                result.status = OperationStatus::Failed;
                result.addDiagnostic(makeSplineCompileDiagnostic
                (
                    source, spline, context, DiagnosticCode::SplinePointLimitExceeded,
                    DiagnosticSeverity::Error,
                    QStringLiteral("adaptive subdivision exceeded maximumPoints"),
                    state.deepestSubdivision, path.vertices.size()
                ));
                return result;
            }
            if (path.vertices.size() < 2U)
            {
                return compileSplineFitFallback
                    (source, spline, samplingPolicy, options, context);
            }

            const double duplicateTolerance = std::max
            (
                policy.minimumTolerance,
                distance(path.vertices.front().position, path.vertices.back().position) * 1.0e-9
            );
            if (path.closed && path.vertices.size() > 2U
                && distance(path.vertices.front().position, path.vertices.back().position)
                    <= duplicateTolerance)
            {
                path.vertices.pop_back();
            }
            if (options.reverse)
            {
                std::reverse(path.vertices.begin(), path.vertices.end());
            }

            OperationReport validation = validatePath3D(path, context);
            if (!validation.succeeded())
            {
                result.status = validation.status;
                result.mergeDiagnostics(validation);
                return result;
            }
            result.status = OperationStatus::Success;
            result.value = std::move(path);
            return result;
        }

        bool validPolicy(const SamplingPolicy& policy)
        {
            return isFinite(policy.chordTolerance)
                && isFinite(policy.maximumSegmentLength)
                && isFinite(policy.maximumAngularStep)
                && policy.chordTolerance >= 0.0
                && policy.maximumSegmentLength >= 0.0
                && policy.maximumAngularStep >= 0.0
                && policy.minimumSegments >= 1
                && policy.fullTurnSegments >= 3
                && policy.minimumBulgeSegments >= 1
                && policy.maximumSegments >= policy.minimumSegments
                && isFinite(policy.spline.minimumTolerance)
                && isFinite(policy.spline.relativeChordTolerance)
                && isFinite(policy.spline.relativeMaximumSegmentLength)
                && isFinite(policy.spline.knotSpanTolerance)
                && policy.spline.minimumTolerance > 0.0
                && policy.spline.relativeChordTolerance >= 0.0
                && policy.spline.relativeMaximumSegmentLength > 0.0
                && policy.spline.knotSpanTolerance >= 0.0
                && policy.spline.maximumSubdivisionDepth >= 0
                && policy.spline.maximumPoints >= 2
                && policy.spline.fitFallbackSamplesPerSpan >= 1;
        }

        int segmentCountForSpan
        (
            double span,
            double characteristicRadius,
            const SamplingPolicy& policy,
            bool useFullTurnDensity
        )
        {
            int segments = policy.minimumSegments;
            const double absoluteSpan = std::abs(span);

            if (useFullTurnDensity)
            {
                segments = std::max
                (
                    segments,
                    static_cast<int>(std::ceil
                    (
                        absoluteSpan / kTwoPi * static_cast<double>(policy.fullTurnSegments)
                    ))
                );
            }

            if (policy.maximumAngularStep > 0.0)
            {
                segments = std::max
                (
                    segments,
                    static_cast<int>(std::ceil(absoluteSpan / policy.maximumAngularStep))
                );
            }

            if (characteristicRadius > kGeometryTolerance && policy.chordTolerance > 0.0)
            {
                const double ratio = std::clamp
                (
                    1.0 - policy.chordTolerance / characteristicRadius,
                    -1.0,
                    1.0
                );
                const double angularStep = 2.0 * std::acos(ratio);
                if (angularStep > kGeometryTolerance)
                {
                    segments = std::max
                    (
                        segments,
                        static_cast<int>(std::ceil(absoluteSpan / angularStep))
                    );
                }
            }

            if (characteristicRadius > kGeometryTolerance && policy.maximumSegmentLength > 0.0)
            {
                const double ratio = std::clamp
                (
                    policy.maximumSegmentLength / (2.0 * characteristicRadius),
                    0.0,
                    1.0
                );
                const double angularStep = 2.0 * std::asin(ratio);
                if (angularStep > kGeometryTolerance)
                {
                    segments = std::max
                    (
                        segments,
                        static_cast<int>(std::ceil(absoluteSpan / angularStep))
                    );
                }
            }

            return segments;
        }

        double normalizedArcEnd(double start, double end, bool reverse)
        {
            if (reverse)
            {
                while (end >= start)
                {
                    end -= kTwoPi;
                }
            }
            else
            {
                while (end <= start)
                {
                    end += kTwoPi;
                }
            }
            return end;
        }
    }

    OperationResult<Path3D> GeometryCompiler::compile
    (
        const SourceEntity& source,
        const SamplingPolicy& policy,
        const PathCompileOptions& options,
        const OperationContext& context
    ) const
    {
        OperationResult<Path3D> result;

        if (source.id == 0)
        {
            result.status = OperationStatus::InvalidInput;
            result.addDiagnostic(makeCompileDiagnostic
            (
                source,
                context,
                DiagnosticCode::InvalidArgument,
                QStringLiteral("ValidateInput"),
                QStringLiteral("源图元编号无效。"),
                QStringLiteral("SourceEntity::id is zero")
            ));
            return result;
        }

        if (!validPolicy(policy))
        {
            result.status = OperationStatus::InvalidInput;
            result.addDiagnostic(makeCompileDiagnostic
            (
                source,
                context,
                DiagnosticCode::InvalidSamplingPolicy,
                QStringLiteral("ValidateSamplingPolicy"),
                QStringLiteral("路径采样策略无效。"),
                QStringLiteral("Sampling policy contains an invalid limit"),
                {
                    { QStringLiteral("samplingTolerance"), policy.chordTolerance },
                    { QStringLiteral("maximumAngularStep"), policy.maximumAngularStep },
                    { QStringLiteral("minimumSegments"), policy.minimumSegments },
                    { QStringLiteral("minimumBulgeSegments"), policy.minimumBulgeSegments },
                    { QStringLiteral("maximumSegments"), policy.maximumSegments }
                }
            ));
            return result;
        }

        if (source.kind != SourceGeometryKind::Line
            && source.kind != SourceGeometryKind::Circle
            && source.kind != SourceGeometryKind::Arc
            && source.kind != SourceGeometryKind::Ellipse
            && source.kind != SourceGeometryKind::Polyline
            && source.kind != SourceGeometryKind::Spline)
        {
            result.status = OperationStatus::NotSupported;
            result.addDiagnostic(makeCompileDiagnostic
            (
                source,
                context,
                DiagnosticCode::UnsupportedGeometry,
                QStringLiteral("ValidateInput"),
                QStringLiteral("当前几何编译器尚不支持该图元。"),
                QStringLiteral("Source geometry kind is outside the phase-two scope")
            ));
            return result;
        }

        if (source.kind == SourceGeometryKind::Spline)
        {
            if (!std::holds_alternative<SplineGeometry>(source.geometry))
            {
                result.status = OperationStatus::InvalidInput;
                result.addDiagnostic(makeCompileDiagnostic
                (
                    source,
                    context,
                    DiagnosticCode::InvalidSpline,
                    QStringLiteral("ValidateInput"),
                    QStringLiteral("样条源类型与核心几何数据不匹配。"),
                    QStringLiteral("SourceGeometryKind::Spline does not hold SplineGeometry")
                ));
                return result;
            }
            return compileSplineGeometry
            (
                source,
                std::get<SplineGeometry>(source.geometry),
                policy,
                options,
                context
            );
        }

        Path3D path;
        path.sourceEntityId = source.id;
        path.sourceKind = source.kind;
        path.samplingTolerance = policy.chordTolerance;

        int segmentCount = 0;
        bool geometryValid = true;
        bool segmentCountWithinPolicy = true;

        std::visit
        (
            [&](const auto& geometry)
            {
                using Geometry = std::decay_t<decltype(geometry)>;

                if constexpr (std::is_same_v<Geometry, LineGeometry>)
                {
                    if (source.kind != SourceGeometryKind::Line
                        || squaredLength
                        ({
                            geometry.end.x - geometry.start.x,
                            geometry.end.y - geometry.start.y,
                            geometry.end.z - geometry.start.z
                        }) <= kGeometryTolerance * kGeometryTolerance)
                    {
                        geometryValid = false;
                        return;
                    }

                    path.closed = false;
                    path.vertices = options.reverse
                        ? std::vector<PathVertex3D>
                            { { geometry.end, 1.0 }, { geometry.start, 0.0 } }
                        : std::vector<PathVertex3D>
                            { { geometry.start, 0.0 }, { geometry.end, 1.0 } };
                    segmentCount = 1;
                }
                else if constexpr (std::is_same_v<Geometry, CircleGeometry>)
                {
                    if (source.kind != SourceGeometryKind::Circle
                        || !isFinite(geometry.radius)
                        || geometry.radius <= 0.0
                        || squaredLength(geometry.axisU) <= kGeometryTolerance * kGeometryTolerance
                        || squaredLength(geometry.axisV) <= kGeometryTolerance * kGeometryTolerance)
                    {
                        geometryValid = false;
                        return;
                    }

                    const double start = options.startParameter.value_or(kHalfPi);
                    const double sweep = options.reverse ? -kTwoPi : kTwoPi;
                    segmentCount = segmentCountForSpan
                    (
                        sweep,
                        geometry.radius,
                        policy,
                        true
                    );
                    if (segmentCount > policy.maximumSegments)
                    {
                        segmentCountWithinPolicy = false;
                        return;
                    }
                    path.closed = true;
                    path.vertices.reserve(static_cast<std::size_t>(segmentCount));
                    for (int index = 0; index < segmentCount; ++index)
                    {
                        const double parameter = start
                            + sweep * static_cast<double>(index) / static_cast<double>(segmentCount);
                        path.vertices.push_back
                        ({
                            addScaled
                            (
                                geometry.center,
                                geometry.axisU,
                                std::cos(parameter) * geometry.radius,
                                geometry.axisV,
                                std::sin(parameter) * geometry.radius
                            ),
                            parameter
                        });
                    }
                }
                else if constexpr (std::is_same_v<Geometry, ArcGeometry>)
                {
                    if (source.kind != SourceGeometryKind::Arc
                        || !isFinite(geometry.radius)
                        || geometry.radius <= 0.0
                        || squaredLength(geometry.axisU) <= kGeometryTolerance * kGeometryTolerance
                        || squaredLength(geometry.axisV) <= kGeometryTolerance * kGeometryTolerance)
                    {
                        geometryValid = false;
                        return;
                    }

                    const double start = options.reverse
                        ? geometry.endParameter
                        : geometry.startParameter;
                    const double rawEnd = options.reverse
                        ? geometry.startParameter
                        : geometry.endParameter;
                    const double end = normalizedArcEnd(start, rawEnd, options.reverse);
                    const double span = end - start;
                    segmentCount = segmentCountForSpan
                    (
                        span,
                        geometry.radius,
                        policy,
                        false
                    );
                    if (segmentCount > policy.maximumSegments)
                    {
                        segmentCountWithinPolicy = false;
                        return;
                    }
                    path.closed = false;
                    path.vertices.reserve(static_cast<std::size_t>(segmentCount) + 1U);
                    for (int index = 0; index <= segmentCount; ++index)
                    {
                        const double parameter = start
                            + span * static_cast<double>(index) / static_cast<double>(segmentCount);
                        path.vertices.push_back
                        ({
                            addScaled
                            (
                                geometry.center,
                                geometry.axisU,
                                std::cos(parameter) * geometry.radius,
                                geometry.axisV,
                                std::sin(parameter) * geometry.radius
                            ),
                            parameter
                        });
                    }
                }
                else if constexpr (std::is_same_v<Geometry, EllipseGeometry>)
                {
                    const double majorLength = std::sqrt(squaredLength(geometry.majorAxis));
                    const double minorLength = std::sqrt(squaredLength(geometry.minorAxis));
                    if (source.kind != SourceGeometryKind::Ellipse
                        || majorLength <= kGeometryTolerance
                        || minorLength <= kGeometryTolerance)
                    {
                        geometryValid = false;
                        return;
                    }

                    double start = geometry.startParameter;
                    double end = geometry.endParameter;
                    if (geometry.fullEllipse)
                    {
                        start = options.startParameter.value_or(kHalfPi);
                        end = start + (options.reverse ? -kTwoPi : kTwoPi);
                    }
                    else if (options.reverse)
                    {
                        start = geometry.endParameter;
                        end = normalizedArcEnd(start, geometry.startParameter, true);
                    }
                    else
                    {
                        end = normalizedArcEnd(start, end, false);
                    }

                    const double span = end - start;
                    segmentCount = segmentCountForSpan
                    (
                        span,
                        std::max(majorLength, minorLength),
                        policy,
                        true
                    );
                    if (segmentCount > policy.maximumSegments)
                    {
                        segmentCountWithinPolicy = false;
                        return;
                    }
                    path.closed = geometry.fullEllipse;
                    const int lastIndex = path.closed ? segmentCount - 1 : segmentCount;
                    path.vertices.reserve(static_cast<std::size_t>(lastIndex) + 1U);
                    for (int index = 0; index <= lastIndex; ++index)
                    {
                        const double parameter = start
                            + span * static_cast<double>(index) / static_cast<double>(segmentCount);
                        path.vertices.push_back
                        ({
                            addScaled
                            (
                                geometry.center,
                                geometry.majorAxis,
                                std::cos(parameter),
                                geometry.minorAxis,
                                std::sin(parameter)
                            ),
                            parameter
                        });
                    }
                }
                else if constexpr (std::is_same_v<Geometry, PolylineGeometry>)
                {
                    if (source.kind != SourceGeometryKind::Polyline
                        || !validPolylineGeometry(geometry))
                    {
                        geometryValid = false;
                        return;
                    }

                    const std::size_t primitiveCount = geometry.segments.size();
                    std::size_t startVertexIndex = 0U;
                    if (geometry.closed && options.startParameter.has_value())
                    {
                        const double requestedStart = *options.startParameter;
                        if (!isFinite(requestedStart)
                            || std::abs(requestedStart)
                                > static_cast<double>((std::numeric_limits<long long>::max)()))
                        {
                            geometryValid = false;
                            return;
                        }
                        const long long rawIndex = std::llround(requestedStart);
                        const long long vertexCount = static_cast<long long>(geometry.sourceVertexCount);
                        startVertexIndex = static_cast<std::size_t>
                        (
                            ((rawIndex % vertexCount) + vertexCount) % vertexCount
                        );
                    }

                    path.closed = geometry.closed;
                    path.vertices.clear();
                    segmentCount = 0;

                    for (std::size_t step = 0; step < primitiveCount; ++step)
                    {
                        const std::size_t segmentIndex = geometry.closed
                            ? (options.reverse
                                ? (startVertexIndex + primitiveCount - 1U - step) % primitiveCount
                                : (startVertexIndex + step) % primitiveCount)
                            : (options.reverse ? primitiveCount - 1U - step : step);
                        const bool reversePrimitive = options.reverse;
                        const PolylinePrimitive& primitive = geometry.segments[segmentIndex];

                        std::visit
                        (
                            [&](const auto& segment)
                            {
                                using Segment = std::decay_t<decltype(segment)>;
                                if constexpr (std::is_same_v<Segment, LineGeometry>)
                                {
                                    const Vector3d& start = reversePrimitive ? segment.end : segment.start;
                                    const Vector3d& end = reversePrimitive ? segment.start : segment.end;
                                    if (path.vertices.empty())
                                    {
                                        path.vertices.push_back
                                        ({
                                            start,
                                            static_cast<double>(segmentIndex)
                                                + (reversePrimitive ? 1.0 : 0.0)
                                        });
                                    }
                                    ++segmentCount;
                                    if (!(geometry.closed && step + 1U == primitiveCount))
                                    {
                                        path.vertices.push_back
                                        ({
                                            end,
                                            static_cast<double>(segmentIndex)
                                                + (reversePrimitive ? 0.0 : 1.0)
                                        });
                                    }
                                }
                                else
                                {
                                    const double start = reversePrimitive
                                        ? segment.endParameter
                                        : segment.startParameter;
                                    const double span = reversePrimitive
                                        ? segment.startParameter - segment.endParameter
                                        : segment.endParameter - segment.startParameter;
                                    const int bulgeSegments = std::max
                                    (
                                        policy.minimumBulgeSegments,
                                        static_cast<int>(std::ceil
                                        (
                                            std::abs(span) / kTwoPi
                                            * static_cast<double>(policy.fullTurnSegments)
                                        ))
                                    );
                                    segmentCount += bulgeSegments;
                                    if (path.vertices.empty())
                                    {
                                        path.vertices.push_back
                                        ({
                                            arcPoint(segment, start),
                                            static_cast<double>(segmentIndex)
                                                + (reversePrimitive ? 1.0 : 0.0)
                                        });
                                    }
                                    for (int sample = 1; sample <= bulgeSegments; ++sample)
                                    {
                                        if (geometry.closed && step + 1U == primitiveCount
                                            && sample == bulgeSegments)
                                        {
                                            continue;
                                        }
                                        const double traversalFraction = static_cast<double>(sample)
                                            / static_cast<double>(bulgeSegments);
                                        const double originalFraction = reversePrimitive
                                            ? 1.0 - traversalFraction
                                            : traversalFraction;
                                        path.vertices.push_back
                                        ({
                                            arcPoint
                                            (
                                                segment,
                                                start + span * traversalFraction
                                            ),
                                            static_cast<double>(segmentIndex) + originalFraction
                                        });
                                    }
                                }
                            },
                            primitive
                        );

                        if (segmentCount > policy.maximumSegments)
                        {
                            segmentCountWithinPolicy = false;
                            return;
                        }
                    }
                }
            },
            source.geometry
        );

        if (!geometryValid)
        {
            result.status = OperationStatus::Failed;
            result.addDiagnostic(makeCompileDiagnostic
            (
                source,
                context,
                DiagnosticCode::DegenerateGeometry,
                QStringLiteral("CompileGeometry"),
                QStringLiteral("源几何已退化，无法生成路径。"),
                QStringLiteral("Geometry dimensions or axes are invalid")
            ));
            return result;
        }

        if (!segmentCountWithinPolicy || segmentCount < 1 || segmentCount > policy.maximumSegments)
        {
            result.status = OperationStatus::Failed;
            result.addDiagnostic(makeCompileDiagnostic
            (
                source,
                context,
                DiagnosticCode::GeometryCompilationFailure,
                QStringLiteral("ResolveSegments"),
                QStringLiteral("路径采样段数超出允许范围。"),
                QStringLiteral("Resolved segment count exceeds the sampling policy"),
                {
                    { QStringLiteral("segmentCount"), segmentCount },
                    { QStringLiteral("maximumSegments"), policy.maximumSegments },
                    { QStringLiteral("samplingTolerance"), policy.chordTolerance },
                    { QStringLiteral("maximumAngularStep"), policy.maximumAngularStep }
                }
            ));
            return result;
        }

        OperationReport validation = validatePath3D(path, context);
        if (!validation.succeeded())
        {
            result.status = validation.status;
            result.mergeDiagnostics(validation);
            return result;
        }

        result.status = OperationStatus::Success;
        result.value = std::move(path);
        return result;
    }
}
