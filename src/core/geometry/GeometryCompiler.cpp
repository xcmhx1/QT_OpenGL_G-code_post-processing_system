#include "core/geometry/GeometryCompiler.h"

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
            double scaleV,
            bool singlePrecisionEvaluation,
            bool offsetFirst
        )
        {
            if (singlePrecisionEvaluation)
            {
                const auto evaluate =
                    [offsetFirst](double originValue, double axisUValue, double u, double axisVValue, double v)
                    {
                        const float scaledU = static_cast<float>(axisUValue) * static_cast<float>(u);
                        const float scaledV = static_cast<float>(axisVValue) * static_cast<float>(v);
                        return offsetFirst
                            ? static_cast<double>(static_cast<float>(originValue) + (scaledU + scaledV))
                            : static_cast<double>((static_cast<float>(originValue) + scaledU) + scaledV);
                    };
                return
                {
                    evaluate(origin.x, axisU.x, scaleU, axisV.x, scaleV),
                    evaluate(origin.y, axisU.y, scaleU, axisV.y, scaleV),
                    evaluate(origin.z, axisU.z, scaleU, axisV.z, scaleV)
                };
            }

            return
            {
                origin.x + axisU.x * scaleU + axisV.x * scaleV,
                origin.y + axisU.y * scaleU + axisV.y * scaleV,
                origin.z + axisU.z * scaleU + axisV.z * scaleV
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

        Vector3d arcPoint(const ArcGeometry& arc, double parameter, bool singlePrecision)
        {
            return addScaled
            (
                arc.center,
                arc.axisU,
                std::cos(parameter) * arc.radius,
                arc.axisV,
                std::sin(parameter) * arc.radius,
                singlePrecision,
                true
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
                        return arcPoint(geometry, geometry.startParameter, false);
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
                        return arcPoint(geometry, geometry.endParameter, false);
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
                && policy.maximumSegments >= policy.minimumSegments;
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
            && source.kind != SourceGeometryKind::Polyline)
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
                                std::sin(parameter) * geometry.radius,
                                policy.singlePrecisionEvaluation,
                                true
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
                                std::sin(parameter) * geometry.radius,
                                policy.singlePrecisionEvaluation,
                                true
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
                                std::sin(parameter),
                                policy.singlePrecisionEvaluation,
                                false
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
                                            arcPoint(segment, start, policy.singlePrecisionEvaluation),
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
                                                start + span * traversalFraction,
                                                policy.singlePrecisionEvaluation
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
