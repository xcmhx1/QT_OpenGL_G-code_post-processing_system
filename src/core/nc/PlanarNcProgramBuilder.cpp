#include "core/nc/PlanarNcProgramBuilder.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <type_traits>

namespace cadcam::nc
{
    namespace
    {
        constexpr double kHalfPi = 1.57079632679489661923;
        constexpr double kTwoPi = 6.28318530717958647692;

        Diagnostic planarDiagnostic
        (
            DiagnosticCode code,
            DiagnosticSeverity severity,
            const QString& message,
            const PlanarNcEntityInput& input,
            const OperationContext& context,
            int motionIndex = -1
        )
        {
            Diagnostic diagnostic;
            diagnostic.code = code;
            diagnostic.severity = severity;
            diagnostic.component = QStringLiteral("PlanarNcProgramBuilder");
            diagnostic.operation = context.operationName;
            diagnostic.stage = QStringLiteral("build-planar-program");
            diagnostic.userMessage = message;
            diagnostic.correlationId = context.correlationId;
            diagnostic.entityId = input.sourceEntity.id;
            diagnostic.context =
            {
                { QStringLiteral("entityId"), static_cast<qulonglong>(input.sourceEntity.id) },
                { QStringLiteral("sourceIndex"), static_cast<qulonglong>(input.metadata.sourceIndex) },
                { QStringLiteral("processOrder"), input.metadata.processOrder },
                { QStringLiteral("sourceKind"), QString::fromLatin1
                    (geometry::sourceGeometryKindName(input.sourceEntity.kind)) },
                { QStringLiteral("motionIndex"), motionIndex },
                { QStringLiteral("plane"), QStringLiteral("Unknown") },
                { QStringLiteral("reverse"), input.reverse },
                { QStringLiteral("startParameter"), input.startParameter.value_or(0.0) }
            };
            return diagnostic;
        }

        void quantizeAxes(NcAxisWords& axes, bool quantize)
        {
            if (!quantize) return;
            auto apply = [](std::optional<double>& value)
            {
                if (value.has_value()) *value = static_cast<double>(static_cast<float>(*value));
            };
            apply(axes.x); apply(axes.y); apply(axes.z); apply(axes.a);
            apply(axes.i); apply(axes.j); apply(axes.k); apply(axes.r);
        }

        geometry::Vector3d cross
        (
            const geometry::Vector3d& left,
            const geometry::Vector3d& right
        )
        {
            return
            {
                left.y * right.z - left.z * right.y,
                left.z * right.x - left.x * right.z,
                left.x * right.y - left.y * right.x
            };
        }

        geometry::Vector3d curvePoint
        (
            const geometry::Vector3d& center,
            const geometry::Vector3d& axisU,
            const geometry::Vector3d& axisV,
            double radius,
            double parameter
        )
        {
            return
            {
                center.x + radius * (axisU.x * std::cos(parameter) + axisV.x * std::sin(parameter)),
                center.y + radius * (axisU.y * std::cos(parameter) + axisV.y * std::sin(parameter)),
                center.z + radius * (axisU.z * std::cos(parameter) + axisV.z * std::sin(parameter))
            };
        }

        enum class PrincipalPlane { XY, ZX, YZ, Other };

        PrincipalPlane principalPlane
        (
            const geometry::Vector3d& normal,
            double tolerance
        )
        {
            const double length = std::sqrt
                (normal.x * normal.x + normal.y * normal.y + normal.z * normal.z);
            if (!std::isfinite(length) || length <= tolerance) return PrincipalPlane::Other;
            const double x = std::abs(normal.x / length);
            const double y = std::abs(normal.y / length);
            const double z = std::abs(normal.z / length);
            if (z >= 1.0 - tolerance && x <= tolerance && y <= tolerance) return PrincipalPlane::XY;
            if (y >= 1.0 - tolerance && x <= tolerance && z <= tolerance) return PrincipalPlane::ZX;
            if (x >= 1.0 - tolerance && y <= tolerance && z <= tolerance) return PrincipalPlane::YZ;
            return PrincipalPlane::Other;
        }

        NcMotion makeMotion
        (
            NcMotionKind kind,
            const NcAxisWords& axes,
            const PlanarNcEntityInput& input,
            NcPlane plane,
            bool quantize
        )
        {
            NcMotion motion;
            motion.kind = kind;
            motion.sourceKind = kind == NcMotionKind::Rapid
                ? NcSourceMoveKind::Rapid : NcSourceMoveKind::Cutting;
            motion.plane = plane;
            motion.axes = axes;
            quantizeAxes(motion.axes, quantize);
            motion.entityId = input.sourceEntity.id;
            motion.processGroupId = input.metadata.processGroupId;
            return motion;
        }

        NcAxisWords pointAxes(const geometry::Vector3d& point, bool includeZ)
        {
            NcAxisWords axes;
            axes.x = point.x;
            axes.y = point.y;
            if (includeZ) axes.z = point.z;
            return axes;
        }

        bool appendCompiledPath
        (
            const PlanarNcEntityInput& input,
            const PlanarNcBuildPolicy& policy,
            const OperationContext& context,
            bool includeZ,
            NcEntityBlock& block,
            QVector<Diagnostic>& diagnostics
        )
        {
            geometry::PathCompileOptions options;
            options.reverse = input.reverse;
            options.startParameter = input.startParameter;
            geometry::GeometryCompiler compiler;
            auto compiled = compiler.compile
                (input.sourceEntity, policy.samplingPolicy, options, context);
            if (!compiled.succeeded() || !compiled.value.has_value()
                || compiled.value->vertices.size() < 2U)
            {
                for (Diagnostic diagnostic : compiled.diagnostics)
                {
                    diagnostic.severity = DiagnosticSeverity::Warning;
                    diagnostics.push_back(std::move(diagnostic));
                }
                diagnostics.push_back(planarDiagnostic(DiagnosticCode::PlanarNcGeometryCompileFailed,
                    DiagnosticSeverity::Warning,
                    QStringLiteral("图元无法编译为三轴加工路径，已跳过。"), input, context));
                return false;
            }
            diagnostics += compiled.diagnostics;

            const auto& path = *compiled.value;
            block.motions.push_back(makeMotion(NcMotionKind::Rapid,
                pointAxes(path.vertices.front().position, includeZ), input, NcPlane::XY,
                policy.preserveCurrentOutputQuantization));
            for (std::size_t index = 1; index < path.vertices.size(); ++index)
            {
                block.motions.push_back(makeMotion(NcMotionKind::Linear,
                    pointAxes(path.vertices[index].position, includeZ), input, NcPlane::XY,
                    policy.preserveCurrentOutputQuantization));
            }
            if (path.closed)
            {
                block.motions.push_back(makeMotion(NcMotionKind::Linear,
                    pointAxes(path.vertices.front().position, includeZ), input, NcPlane::XY,
                    policy.preserveCurrentOutputQuantization));
            }
            return block.motions.size() >= 2U;
        }

        bool appendExactArc
        (
            const geometry::ArcGeometry& arc,
            bool reverse,
            const PlanarNcEntityInput& input,
            const PlanarNcBuildPolicy& policy,
            NcEntityBlock& block,
            PrincipalPlane forcedPlane = PrincipalPlane::Other
        )
        {
            const geometry::Vector3d normal = cross(arc.axisU, arc.axisV);
            const PrincipalPlane plane = forcedPlane == PrincipalPlane::Other
                ? principalPlane(normal, policy.principalPlaneTolerance) : forcedPlane;
            if (plane == PrincipalPlane::Other) return false;

            const double startParameter = reverse ? arc.endParameter : arc.startParameter;
            const double endParameter = reverse ? arc.startParameter : arc.endParameter;
            const geometry::Vector3d start = curvePoint
                (arc.center, arc.axisU, arc.axisV, arc.radius, startParameter);
            const geometry::Vector3d end = curvePoint
                (arc.center, arc.axisU, arc.axisV, arc.radius, endParameter);
            double orientation = 0.0;
            NcPlane ncPlane = NcPlane::XY;
            NcAxisWords rapid;
            NcAxisWords circular;
            if (plane == PrincipalPlane::XY)
            {
                orientation = normal.z;
                rapid = pointAxes(start, false);
                circular.x = end.x; circular.y = end.y;
                circular.i = arc.center.x - start.x;
                circular.j = arc.center.y - start.y;
            }
            else if (plane == PrincipalPlane::ZX)
            {
                orientation = -normal.y;
                ncPlane = NcPlane::ZX;
                rapid = pointAxes(start, true);
                circular.x = end.x; circular.z = end.z;
                circular.i = arc.center.x - start.x;
                circular.k = arc.center.z - start.z;
            }
            else
            {
                orientation = normal.x;
                ncPlane = NcPlane::YZ;
                rapid = pointAxes(start, true);
                circular.y = end.y; circular.z = end.z;
                circular.j = arc.center.y - start.y;
                circular.k = arc.center.z - start.z;
            }
            if (reverse) orientation = -orientation;
            const NcMotionKind kind = orientation < 0.0
                ? NcMotionKind::CircularClockwise
                : NcMotionKind::CircularCounterclockwise;
            block.motions.push_back(makeMotion(NcMotionKind::Rapid, rapid, input, ncPlane,
                policy.preserveCurrentOutputQuantization));
            block.motions.push_back(makeMotion(kind, circular, input, ncPlane,
                policy.preserveCurrentOutputQuantization));
            return true;
        }

        bool appendPolyline
        (
            const geometry::PolylineGeometry& polyline,
            const PlanarNcEntityInput& input,
            const PlanarNcBuildPolicy& policy,
            NcEntityBlock& block
        )
        {
            if (polyline.segments.empty() || polyline.sourceVertexCount < 2U) return false;
            const std::size_t count = polyline.segments.size();
            std::size_t startIndex = 0U;
            if (polyline.closed && input.startParameter.has_value())
            {
                if (!std::isfinite(*input.startParameter)) return false;
                const long long raw = std::llround(*input.startParameter);
                const long long vertexCount = static_cast<long long>(polyline.sourceVertexCount);
                startIndex = static_cast<std::size_t>(((raw % vertexCount) + vertexCount) % vertexCount);
            }

            for (std::size_t step = 0; step < count; ++step)
            {
                const std::size_t index = polyline.closed
                    ? (input.reverse ? (startIndex + count - 1U - step) % count
                                     : (startIndex + step) % count)
                    : (input.reverse ? count - 1U - step : step);
                std::visit([&](const auto& segment)
                {
                    using Segment = std::decay_t<decltype(segment)>;
                    if constexpr (std::is_same_v<Segment, geometry::LineGeometry>)
                    {
                        const auto& start = input.reverse ? segment.end : segment.start;
                        const auto& end = input.reverse ? segment.start : segment.end;
                        if (block.motions.empty())
                            block.motions.push_back(makeMotion(NcMotionKind::Rapid,
                                pointAxes(start, false), input, NcPlane::XY,
                                policy.preserveCurrentOutputQuantization));
                        block.motions.push_back(makeMotion(NcMotionKind::Linear,
                            pointAxes(end, false), input, NcPlane::XY,
                            policy.preserveCurrentOutputQuantization));
                    }
                    else
                    {
                        const double startParameter = input.reverse
                            ? segment.endParameter : segment.startParameter;
                        const double endParameter = input.reverse
                            ? segment.startParameter : segment.endParameter;
                        const auto start = curvePoint(segment.center, segment.axisU, segment.axisV,
                            segment.radius, startParameter);
                        const auto end = curvePoint(segment.center, segment.axisU, segment.axisV,
                            segment.radius, endParameter);
                        if (block.motions.empty())
                            block.motions.push_back(makeMotion(NcMotionKind::Rapid,
                                pointAxes(start, false), input, NcPlane::XY,
                                policy.preserveCurrentOutputQuantization));
                        const geometry::Vector3d normal = cross(segment.axisU, segment.axisV);
                        double orientation = normal.z * (segment.endParameter - segment.startParameter);
                        if (input.reverse) orientation = -orientation;
                        NcAxisWords axes;
                        axes.x = end.x; axes.y = end.y;
                        axes.i = segment.center.x - start.x;
                        axes.j = segment.center.y - start.y;
                        block.motions.push_back(makeMotion(orientation < 0.0
                                ? NcMotionKind::CircularClockwise
                                : NcMotionKind::CircularCounterclockwise,
                            axes, input, NcPlane::XY, policy.preserveCurrentOutputQuantization));
                    }
                }, polyline.segments[index]);
            }
            return block.motions.size() >= 2U;
        }
    }

    OperationResult<NcProgram> PlanarNcProgramBuilder::build
    (
        std::uint64_t contentRevision,
        const std::vector<PlanarNcEntityInput>& entities,
        const PlanarNcBuildPolicy& policy,
        const OperationContext& context
    )
    {
        OperationResult<NcProgram> result;
        if (contentRevision == 0 || entities.empty())
        {
            result.status = OperationStatus::InvalidInput;
            if (!entities.empty())
            {
                result.addDiagnostic(planarDiagnostic(DiagnosticCode::PlanarNcInputInvalid,
                    DiagnosticSeverity::Error, QStringLiteral("三轴 NC 输入无效。"),
                    entities.front(), context));
                result.diagnostics.back().context.insert(QStringLiteral("contentRevision"),
                    QVariant::fromValue<qulonglong>(contentRevision));
            }
            return result;
        }

        NcProgram program;
        program.contentRevision = contentRevision;
        program.mode = NcProgramMode::Planar3Axis;
        bool skipped = false;
        for (const PlanarNcEntityInput& input : entities)
        {
            NcEntityBlock block;
            block.metadata = input.metadata;
            bool built = false;
            if (input.sourceEntity.id != input.metadata.entityId || input.sourceEntity.id == 0)
            {
                result.addDiagnostic(planarDiagnostic(DiagnosticCode::PlanarNcInvariantViolation,
                    DiagnosticSeverity::Warning, QStringLiteral("图元编号与 NC 元数据不一致，已跳过。"),
                    input, context));
                skipped = true;
                continue;
            }

            switch (input.sourceEntity.kind)
            {
            case geometry::SourceGeometryKind::Line:
            {
                const auto* line = std::get_if<geometry::LineGeometry>(&input.sourceEntity.geometry);
                if (line != nullptr)
                {
                    const auto& start = input.reverse ? line->end : line->start;
                    const auto& end = input.reverse ? line->start : line->end;
                    block.motions.push_back(makeMotion(NcMotionKind::Rapid, pointAxes(start, false),
                        input, NcPlane::XY, policy.preserveCurrentOutputQuantization));
                    block.motions.push_back(makeMotion(NcMotionKind::Linear, pointAxes(end, false),
                        input, NcPlane::XY, policy.preserveCurrentOutputQuantization));
                    built = true;
                }
                break;
            }
            case geometry::SourceGeometryKind::Arc:
            {
                const auto* arc = std::get_if<geometry::ArcGeometry>(&input.sourceEntity.geometry);
                if (arc != nullptr)
                {
                    const PrincipalPlane plane = principalPlane(cross(arc->axisU, arc->axisV),
                        policy.principalPlaneTolerance);
                    built = plane == PrincipalPlane::Other
                        ? appendCompiledPath(input, policy, context, true, block, result.diagnostics)
                        : appendExactArc(*arc, input.reverse, input, policy, block);
                }
                break;
            }
            case geometry::SourceGeometryKind::Circle:
            {
                const auto* circle = std::get_if<geometry::CircleGeometry>(&input.sourceEntity.geometry);
                if (circle != nullptr)
                {
                    const PrincipalPlane plane = principalPlane(cross(circle->axisU, circle->axisV),
                        policy.principalPlaneTolerance);
                    if (plane == PrincipalPlane::XY)
                    {
                        const double startParameter = input.startParameter.value_or(kHalfPi);
                        const auto start = curvePoint(circle->center, circle->axisU, circle->axisV,
                            circle->radius, startParameter);
                        NcAxisWords circular;
                        circular.x = start.x; circular.y = start.y;
                        circular.i = circle->center.x - start.x;
                        circular.j = circle->center.y - start.y;
                        const double orientation = cross(circle->axisU, circle->axisV).z
                            * (input.reverse ? -1.0 : 1.0);
                        block.motions.push_back(makeMotion(NcMotionKind::Rapid,
                            pointAxes(start, false), input, NcPlane::XY,
                            policy.preserveCurrentOutputQuantization));
                        block.motions.push_back(makeMotion(orientation < 0.0
                                ? NcMotionKind::CircularClockwise
                                : NcMotionKind::CircularCounterclockwise,
                            circular, input, NcPlane::XY, policy.preserveCurrentOutputQuantization));
                        built = true;
                    }
                    else
                    {
                        built = appendCompiledPath
                            (input, policy, context, true, block, result.diagnostics);
                    }
                }
                break;
            }
            case geometry::SourceGeometryKind::Polyline:
            {
                const auto* polyline = std::get_if<geometry::PolylineGeometry>
                    (&input.sourceEntity.geometry);
                if (polyline != nullptr) built = appendPolyline(*polyline, input, policy, block);
                break;
            }
            case geometry::SourceGeometryKind::Ellipse:
                built = appendCompiledPath(input, policy, context, false, block, result.diagnostics);
                break;
            case geometry::SourceGeometryKind::Spline:
                built = appendCompiledPath(input, policy, context, true, block, result.diagnostics);
                break;
            default:
                break;
            }

            if (!built || block.motions.size() < 2U)
            {
                result.addDiagnostic(planarDiagnostic(DiagnosticCode::PlanarNcUnsupportedGeometry,
                    DiagnosticSeverity::Warning,
                    QStringLiteral("图元无法生成有效的三轴 NC 运动，已跳过。"), input, context));
                skipped = true;
                continue;
            }
            block.metadata.processOrder = static_cast<int>(program.entities.size());
            program.entities.push_back(std::move(block));
        }

        if (program.entities.empty())
        {
            result.status = OperationStatus::Failed;
            result.addDiagnostic(planarDiagnostic(DiagnosticCode::PlanarNcInvariantViolation,
                DiagnosticSeverity::Error,
                QStringLiteral("所有图元均无法生成三轴 NC 运动。"),
                entities.front(), context));
            for (Diagnostic& diagnostic : result.diagnostics)
                diagnostic.context.insert(QStringLiteral("contentRevision"),
                    QVariant::fromValue<qulonglong>(contentRevision));
            return result;
        }
        for (Diagnostic& diagnostic : result.diagnostics)
            diagnostic.context.insert(QStringLiteral("contentRevision"),
                QVariant::fromValue<qulonglong>(contentRevision));
        result.status = skipped ? OperationStatus::PartialSuccess : OperationStatus::Success;
        result.value = std::move(program);
        return result;
    }
}
