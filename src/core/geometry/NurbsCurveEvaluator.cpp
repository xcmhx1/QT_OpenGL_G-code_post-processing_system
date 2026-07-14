#include "core/geometry/NurbsCurveEvaluator.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace cadcam::geometry
{
    namespace
    {
        constexpr double kWeightTolerance = 1.0e-15;

        struct HomogeneousPoint
        {
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
            double w = 1.0;
        };

        bool finiteVector(const Vector3d& point)
        {
            return std::isfinite(point.x)
                && std::isfinite(point.y)
                && std::isfinite(point.z);
        }

        QVariantMap splineContext(const SplineGeometry& spline)
        {
            return
            {
                { QStringLiteral("entityId"), 0 },
                { QStringLiteral("degree"), spline.degree },
                { QStringLiteral("controlPointCount"),
                    static_cast<qulonglong>(spline.controlPoints.size()) },
                { QStringLiteral("knotCount"),
                    static_cast<qulonglong>(spline.knots.size()) },
                { QStringLiteral("weightCount"),
                    static_cast<qulonglong>(spline.weights.size()) },
                { QStringLiteral("fitPointCount"),
                    static_cast<qulonglong>(spline.fitPoints.size()) },
                { QStringLiteral("parameterStart"), spline.parameterStart },
                { QStringLiteral("parameterEnd"), spline.parameterEnd },
                { QStringLiteral("subdivisionDepth"), -1 },
                { QStringLiteral("generatedPointCount"), 0 },
                { QStringLiteral("closed"), spline.closed },
                { QStringLiteral("periodic"), spline.periodic },
                { QStringLiteral("rational"), spline.rational }
            };
        }

        Diagnostic makeDiagnostic
        (
            const SplineGeometry& spline,
            const OperationContext& context,
            DiagnosticCode code,
            const QString& detail
        )
        {
            Diagnostic diagnostic;
            diagnostic.code = code;
            diagnostic.severity = DiagnosticSeverity::Error;
            diagnostic.component = QStringLiteral("NurbsCurveEvaluator");
            diagnostic.operation = QStringLiteral("EvaluateSpline");
            diagnostic.stage = QStringLiteral("EvaluateNurbs");
            diagnostic.userMessage = QStringLiteral("样条曲线参数无效，无法进行精确求值。");
            diagnostic.technicalDetail = detail;
            diagnostic.correlationId = context.correlationId;
            diagnostic.context = splineContext(spline);
            return diagnostic;
        }

        OperationResult<Vector3d> fail
        (
            const SplineGeometry& spline,
            const OperationContext& context,
            DiagnosticCode code,
            const QString& detail
        )
        {
            OperationResult<Vector3d> result;
            result.status = OperationStatus::InvalidInput;
            result.addDiagnostic(makeDiagnostic(spline, context, code, detail));
            return result;
        }
    }

    OperationResult<Vector3d> NurbsCurveEvaluator::evaluate
    (
        const SplineGeometry& spline,
        double parameter,
        const OperationContext& context
    ) const
    {
        const std::size_t controlCount = spline.controlPoints.size();
        if (spline.degree < 1)
        {
            return fail(spline, context, DiagnosticCode::InvalidSplineDegree,
                QStringLiteral("degree must be at least one"));
        }
        if (controlCount <= static_cast<std::size_t>(spline.degree))
        {
            return fail(spline, context, DiagnosticCode::InvalidSplineControlPoints,
                QStringLiteral("controlPointCount must be greater than degree"));
        }
        for (const Vector3d& point : spline.controlPoints)
        {
            if (!finiteVector(point))
            {
                return fail(spline, context, DiagnosticCode::InvalidSplineControlPoints,
                    QStringLiteral("control point contains NaN or infinity"));
            }
        }

        const std::size_t requiredKnotCount =
            controlCount + static_cast<std::size_t>(spline.degree) + 1U;
        if (spline.knots.size() < requiredKnotCount)
        {
            return fail(spline, context, DiagnosticCode::InvalidSplineKnots,
                QStringLiteral("knot vector is shorter than controlPointCount + degree + 1"));
        }
        for (std::size_t index = 0; index < spline.knots.size(); ++index)
        {
            if (!std::isfinite(spline.knots[index])
                || (index > 0U && spline.knots[index] < spline.knots[index - 1U]))
            {
                return fail(spline, context, DiagnosticCode::InvalidSplineKnots,
                    QStringLiteral("knot vector must be finite and nondecreasing"));
            }
        }
        for (double weight : spline.weights)
        {
            if (!std::isfinite(weight) || std::abs(weight) <= kWeightTolerance)
            {
                return fail(spline, context, DiagnosticCode::InvalidSplineWeights,
                    QStringLiteral("weights must be finite and nonzero"));
            }
        }

        const double knotStart = spline.knots[static_cast<std::size_t>(spline.degree)];
        const double knotEnd = spline.knots[controlCount];
        if (!std::isfinite(spline.parameterStart)
            || !std::isfinite(spline.parameterEnd)
            || spline.parameterEnd <= spline.parameterStart
            || spline.parameterStart != knotStart
            || spline.parameterEnd != knotEnd
            || !std::isfinite(parameter)
            || parameter < spline.parameterStart
            || parameter > spline.parameterEnd)
        {
            return fail(spline, context, DiagnosticCode::InvalidSplineParameterDomain,
                QStringLiteral("parameter or effective knot domain is invalid"));
        }

        const int lastControlIndex = static_cast<int>(controlCount) - 1;
        int span = lastControlIndex;
        if (parameter < spline.parameterEnd)
        {
            int low = spline.degree;
            int high = lastControlIndex + 1;
            span = (low + high) / 2;
            while (parameter < spline.knots[static_cast<std::size_t>(span)]
                || parameter >= spline.knots[static_cast<std::size_t>(span + 1)])
            {
                if (parameter < spline.knots[static_cast<std::size_t>(span)])
                {
                    high = span;
                }
                else
                {
                    low = span;
                }
                span = (low + high) / 2;
            }
        }

        std::vector<HomogeneousPoint> working
            (static_cast<std::size_t>(spline.degree) + 1U);
        for (int index = 0; index <= spline.degree; ++index)
        {
            const std::size_t controlIndex =
                static_cast<std::size_t>(span - spline.degree + index);
            const double weight = controlIndex < spline.weights.size()
                ? spline.weights[controlIndex]
                : 1.0;
            const Vector3d& control = spline.controlPoints[controlIndex];
            working[static_cast<std::size_t>(index)] =
            {
                control.x * weight,
                control.y * weight,
                control.z * weight,
                weight
            };
        }

        for (int level = 1; level <= spline.degree; ++level)
        {
            for (int index = spline.degree; index >= level; --index)
            {
                const int knotIndex = span - spline.degree + index;
                const double leftKnot = spline.knots[static_cast<std::size_t>(knotIndex)];
                const double rightKnot = spline.knots
                    [static_cast<std::size_t>(knotIndex + spline.degree - level + 1)];
                const double denominator = rightKnot - leftKnot;
                const double alpha = std::abs(denominator) > kWeightTolerance
                    ? std::clamp((parameter - leftKnot) / denominator, 0.0, 1.0)
                    : 0.0;
                HomogeneousPoint& current = working[static_cast<std::size_t>(index)];
                const HomogeneousPoint& previous =
                    working[static_cast<std::size_t>(index - 1)];
                current.x = (1.0 - alpha) * previous.x + alpha * current.x;
                current.y = (1.0 - alpha) * previous.y + alpha * current.y;
                current.z = (1.0 - alpha) * previous.z + alpha * current.z;
                current.w = (1.0 - alpha) * previous.w + alpha * current.w;
            }
        }

        const HomogeneousPoint& evaluated = working[static_cast<std::size_t>(spline.degree)];
        if (!std::isfinite(evaluated.w) || std::abs(evaluated.w) <= kWeightTolerance)
        {
            return fail(spline, context, DiagnosticCode::SplineEvaluationFailure,
                QStringLiteral("homogeneous result has an invalid weight"));
        }

        Vector3d point
        {
            evaluated.x / evaluated.w,
            evaluated.y / evaluated.w,
            evaluated.z / evaluated.w
        };
        if (!finiteVector(point))
        {
            return fail(spline, context, DiagnosticCode::SplineEvaluationFailure,
                QStringLiteral("evaluated point contains NaN or infinity"));
        }

        OperationResult<Vector3d> result;
        result.status = OperationStatus::Success;
        result.value = point;
        return result;
    }
}
