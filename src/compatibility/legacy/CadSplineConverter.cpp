#include "platform/pch.h"

#include "compatibility/legacy/CadSplineConverter.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace
{
    constexpr double kMinimumTolerance = 1.0e-6;
    constexpr int kMaximumSubdivisionDepth = 12;
    constexpr size_t kMaximumPolylinePoints = 65536;
    constexpr int kFitFallbackSamplesPerSpan = 16;

    struct Point3D
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    struct HomogeneousPoint
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double w = 1.0;
    };

    Point3D operator+(const Point3D& left, const Point3D& right)
    {
        return { left.x + right.x, left.y + right.y, left.z + right.z };
    }

    Point3D operator-(const Point3D& left, const Point3D& right)
    {
        return { left.x - right.x, left.y - right.y, left.z - right.z };
    }

    Point3D operator*(const Point3D& point, double factor)
    {
        return { point.x * factor, point.y * factor, point.z * factor };
    }

    double dot(const Point3D& left, const Point3D& right)
    {
        return left.x * right.x + left.y * right.y + left.z * right.z;
    }

    double lengthSquared(const Point3D& point)
    {
        return dot(point, point);
    }

    double distance(const Point3D& left, const Point3D& right)
    {
        return std::sqrt(lengthSquared(left - right));
    }

    bool isFinite(const Point3D& point)
    {
        return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
    }

    Point3D fromCoord(const DRW_Coord& point)
    {
        return { point.x, point.y, point.z };
    }

    double pointToSegmentDistance(const Point3D& point, const Point3D& start, const Point3D& end)
    {
        const Point3D segment = end - start;
        const double segmentLengthSquared = lengthSquared(segment);

        if (segmentLengthSquared <= std::numeric_limits<double>::epsilon())
        {
            return distance(point, start);
        }

        const double factor = std::clamp(dot(point - start, segment) / segmentLengthSquared, 0.0, 1.0);
        return distance(point, start + segment * factor);
    }

    void appendDistinct(std::vector<Point3D>& points, const Point3D& point, double tolerance)
    {
        if (!isFinite(point))
        {
            return;
        }

        if (points.empty() || distance(points.back(), point) > tolerance)
        {
            points.push_back(point);
        }
    }

    bool validateKnotData(const DRW_Spline* spline)
    {
        if (spline == nullptr || spline->degree < 1)
        {
            return false;
        }

        const size_t controlCount = spline->controllist.size();
        const size_t expectedKnotCount = controlCount + static_cast<size_t>(spline->degree) + 1;

        if (controlCount <= static_cast<size_t>(spline->degree)
            || spline->knotslist.size() < expectedKnotCount)
        {
            return false;
        }

        for (size_t index = 0; index < expectedKnotCount; ++index)
        {
            if (!std::isfinite(spline->knotslist[index]))
            {
                return false;
            }

            if (index > 0 && spline->knotslist[index] < spline->knotslist[index - 1])
            {
                return false;
            }
        }

        for (const std::shared_ptr<DRW_Coord>& controlPoint : spline->controllist)
        {
            if (controlPoint == nullptr || !isFinite(fromCoord(*controlPoint)))
            {
                return false;
            }
        }

        return true;
    }

    class SplineEvaluator
    {
    public:
        explicit SplineEvaluator(const DRW_Spline* spline)
            : m_spline(spline)
            , m_degree(spline != nullptr ? spline->degree : 0)
            , m_controlCount(spline != nullptr ? spline->controllist.size() : 0)
        {
        }

        Point3D evaluate(double parameter) const
        {
            const int span = findSpan(parameter);
            std::vector<HomogeneousPoint> working(static_cast<size_t>(m_degree) + 1);

            for (int index = 0; index <= m_degree; ++index)
            {
                const size_t controlIndex = static_cast<size_t>(span - m_degree + index);
                const DRW_Coord& control = *m_spline->controllist[controlIndex];
                double weight = 1.0;

                if (controlIndex < m_spline->weightlist.size()
                    && std::isfinite(m_spline->weightlist[controlIndex])
                    && std::abs(m_spline->weightlist[controlIndex]) > 1.0e-15)
                {
                    weight = m_spline->weightlist[controlIndex];
                }

                working[static_cast<size_t>(index)] =
                {
                    control.x * weight,
                    control.y * weight,
                    control.z * weight,
                    weight
                };
            }

            for (int level = 1; level <= m_degree; ++level)
            {
                for (int index = m_degree; index >= level; --index)
                {
                    const int knotIndex = span - m_degree + index;
                    const double leftKnot = m_spline->knotslist[static_cast<size_t>(knotIndex)];
                    const double rightKnot = m_spline->knotslist[static_cast<size_t>(knotIndex + m_degree - level + 1)];
                    const double denominator = rightKnot - leftKnot;
                    const double alpha = std::abs(denominator) > 1.0e-15
                        ? std::clamp((parameter - leftKnot) / denominator, 0.0, 1.0)
                        : 0.0;
                    HomogeneousPoint& current = working[static_cast<size_t>(index)];
                    const HomogeneousPoint& previous = working[static_cast<size_t>(index - 1)];

                    current.x = (1.0 - alpha) * previous.x + alpha * current.x;
                    current.y = (1.0 - alpha) * previous.y + alpha * current.y;
                    current.z = (1.0 - alpha) * previous.z + alpha * current.z;
                    current.w = (1.0 - alpha) * previous.w + alpha * current.w;
                }
            }

            const HomogeneousPoint& result = working[static_cast<size_t>(m_degree)];
            if (std::abs(result.w) <= 1.0e-15)
            {
                return {};
            }

            return { result.x / result.w, result.y / result.w, result.z / result.w };
        }

    private:
        int findSpan(double parameter) const
        {
            const int lastControlIndex = static_cast<int>(m_controlCount) - 1;
            const double endParameter = m_spline->knotslist[m_controlCount];

            if (parameter >= endParameter)
            {
                return lastControlIndex;
            }

            int low = m_degree;
            int high = lastControlIndex + 1;
            int middle = (low + high) / 2;

            while (parameter < m_spline->knotslist[static_cast<size_t>(middle)]
                || parameter >= m_spline->knotslist[static_cast<size_t>(middle + 1)])
            {
                if (parameter < m_spline->knotslist[static_cast<size_t>(middle)])
                {
                    high = middle;
                }
                else
                {
                    low = middle;
                }

                middle = (low + high) / 2;
            }

            return middle;
        }

        const DRW_Spline* m_spline = nullptr;
        int m_degree = 0;
        size_t m_controlCount = 0;
    };

    void tessellateSpan
    (
        const SplineEvaluator& evaluator,
        double startParameter,
        double endParameter,
        const Point3D& startPoint,
        const Point3D& endPoint,
        double tolerance,
        double maximumSegmentLength,
        int depth,
        std::vector<Point3D>& points
    )
    {
        if (points.size() >= kMaximumPolylinePoints)
        {
            return;
        }

        const double middleParameter = (startParameter + endParameter) * 0.5;
        const double firstQuarterParameter = (startParameter + middleParameter) * 0.5;
        const double thirdQuarterParameter = (middleParameter + endParameter) * 0.5;
        const Point3D middlePoint = evaluator.evaluate(middleParameter);
        const Point3D firstQuarterPoint = evaluator.evaluate(firstQuarterParameter);
        const Point3D thirdQuarterPoint = evaluator.evaluate(thirdQuarterParameter);
        const double deviation = std::max
        ({
            pointToSegmentDistance(middlePoint, startPoint, endPoint),
            pointToSegmentDistance(firstQuarterPoint, startPoint, endPoint),
            pointToSegmentDistance(thirdQuarterPoint, startPoint, endPoint)
        });
        const bool preciseEnough = deviation <= tolerance
            && distance(startPoint, endPoint) <= maximumSegmentLength;

        if (preciseEnough || depth >= kMaximumSubdivisionDepth)
        {
            appendDistinct(points, endPoint, tolerance * 0.01);
            return;
        }

        tessellateSpan
        (
            evaluator,
            startParameter,
            middleParameter,
            startPoint,
            middlePoint,
            tolerance,
            maximumSegmentLength,
            depth + 1,
            points
        );
        tessellateSpan
        (
            evaluator,
            middleParameter,
            endParameter,
            middlePoint,
            endPoint,
            tolerance,
            maximumSegmentLength,
            depth + 1,
            points
        );
    }

    bool controlBounds(const DRW_Spline* spline, Point3D& minimum, Point3D& maximum)
    {
        bool hasPoint = false;

        for (const std::shared_ptr<DRW_Coord>& controlPoint : spline->controllist)
        {
            if (controlPoint == nullptr)
            {
                continue;
            }

            const Point3D point = fromCoord(*controlPoint);
            if (!isFinite(point))
            {
                continue;
            }

            if (!hasPoint)
            {
                minimum = maximum = point;
                hasPoint = true;
                continue;
            }

            minimum.x = std::min(minimum.x, point.x);
            minimum.y = std::min(minimum.y, point.y);
            minimum.z = std::min(minimum.z, point.z);
            maximum.x = std::max(maximum.x, point.x);
            maximum.y = std::max(maximum.y, point.y);
            maximum.z = std::max(maximum.z, point.z);
        }

        return hasPoint;
    }

    std::vector<Point3D> sampleNurbs(const DRW_Spline* spline)
    {
        std::vector<Point3D> points;

        if (!validateKnotData(spline))
        {
            return points;
        }

        Point3D minimum;
        Point3D maximum;
        if (!controlBounds(spline, minimum, maximum))
        {
            return points;
        }

        const double diagonal = distance(minimum, maximum);
        if (diagonal <= std::numeric_limits<double>::epsilon())
        {
            return points;
        }

        // Match the practical density used by arc machining: preserve curve shape
        // without creating excessive polyline vertices that destabilize sorting.
        const double tolerance = std::max(kMinimumTolerance, diagonal * 5.0e-4);
        const double maximumSegmentLength = std::max(tolerance * 32.0, diagonal / 64.0);
        const double knotTolerance = std::max(1.0e-14, std::abs(spline->tolknot));
        const size_t controlCount = spline->controllist.size();
        const int degree = spline->degree;
        const SplineEvaluator evaluator(spline);

        for (size_t knotIndex = static_cast<size_t>(degree); knotIndex < controlCount; ++knotIndex)
        {
            const double startParameter = spline->knotslist[knotIndex];
            const double endParameter = spline->knotslist[knotIndex + 1];

            if (endParameter - startParameter <= knotTolerance)
            {
                continue;
            }

            const Point3D startPoint = evaluator.evaluate(startParameter);
            const Point3D endPoint = evaluator.evaluate(endParameter);
            appendDistinct(points, startPoint, tolerance * 0.01);
            tessellateSpan
            (
                evaluator,
                startParameter,
                endParameter,
                startPoint,
                endPoint,
                tolerance,
                maximumSegmentLength,
                0,
                points
            );

            if (points.size() >= kMaximumPolylinePoints)
            {
                break;
            }
        }

        return points;
    }

    Point3D catmullRom
    (
        const Point3D& p0,
        const Point3D& p1,
        const Point3D& p2,
        const Point3D& p3,
        double parameter
    )
    {
        const double parameter2 = parameter * parameter;
        const double parameter3 = parameter2 * parameter;

        return
        {
            0.5 * ((2.0 * p1.x)
                + (-p0.x + p2.x) * parameter
                + (2.0 * p0.x - 5.0 * p1.x + 4.0 * p2.x - p3.x) * parameter2
                + (-p0.x + 3.0 * p1.x - 3.0 * p2.x + p3.x) * parameter3),
            0.5 * ((2.0 * p1.y)
                + (-p0.y + p2.y) * parameter
                + (2.0 * p0.y - 5.0 * p1.y + 4.0 * p2.y - p3.y) * parameter2
                + (-p0.y + 3.0 * p1.y - 3.0 * p2.y + p3.y) * parameter3),
            0.5 * ((2.0 * p1.z)
                + (-p0.z + p2.z) * parameter
                + (2.0 * p0.z - 5.0 * p1.z + 4.0 * p2.z - p3.z) * parameter2
                + (-p0.z + 3.0 * p1.z - 3.0 * p2.z + p3.z) * parameter3)
        };
    }

    std::vector<Point3D> sampleFitPoints(const DRW_Spline* spline)
    {
        std::vector<Point3D> fitPoints;
        for (const std::shared_ptr<DRW_Coord>& fitPoint : spline->fitlist)
        {
            if (fitPoint != nullptr && isFinite(fromCoord(*fitPoint)))
            {
                fitPoints.push_back(fromCoord(*fitPoint));
            }
        }

        if (fitPoints.size() < 2)
        {
            return {};
        }

        const bool closed = (spline->flags & (1 | 2)) != 0;
        std::vector<Point3D> points;
        const size_t spanCount = closed ? fitPoints.size() : fitPoints.size() - 1;
        points.reserve(std::min(kMaximumPolylinePoints, spanCount * kFitFallbackSamplesPerSpan + 1));

        auto pointAt = [&](long long index) -> const Point3D&
            {
                if (closed)
                {
                    const long long count = static_cast<long long>(fitPoints.size());
                    return fitPoints[static_cast<size_t>((index % count + count) % count)];
                }

                const long long clamped = std::clamp<long long>
                (
                    index,
                    0,
                    static_cast<long long>(fitPoints.size()) - 1
                );
                return fitPoints[static_cast<size_t>(clamped)];
            };

        appendDistinct(points, fitPoints.front(), kMinimumTolerance * 0.01);

        for (size_t span = 0; span < spanCount && points.size() < kMaximumPolylinePoints; ++span)
        {
            for (int sample = 1; sample <= kFitFallbackSamplesPerSpan; ++sample)
            {
                const double parameter = static_cast<double>(sample) / kFitFallbackSamplesPerSpan;
                appendDistinct
                (
                    points,
                    catmullRom
                    (
                        pointAt(static_cast<long long>(span) - 1),
                        pointAt(static_cast<long long>(span)),
                        pointAt(static_cast<long long>(span) + 1),
                        pointAt(static_cast<long long>(span) + 2),
                        parameter
                    ),
                    kMinimumTolerance * 0.01
                );
            }
        }

        return points;
    }

    void copyEntityProperties(const DRW_Spline* source, DRW_Polyline* target)
    {
        target->handle = source->handle;
        target->appData = source->appData;
        target->parentHandle = source->parentHandle;
        target->space = source->space;
        target->layer = source->layer;
        target->lineType = source->lineType;
        target->material = source->material;
        target->color = source->color;
        target->lWeight = source->lWeight;
        target->ltypeScale = source->ltypeScale;
        target->visible = source->visible;
        target->numProxyGraph = source->numProxyGraph;
        target->proxyGraphics = source->proxyGraphics;
        target->color24 = source->color24;
        target->colorName = source->colorName;
        target->transparency = source->transparency;
        target->plotStyle = source->plotStyle;
        target->shadow = source->shadow;
        target->extData = source->extData;
    }
}

std::unique_ptr<DRW_Polyline> convertSplineToPolyline(const DRW_Spline* spline)
{
    if (spline == nullptr)
    {
        return nullptr;
    }

    std::vector<Point3D> points = sampleNurbs(spline);
    if (points.size() < 2)
    {
        points = sampleFitPoints(spline);
    }

    if (points.size() < 2)
    {
        return nullptr;
    }

    const bool closed = (spline->flags & (1 | 2)) != 0;
    const double duplicateTolerance = std::max
    (
        kMinimumTolerance,
        distance(points.front(), points.back()) * 1.0e-9
    );

    if (closed && points.size() > 2 && distance(points.front(), points.back()) <= duplicateTolerance)
    {
        points.pop_back();
    }

    auto polyline = std::make_unique<DRW_Polyline>();
    copyEntityProperties(spline, polyline.get());
    polyline->flags = 8 | (closed ? 1 : 0);
    polyline->vertlist.reserve(points.size());

    for (const Point3D& point : points)
    {
        auto vertex = std::make_shared<DRW_Vertex>(point.x, point.y, point.z, 0.0);
        vertex->flags = 32;
        polyline->vertlist.push_back(std::move(vertex));
    }

    polyline->vertexcount = static_cast<int>(polyline->vertlist.size());
    return polyline;
}
