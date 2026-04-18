#include "pch.h"

#include "Rotary4AxisPlanner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace
{
    constexpr double kEpsilon = 1.0e-9;

    double clamp01(double value)
    {
        return std::max(0.0, std::min(1.0, value));
    }

    double maxOrDefault(double value, double fallback)
    {
        return value > kEpsilon ? value : fallback;
    }

    ModeStatistics computeModeStatistics
    (
        const QVector<SamplePoint>& samples,
        const QVector<double>& continuousA
    )
    {
        ModeStatistics statistics;

        if (samples.size() < 2 || samples.size() != continuousA.size())
        {
            return statistics;
        }

        double minA = continuousA.front();
        double maxA = continuousA.front();
        double maxDeltaA = 0.0;
        double maxDeltaARate = 0.0;

        for (int index = 1; index < continuousA.size(); ++index)
        {
            minA = std::min(minA, continuousA.at(index));
            maxA = std::max(maxA, continuousA.at(index));

            const double deltaA = std::abs(continuousA.at(index) - continuousA.at(index - 1));
            maxDeltaA = std::max(maxDeltaA, deltaA);

            const double segmentLength = static_cast<double>((samples.at(index).pos - samples.at(index - 1).pos).length());
            const double rate = deltaA / maxOrDefault(segmentLength, 1.0e-6);
            maxDeltaARate = std::max(maxDeltaARate, rate);
        }

        statistics.aRangeDeg = maxA - minA;
        statistics.maxDeltaADeg = maxDeltaA;
        statistics.maxDeltaARateDegPerUnit = maxDeltaARate;
        return statistics;
    }

    bool edgeIsAFixed(const ToolpathPoint4Axis& left, const ToolpathPoint4Axis& right, const ProcessTolerance& tolerance)
    {
        return std::abs(right.aDeg - left.aDeg) <= tolerance.aConstTolDeg;
    }

    ToolpathSegment4Axis buildSegment
    (
        const QVector<ToolpathPoint4Axis>& points,
        int beginIndex,
        int endIndex,
        SegmentMotionMode mode
    )
    {
        ToolpathSegment4Axis segment;
        segment.mode = mode;

        for (int index = beginIndex; index <= endIndex; ++index)
        {
            segment.points.append(points.at(index));
        }

        return segment;
    }
}

ProcessMode FeatureClassifier::classify
(
    const QVector<SamplePoint>& samples,
    const RotaryConfig& rotary,
    const ProcessTolerance& tolerance,
    ReachabilityResult* reachability,
    ModeStatistics* statistics
) const
{
    ReachabilityResult localReachability;
    const QVector<double> continuousAngles = computeContinuousAAngles(samples, rotary, tolerance, &localReachability);
    const ModeStatistics localStatistics = computeModeStatistics(samples, continuousAngles);

    if (reachability != nullptr)
    {
        *reachability = localReachability;
    }

    if (statistics != nullptr)
    {
        *statistics = localStatistics;
    }

    if (samples.size() < 2 || continuousAngles.size() < 2)
    {
        return ProcessMode::XYZ_3Axis;
    }

    if (!localReachability.aOnlyReachable || localReachability.hasDegenerateNormal)
    {
        return ProcessMode::XYZ_3Axis;
    }

    const double averageA = std::accumulate(continuousAngles.cbegin(), continuousAngles.cend(), 0.0) / static_cast<double>(continuousAngles.size());

    if (localStatistics.aRangeDeg <= tolerance.aConstTolDeg
        && localStatistics.maxDeltaADeg <= tolerance.aConstTolDeg
        && localStatistics.maxDeltaARateDegPerUnit <= tolerance.aConstTolDeg)
    {
        return std::abs(averageA) <= tolerance.aConstTolDeg
            ? ProcessMode::XYZ_3Axis
            : ProcessMode::A_Indexed_XYZ;
    }

    return ProcessMode::XYZA_Continuous;
}

bool buildSamplesFromAttachedCurve
(
    const QVector<AttachedCurveSample>& attachedSamples,
    QVector<SamplePoint>& outSamples
)
{
    outSamples.clear();

    if (attachedSamples.isEmpty())
    {
        return false;
    }

    outSamples.reserve(attachedSamples.size());

    for (const AttachedCurveSample& attached : attachedSamples)
    {
        SamplePoint sample;
        sample.pos = attached.worldPos;
        sample.normal = attached.surfaceNormal;
        sample.hasNormal = attached.hasSurfaceNormal;
        outSamples.append(sample);
    }

    return !outSamples.isEmpty();
}

bool buildSamplesFromSectionByNearestProjection
(
    const QVector<QVector3D>& pathPoints,
    const CrossSection2D& section,
    QVector<SamplePoint>& outSamples,
    QStringList* warnings
)
{
    outSamples.clear();

    if (pathPoints.isEmpty())
    {
        return false;
    }

    constexpr int kCoarseSampleCount = 720;
    outSamples.reserve(pathPoints.size());

    for (int pointIndex = 0; pointIndex < pathPoints.size(); ++pointIndex)
    {
        const QVector3D& point = pathPoints.at(pointIndex);
        const QVector2D yzPoint(point.y(), point.z());
        double bestU = 0.0;
        double bestDistanceSquared = std::numeric_limits<double>::max();

        for (int sampleIndex = 0; sampleIndex < kCoarseSampleCount; ++sampleIndex)
        {
            const double u = static_cast<double>(sampleIndex) / static_cast<double>(kCoarseSampleCount);
            const QVector2D sectionPoint = section.pointAt(u);
            const QVector2D delta = yzPoint - sectionPoint;
            const double distanceSquared = static_cast<double>(delta.lengthSquared());

            if (distanceSquared < bestDistanceSquared)
            {
                bestDistanceSquared = distanceSquared;
                bestU = u;
            }
        }

        const QVector2D sectionNormal = section.outwardNormalAt(bestU);
        SamplePoint sample;
        sample.pos = point;
        sample.normal = QVector3D(0.0f, sectionNormal.x(), sectionNormal.y());
        sample.hasNormal = sectionNormal.lengthSquared() > 1.0e-12f;
        outSamples.append(sample);

        if (!sample.hasNormal && warnings != nullptr)
        {
            warnings->append(QStringLiteral("第 %1 个截面投影点法向退化，已降级为不可用点。").arg(pointIndex));
        }
    }

    return !outSamples.isEmpty();
}

QVector<ToolpathPoint4Axis> resampleForRotaryMotion
(
    const QVector<SamplePoint>& samples,
    const QVector<double>& continuousA,
    const RotaryConfig& rotary,
    const ProcessTolerance& tolerance,
    double feed,
    double power
)
{
    QVector<ToolpathPoint4Axis> points;

    if (samples.isEmpty() || samples.size() != continuousA.size())
    {
        return points;
    }

    points.reserve(samples.size() * 2);

    auto appendPoint = [&](const QVector3D& localPos, double aDeg)
    {
        ToolpathPoint4Axis point;
        point.localPos = localPos;
        point.aDeg = aDeg;
        point.machinePos = rotatePointByA(localPos, aDeg, rotary);
        point.feed = feed;
        point.power = power;
        point.laserOn = true;
        points.append(point);
    };

    appendPoint(samples.front().pos, continuousA.front());

    for (int index = 1; index < samples.size(); ++index)
    {
        const QVector3D startPos = samples.at(index - 1).pos;
        const QVector3D endPos = samples.at(index).pos;
        const double startA = continuousA.at(index - 1);
        const double endA = continuousA.at(index);
        const double linearDistance = static_cast<double>((endPos - startPos).length());
        const double deltaA = std::abs(endA - startA);
        const int stepsByLinear = static_cast<int>(std::ceil(linearDistance / maxOrDefault(tolerance.maxLinearStep, 1.0)));
        const int stepsByA = static_cast<int>(std::ceil(deltaA / maxOrDefault(tolerance.maxDeltaADegPerStep, 1.0)));
        const int stepCount = std::max(1, std::max(stepsByLinear, stepsByA));

        for (int step = 1; step <= stepCount; ++step)
        {
            const double t = static_cast<double>(step) / static_cast<double>(stepCount);
            const QVector3D localPos
            (
                static_cast<float>(startPos.x() + (endPos.x() - startPos.x()) * t),
                static_cast<float>(startPos.y() + (endPos.y() - startPos.y()) * t),
                static_cast<float>(startPos.z() + (endPos.z() - startPos.z()) * t)
            );
            const double interpolatedA = startA + (endA - startA) * t;
            appendPoint(localPos, interpolatedA);
        }
    }

    return points;
}

QVector<ToolpathSegment4Axis> segmentByAMode
(
    const QVector<ToolpathPoint4Axis>& points,
    const RotaryConfig& rotary,
    const ProcessTolerance& tolerance
)
{
    QVector<ToolpathSegment4Axis> segments;

    if (points.size() < 2)
    {
        return segments;
    }

    const int edgeCount = points.size() - 1;
    SegmentMotionMode currentMode = edgeIsAFixed(points.at(0), points.at(1), tolerance)
        ? SegmentMotionMode::AFixed
        : SegmentMotionMode::AContinuous;
    int segmentStartEdge = 0;

    for (int edgeIndex = 1; edgeIndex < edgeCount; ++edgeIndex)
    {
        const SegmentMotionMode edgeMode = edgeIsAFixed(points.at(edgeIndex), points.at(edgeIndex + 1), tolerance)
            ? SegmentMotionMode::AFixed
            : SegmentMotionMode::AContinuous;

        if (edgeMode == currentMode)
        {
            continue;
        }

        segments.append(buildSegment(points, segmentStartEdge, edgeIndex, currentMode));
        segmentStartEdge = edgeIndex;
        currentMode = edgeMode;
    }

    segments.append(buildSegment(points, segmentStartEdge, edgeCount, currentMode));

    for (ToolpathSegment4Axis& segment : segments)
    {
        if (segment.mode != SegmentMotionMode::AFixed || segment.points.isEmpty())
        {
            continue;
        }

        double averageA = 0.0;

        for (const ToolpathPoint4Axis& point : segment.points)
        {
            averageA += point.aDeg;
        }

        averageA /= static_cast<double>(segment.points.size());

        for (ToolpathPoint4Axis& point : segment.points)
        {
            point.aDeg = averageA;
            point.machinePos = rotatePointByA(point.localPos, point.aDeg, rotary);
        }
    }

    return segments;
}

QVector<SamplePoint> buildRoundedRectSectionDemoSamples
(
    const RoundedRectSection2D& section,
    double xValue,
    int sampleCount
)
{
    QVector<SamplePoint> samples;
    const int count = std::max(8, sampleCount);
    samples.reserve(count + 1);

    for (int index = 0; index <= count; ++index)
    {
        const double u = static_cast<double>(index) / static_cast<double>(count);
        const QVector2D sectionPoint = section.pointAt(u);
        const QVector2D sectionNormal = section.outwardNormalAt(u);

        SamplePoint sample;
        sample.pos = QVector3D
        (
            static_cast<float>(xValue),
            sectionPoint.x(),
            sectionPoint.y()
        );
        sample.normal = QVector3D
        (
            0.0f,
            sectionNormal.x(),
            sectionNormal.y()
        );
        sample.hasNormal = sectionNormal.lengthSquared() > static_cast<float>(kEpsilon);
        samples.append(sample);
    }

    return samples;
}
