#include "pch.h"

#include "Rotary4AxisPlanner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace
{
    constexpr double kEpsilon = 1.0e-9;

    double maxOrDefault(double value, double fallback)
    {
        return value > kEpsilon ? value : fallback;
    }

    double snapAngleToStepAroundNeutral
    (
        double commandAngleDeg,
        const RotaryConfig& rotary,
        double stepDeg
    )
    {
        const double safeStep = std::max(stepDeg, 1.0e-6);
        const double neutralA = commandAngleFromMathDeg(0.0, rotary);
        const double deltaToNeutral = normalizeAngleDeg(commandAngleDeg - neutralA);
        const double snappedDelta = std::round(deltaToNeutral / safeStep) * safeStep;
        return neutralA + snappedDelta;
    }

    bool hasSectionRegionInfo(const QVector<SamplePoint>& samples)
    {
        for (const SamplePoint& sample : samples)
        {
            if (sample.regionType != SectionRegionType::Unknown)
            {
                return true;
            }
        }

        return false;
    }

    bool isFlatSideSample(const SamplePoint& sample)
    {
        return sample.regionType == SectionRegionType::FlatSide && sample.sideIndex >= 0;
    }

    bool isSameFlatSidePair(const SamplePoint& left, const SamplePoint& right)
    {
        return isFlatSideSample(left)
            && isFlatSideSample(right)
            && left.sideIndex == right.sideIndex;
    }

    bool isStraightPath(const QVector<QVector3D>& pathPoints)
    {
        if (pathPoints.size() < 2)
        {
            return false;
        }

        if (pathPoints.size() == 2)
        {
            return true;
        }

        const QVector3D base = pathPoints.back() - pathPoints.front();

        if (base.lengthSquared() <= 1.0e-12f)
        {
            return false;
        }

        for (int index = 1; index + 1 < pathPoints.size(); ++index)
        {
            const QVector3D delta = pathPoints.at(index) - pathPoints.front();

            if (QVector3D::crossProduct(base, delta).lengthSquared() > 1.0e-8f)
            {
                return false;
            }
        }

        return true;
    }

    void assignFlatSideInvariant
    (
        SamplePoint& sample,
        int sideIndex,
        const RoundedRectSection2D& section,
        const RotaryConfig& rotary,
        const ProcessTolerance& tolerance
    )
    {
        sample.regionType = SectionRegionType::FlatSide;
        sample.sideIndex = sideIndex;
        const QVector2D flatNormal = section.flatSideNormal(sideIndex);
        sample.normal = QVector3D(0.0f, flatNormal.x(), flatNormal.y());
        sample.hasNormal = flatNormal.lengthSquared() > 1.0e-12f;

        bool valid = false;
        const double rawA = computeTargetAFromNormal(sample.normal, rotary, &valid);
        sample.hasRawA = valid;
        sample.rawA = valid ? rawA : 0.0;

        if (valid && tolerance.enableFlatSideQuadrantSnap)
        {
            sample.snappedA = snapAngleToStepAroundNeutral(rawA, rotary, tolerance.flatSideSnapStepDeg);
            sample.hasSnappedA = true;
            sample.targetA = sample.snappedA;
            sample.hasTargetA = true;
        }
        else
        {
            sample.hasSnappedA = false;
            sample.snappedA = 0.0;
            sample.hasTargetA = false;
            sample.targetA = 0.0;
        }
    }

    bool resolveFixedAForSampleRange
    (
        const QVector<SamplePoint>& samples,
        int beginIndex,
        int endIndex,
        double& fixedA
    )
    {
        if (samples.isEmpty() || beginIndex < 0 || endIndex >= samples.size() || beginIndex > endIndex)
        {
            return false;
        }

        int sideIndex = -1;
        bool firstA = true;
        double invariantA = 0.0;

        for (int sampleIndex = beginIndex; sampleIndex <= endIndex; ++sampleIndex)
        {
            const SamplePoint& sample = samples.at(sampleIndex);

            if (!isFlatSideSample(sample) || !sample.hasTargetA)
            {
                return false;
            }

            if (sideIndex < 0)
            {
                sideIndex = sample.sideIndex;
            }
            else if (sideIndex != sample.sideIndex)
            {
                return false;
            }

            if (firstA)
            {
                invariantA = sample.targetA;
                firstA = false;
            }
            else if (std::abs(sample.targetA - invariantA) > 1.0e-9)
            {
                return false;
            }
        }

        if (firstA)
        {
            return false;
        }

        fixedA = invariantA;
        return true;
    }

    void enforceFixedASegmentInvariant
    (
        ToolpathSegment4Axis& segment,
        const QVector<SamplePoint>& samples,
        int beginIndex,
        const RotaryConfig& rotary
    )
    {
        double fixedA = 0.0;

        if (!resolveFixedAForSampleRange(samples, beginIndex, beginIndex + segment.points.size() - 1, fixedA))
        {
            segment.hasFixedA = false;
            segment.fixedADeg = 0.0;
            segment.emitAInEachLine = true;
            segment.prePositionAOnly = false;
            return;
        }

        segment.mode = SegmentMotionMode::AFixed;
        segment.hasFixedA = true;
        segment.fixedADeg = fixedA;
        segment.emitAInEachLine = false;
        segment.prePositionAOnly = true;

        for (ToolpathPoint4Axis& point : segment.points)
        {
            point.aDeg = fixedA;
            point.machinePos = rotatePointByA(point.localPos, point.aDeg, rotary);
        }
    }

    void updateSegmentMetadata(ToolpathSegment4Axis& segment, const QVector<SamplePoint>& samples, int beginIndex)
    {
        if (segment.points.isEmpty())
        {
            segment.startADeg = 0.0;
            segment.endADeg = 0.0;
            segment.aRangeDeg = 0.0;
            segment.regionSummary = QStringLiteral("Unknown");
            return;
        }

        segment.startADeg = segment.points.front().aDeg;
        segment.endADeg = segment.points.back().aDeg;
        double minA = segment.startADeg;
        double maxA = segment.startADeg;

        for (const ToolpathPoint4Axis& point : segment.points)
        {
            minA = std::min(minA, point.aDeg);
            maxA = std::max(maxA, point.aDeg);
        }

        segment.aRangeDeg = maxA - minA;

        int flatCount = 0;
        int cornerCount = 0;
        int unknownCount = 0;
        int sideIndex = -1;
        bool sameSide = true;
        const int endIndex = beginIndex + segment.points.size() - 1;

        for (int sampleIndex = beginIndex; sampleIndex <= endIndex && sampleIndex < samples.size(); ++sampleIndex)
        {
            const SamplePoint& sample = samples.at(sampleIndex);

            if (sample.regionType == SectionRegionType::FlatSide)
            {
                ++flatCount;

                if (sideIndex < 0)
                {
                    sideIndex = sample.sideIndex;
                }
                else if (sample.sideIndex != sideIndex)
                {
                    sameSide = false;
                }
            }
            else if (sample.regionType == SectionRegionType::CornerTransition)
            {
                ++cornerCount;
            }
            else
            {
                ++unknownCount;
            }
        }

        if (flatCount > 0 && cornerCount == 0 && unknownCount == 0)
        {
            segment.regionSummary = sameSide && sideIndex >= 0
                ? QStringLiteral("FlatSide(side=%1)").arg(sideIndex)
                : QStringLiteral("FlatSide");
        }
        else if (cornerCount > 0 && flatCount == 0 && unknownCount == 0)
        {
            segment.regionSummary = QStringLiteral("CornerTransition");
        }
        else if (unknownCount > 0 && flatCount == 0 && cornerCount == 0)
        {
            segment.regionSummary = QStringLiteral("Unknown");
        }
        else
        {
            segment.regionSummary = QStringLiteral("Mixed(flat=%1,corner=%2,unknown=%3)")
                .arg(flatCount)
                .arg(cornerCount)
                .arg(unknownCount);
        }
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
    const double neutralA = commandAngleFromMathDeg(0.0, rotary);
    const double neutralDelta = std::abs(averageA - neutralA);
    const bool nearNeutral = neutralDelta <= std::max(tolerance.aEnableThresholdDeg, tolerance.aNeutralToleranceDeg);
    bool hasFlat = false;
    bool hasCorner = false;

    for (const SamplePoint& sample : samples)
    {
        hasFlat = hasFlat || sample.regionType == SectionRegionType::FlatSide;
        hasCorner = hasCorner || sample.regionType == SectionRegionType::CornerTransition;
    }

    if (hasCorner && hasFlat)
    {
        return ProcessMode::XYZA_Continuous;
    }

    if (localStatistics.aRangeDeg <= tolerance.aConstTolDeg
        && localStatistics.maxDeltaADeg <= tolerance.aConstTolDeg
        && localStatistics.maxDeltaARateDegPerUnit <= tolerance.aConstTolDeg)
    {
        return nearNeutral
            ? ProcessMode::XYZ_3Axis
            : ProcessMode::A_Indexed_XYZ;
    }

    if (hasCorner && !hasFlat)
    {
        return ProcessMode::XYZA_Continuous;
    }

    if (localStatistics.aRangeDeg >= tolerance.aRangeForContinuousDeg
        || localStatistics.maxDeltaADeg > tolerance.aConstTolDeg)
    {
        return ProcessMode::XYZA_Continuous;
    }

    if (nearNeutral)
    {
        return ProcessMode::XYZ_3Axis;
    }

    return ProcessMode::A_Indexed_XYZ;
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
        sample.hasRawA = false;
        sample.rawA = 0.0;
        sample.hasSnappedA = false;
        sample.snappedA = 0.0;
        sample.hasTargetA = false;
        sample.targetA = 0.0;
        sample.regionType = SectionRegionType::Unknown;
        sample.sideIndex = -1;
        outSamples.append(sample);
    }

    return !outSamples.isEmpty();
}

bool buildSamplesFromSectionByNearestProjection
(
    const QVector<QVector3D>& pathPoints,
    const CrossSection2D& section,
    const RotaryConfig& rotary,
    const ProcessTolerance& tolerance,
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
    const RoundedRectSection2D* roundedRectSection = dynamic_cast<const RoundedRectSection2D*>(&section);

    for (int pointIndex = 0; pointIndex < pathPoints.size(); ++pointIndex)
    {
        const QVector3D& point = pathPoints.at(pointIndex);
        const QVector2D yzPoint(point.y(), point.z());
        QVector2D sectionNormal;
        SectionRegionType regionType = SectionRegionType::Unknown;
        int sideIndex = -1;

        if (roundedRectSection != nullptr)
        {
            RoundedRectSection2D::ProjectionResult projection;

            if (roundedRectSection->projectPoint(yzPoint, projection))
            {
                sectionNormal = projection.normal;
                regionType = projection.regionType;
                sideIndex = projection.sideIndex;
            }
        }

        if (regionType == SectionRegionType::Unknown)
        {
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

            sectionNormal = section.outwardNormalAt(bestU);
            regionType = section.regionTypeAt(bestU);
            sideIndex = section.sideIndexAt(bestU);
        }

        SamplePoint sample;
        sample.pos = point;
        sample.normal = QVector3D(0.0f, sectionNormal.x(), sectionNormal.y());
        sample.hasNormal = sectionNormal.lengthSquared() > 1.0e-12f;
        sample.regionType = regionType;
        sample.sideIndex = sideIndex;
        sample.hasRawA = false;
        sample.rawA = 0.0;
        sample.hasSnappedA = false;
        sample.snappedA = 0.0;
        sample.hasTargetA = false;

        if (sample.hasNormal)
        {
            bool valid = false;
            const double rawA = computeTargetAFromNormal(sample.normal, rotary, &valid);
            sample.hasRawA = valid;
            sample.rawA = valid ? rawA : 0.0;

            if (valid && sample.regionType == SectionRegionType::FlatSide && tolerance.enableFlatSideQuadrantSnap)
            {
                sample.snappedA = snapAngleToStepAroundNeutral(rawA, rotary, tolerance.flatSideSnapStepDeg);
                sample.hasSnappedA = true;
                sample.targetA = sample.snappedA;
                sample.hasTargetA = true;
            }
        }

        outSamples.append(sample);

        if (!sample.hasNormal && warnings != nullptr)
        {
            warnings->append(QStringLiteral("第 %1 个截面投影点法向退化，已降级为不可用点。").arg(pointIndex));
        }
    }

    if (roundedRectSection != nullptr
        && isStraightPath(pathPoints)
        && outSamples.size() >= 2
        && isFlatSideSample(outSamples.front())
        && isFlatSideSample(outSamples.back())
        && outSamples.front().sideIndex == outSamples.back().sideIndex)
    {
        const int lockedSideIndex = outSamples.front().sideIndex;

        for (SamplePoint& sample : outSamples)
        {
            assignFlatSideInvariant(sample, lockedSideIndex, *roundedRectSection, rotary, tolerance);
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
    Q_UNUSED(tolerance);

    QVector<ToolpathPoint4Axis> points;

    if (samples.isEmpty() || samples.size() != continuousA.size())
    {
        return points;
    }

    points.reserve(samples.size());

    for (int index = 0; index < samples.size(); ++index)
    {
        ToolpathPoint4Axis point;
        point.localPos = samples.at(index).pos;
        point.aDeg = continuousA.at(index);
        point.machinePos = rotatePointByA(point.localPos, point.aDeg, rotary);
        point.feed = feed;
        point.power = power;
        point.laserOn = true;
        points.append(point);
    }

    return points;
}

QVector<ToolpathSegment4Axis> segmentByAMode
(
    const QVector<ToolpathPoint4Axis>& points,
    const QVector<SamplePoint>& samples,
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
    const bool useRegionPreferredSplit = samples.size() == points.size() && hasSectionRegionInfo(samples);

    if (useRegionPreferredSplit)
    {
        int segmentStartEdge = 0;

        while (segmentStartEdge < edgeCount)
        {
            if (isSameFlatSidePair(samples.at(segmentStartEdge), samples.at(segmentStartEdge + 1)))
            {
                const int fixedSideIndex = samples.at(segmentStartEdge).sideIndex;
                int segmentEndEdge = segmentStartEdge;

                while (segmentEndEdge + 1 < edgeCount
                    && isSameFlatSidePair(samples.at(segmentEndEdge + 1), samples.at(segmentEndEdge + 2))
                    && samples.at(segmentEndEdge + 1).sideIndex == fixedSideIndex)
                {
                    ++segmentEndEdge;
                }

                segments.append(buildSegment(points, segmentStartEdge, segmentEndEdge + 1, SegmentMotionMode::AFixed));
                segmentStartEdge = segmentEndEdge + 1;
                continue;
            }

            int segmentEndEdge = segmentStartEdge;

            while (segmentEndEdge + 1 < edgeCount
                && !isSameFlatSidePair(samples.at(segmentEndEdge + 1), samples.at(segmentEndEdge + 2)))
            {
                ++segmentEndEdge;
            }

            segments.append(buildSegment(points, segmentStartEdge, segmentEndEdge + 1, SegmentMotionMode::AContinuous));
            segmentStartEdge = segmentEndEdge + 1;
        }
    }
    else
    {
        int segmentStartEdge = 0;

        while (segmentStartEdge < edgeCount)
        {
            double minA = points.at(segmentStartEdge).aDeg;
            double maxA = minA;
            int edgeIndex = segmentStartEdge;
            SegmentMotionMode mode = SegmentMotionMode::AFixed;

            for (; edgeIndex < edgeCount; ++edgeIndex)
            {
                const double leftA = points.at(edgeIndex).aDeg;
                const double rightA = points.at(edgeIndex + 1).aDeg;
                minA = std::min(minA, std::min(leftA, rightA));
                maxA = std::max(maxA, std::max(leftA, rightA));

                if ((maxA - minA) > tolerance.aConstTolDeg)
                {
                    mode = SegmentMotionMode::AContinuous;
                    break;
                }
            }

            if (mode == SegmentMotionMode::AFixed)
            {
                segments.append(buildSegment(points, segmentStartEdge, edgeCount, mode));
                break;
            }

            if (edgeIndex > segmentStartEdge)
            {
                segments.append(buildSegment(points, segmentStartEdge, edgeIndex, SegmentMotionMode::AFixed));
            }

            int continuousEndEdge = edgeIndex;
            double continuousMinA = std::min(points.at(edgeIndex).aDeg, points.at(edgeIndex + 1).aDeg);
            double continuousMaxA = std::max(points.at(edgeIndex).aDeg, points.at(edgeIndex + 1).aDeg);

            while (continuousEndEdge + 1 < edgeCount)
            {
                const double nextA = points.at(continuousEndEdge + 2).aDeg;
                const double candidateMin = std::min(continuousMinA, nextA);
                const double candidateMax = std::max(continuousMaxA, nextA);

                if ((candidateMax - candidateMin) <= tolerance.aConstTolDeg)
                {
                    break;
                }

                continuousMinA = candidateMin;
                continuousMaxA = candidateMax;
                ++continuousEndEdge;
            }

            segments.append(buildSegment(points, edgeIndex, continuousEndEdge + 1, SegmentMotionMode::AContinuous));
            segmentStartEdge = continuousEndEdge + 1;
        }
    }

    int sampleBeginIndex = 0;
    for (ToolpathSegment4Axis& segment : segments)
    {
        if (segment.mode == SegmentMotionMode::AFixed && !segment.points.isEmpty())
        {
            enforceFixedASegmentInvariant(segment, samples, sampleBeginIndex, rotary);
        }

        updateSegmentMetadata(segment, samples, sampleBeginIndex);
        sampleBeginIndex += segment.points.size() > 1 ? (segment.points.size() - 1) : 0;
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
        sample.hasRawA = false;
        sample.rawA = 0.0;
        sample.hasSnappedA = false;
        sample.snappedA = 0.0;
        sample.regionType = section.regionTypeAt(u);
        sample.sideIndex = section.sideIndexAt(u);
        sample.hasTargetA = false;
        sample.targetA = 0.0;
        samples.append(sample);
    }

    return samples;
}
