#include "pch.h"

#include "Rotary4AxisExample.h"

#include "GCodeGenerator4Axis.h"
#include "Rotary4AxisPlanner.h"

QString buildRoundedRect4AxisDemoGCode(QStringList* warnings)
{
    RoundedRectSection2D section(20.0, 15.0, 4.0, 3.0);
    const QVector<SamplePoint> samples = buildRoundedRectSectionDemoSamples(section, 0.0, 160);

    RotaryConfig rotary;
    rotary.yCenter = 0.0;
    rotary.zCenter = 0.0;
    rotary.aOffsetDeg = 0.0;
    rotary.aPositiveCCW = true;
    rotary.targetNormalYZ = QVector2D(0.0f, 1.0f);

    ProcessTolerance tolerance;
    tolerance.nxThreshold = 0.25;
    tolerance.normalEps = 1.0e-6;
    tolerance.aConstTolDeg = 0.5;
    tolerance.maxDeltaADegPerStep = 3.0;
    tolerance.maxLinearStep = 0.8;
    tolerance.fixedASwitchThresholdDeg = 0.5;

    ReachabilityResult reachability;
    ModeStatistics statistics;
    FeatureClassifier classifier;
    const ProcessMode mode = classifier.classify(samples, rotary, tolerance, &reachability, &statistics);
    const QVector<double> continuousA = computeContinuousAAngles(samples, rotary, tolerance, &reachability);
    const QVector<ToolpathPoint4Axis> resampled = resampleForRotaryMotion(samples, continuousA, rotary, tolerance, 1200.0, 100.0);
    QVector<ToolpathSegment4Axis> segments = segmentByAMode(resampled, rotary, tolerance);

    if (mode == ProcessMode::A_Indexed_XYZ && !segments.isEmpty())
    {
        for (ToolpathSegment4Axis& segment : segments)
        {
            segment.mode = SegmentMotionMode::AFixed;
        }
    }

    MachineConfig4Axis machine;
    machine.safeZ = 50.0;
    machine.useSafeZBeforeRapid = true;

    if (warnings != nullptr)
    {
        warnings->append(QStringLiteral("分类结果: %1")
            .arg(mode == ProcessMode::XYZ_3Axis
                ? QStringLiteral("XYZ_3Axis")
                : (mode == ProcessMode::A_Indexed_XYZ ? QStringLiteral("A_Indexed_XYZ") : QStringLiteral("XYZA_Continuous"))));
        warnings->append(QStringLiteral("A范围: %1 deg, 最大相邻A差: %2 deg, 最大A变化率: %3")
            .arg(statistics.aRangeDeg, 0, 'f', 3)
            .arg(statistics.maxDeltaADeg, 0, 'f', 3)
            .arg(statistics.maxDeltaARateDegPerUnit, 0, 'f', 3));
        warnings->append(QStringLiteral("A-only可达: %1, |nx|max: %2, 法向退化: %3")
            .arg(reachability.aOnlyReachable ? QStringLiteral("是") : QStringLiteral("否"))
            .arg(reachability.nxAbsMax, 0, 'f', 3)
            .arg(reachability.hasDegenerateNormal ? QStringLiteral("是") : QStringLiteral("否")));
    }

    GCodeGenerator4Axis generator;
    return generator.generate(segments, rotary, machine, warnings);
}

