#pragma once

#include "Rotary4AxisMath.h"
#include "Rotary4AxisSection.h"

#include <QStringList>

class FeatureClassifier
{
public:
    ProcessMode classify
    (
        const QVector<SamplePoint>& samples,
        const RotaryConfig& rotary,
        const ProcessTolerance& tolerance,
        ReachabilityResult* reachability = nullptr,
        ModeStatistics* statistics = nullptr
    ) const;
};

bool buildSamplesFromAttachedCurve
(
    const QVector<AttachedCurveSample>& attachedSamples,
    QVector<SamplePoint>& outSamples
);

bool buildSamplesFromSectionByNearestProjection
(
    const QVector<QVector3D>& pathPoints,
    const CrossSection2D& section,
    QVector<SamplePoint>& outSamples,
    QStringList* warnings = nullptr
);

QVector<ToolpathPoint4Axis> resampleForRotaryMotion
(
    const QVector<SamplePoint>& samples,
    const QVector<double>& continuousA,
    const RotaryConfig& rotary,
    const ProcessTolerance& tolerance,
    double feed,
    double power
);

QVector<ToolpathSegment4Axis> segmentByAMode
(
    const QVector<ToolpathPoint4Axis>& points,
    const RotaryConfig& rotary,
    const ProcessTolerance& tolerance
);

QVector<SamplePoint> buildRoundedRectSectionDemoSamples
(
    const RoundedRectSection2D& section,
    double xValue,
    int sampleCount
);

