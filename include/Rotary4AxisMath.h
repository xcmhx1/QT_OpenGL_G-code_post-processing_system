#pragma once

#include "Rotary4AxisTypes.h"

double normalizeAngleDeg(double angleDeg);
double unwrapAngleDeg(double prev, double current);

double commandAngleFromMathDeg(double mathAngleDeg, const RotaryConfig& config);
double mathAngleFromCommandDeg(double commandAngleDeg, const RotaryConfig& config);

double computeTargetAFromNormal
(
    const QVector3D& surfaceNormal,
    const RotaryConfig& config,
    bool* valid = nullptr
);

QVector<double> computeContinuousAAngles
(
    const QVector<SamplePoint>& samples,
    const RotaryConfig& config,
    const ProcessTolerance& tolerance,
    ReachabilityResult* reachability = nullptr
);

QVector3D rotatePointByA
(
    const QVector3D& localPoint,
    double commandAngleDeg,
    const RotaryConfig& config
);

