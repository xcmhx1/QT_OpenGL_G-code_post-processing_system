#include "pch.h"

#include "Rotary4AxisMath.h"

#include <cmath>

namespace
{
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kDegToRad = kPi / 180.0;
    constexpr double kRadToDeg = 180.0 / kPi;
}

double normalizeAngleDeg(double angleDeg)
{
    double normalized = std::fmod(angleDeg, 360.0);

    if (normalized > 180.0)
    {
        normalized -= 360.0;
    }
    else if (normalized <= -180.0)
    {
        normalized += 360.0;
    }

    return normalized;
}

double unwrapAngleDeg(double prev, double current)
{
    while (current - prev > 180.0)
    {
        current -= 360.0;
    }

    while (current - prev < -180.0)
    {
        current += 360.0;
    }

    return current;
}

double commandAngleFromMathDeg(double mathAngleDeg, const RotaryConfig& config)
{
    const double directionSign = config.aPositiveCCW ? 1.0 : -1.0;
    return directionSign * (mathAngleDeg + config.aOffsetDeg);
}

double mathAngleFromCommandDeg(double commandAngleDeg, const RotaryConfig& config)
{
    const double directionSign = config.aPositiveCCW ? 1.0 : -1.0;
    return directionSign * commandAngleDeg - config.aOffsetDeg;
}

double computeTargetAFromNormal
(
    const QVector3D& surfaceNormal,
    const RotaryConfig& config,
    bool* valid
)
{
    const QVector2D projectedNormal(surfaceNormal.y(), surfaceNormal.z());
    const double projectedLength = static_cast<double>(projectedNormal.length());

    if (projectedLength <= 1.0e-12)
    {
        if (valid != nullptr)
        {
            *valid = false;
        }

        return 0.0;
    }

    QVector2D targetNormal = config.targetNormalYZ;

    if (targetNormal.lengthSquared() <= 1.0e-12f)
    {
        targetNormal = QVector2D(0.0f, 1.0f);
    }
    else
    {
        targetNormal.normalize();
    }

    // 约定：
    // - 法向角与目标角均在 YZ 平面按 atan2(z, y) 计算。
    // - mathA 是把法向旋转到目标受光法向所需的几何角。
    // - commandA 再叠加机床正方向映射和零位偏移。
    const double normalAngleDeg = std::atan2(surfaceNormal.z(), surfaceNormal.y()) * kRadToDeg;
    const double targetAngleDeg = std::atan2(targetNormal.y(), targetNormal.x()) * kRadToDeg;
    const double mathAngleDeg = normalizeAngleDeg(targetAngleDeg - normalAngleDeg);
    const double commandAngleDeg = commandAngleFromMathDeg(mathAngleDeg, config);

    if (valid != nullptr)
    {
        *valid = true;
    }

    return normalizeAngleDeg(commandAngleDeg);
}

QVector<double> computeContinuousAAngles
(
    const QVector<SamplePoint>& samples,
    const RotaryConfig& config,
    const ProcessTolerance& tolerance,
    ReachabilityResult* reachability
)
{
    ReachabilityResult localReachability;
    QVector<double> wrappedAngles;
    wrappedAngles.reserve(samples.size());

    double previousAngle = 0.0;
    bool hasPreviousAngle = false;

    for (const SamplePoint& sample : samples)
    {
        if (!sample.hasNormal)
        {
            localReachability.hasDegenerateNormal = true;
            wrappedAngles.append(hasPreviousAngle ? previousAngle : 0.0);
            continue;
        }

        const double nxAbs = std::abs(static_cast<double>(sample.normal.x()));
        localReachability.nxAbsMax = std::max(localReachability.nxAbsMax, nxAbs);

        if (nxAbs > tolerance.nxThreshold)
        {
            localReachability.aOnlyReachable = false;
        }

        const QVector2D projectedNormal(sample.normal.y(), sample.normal.z());

        if (static_cast<double>(projectedNormal.length()) <= tolerance.normalEps)
        {
            localReachability.hasDegenerateNormal = true;
            wrappedAngles.append(hasPreviousAngle ? previousAngle : 0.0);
            continue;
        }

        bool valid = false;
        const double angle = sample.hasTargetA
            ? sample.targetA
            : computeTargetAFromNormal(sample.normal, config, &valid);

        if (sample.hasTargetA)
        {
            valid = true;
        }

        if (!valid)
        {
            localReachability.hasDegenerateNormal = true;
            wrappedAngles.append(hasPreviousAngle ? previousAngle : 0.0);
            continue;
        }

        wrappedAngles.append(angle);
        previousAngle = angle;
        hasPreviousAngle = true;
    }

    QVector<double> continuousAngles;
    continuousAngles.reserve(wrappedAngles.size());

    for (int index = 0; index < wrappedAngles.size(); ++index)
    {
        if (index == 0)
        {
            continuousAngles.append(wrappedAngles.at(index));
            continue;
        }

        continuousAngles.append(unwrapAngleDeg(continuousAngles.back(), wrappedAngles.at(index)));
    }

    if (reachability != nullptr)
    {
        *reachability = localReachability;
    }

    return continuousAngles;
}

QVector3D rotatePointByA
(
    const QVector3D& localPoint,
    double commandAngleDeg,
    const RotaryConfig& config
)
{
    const double mathAngleDeg = mathAngleFromCommandDeg(commandAngleDeg, config);
    const double angleRad = mathAngleDeg * kDegToRad;
    const double cosA = std::cos(angleRad);
    const double sinA = std::sin(angleRad);
    const double localY = static_cast<double>(localPoint.y()) - config.yCenter;
    const double localZ = static_cast<double>(localPoint.z()) - config.zCenter;
    const double machineY = config.yCenter + localY * cosA - localZ * sinA;
    const double machineZ = config.zCenter + localY * sinA + localZ * cosA;

    return QVector3D
    (
        localPoint.x(),
        static_cast<float>(machineY),
        static_cast<float>(machineZ)
    );
}
