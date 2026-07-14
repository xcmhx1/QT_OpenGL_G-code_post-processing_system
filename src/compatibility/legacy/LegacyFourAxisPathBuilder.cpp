#include "compatibility/legacy/LegacyFourAxisPathBuilder.h"

#include <algorithm>
#include <cmath>

namespace
{
    constexpr double kAxisEps = 1.0e-8;
    constexpr double kPlaneEps = 1.0e-8;
    constexpr double kRadToDeg = 57.2957795130823208768;

    double normalizeAngle180(double angleDeg)
    {
        while (angleDeg > 180.0)
        {
            angleDeg -= 360.0;
        }
        while (angleDeg <= -180.0)
        {
            angleDeg += 360.0;
        }
        return angleDeg;
    }

    double unwrapAngleNear(double previousDeg, double currentDeg)
    {
        while (currentDeg - previousDeg > 180.0)
        {
            currentDeg -= 360.0;
        }
        while (currentDeg - previousDeg < -180.0)
        {
            currentDeg += 360.0;
        }
        return currentDeg;
    }

    Diagnostic emptyPathDiagnostic
    (
        cadcam::geometry::EntityId entityId,
        const OperationContext& context
    )
    {
        Diagnostic diagnostic;
        diagnostic.code = DiagnosticCode::InvalidGeometry;
        diagnostic.severity = DiagnosticSeverity::Error;
        diagnostic.component = QStringLiteral("LegacyFourAxisPathBuilder");
        diagnostic.operation = QStringLiteral("BuildFourAxisPath");
        diagnostic.stage = QStringLiteral("ValidateRawPath");
        diagnostic.userMessage = QStringLiteral("原始路径点集为空，无法生成四轴控制点。");
        diagnostic.technicalDetail = QStringLiteral("RawPathPoint3D vector is empty");
        diagnostic.correlationId = context.correlationId;
        diagnostic.entityId = entityId;
        return diagnostic;
    }
}

OperationResult<std::vector<ControlPoint4Axis>> LegacyFourAxisPathBuilder::build
(
    const std::vector<RawPathPoint3D>& rawPath,
    const LegacyFourAxisPathOptions& options,
    cadcam::geometry::EntityId entityId,
    const OperationContext& context
)
{
    OperationResult<std::vector<ControlPoint4Axis>> result;
    if (rawPath.empty())
    {
        result.status = OperationStatus::InvalidInput;
        result.addDiagnostic(emptyPathDiagnostic(entityId, context));
        return result;
    }

    std::vector<ControlPoint4Axis> controlPoints;
    controlPoints.reserve(rawPath.size());
    auto applyAnglePolicy = [&options]
    (double rawA, bool hasPrevious, double previousA)
    {
        if (options.invertAAxisDirection)
        {
            rawA = -rawA;
        }
        rawA += options.aAxisOffsetDegrees;
        rawA = normalizeAngle180(rawA);
        if (hasPrevious && options.keepContinuousAngle)
        {
            rawA = unwrapAngleNear(previousA, rawA);
        }
        return rawA;
    };

    double minY = rawPath.front().y;
    double maxY = rawPath.front().y;
    double minZ = rawPath.front().z;
    double maxZ = rawPath.front().z;
    double sumY = 0.0;
    double sumZ = 0.0;
    for (const RawPathPoint3D& point : rawPath)
    {
        minY = std::min(minY, point.y);
        maxY = std::max(maxY, point.y);
        minZ = std::min(minZ, point.z);
        maxZ = std::max(maxZ, point.z);
        sumY += point.y;
        sumZ += point.z;
    }

    const double avgY = sumY / static_cast<double>(rawPath.size());
    const double avgZ = sumZ / static_cast<double>(rawPath.size());
    const bool inPlaneParallelXZ = (maxY - minY) < kPlaneEps;
    const bool inPlaneParallelXY = (maxZ - minZ) < kPlaneEps;
    bool useFixedA = false;
    double fixedRawA = 0.0;

    if (inPlaneParallelXZ)
    {
        if (avgY > options.judgeCenterY + kPlaneEps)
        {
            fixedRawA = 90.0;
        }
        else if (avgY < options.judgeCenterY - kPlaneEps)
        {
            fixedRawA = -90.0;
        }
        else
        {
            fixedRawA = avgZ >= options.judgeCenterZ ? 0.0 : 180.0;
        }
        useFixedA = true;
    }
    else if (inPlaneParallelXY)
    {
        if (avgZ > options.judgeCenterZ + kPlaneEps)
        {
            fixedRawA = 0.0;
        }
        else if (avgZ < options.judgeCenterZ - kPlaneEps)
        {
            fixedRawA = 180.0;
        }
        else
        {
            fixedRawA = avgY >= options.judgeCenterY ? 90.0 : -90.0;
        }
        useFixedA = true;
    }

    if (useFixedA)
    {
        const double fixedA = applyAnglePolicy(fixedRawA, false, 0.0);
        const double angleRad = fixedA / kRadToDeg;
        const double cosine = std::cos(angleRad);
        const double sine = std::sin(angleRad);
        for (const RawPathPoint3D& point : rawPath)
        {
            const double dy = point.y - options.axisY;
            const double dz = point.z - options.axisZ;
            controlPoints.push_back
            ({
                point.x,
                options.axisY + dy * cosine - dz * sine,
                options.axisZ + dy * sine + dz * cosine,
                fixedA
            });
        }
    }
    else
    {
        bool hasPrevious = false;
        double previousA = 0.0;
        for (const RawPathPoint3D& point : rawPath)
        {
            const double dy = point.y - options.axisY;
            const double dz = point.z - options.axisZ;
            const double radiusSquared = dy * dy + dz * dz;
            const double radius = std::sqrt(radiusSquared);
            double aDeg = 0.0;
            if (radiusSquared < kAxisEps * kAxisEps)
            {
                aDeg = hasPrevious
                    ? previousA
                    : applyAnglePolicy(0.0, false, 0.0);
            }
            else
            {
                const double rawA = std::atan2(dy, dz) * kRadToDeg;
                aDeg = applyAnglePolicy(rawA, hasPrevious, previousA);
            }
            controlPoints.push_back
                ({ point.x, options.axisY, options.axisZ + radius, aDeg });
            previousA = aDeg;
            hasPrevious = true;
        }
    }

    result.status = OperationStatus::Success;
    result.value = std::move(controlPoints);
    return result;
}
