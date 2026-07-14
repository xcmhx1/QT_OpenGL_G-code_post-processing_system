#include "core/machine/RotaryKinematics.h"

#include <algorithm>
#include <cmath>

namespace cadcam::machine
{
    namespace
    {
        constexpr double kAxisEpsilon = 1.0e-8;
        constexpr double kPlaneEpsilon = 1.0e-8;
        constexpr double kRadiansToDegrees = 57.2957795130823208768;

        double normalizeAngle(double value)
        {
            while (value > 180.0) value -= 360.0;
            while (value <= -180.0) value += 360.0;
            return value;
        }

        double unwrapNear(double previous, double value)
        {
            while (value - previous > 180.0) value -= 360.0;
            while (value - previous < -180.0) value += 360.0;
            return value;
        }

        double applyAnglePolicy
        (
            double raw,
            const RotaryMachinePolicy& policy,
            bool hasPrevious,
            double previous
        )
        {
            if (policy.invertAAxisDirection) raw = -raw;
            raw = normalizeAngle(raw + policy.aAxisOffsetDegrees);
            return hasPrevious && policy.keepContinuousAngle ? unwrapNear(previous, raw) : raw;
        }

        const machining::TubeCornerGeometry* matchingCorner
        (
            const geometry::Vector3d& point,
            const std::optional<machining::TubeSectionModel>& section
        )
        {
            if (!section.has_value()) return nullptr;
            for (const auto& corner : section->corners)
            {
                if (!std::isfinite(corner.radius) || corner.radius <= 0.0) continue;
                const double tolerance = std::max(0.01, corner.radius * 0.01);
                const double dy = point.y - corner.center.x;
                const double dz = point.z - corner.center.y;
                if (dy * corner.yDirection < -tolerance || dz * corner.zDirection < -tolerance) continue;
                if (std::abs(std::hypot(dy, dz) - corner.radius) <= tolerance) return &corner;
            }
            return nullptr;
        }

        MachinePose4D rotatePoint
        (
            const geometry::Vector3d& point,
            double angle,
            const RotaryMachinePolicy& policy
        )
        {
            const double radians = angle / kRadiansToDegrees;
            const double cosine = std::cos(radians);
            const double sine = std::sin(radians);
            const double dy = point.y - policy.rotaryAxisY;
            const double dz = point.z - policy.rotaryAxisZ;
            return
            {
                point.x,
                policy.rotaryAxisY + dy * cosine - dz * sine,
                policy.rotaryAxisZ + dy * sine + dz * cosine + policy.machiningPlaneZOffset,
                angle
            };
        }

        Diagnostic failureDiagnostic(const geometry::Path3D& path, const OperationContext& context)
        {
            Diagnostic diagnostic;
            diagnostic.code = DiagnosticCode::RotaryKinematicsFailed;
            diagnostic.severity = DiagnosticSeverity::Error;
            diagnostic.component = QStringLiteral("RotaryKinematics");
            diagnostic.operation = context.operationName;
            diagnostic.stage = QStringLiteral("transform");
            diagnostic.userMessage = QStringLiteral("加工路径无法转换为四轴机床轨迹。");
            diagnostic.technicalDetail = QStringLiteral("Path is empty or contains non-finite coordinates.");
            diagnostic.correlationId = context.correlationId;
            diagnostic.entityId = path.sourceEntityId;
            return diagnostic;
        }
    }

    OperationResult<std::vector<MachinePose4D>> RotaryKinematics::transform
    (
        const geometry::Path3D& path,
        const RotaryMachinePolicy& policy,
        const std::optional<machining::TubeSectionModel>& section,
        const OperationContext& context
    )
    {
        OperationResult<std::vector<MachinePose4D>> result;
        if (path.vertices.empty())
        {
            result.status = OperationStatus::InvalidInput;
            result.addDiagnostic(failureDiagnostic(path, context));
            return result;
        }

        double minimumY = path.vertices.front().position.y;
        double maximumY = minimumY;
        double minimumZ = path.vertices.front().position.z;
        double maximumZ = minimumZ;
        double sumY = 0.0;
        double sumZ = 0.0;
        for (const auto& vertex : path.vertices)
        {
            const auto& point = vertex.position;
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
            {
                result.status = OperationStatus::Failed;
                result.addDiagnostic(failureDiagnostic(path, context));
                return result;
            }
            minimumY = std::min(minimumY, point.y);
            maximumY = std::max(maximumY, point.y);
            minimumZ = std::min(minimumZ, point.z);
            maximumZ = std::max(maximumZ, point.z);
            sumY += point.y;
            sumZ += point.z;
        }

        const double averageY = sumY / path.vertices.size();
        const double averageZ = sumZ / path.vertices.size();
        bool fixed = false;
        double fixedRawAngle = 0.0;
        if (maximumY - minimumY < kPlaneEpsilon)
        {
            fixedRawAngle = averageY > policy.tubeCenterY + kPlaneEpsilon ? 90.0
                : averageY < policy.tubeCenterY - kPlaneEpsilon ? -90.0
                : averageZ >= policy.tubeCenterZ ? 0.0 : 180.0;
            fixed = true;
        }
        else if (maximumZ - minimumZ < kPlaneEpsilon)
        {
            fixedRawAngle = averageZ > policy.tubeCenterZ + kPlaneEpsilon ? 0.0
                : averageZ < policy.tubeCenterZ - kPlaneEpsilon ? 180.0
                : averageY >= policy.tubeCenterY ? 90.0 : -90.0;
            fixed = true;
        }

        std::vector<MachinePose4D> poses;
        poses.reserve(path.vertices.size());
        bool hasPrevious = false;
        double previous = 0.0;
        const double fixedAngle = applyAnglePolicy(fixedRawAngle, policy, false, 0.0);
        for (const auto& vertex : path.vertices)
        {
            const auto& point = vertex.position;
            const auto* corner = matchingCorner(point, section);
            double angle = fixedAngle;
            bool radialMachineCoordinates = false;
            if (corner != nullptr)
            {
                angle = applyAnglePolicy
                (
                    std::atan2(point.y - corner->center.x, point.z - corner->center.y) * kRadiansToDegrees,
                    policy,
                    hasPrevious,
                    previous
                );
            }
            else if (!fixed)
            {
                const double dy = point.y - policy.rotaryAxisY;
                const double dz = point.z - policy.rotaryAxisZ;
                const double radiusSquared = dy * dy + dz * dz;
                angle = radiusSquared < kAxisEpsilon * kAxisEpsilon
                    ? (hasPrevious ? previous : applyAnglePolicy(0.0, policy, false, 0.0))
                    : applyAnglePolicy(std::atan2(dy, dz) * kRadiansToDegrees, policy, hasPrevious, previous);
                radialMachineCoordinates = true;
            }

            if (radialMachineCoordinates)
            {
                const double radius = std::hypot
                    (point.y - policy.rotaryAxisY, point.z - policy.rotaryAxisZ);
                poses.push_back
                ({ point.x, policy.rotaryAxisY,
                   policy.rotaryAxisZ + radius + policy.machiningPlaneZOffset, angle });
            }
            else
            {
                poses.push_back(rotatePoint(point, angle, policy));
            }
            previous = angle;
            hasPrevious = true;
        }

        result.status = OperationStatus::Success;
        result.value = std::move(poses);
        return result;
    }
}
