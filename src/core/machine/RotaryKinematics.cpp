#include "core/machine/RotaryKinematics.h"

#include "core/machining/TubeSectionProjector.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace cadcam::machine
{
    namespace
    {
        constexpr double kAxisEpsilon = 1.0e-8;
        constexpr double kRadiansToDegrees = 57.2957795130823208768;

        struct PathStatistics
        {
            double minimumY = 0.0;
            double maximumY = 0.0;
            double minimumZ = 0.0;
            double maximumZ = 0.0;
            double averageY = 0.0;
            double averageZ = 0.0;
        };

        struct SurfaceClassification
        {
            RotarySurfaceRegion region = RotarySurfaceRegion::Unknown;
            const machining::TubeCornerGeometry* corner = nullptr;
        };

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
                if (dy * corner.yDirection < -tolerance
                    || dz * corner.zDirection < -tolerance) continue;
                if (std::abs(std::hypot(dy, dz) - corner.radius) <= tolerance)
                    return &corner;
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
                policy.rotaryAxisZ + dy * sine + dz * cosine
                    + policy.machiningPlaneZOffset,
                angle
            };
        }

        Diagnostic failureDiagnostic
        (
            const geometry::Path3D& path,
            const OperationContext& context,
            DiagnosticCode code = DiagnosticCode::RotaryKinematicsFailed,
            const QString& detail = QStringLiteral("Path is empty or contains non-finite coordinates.")
        )
        {
            Diagnostic diagnostic;
            diagnostic.code = code;
            diagnostic.severity = DiagnosticSeverity::Error;
            diagnostic.component = QStringLiteral("RotaryKinematics");
            diagnostic.operation = context.operationName;
            diagnostic.stage = QStringLiteral("transform");
            diagnostic.userMessage = code == DiagnosticCode::RotarySurfaceClassificationFailed
                ? QStringLiteral("加工路径无法确定稳定的方管表面。")
                : QStringLiteral("加工路径无法转换为四轴机床轨迹。");
            diagnostic.technicalDetail = detail;
            diagnostic.correlationId = context.correlationId;
            diagnostic.entityId = path.sourceEntityId;
            return diagnostic;
        }

        std::optional<PathStatistics> pathStatistics(const geometry::Path3D& path)
        {
            if (path.vertices.empty()) return std::nullopt;
            PathStatistics statistics;
            statistics.minimumY = statistics.maximumY = path.vertices.front().position.y;
            statistics.minimumZ = statistics.maximumZ = path.vertices.front().position.z;
            double sumY = 0.0;
            double sumZ = 0.0;
            for (const auto& vertex : path.vertices)
            {
                const auto& point = vertex.position;
                if (!std::isfinite(point.x) || !std::isfinite(point.y)
                    || !std::isfinite(point.z)) return std::nullopt;
                statistics.minimumY = std::min(statistics.minimumY, point.y);
                statistics.maximumY = std::max(statistics.maximumY, point.y);
                statistics.minimumZ = std::min(statistics.minimumZ, point.z);
                statistics.maximumZ = std::max(statistics.maximumZ, point.z);
                sumY += point.y;
                sumZ += point.z;
            }
            statistics.averageY = sumY / static_cast<double>(path.vertices.size());
            statistics.averageZ = sumZ / static_cast<double>(path.vertices.size());
            return statistics;
        }

        std::optional<RotarySurfaceRegion> classifyFlatSurface
        (
            const geometry::Path3D& path,
            const machining::TubeSectionModel& section,
            double tolerance,
            double numericalEpsilon,
            bool& ambiguous
        )
        {
            ambiguous = false;
            if (section.geometry.boundary.empty()) return std::nullopt;
            double minimumY = section.geometry.boundary.front().x;
            double maximumY = minimumY;
            double minimumZ = section.geometry.boundary.front().y;
            double maximumZ = minimumZ;
            for (const auto& point : section.geometry.boundary)
            {
                minimumY = std::min(minimumY, point.x);
                maximumY = std::max(maximumY, point.x);
                minimumZ = std::min(minimumZ, point.y);
                maximumZ = std::max(maximumZ, point.y);
            }

            struct Candidate
            {
                RotarySurfaceRegion region = RotarySurfaceRegion::Unknown;
                double maximumDistance = 0.0;
                double averageDistance = 0.0;
            };
            std::vector<Candidate> candidates;
            const std::array<std::pair<RotarySurfaceRegion, double>, 4> planes
            {{
                { RotarySurfaceRegion::Top, maximumZ },
                { RotarySurfaceRegion::Right, maximumY },
                { RotarySurfaceRegion::Bottom, minimumZ },
                { RotarySurfaceRegion::Left, minimumY }
            }};
            for (const auto& [region, coordinate] : planes)
            {
                double maximumDistance = 0.0;
                double totalDistance = 0.0;
                for (const auto& vertex : path.vertices)
                {
                    const double value = region == RotarySurfaceRegion::Top
                        || region == RotarySurfaceRegion::Bottom
                        ? vertex.position.z : vertex.position.y;
                    const double pointDistance = std::abs(value - coordinate);
                    maximumDistance = std::max(maximumDistance, pointDistance);
                    totalDistance += pointDistance;
                }
                if (maximumDistance <= tolerance)
                {
                    candidates.push_back
                    ({ region, maximumDistance,
                        totalDistance / static_cast<double>(path.vertices.size()) });
                }
            }
            if (candidates.empty()) return std::nullopt;
            std::sort(candidates.begin(), candidates.end(), [](const Candidate& left,
                const Candidate& right)
            {
                if (left.maximumDistance != right.maximumDistance)
                    return left.maximumDistance < right.maximumDistance;
                if (left.averageDistance != right.averageDistance)
                    return left.averageDistance < right.averageDistance;
                return static_cast<int>(left.region) < static_cast<int>(right.region);
            });
            if (candidates.size() == 1U) return candidates.front().region;
            const Candidate& first = candidates[0];
            const Candidate& second = candidates[1];
            const bool uniqueMaximum = first.maximumDistance
                < second.maximumDistance - numericalEpsilon;
            const bool uniqueAverage = std::abs(first.maximumDistance - second.maximumDistance)
                    <= numericalEpsilon
                && first.averageDistance < second.averageDistance - numericalEpsilon;
            if (uniqueMaximum || uniqueAverage) return first.region;
            ambiguous = true;
            return std::nullopt;
        }

        const machining::TubeCornerGeometry* classifyCorner
        (
            const geometry::Path3D& path,
            const std::optional<machining::TubeSectionModel>& section
        )
        {
            const machining::TubeCornerGeometry* selected = nullptr;
            for (const auto& vertex : path.vertices)
            {
                const auto* corner = matchingCorner(vertex.position, section);
                if (corner == nullptr || (selected != nullptr && selected != corner)) return nullptr;
                selected = corner;
            }
            return selected;
        }

        SurfaceClassification classifySurface
        (
            const geometry::Path3D& path,
            const PathStatistics& statistics,
            const RotaryMachinePolicy& policy,
            const std::optional<machining::TubeSectionModel>& section
        )
        {
            if (section.has_value() && !section->geometry.boundary.empty())
            {
                bool flatAmbiguous = false;
                const auto flat = classifyFlatSurface
                (
                    path, *section, policy.surfaceClassificationTolerance,
                    policy.numericalEpsilon, flatAmbiguous
                );
                if (flat.has_value()) return { *flat, nullptr };
                if (const auto* corner = classifyCorner(path, section))
                    return { RotarySurfaceRegion::Corner, corner };
                return { flatAmbiguous
                    ? RotarySurfaceRegion::Unknown : RotarySurfaceRegion::Radial, nullptr };
            }

            const double ySpan = statistics.maximumY - statistics.minimumY;
            const double zSpan = statistics.maximumZ - statistics.minimumZ;
            const double tolerance = policy.surfaceClassificationTolerance;
            const bool yConstant = ySpan <= tolerance;
            const bool zConstant = zSpan <= tolerance;
            if (yConstant && zConstant)
            {
                const double relativeY = statistics.averageY - policy.tubeCenterY;
                const double relativeZ = statistics.averageZ - policy.tubeCenterZ;
                const double absoluteY = std::abs(relativeY);
                const double absoluteZ = std::abs(relativeZ);
                if (absoluteZ > absoluteY + tolerance)
                    return { relativeZ >= 0.0
                        ? RotarySurfaceRegion::Top : RotarySurfaceRegion::Bottom, nullptr };
                if (absoluteY > absoluteZ + tolerance)
                    return { relativeY >= 0.0
                        ? RotarySurfaceRegion::Right : RotarySurfaceRegion::Left, nullptr };
                return {};
            }
            if (yConstant)
            {
                if (statistics.averageY > policy.tubeCenterY + tolerance)
                    return { RotarySurfaceRegion::Right, nullptr };
                if (statistics.averageY < policy.tubeCenterY - tolerance)
                    return { RotarySurfaceRegion::Left, nullptr };
                return { statistics.averageZ >= policy.tubeCenterZ
                    ? RotarySurfaceRegion::Top : RotarySurfaceRegion::Bottom, nullptr };
            }
            if (zConstant)
            {
                if (statistics.averageZ > policy.tubeCenterZ + tolerance)
                    return { RotarySurfaceRegion::Top, nullptr };
                if (statistics.averageZ < policy.tubeCenterZ - tolerance)
                    return { RotarySurfaceRegion::Bottom, nullptr };
                return { statistics.averageY >= policy.tubeCenterY
                    ? RotarySurfaceRegion::Right : RotarySurfaceRegion::Left, nullptr };
            }
            return { RotarySurfaceRegion::Radial, nullptr };
        }

        bool fixedSurfaceAngle(RotarySurfaceRegion region, double& angle)
        {
            switch (region)
            {
            case RotarySurfaceRegion::Top: angle = 0.0; return true;
            case RotarySurfaceRegion::Right: angle = 90.0; return true;
            case RotarySurfaceRegion::Bottom: angle = 180.0; return true;
            case RotarySurfaceRegion::Left: angle = -90.0; return true;
            default: return false;
            }
        }
    }

    geometry::Vector3d RotaryKinematics::sourceRetractPose
    (
        const geometry::Vector3d& cutEnd,
        double outwardDistance,
        const std::optional<machining::TubeSectionModel>& section,
        double tubeCenterY,
        double tubeCenterZ,
        double tolerance
    )
    {
        geometry::Vector3d normal
        {
            0.0,
            cutEnd.y - tubeCenterY,
            cutEnd.z - tubeCenterZ
        };
        if (section.has_value())
        {
            const machining::TubeSectionProjection projection =
                machining::TubeSectionProjector::project
                (
                    *section,
                    { cutEnd.y, cutEnd.z },
                    std::max(tolerance, 1.0e-8)
                );
            if (projection.valid)
            {
                switch (projection.zone)
                {
                case machining::TubeZone16::TopFace:
                    normal = { 0.0, 0.0, 1.0 };
                    break;
                case machining::TubeZone16::RightFace:
                    normal = { 0.0, 1.0, 0.0 };
                    break;
                case machining::TubeZone16::BottomFace:
                    normal = { 0.0, 0.0, -1.0 };
                    break;
                case machining::TubeZone16::LeftFace:
                    normal = { 0.0, -1.0, 0.0 };
                    break;
                case machining::TubeZone16::TopRightCorner:
                case machining::TubeZone16::BottomRightCorner:
                case machining::TubeZone16::BottomLeftCorner:
                case machining::TubeZone16::TopLeftCorner:
                {
                    const int yDirection =
                        projection.zone == machining::TubeZone16
                            ::TopRightCorner
                        || projection.zone == machining::TubeZone16
                            ::BottomRightCorner
                        ? 1 : -1;
                    const int zDirection =
                        projection.zone == machining::TubeZone16
                            ::TopRightCorner
                        || projection.zone == machining::TubeZone16
                            ::TopLeftCorner
                        ? 1 : -1;
                    const auto corner = std::find_if
                    (
                        section->corners.cbegin(),
                        section->corners.cend(),
                        [yDirection, zDirection]
                        (const machining::TubeCornerGeometry& value)
                        {
                            return value.yDirection == yDirection
                                && value.zDirection == zDirection;
                        }
                    );
                    if (corner != section->corners.cend())
                    {
                        normal =
                        {
                            0.0,
                            cutEnd.y - corner->center.x,
                            cutEnd.z - corner->center.y
                        };
                    }
                    break;
                }
                default:
                    break;
                }
            }
        }

        const double length = std::hypot(normal.y, normal.z);
        if (!std::isfinite(outwardDistance) || outwardDistance <= 0.0
            || !std::isfinite(length) || length <= std::max(tolerance, 1.0e-12))
        {
            return cutEnd;
        }
        return
        {
            cutEnd.x,
            cutEnd.y + normal.y / length * outwardDistance,
            cutEnd.z + normal.z / length * outwardDistance
        };
    }

    OperationResult<RotaryKinematicsResult> RotaryKinematics::transform
    (
        const geometry::Path3D& path,
        const RotaryMachinePolicy& policy,
        const std::optional<machining::TubeSectionModel>& section,
        const OperationContext& context
    )
    {
        OperationResult<RotaryKinematicsResult> result;
        const auto statistics = pathStatistics(path);
        if (!statistics.has_value())
        {
            result.status = OperationStatus::InvalidInput;
            result.addDiagnostic(failureDiagnostic(path, context));
            return result;
        }
        if (!std::isfinite(policy.surfaceClassificationTolerance)
            || policy.surfaceClassificationTolerance <= 0.0)
        {
            result.status = OperationStatus::InvalidInput;
            result.addDiagnostic(failureDiagnostic(path, context,
                DiagnosticCode::RotarySurfaceClassificationFailed,
                QStringLiteral("surfaceClassificationTolerance must be finite and positive.")));
            return result;
        }

        const SurfaceClassification classification = classifySurface
            (path, *statistics, policy, section);
        if (classification.region == RotarySurfaceRegion::Unknown)
        {
            result.status = OperationStatus::Failed;
            Diagnostic diagnostic = failureDiagnostic(path, context,
                DiagnosticCode::RotarySurfaceClassificationFailed,
                QStringLiteral("Path is equally close to multiple surface regions."));
            diagnostic.context.insert(QStringLiteral("ySpan"),
                statistics->maximumY - statistics->minimumY);
            diagnostic.context.insert(QStringLiteral("zSpan"),
                statistics->maximumZ - statistics->minimumZ);
            diagnostic.context.insert(QStringLiteral("surfaceTolerance"),
                policy.surfaceClassificationTolerance);
            result.addDiagnostic(diagnostic);
            return result;
        }

        std::vector<double> rawAngles;
        rawAngles.reserve(path.vertices.size());
        double fixedAngle = 0.0;
        const bool fixed = fixedSurfaceAngle(classification.region, fixedAngle);
        for (const auto& vertex : path.vertices)
        {
            const auto& point = vertex.position;
            if (fixed)
            {
                rawAngles.push_back(fixedAngle);
            }
            else if (classification.region == RotarySurfaceRegion::Corner
                && classification.corner != nullptr)
            {
                rawAngles.push_back(std::atan2
                (
                    point.y - classification.corner->center.x,
                    point.z - classification.corner->center.y
                ) * kRadiansToDegrees);
            }
            else
            {
                const double dy = point.y - policy.rotaryAxisY;
                const double dz = point.z - policy.rotaryAxisZ;
                const double radiusSquared = dy * dy + dz * dz;
                rawAngles.push_back(radiusSquared < kAxisEpsilon * kAxisEpsilon
                    ? 0.0 : std::atan2(dy, dz) * kRadiansToDegrees);
            }
        }

        RotaryKinematicsResult transformed;
        transformed.poses.reserve(path.vertices.size());
        bool hasPrevious = false;
        double previous = 0.0;
        for (std::size_t index = 0; index < path.vertices.size(); ++index)
        {
            const auto& point = path.vertices[index].position;
            const double angle = applyAnglePolicy
                (rawAngles[index], policy, hasPrevious, previous);
            if (classification.region == RotarySurfaceRegion::Radial)
            {
                const double radius = std::hypot
                    (point.y - policy.rotaryAxisY, point.z - policy.rotaryAxisZ);
                transformed.poses.push_back
                ({ point.x, policy.rotaryAxisY,
                   policy.rotaryAxisZ + radius + policy.machiningPlaneZOffset, angle });
            }
            else
            {
                transformed.poses.push_back(rotatePoint(point, angle, policy));
            }
            previous = angle;
            hasPrevious = true;
        }

        transformed.surface.entityId = path.sourceEntityId;
        transformed.surface.sourceKind = path.sourceKind;
        transformed.surface.pointCount = path.vertices.size();
        transformed.surface.ySpan = statistics->maximumY - statistics->minimumY;
        transformed.surface.zSpan = statistics->maximumZ - statistics->minimumZ;
        transformed.surface.classification = classification.region;
        transformed.surface.surfaceTolerance = policy.surfaceClassificationTolerance;
        transformed.surface.rawAStart = rawAngles.front();
        transformed.surface.rawAEnd = rawAngles.back();
        transformed.surface.alignedAStart = transformed.poses.front().aDegrees;
        transformed.surface.alignedAEnd = transformed.poses.back().aDegrees;
        result.status = OperationStatus::Success;
        result.value = std::move(transformed);
        return result;
    }
}
