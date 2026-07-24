#include "core/machining/TubeSectionProjector.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <utility>

namespace cadcam::machining
{
    namespace
    {
        using geometry::Path3D;
        using geometry::Vector2d;
        using geometry::Vector3d;

        constexpr double kPi = 3.1415926535897932384626433832795;
        constexpr double kTwoPi = 2.0 * kPi;
        constexpr int kMaximumClassificationDepth = 8;

        struct RegionPrimitive
        {
            TubeZone16 zone = TubeZone16::TopFace;
            Vector2d start;
            Vector2d end;
            Vector2d center;
            double radius = 0.0;
            double startAngle = 0.0;
            double clockwiseSweep = 0.0;
            double perimeterStart = 0.0;
            double length = 0.0;
            bool arc = false;
            bool valid = false;
        };

        struct SectionLayout
        {
            std::array<RegionPrimitive, 8> regions;
            std::array<Vector2d, 8> boundaryPoints;
            std::array<double, 8> boundaryPerimeterPositions{};
            std::vector<Vector2d> shellBoundary;
            double perimeter = 0.0;
            double scale = 0.0;
            bool valid = false;
        };

        struct ProjectionCandidate
        {
            TubeZone16 zone = TubeZone16::TopFace;
            Vector2d shellPoint;
            double distance = std::numeric_limits<double>::max();
            double perimeterPosition = 0.0;
            bool boundary = false;
        };

        struct ProfileAccumulator
        {
            ProcessUnitZoneProfile profile;
            std::array<bool, kTubeZone16Count> spanInitialized{};
            double weightedDeviation = 0.0;
            double weightedLength = 0.0;
            double sampledDeviation = 0.0;
            std::size_t sampledDeviationCount = 0U;
            double totalProjectedLength = 0.0;
            bool hasEntry = false;
            bool hasExit = false;
            bool uncertain = false;
        };

        bool finite(const Vector2d& point)
        {
            return std::isfinite(point.x) && std::isfinite(point.y);
        }

        bool finite(const Vector3d& point)
        {
            return std::isfinite(point.x) && std::isfinite(point.y)
                && std::isfinite(point.z);
        }

        double distanceSquared(const Vector2d& left, const Vector2d& right)
        {
            const double dx = left.x - right.x;
            const double dy = left.y - right.y;
            return dx * dx + dy * dy;
        }

        double distance(const Vector2d& left, const Vector2d& right)
        {
            return std::sqrt(distanceSquared(left, right));
        }

        double distance(const Vector3d& left, const Vector3d& right)
        {
            const double dx = left.x - right.x;
            const double dy = left.y - right.y;
            const double dz = left.z - right.z;
            return std::sqrt(dx * dx + dy * dy + dz * dz);
        }

        double wrapPositive(double value, double period)
        {
            if (!std::isfinite(value) || !std::isfinite(period) || period <= 0.0)
                return 0.0;
            value = std::fmod(value, period);
            if (value < 0.0) value += period;
            return value;
        }

        double shortestPerimeterDelta(double from, double to, double perimeter)
        {
            double delta = wrapPositive(to - from, perimeter);
            if (delta > perimeter * 0.5) delta -= perimeter;
            return delta;
        }

        double clockwiseAngle(double startAngle, double angle)
        {
            return wrapPositive(startAngle - angle, kTwoPi);
        }

        bool pointInsideShell(const Vector2d& point, const std::vector<Vector2d>& boundary)
        {
            bool inside = false;
            if (boundary.size() < 3U) return false;
            for (std::size_t index = 0U, previous = boundary.size() - 1U;
                index < boundary.size(); previous = index++)
            {
                const Vector2d& current = boundary[index];
                const Vector2d& prior = boundary[previous];
                const bool crosses = (current.y > point.y) != (prior.y > point.y);
                if (!crosses) continue;
                const double denominator = prior.y - current.y;
                if (std::abs(denominator) <= std::numeric_limits<double>::epsilon())
                    continue;
                const double intersectionX = current.x
                    + (point.y - current.y) * (prior.x - current.x) / denominator;
                if (point.x < intersectionX) inside = !inside;
            }
            return inside;
        }

        TubeCornerGeometry cornerFor
        (
            const TubeSectionModel& section,
            int yDirection,
            int zDirection,
            double minimumY,
            double maximumY,
            double minimumZ,
            double maximumZ
        )
        {
            for (const TubeCornerGeometry& corner : section.corners)
            {
                if (corner.yDirection == yDirection && corner.zDirection == zDirection
                    && finite(corner.center) && std::isfinite(corner.radius)
                    && corner.radius > 0.0)
                {
                    return corner;
                }
            }

            TubeCornerGeometry fallback;
            fallback.yDirection = yDirection;
            fallback.zDirection = zDirection;
            const double maximumRadius = std::max(0.0,
                std::min(maximumY - minimumY, maximumZ - minimumZ) * 0.5);
            fallback.radius = std::clamp(section.cornerRadius, 0.0, maximumRadius);
            fallback.center =
            {
                yDirection > 0 ? maximumY - fallback.radius : minimumY + fallback.radius,
                zDirection > 0 ? maximumZ - fallback.radius : minimumZ + fallback.radius
            };
            return fallback;
        }

        RegionPrimitive lineRegion(TubeZone16 zone, const Vector2d& start, const Vector2d& end)
        {
            RegionPrimitive region;
            region.zone = zone;
            region.start = start;
            region.end = end;
            region.length = distance(start, end);
            region.valid = finite(start) && finite(end)
                && std::isfinite(region.length) && region.length > 0.0;
            return region;
        }

        RegionPrimitive arcRegion
        (
            TubeZone16 zone,
            const TubeCornerGeometry& corner,
            double startAngle
        )
        {
            RegionPrimitive region;
            region.zone = zone;
            region.center = corner.center;
            region.radius = corner.radius;
            region.startAngle = startAngle;
            region.clockwiseSweep = kPi * 0.5;
            region.length = region.radius * region.clockwiseSweep;
            region.arc = true;
            region.valid = finite(region.center) && std::isfinite(region.radius)
                && region.radius > 0.0 && std::isfinite(region.length);
            if (region.valid)
            {
                region.start =
                {
                    region.center.x + region.radius * std::cos(startAngle),
                    region.center.y + region.radius * std::sin(startAngle)
                };
                const double endAngle = startAngle - region.clockwiseSweep;
                region.end =
                {
                    region.center.x + region.radius * std::cos(endAngle),
                    region.center.y + region.radius * std::sin(endAngle)
                };
            }
            return region;
        }

        SectionLayout buildLayout(const TubeSectionModel& section)
        {
            SectionLayout layout;
            if (section.geometry.boundary.size() < 4U) return layout;

            double minimumY = std::numeric_limits<double>::max();
            double maximumY = std::numeric_limits<double>::lowest();
            double minimumZ = std::numeric_limits<double>::max();
            double maximumZ = std::numeric_limits<double>::lowest();
            for (const Vector2d& point : section.geometry.boundary)
            {
                if (!finite(point)) return layout;
                minimumY = std::min(minimumY, point.x);
                maximumY = std::max(maximumY, point.x);
                minimumZ = std::min(minimumZ, point.y);
                maximumZ = std::max(maximumZ, point.y);
            }
            layout.scale = std::max(maximumY - minimumY, maximumZ - minimumZ);
            if (!std::isfinite(layout.scale) || layout.scale <= 0.0) return layout;

            const TubeCornerGeometry topRight = cornerFor
                (section, 1, 1, minimumY, maximumY, minimumZ, maximumZ);
            const TubeCornerGeometry bottomRight = cornerFor
                (section, 1, -1, minimumY, maximumY, minimumZ, maximumZ);
            const TubeCornerGeometry bottomLeft = cornerFor
                (section, -1, -1, minimumY, maximumY, minimumZ, maximumZ);
            const TubeCornerGeometry topLeft = cornerFor
                (section, -1, 1, minimumY, maximumY, minimumZ, maximumZ);

            const Vector2d topLeftTop
                { topLeft.center.x, topLeft.center.y + topLeft.radius };
            const Vector2d topRightTop
                { topRight.center.x, topRight.center.y + topRight.radius };
            const Vector2d topRightRight
                { topRight.center.x + topRight.radius, topRight.center.y };
            const Vector2d bottomRightRight
                { bottomRight.center.x + bottomRight.radius, bottomRight.center.y };
            const Vector2d bottomRightBottom
                { bottomRight.center.x, bottomRight.center.y - bottomRight.radius };
            const Vector2d bottomLeftBottom
                { bottomLeft.center.x, bottomLeft.center.y - bottomLeft.radius };
            const Vector2d bottomLeftLeft
                { bottomLeft.center.x - bottomLeft.radius, bottomLeft.center.y };
            const Vector2d topLeftLeft
                { topLeft.center.x - topLeft.radius, topLeft.center.y };

            layout.regions =
            {{
                lineRegion(TubeZone16::TopFace, topLeftTop, topRightTop),
                arcRegion(TubeZone16::TopRightCorner, topRight, kPi * 0.5),
                lineRegion(TubeZone16::RightFace, topRightRight, bottomRightRight),
                arcRegion(TubeZone16::BottomRightCorner, bottomRight, 0.0),
                lineRegion(TubeZone16::BottomFace, bottomRightBottom, bottomLeftBottom),
                arcRegion(TubeZone16::BottomLeftCorner, bottomLeft, -kPi * 0.5),
                lineRegion(TubeZone16::LeftFace, bottomLeftLeft, topLeftLeft),
                arcRegion(TubeZone16::TopLeftCorner, topLeft, -kPi)
            }};

            double perimeter = 0.0;
            Vector2d lastPoint = topLeftTop;
            for (std::size_t index = 0U; index < layout.regions.size(); ++index)
            {
                RegionPrimitive& region = layout.regions[index];
                region.perimeterStart = perimeter;
                if (region.valid)
                {
                    perimeter += region.length;
                    lastPoint = region.end;
                }
                layout.boundaryPoints[index] = lastPoint;
                layout.boundaryPerimeterPositions[index] = perimeter;
            }
            if (!std::isfinite(perimeter) || perimeter <= 0.0) return layout;
            layout.perimeter = perimeter;
            layout.boundaryPerimeterPositions.back() = 0.0;

            layout.shellBoundary = section.geometry.boundary;
            if (layout.shellBoundary.size() > 1U
                && distance(layout.shellBoundary.front(), layout.shellBoundary.back())
                    <= std::max(1.0e-12, layout.scale * 1.0e-12))
            {
                layout.shellBoundary.pop_back();
            }
            layout.valid = layout.shellBoundary.size() >= 3U;
            return layout;
        }

        ProjectionCandidate projectToLine
        (
            const Vector2d& point,
            const RegionPrimitive& region
        )
        {
            ProjectionCandidate candidate;
            candidate.zone = region.zone;
            const double dx = region.end.x - region.start.x;
            const double dy = region.end.y - region.start.y;
            const double denominator = dx * dx + dy * dy;
            double parameter = denominator > 0.0
                ? ((point.x - region.start.x) * dx + (point.y - region.start.y) * dy)
                    / denominator
                : 0.0;
            parameter = std::clamp(parameter, 0.0, 1.0);
            candidate.shellPoint =
            {
                region.start.x + dx * parameter,
                region.start.y + dy * parameter
            };
            candidate.distance = distance(point, candidate.shellPoint);
            candidate.perimeterPosition =
                region.perimeterStart + region.length * parameter;
            return candidate;
        }

        ProjectionCandidate projectToArc
        (
            const Vector2d& point,
            const RegionPrimitive& region
        )
        {
            ProjectionCandidate candidate;
            candidate.zone = region.zone;
            const double angle = std::atan2
                (point.y - region.center.y, point.x - region.center.x);
            double travel = clockwiseAngle(region.startAngle, angle);
            if (travel > region.clockwiseSweep)
            {
                const double startDistance = distance(point, region.start);
                const double endDistance = distance(point, region.end);
                travel = startDistance <= endDistance ? 0.0 : region.clockwiseSweep;
            }
            const double projectedAngle = region.startAngle - travel;
            candidate.shellPoint =
            {
                region.center.x + region.radius * std::cos(projectedAngle),
                region.center.y + region.radius * std::sin(projectedAngle)
            };
            candidate.distance = distance(point, candidate.shellPoint);
            candidate.perimeterPosition =
                region.perimeterStart + region.radius * travel;
            return candidate;
        }

        TubeSectionProjection projectWithLayout
        (
            const SectionLayout& layout,
            const Vector2d& point,
            double projectionTolerance
        )
        {
            TubeSectionProjection projection;
            if (!layout.valid || !finite(point) || !std::isfinite(projectionTolerance)
                || projectionTolerance <= 0.0)
            {
                return projection;
            }

            std::vector<ProjectionCandidate> regionCandidates;
            std::vector<ProjectionCandidate> boundaryCandidates;
            regionCandidates.reserve(8U);
            boundaryCandidates.reserve(8U);
            for (const RegionPrimitive& region : layout.regions)
            {
                if (!region.valid) continue;
                regionCandidates.push_back(region.arc
                    ? projectToArc(point, region) : projectToLine(point, region));
            }
            for (std::size_t index = 0U; index < layout.boundaryPoints.size(); ++index)
            {
                ProjectionCandidate candidate;
                candidate.zone = static_cast<TubeZone16>(index * 2U + 1U);
                candidate.shellPoint = layout.boundaryPoints[index];
                candidate.distance = distance(point, candidate.shellPoint);
                candidate.perimeterPosition = layout.boundaryPerimeterPositions[index];
                candidate.boundary = true;
                boundaryCandidates.push_back(candidate);
            }
            if (regionCandidates.empty()) return projection;

            const auto candidateLess = [](const ProjectionCandidate& left,
                const ProjectionCandidate& right)
            {
                if (left.distance != right.distance) return left.distance < right.distance;
                return static_cast<std::uint8_t>(left.zone)
                    < static_cast<std::uint8_t>(right.zone);
            };
            std::sort(regionCandidates.begin(), regionCandidates.end(), candidateLess);
            std::sort(boundaryCandidates.begin(), boundaryCandidates.end(), candidateLess);

            const double boundaryBand = std::max(1.0e-9,
                std::min(projectionTolerance * 0.1, layout.scale * 1.0e-8));
            const double tieTolerance = std::max
                (1.0e-12, std::min(boundaryBand, projectionTolerance * 1.0e-3));
            ProjectionCandidate selected = regionCandidates.front();
            bool boundarySelected = false;
            if (!boundaryCandidates.empty()
                && boundaryCandidates.front().distance <= boundaryBand
                && boundaryCandidates.front().distance
                    <= regionCandidates.front().distance + tieTolerance)
            {
                selected = boundaryCandidates.front();
                boundarySelected = true;
            }

            bool ambiguous = false;
            if (boundarySelected)
            {
                if (boundaryCandidates.size() > 1U
                    && boundaryCandidates[1].distance
                        <= boundaryCandidates.front().distance + tieTolerance)
                {
                    ambiguous = true;
                }
            }
            else if (regionCandidates.size() > 1U
                && regionCandidates[1].distance
                    <= regionCandidates.front().distance + tieTolerance)
            {
                ambiguous = true;
            }

            projection.zone = selected.zone;
            projection.perimeterPosition = wrapPositive
                (selected.perimeterPosition, layout.perimeter);
            projection.absoluteDistanceToShell = selected.distance;
            const bool inside = pointInsideShell(point, layout.shellBoundary);
            projection.signedDistanceToShell = selected.distance <= tieTolerance
                ? 0.0 : inside ? -selected.distance : selected.distance;
            projection.valid = std::isfinite(selected.distance)
                && selected.distance <= projectionTolerance;
            projection.onBoundary = boundarySelected;
            if (projection.valid && selected.distance >= projectionTolerance * 0.8)
                ambiguous = true;
            projection.ambiguous = ambiguous;

            const double shellConfidence = projection.valid
                ? std::clamp(1.0 - selected.distance / projectionTolerance, 0.0, 1.0)
                : 0.0;
            double separationConfidence = 1.0;
            if (!boundarySelected && regionCandidates.size() > 1U)
            {
                separationConfidence = std::clamp
                (
                    (regionCandidates[1].distance - regionCandidates.front().distance)
                        / std::max(projectionTolerance, tieTolerance),
                    0.0, 1.0
                );
            }
            projection.confidence = shellConfidence
                * (0.5 + 0.5 * separationConfidence);
            if (projection.ambiguous)
                projection.confidence = std::min(projection.confidence, 0.49);
            return projection;
        }

        Diagnostic profileDiagnostic
        (
            DiagnosticSeverity severity,
            const QString& message,
            const QString& detail,
            const OperationContext& context,
            const ProcessUnitZoneProfile* profile,
            double projectionTolerance,
            std::size_t pathCount
        )
        {
            Diagnostic diagnostic;
            diagnostic.code = DiagnosticCode::TubeSectionProjectionFailed;
            diagnostic.severity = severity;
            diagnostic.component = QStringLiteral("TubeSectionProjector");
            diagnostic.operation = context.operationName;
            diagnostic.stage = QStringLiteral("build-process-unit-zone-profile");
            diagnostic.userMessage = message;
            diagnostic.technicalDetail = detail;
            diagnostic.correlationId = context.correlationId;
            diagnostic.context.insert(QStringLiteral("projectionTolerance"),
                projectionTolerance);
            diagnostic.context.insert(QStringLiteral("pathCount"),
                QVariant::fromValue<qulonglong>(pathCount));
            diagnostic.context.insert(QStringLiteral("occupancyMask"),
                profile != nullptr ? static_cast<int>(profile->occupancyMask) : 0);
            diagnostic.context.insert(QStringLiteral("uncertain"),
                profile != nullptr && profile->uncertain);
            return diagnostic;
        }

        std::size_t zoneIndex(TubeZone16 zone)
        {
            return static_cast<std::size_t>(static_cast<std::uint8_t>(zone));
        }

        bool boundaryZone(TubeZone16 zone)
        {
            return (zoneIndex(zone) % 2U) == 1U;
        }

        void recordZone
        (
            ProfileAccumulator& accumulator,
            TubeZone16 zone,
            double minimumX,
            double maximumX,
            double projectedLength,
            double shellDeviation
        )
        {
            const std::size_t index = zoneIndex(zone);
            TubeZoneSpan& span = accumulator.profile.zoneSpans[index];
            if (!accumulator.spanInitialized[index])
            {
                span.minimumX = std::min(minimumX, maximumX);
                span.maximumX = std::max(minimumX, maximumX);
                accumulator.spanInitialized[index] = true;
            }
            else
            {
                span.minimumX = std::min(span.minimumX, std::min(minimumX, maximumX));
                span.maximumX = std::max(span.maximumX, std::max(minimumX, maximumX));
            }
            span.projectedLength += std::max(0.0, projectedLength);
            span.maximumShellDeviation = std::max
                (span.maximumShellDeviation, std::max(0.0, shellDeviation));
            ++span.contributingSegmentCount;
            if (boundaryZone(zone))
            {
                span.occupied = true;
                accumulator.profile.occupancyMask |= tubeZoneBit(zone);
            }
        }

        void recordBoundaryTransition
        (
            ProfileAccumulator& accumulator,
            TubeZone16 from,
            TubeZone16 to,
            double x,
            double shellDeviation
        )
        {
            const int fromIndex = static_cast<int>(zoneIndex(from));
            const int toIndex = static_cast<int>(zoneIndex(to));
            if (fromIndex == toIndex) return;
            const int forward = (toIndex - fromIndex + 16) % 16;
            const int backward = (fromIndex - toIndex + 16) % 16;
            const int direction = forward <= backward ? 1 : -1;
            const int steps = std::min(forward, backward);
            int current = fromIndex;
            for (int step = 0; step < steps; ++step)
            {
                current = (current + direction + 16) % 16;
                if ((current % 2) == 1)
                {
                    recordZone(accumulator, static_cast<TubeZone16>(current),
                        x, x, 0.0, shellDeviation);
                }
            }
        }

        Vector3d midpoint(const Vector3d& left, const Vector3d& right)
        {
            return
            {
                (left.x + right.x) * 0.5,
                (left.y + right.y) * 0.5,
                (left.z + right.z) * 0.5
            };
        }

        void accumulateLeaf
        (
            ProfileAccumulator& accumulator,
            const SectionLayout& layout,
            const Vector3d& start,
            const Vector3d& end,
            const TubeSectionProjection& startProjection,
            const TubeSectionProjection& middleProjection,
            const TubeSectionProjection& endProjection,
            double minimumSegmentLength
        )
        {
            const double sourceLength = distance(start, end);
            if (!std::isfinite(sourceLength) || sourceLength <= minimumSegmentLength)
                return;

            const double perimeterDelta = startProjection.valid && endProjection.valid
                ? shortestPerimeterDelta(startProjection.perimeterPosition,
                    endProjection.perimeterPosition, layout.perimeter)
                : 0.0;
            const double projectedLength = std::sqrt
            (
                (end.x - start.x) * (end.x - start.x)
                + perimeterDelta * perimeterDelta
            );
            const double shellDeviation = std::max
            ({
                startProjection.absoluteDistanceToShell,
                middleProjection.absoluteDistanceToShell,
                endProjection.absoluteDistanceToShell
            });
            accumulator.profile.maximumShellDeviation = std::max
                (accumulator.profile.maximumShellDeviation, shellDeviation);
            accumulator.sampledDeviation += startProjection.absoluteDistanceToShell
                + middleProjection.absoluteDistanceToShell
                + endProjection.absoluteDistanceToShell;
            accumulator.sampledDeviationCount += 3U;
            if (std::isfinite(projectedLength) && projectedLength > 0.0)
            {
                accumulator.totalProjectedLength += projectedLength;
                accumulator.weightedLength += projectedLength;
                accumulator.weightedDeviation +=
                    middleProjection.absoluteDistanceToShell * projectedLength;
            }

            if (startProjection.valid && startProjection.onBoundary)
                recordZone(accumulator, startProjection.zone, start.x, start.x,
                    0.0, startProjection.absoluteDistanceToShell);
            if (endProjection.valid && endProjection.onBoundary)
                recordZone(accumulator, endProjection.zone, end.x, end.x,
                    0.0, endProjection.absoluteDistanceToShell);

            const TubeSectionProjection* mainProjection = middleProjection.valid
                ? &middleProjection : startProjection.valid
                    ? &startProjection : endProjection.valid ? &endProjection : nullptr;
            if (mainProjection != nullptr)
            {
                recordZone(accumulator, mainProjection->zone, start.x, end.x,
                    projectedLength, shellDeviation);
                if (!accumulator.hasEntry)
                {
                    accumulator.profile.entryZone = mainProjection->zone;
                    accumulator.profile.entryPerimeterPosition =
                        mainProjection->perimeterPosition;
                    accumulator.hasEntry = true;
                }
                accumulator.profile.exitZone = mainProjection->zone;
                accumulator.profile.exitPerimeterPosition =
                    mainProjection->perimeterPosition;
                accumulator.hasExit = true;
            }
            else
            {
                accumulator.uncertain = true;
            }

            if (startProjection.valid && middleProjection.valid)
                recordBoundaryTransition(accumulator, startProjection.zone,
                    middleProjection.zone, (start.x + end.x) * 0.5, shellDeviation);
            if (middleProjection.valid && endProjection.valid)
                recordBoundaryTransition(accumulator, middleProjection.zone,
                    endProjection.zone, (start.x + end.x) * 0.5, shellDeviation);
            accumulator.uncertain = accumulator.uncertain
                || startProjection.ambiguous || middleProjection.ambiguous
                || endProjection.ambiguous;
        }

        void accumulateSegment
        (
            ProfileAccumulator& accumulator,
            const SectionLayout& layout,
            const Vector3d& start,
            const Vector3d& end,
            double projectionTolerance,
            double minimumSegmentLength,
            int depth
        )
        {
            const Vector3d middle = midpoint(start, end);
            const TubeSectionProjection startProjection = projectWithLayout
                (layout, { start.y, start.z }, projectionTolerance);
            const TubeSectionProjection middleProjection = projectWithLayout
                (layout, { middle.y, middle.z }, projectionTolerance);
            const TubeSectionProjection endProjection = projectWithLayout
                (layout, { end.y, end.z }, projectionTolerance);
            const double sourceLength = distance(start, end);
            if (!std::isfinite(sourceLength) || sourceLength <= minimumSegmentLength)
                return;

            const bool differentZones = startProjection.zone != middleProjection.zone
                || middleProjection.zone != endProjection.zone;
            const bool invalidProjection = !startProjection.valid
                || !middleProjection.valid || !endProjection.valid;
            const double perimeterDelta = startProjection.valid && endProjection.valid
                ? std::abs(shortestPerimeterDelta(startProjection.perimeterPosition,
                    endProjection.perimeterPosition, layout.perimeter))
                : layout.perimeter;
            const bool largePerimeterStep = perimeterDelta > layout.perimeter / 32.0;
            const bool deviationChange =
                std::max({ startProjection.absoluteDistanceToShell,
                    middleProjection.absoluteDistanceToShell,
                    endProjection.absoluteDistanceToShell })
                - std::min({ startProjection.absoluteDistanceToShell,
                    middleProjection.absoluteDistanceToShell,
                    endProjection.absoluteDistanceToShell })
                > projectionTolerance * 0.25;
            if (depth < kMaximumClassificationDepth
                && (differentZones || invalidProjection || largePerimeterStep
                    || deviationChange))
            {
                accumulateSegment(accumulator, layout, start, middle,
                    projectionTolerance, minimumSegmentLength, depth + 1);
                accumulateSegment(accumulator, layout, middle, end,
                    projectionTolerance, minimumSegmentLength, depth + 1);
                return;
            }

            accumulateLeaf(accumulator, layout, start, end, startProjection,
                middleProjection, endProjection, minimumSegmentLength);
        }
    }

    QString tubeZoneName(TubeZone16 zone)
    {
        switch (zone)
        {
        case TubeZone16::TopFace: return QStringLiteral("TopFace");
        case TubeZone16::TopToTopRightBoundary:
            return QStringLiteral("TopToTopRightBoundary");
        case TubeZone16::TopRightCorner: return QStringLiteral("TopRightCorner");
        case TubeZone16::TopRightToRightBoundary:
            return QStringLiteral("TopRightToRightBoundary");
        case TubeZone16::RightFace: return QStringLiteral("RightFace");
        case TubeZone16::RightToBottomRightBoundary:
            return QStringLiteral("RightToBottomRightBoundary");
        case TubeZone16::BottomRightCorner: return QStringLiteral("BottomRightCorner");
        case TubeZone16::BottomRightToBottomBoundary:
            return QStringLiteral("BottomRightToBottomBoundary");
        case TubeZone16::BottomFace: return QStringLiteral("BottomFace");
        case TubeZone16::BottomToBottomLeftBoundary:
            return QStringLiteral("BottomToBottomLeftBoundary");
        case TubeZone16::BottomLeftCorner: return QStringLiteral("BottomLeftCorner");
        case TubeZone16::BottomLeftToLeftBoundary:
            return QStringLiteral("BottomLeftToLeftBoundary");
        case TubeZone16::LeftFace: return QStringLiteral("LeftFace");
        case TubeZone16::LeftToTopLeftBoundary:
            return QStringLiteral("LeftToTopLeftBoundary");
        case TubeZone16::TopLeftCorner: return QStringLiteral("TopLeftCorner");
        case TubeZone16::TopLeftToTopBoundary:
            return QStringLiteral("TopLeftToTopBoundary");
        }
        return QStringLiteral("Unknown");
    }

    TubeSectionProjection TubeSectionProjector::project
    (
        const TubeSectionModel& section,
        const Vector2d& yzPoint,
        double projectionTolerance
    )
    {
        return projectWithLayout(buildLayout(section), yzPoint, projectionTolerance);
    }

    OperationResult<ProcessUnitZoneProfile> TubeSectionProjector::buildProfile
    (
        const TubeSectionModel& section,
        const std::vector<Path3D>& orderedPaths,
        bool closed,
        double projectionTolerance,
        const OperationContext& context
    )
    {
        OperationResult<ProcessUnitZoneProfile> result;
        const SectionLayout layout = buildLayout(section);
        if (!layout.valid || !std::isfinite(projectionTolerance)
            || projectionTolerance <= 0.0 || orderedPaths.empty())
        {
            result.status = OperationStatus::InvalidInput;
            result.addDiagnostic(profileDiagnostic
            (
                DiagnosticSeverity::Error,
                QStringLiteral("方管 16 区位画像输入无效。"),
                QStringLiteral("Section layout, projection tolerance, or ordered paths are invalid."),
                context, nullptr, projectionTolerance, orderedPaths.size()
            ));
            return result;
        }

        double pathScale = 0.0;
        for (const Path3D& path : orderedPaths)
        {
            for (std::size_t index = 1U; index < path.vertices.size(); ++index)
            {
                const Vector3d& previous = path.vertices[index - 1U].position;
                const Vector3d& current = path.vertices[index].position;
                if (finite(previous) && finite(current))
                    pathScale += distance(previous, current);
            }
            if (path.closed && path.vertices.size() > 1U)
            {
                const Vector3d& last = path.vertices.back().position;
                const Vector3d& first = path.vertices.front().position;
                if (finite(last) && finite(first)) pathScale += distance(last, first);
            }
        }
        const double minimumSegmentLength = std::max
            ({ 1.0e-12, projectionTolerance * 1.0e-4, pathScale * 1.0e-12 });

        ProfileAccumulator accumulator;
        accumulator.profile.closed = closed;
        const Vector3d* firstTraversalPoint = nullptr;
        for (const Path3D& path : orderedPaths)
        {
            if (path.vertices.empty())
            {
                accumulator.uncertain = true;
                continue;
            }
            if (firstTraversalPoint == nullptr)
                firstTraversalPoint = &path.vertices.front().position;
            for (std::size_t index = 1U; index < path.vertices.size(); ++index)
            {
                const Vector3d& previous = path.vertices[index - 1U].position;
                const Vector3d& current = path.vertices[index].position;
                if (!finite(previous) || !finite(current))
                {
                    accumulator.uncertain = true;
                    continue;
                }
                accumulateSegment(accumulator, layout, previous, current,
                    projectionTolerance, minimumSegmentLength, 0);
            }
            if (path.closed && path.vertices.size() > 1U)
            {
                const Vector3d& last = path.vertices.back().position;
                const Vector3d& first = path.vertices.front().position;
                if (finite(last) && finite(first))
                {
                    accumulateSegment(accumulator, layout, last, first,
                        projectionTolerance, minimumSegmentLength, 0);
                }
                else
                {
                    accumulator.uncertain = true;
                }
            }
        }

        const double strongOccupancyThreshold = std::max
            ({ 1.0e-12, projectionTolerance * 1.0e-3,
                accumulator.totalProjectedLength * 1.0e-9 });
        for (std::size_t index = 0U; index < kTubeZone16Count; ++index)
        {
            TubeZoneSpan& span = accumulator.profile.zoneSpans[index];
            const TubeZone16 zone = static_cast<TubeZone16>(index);
            if (!boundaryZone(zone)
                && span.projectedLength > strongOccupancyThreshold)
            {
                span.occupied = true;
                accumulator.profile.occupancyMask |= tubeZoneBit(zone);
            }
            if (!span.occupied)
                accumulator.profile.occupancyMask &= ~tubeZoneBit(zone);
        }

        accumulator.profile.averageShellDeviation =
            accumulator.weightedLength > 0.0
            ? accumulator.weightedDeviation / accumulator.weightedLength
            : accumulator.sampledDeviationCount > 0U
                ? accumulator.sampledDeviation
                    / static_cast<double>(accumulator.sampledDeviationCount)
                : 0.0;
        if (!accumulator.hasEntry || !accumulator.hasExit
            || accumulator.profile.occupancyMask == 0U)
        {
            accumulator.uncertain = true;
        }
        if (closed && firstTraversalPoint != nullptr)
        {
            const TubeSectionProjection closureProjection = projectWithLayout
                (layout, { firstTraversalPoint->y, firstTraversalPoint->z },
                    projectionTolerance);
            if (closureProjection.valid && closureProjection.onBoundary)
            {
                accumulator.profile.entryZone = closureProjection.zone;
                accumulator.profile.exitZone = closureProjection.zone;
                accumulator.profile.entryPerimeterPosition =
                    closureProjection.perimeterPosition;
                accumulator.profile.exitPerimeterPosition =
                    closureProjection.perimeterPosition;
            }
            else if (accumulator.hasEntry && accumulator.hasExit
                && accumulator.profile.entryZone != accumulator.profile.exitZone)
            {
                accumulator.uncertain = true;
            }
        }
        accumulator.profile.uncertain = accumulator.uncertain;

        result.status = accumulator.profile.uncertain
            ? OperationStatus::PartialSuccess : OperationStatus::Success;
        result.value = accumulator.profile;
        if (accumulator.profile.uncertain)
        {
            result.addDiagnostic(profileDiagnostic
            (
                DiagnosticSeverity::Warning,
                QStringLiteral("方管 16 区位画像存在不确定投影。"),
                QStringLiteral("At least one path segment was outside the reliable projection band or could not be disambiguated."),
                context, &accumulator.profile, projectionTolerance, orderedPaths.size()
            ));
        }
        return result;
    }
}
