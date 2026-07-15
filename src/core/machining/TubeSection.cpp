#include "platform/pch.h"

#include "core/machining/TubeSection.h"
#include "core/geometry/LocalCoordinateFrame.h"

#include <QVariantList>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>

namespace cadcam::machining
{
    namespace
    {
        using geometry::EntityId;
        using geometry::LocalFrame2d;
        using geometry::LocalFrame3d;
        using geometry::Vector2d;
        using geometry::Vector3d;
        using topology::TopologyInput;
        using topology::TopologyLoopResult;
        using topology::TopologyPathRecord;
        using topology::PathTopologyTolerance;

        struct SectionCandidate
        {
            TubeSectionModel model;
            double area = 0.0;
            std::size_t minimumSourceIndex = std::numeric_limits<std::size_t>::max();
            EntityId minimumEntityId = std::numeric_limits<EntityId>::max();
        };

        struct CornerAnalysis
        {
            int count = 0;
            std::vector<double> radii;
            double radius = 0.0;
            double confidence = 0.0;
        };

        double distance2D(const Vector2d& left, const Vector2d& right)
        {
            return std::hypot(left.x - right.x, left.y - right.y);
        }

        double distance3D(const Vector3d& left, const Vector3d& right)
        {
            return std::sqrt
            (
                (left.x - right.x) * (left.x - right.x)
                + (left.y - right.y) * (left.y - right.y)
                + (left.z - right.z) * (left.z - right.z)
            );
        }

        double cross(const Vector2d& origin, const Vector2d& first, const Vector2d& second)
        {
            return (first.x - origin.x) * (second.y - origin.y)
                - (first.y - origin.y) * (second.x - origin.x);
        }

        double signedArea(const std::vector<Vector2d>& boundary)
        {
            if (boundary.empty()) return 0.0;
            const LocalFrame2d frame{ geometry::stableBoundsCenter(boundary) };
            double area = 0.0;
            double correction = 0.0;

            for (std::size_t index = 0; index < boundary.size(); ++index)
            {
                const Vector2d start = frame.toLocal(boundary[index]);
                const Vector2d end = frame.toLocal(boundary[(index + 1) % boundary.size()]);
                const double term = start.x * end.y - end.x * start.y;
                const double adjusted = term - correction;
                const double next = area + adjusted;
                correction = (next - area) - adjusted;
                area = next;
            }

            return area * 0.5;
        }

        double pointSegmentDistance
        (
            const Vector2d& point,
            const Vector2d& start,
            const Vector2d& end
        )
        {
            const double deltaX = end.x - start.x;
            const double deltaY = end.y - start.y;
            const double lengthSquared = deltaX * deltaX + deltaY * deltaY;

            if (lengthSquared <= 1.0e-18)
            {
                return distance2D(point, start);
            }

            const double factor = std::clamp
            (
                ((point.x - start.x) * deltaX + (point.y - start.y) * deltaY)
                    / lengthSquared,
                0.0,
                1.0
            );
            return distance2D
            (
                point,
                { start.x + deltaX * factor, start.y + deltaY * factor }
            );
        }

        double distanceToBoundary
        (
            const Vector2d& point,
            const std::vector<Vector2d>& boundary
        )
        {
            double distance = std::numeric_limits<double>::max();

            for (std::size_t index = 0; index < boundary.size(); ++index)
            {
                distance = std::min
                (
                    distance,
                    pointSegmentDistance
                    (
                        point,
                        boundary[index],
                        boundary[(index + 1) % boundary.size()]
                    )
                );
            }

            return distance;
        }

        bool pointInside(const Vector2d& point, const std::vector<Vector2d>& boundary)
        {
            const LocalFrame2d frame{ geometry::stableBoundsCenter(boundary) };
            const Vector2d localPoint = frame.toLocal(point);
            bool inside = false;

            for (std::size_t current = 0, previous = boundary.size() - 1;
                current < boundary.size(); previous = current++)
            {
                const Vector2d first = frame.toLocal(boundary[previous]);
                const Vector2d second = frame.toLocal(boundary[current]);
                const bool crosses = (first.y > localPoint.y) != (second.y > localPoint.y);

                if (crosses)
                {
                    const double intersectionX = (second.x - first.x)
                        * (localPoint.y - first.y) / (second.y - first.y) + first.x;
                    inside = inside != (localPoint.x < intersectionX);
                }
            }

            return inside;
        }

        int orientation
        (
            const Vector2d& first,
            const Vector2d& second,
            const Vector2d& third,
            double epsilon
        )
        {
            const double value = cross(first, second, third);
            return value > epsilon ? 1 : value < -epsilon ? -1 : 0;
        }

        bool pointOnSegment
        (
            const Vector2d& point,
            const Vector2d& start,
            const Vector2d& end,
            double epsilon
        )
        {
            return pointSegmentDistance(point, start, end) <= epsilon
                && point.x >= std::min(start.x, end.x) - epsilon
                && point.x <= std::max(start.x, end.x) + epsilon
                && point.y >= std::min(start.y, end.y) - epsilon
                && point.y <= std::max(start.y, end.y) + epsilon;
        }

        bool segmentsIntersect
        (
            const Vector2d& firstStart,
            const Vector2d& firstEnd,
            const Vector2d& secondStart,
            const Vector2d& secondEnd,
            double epsilon
        )
        {
            const int firstSideStart = orientation(firstStart, firstEnd, secondStart, epsilon);
            const int firstSideEnd = orientation(firstStart, firstEnd, secondEnd, epsilon);
            const int secondSideStart = orientation(secondStart, secondEnd, firstStart, epsilon);
            const int secondSideEnd = orientation(secondStart, secondEnd, firstEnd, epsilon);

            if (firstSideStart != firstSideEnd && secondSideStart != secondSideEnd)
            {
                return true;
            }

            return (firstSideStart == 0 && pointOnSegment(secondStart, firstStart, firstEnd, epsilon))
                || (firstSideEnd == 0 && pointOnSegment(secondEnd, firstStart, firstEnd, epsilon))
                || (secondSideStart == 0 && pointOnSegment(firstStart, secondStart, secondEnd, epsilon))
                || (secondSideEnd == 0 && pointOnSegment(firstEnd, secondStart, secondEnd, epsilon));
        }

        bool selfIntersects(const std::vector<Vector2d>& boundary, double epsilon)
        {
            for (std::size_t first = 0; first < boundary.size(); ++first)
            {
                const std::size_t firstEnd = (first + 1) % boundary.size();

                for (std::size_t second = first + 1; second < boundary.size(); ++second)
                {
                    const std::size_t secondEnd = (second + 1) % boundary.size();

                    if (first == second || firstEnd == second || secondEnd == first)
                    {
                        continue;
                    }

                    if (segmentsIntersect
                    (
                        boundary[first],
                        boundary[firstEnd],
                        boundary[second],
                        boundary[secondEnd],
                        epsilon
                    ))
                    {
                        return true;
                    }
                }
            }

            return false;
        }

        QVariantList entityList(const std::vector<EntityId>& entityIds)
        {
            QVariantList values;

            for (const EntityId entityId : entityIds)
            {
                values.push_back(QVariant::fromValue<qulonglong>(entityId));
            }

            return values;
        }

        QVariantMap contextValues
        (
            std::uint64_t contentRevision,
            int selectedCount,
            int candidateCount,
            const TubeSectionModel* model,
            double sectionArea,
            int physicalInteriorCount = 0,
            int topologicalInteriorCount = 0
        )
        {
            QVariantMap values;
            values.insert(QStringLiteral("contentRevision"), QVariant::fromValue<qulonglong>(contentRevision));
            values.insert(QStringLiteral("selectedCount"), selectedCount);
            values.insert(QStringLiteral("candidateCount"), candidateCount);
            values.insert(QStringLiteral("outerBoundaryEntityIds"), model != nullptr
                ? entityList(model->outerBoundaryEntityIds) : QVariantList{});
            values.insert(QStringLiteral("centerX"), model != nullptr ? model->centerX : 0.0);
            values.insert(QStringLiteral("maximumPlaneDeviation"),
                model != nullptr ? model->maximumPlaneDeviation : 0.0);
            values.insert(QStringLiteral("centerY"), model != nullptr ? model->geometry.centerY : 0.0);
            values.insert(QStringLiteral("centerZ"), model != nullptr ? model->geometry.centerZ : 0.0);
            values.insert(QStringLiteral("yLength"), model != nullptr ? model->geometry.yLength : 0.0);
            values.insert(QStringLiteral("zWidth"), model != nullptr ? model->geometry.zWidth : 0.0);
            values.insert(QStringLiteral("perimeter"), model != nullptr ? model->geometry.perimeter : 0.0);
            values.insert(QStringLiteral("sectionArea"), sectionArea);
            values.insert(QStringLiteral("physicalInteriorCount"), physicalInteriorCount);
            values.insert(QStringLiteral("topologicalInteriorCount"), topologicalInteriorCount);
            return values;
        }

        Diagnostic diagnostic
        (
            DiagnosticCode code,
            DiagnosticSeverity severity,
            const QString& userMessage,
            const QString& technicalDetail,
            const OperationContext& operationContext,
            const QVariantMap& values
        )
        {
            Diagnostic value;
            value.code = code;
            value.severity = severity;
            value.component = QStringLiteral("TubeSectionAnalyzer");
            value.operation = operationContext.operationName;
            value.stage = QStringLiteral("tube-section-analysis");
            value.userMessage = userMessage;
            value.technicalDetail = technicalDetail;
            value.correlationId = operationContext.correlationId;
            value.context = values;
            return value;
        }

        template<typename T>
        OperationResult<T> failure
        (
            DiagnosticCode code,
            const QString& userMessage,
            const QString& technicalDetail,
            const OperationContext& operationContext,
            const QVariantMap& values
        )
        {
            OperationResult<T> result;
            result.status = OperationStatus::Failed;
            result.addDiagnostic(diagnostic
                (code, DiagnosticSeverity::Error, userMessage, technicalDetail, operationContext, values));
            return result;
        }

        bool validPolicy(const TubeSectionPolicy& policy)
        {
            return std::isfinite(policy.connectionTolerance) && policy.connectionTolerance > 0.0
                && std::isfinite(policy.numericalEpsilon) && policy.numericalEpsilon > 0.0
                && std::isfinite(policy.maximumPlaneDeviation) && policy.maximumPlaneDeviation >= 0.0
                && std::isfinite(policy.boundaryDistanceTolerance) && policy.boundaryDistanceTolerance >= 0.0
                && std::isfinite(policy.interiorDistanceTolerance) && policy.interiorDistanceTolerance >= 0.0
                && std::isfinite(policy.minimumSectionArea) && policy.minimumSectionArea > 0.0;
        }

        double median(std::vector<double> values)
        {
            if (values.empty()) return 0.0;
            std::sort(values.begin(), values.end());
            const std::size_t middle = values.size() / 2;
            return values.size() % 2 == 0
                ? (values[middle - 1] + values[middle]) * 0.5
                : values[middle];
        }

        CornerAnalysis analyzeCorners
        (
            const std::vector<Vector2d>& boundary,
            const TubeSectionPolicy& policy
        )
        {
            CornerAnalysis result;

            if (boundary.size() < 4) return result;
            double minimumY = boundary.front().x;
            double maximumY = minimumY;
            double minimumZ = boundary.front().y;
            double maximumZ = minimumZ;

            for (const Vector2d& point : boundary)
            {
                minimumY = std::min(minimumY, point.x);
                maximumY = std::max(maximumY, point.x);
                minimumZ = std::min(minimumZ, point.y);
                maximumZ = std::max(maximumZ, point.y);
            }

            const double maximumRadius = std::min
                (maximumY - minimumY, maximumZ - minimumZ) * 0.5;
            const double supportTolerance = std::max
                (1.0e-5, policy.connectionTolerance * 0.1);

            for (const int ySide : { -1, 1 })
            {
                for (const int zSide : { -1, 1 })
                {
                    const double cornerY = ySide < 0 ? minimumY : maximumY;
                    const double cornerZ = zSide < 0 ? minimumZ : maximumZ;
                    bool sharp = false;
                    bool ySupport = false;
                    bool zSupport = false;
                    std::vector<double> samples;

                    for (const Vector2d& point : boundary)
                    {
                        const double yOffset = std::abs(point.x - cornerY);
                        const double zOffset = std::abs(point.y - cornerZ);

                        if (yOffset > maximumRadius + supportTolerance
                            || zOffset > maximumRadius + supportTolerance) continue;
                        if (std::hypot(yOffset, zOffset) <= supportTolerance)
                        {
                            sharp = true;
                            break;
                        }
                        ySupport = ySupport || (zOffset <= supportTolerance && yOffset > supportTolerance);
                        zSupport = zSupport || (yOffset <= supportTolerance && zOffset > supportTolerance);

                        if (yOffset > supportTolerance && zOffset > supportTolerance)
                        {
                            const double radius = yOffset + zOffset
                                + std::sqrt(2.0 * yOffset * zOffset);
                            if (radius > supportTolerance && radius <= maximumRadius + supportTolerance)
                                samples.push_back(radius);
                        }
                    }

                    double radius = 0.0;
                    if (!sharp && ySupport && zSupport && samples.size() >= 2)
                    {
                        radius = median(samples);
                        const auto limits = std::minmax_element(samples.begin(), samples.end());
                        const double spread = std::max
                            (policy.connectionTolerance, radius * 0.12);
                        if (radius <= supportTolerance || *limits.second - *limits.first > spread)
                            radius = 0.0;
                    }

                    result.radii.push_back(radius);
                    result.count += radius > 0.0 ? 1 : 0;
                }
            }

            std::vector<double> reliable;
            for (const double radius : result.radii)
                if (radius > supportTolerance) reliable.push_back(radius);
            if (result.count >= 3)
            {
                result.radius = median(reliable);
                const auto limits = std::minmax_element(reliable.begin(), reliable.end());
                const double allowedSpread = std::max
                    (policy.connectionTolerance, result.radius * 0.12);
                if (*limits.second - *limits.first <= allowedSpread)
                {
                    result.confidence = static_cast<double>(result.count) / 4.0
                        * std::max(0.0, 1.0 - (*limits.second - *limits.first)
                            / std::max(allowedSpread, 1.0e-12));
                }
                else
                {
                    result.radius = 0.0;
                }
            }
            return result;
        }

        OperationResult<SectionCandidate> buildCandidate
        (
            const TopologyInput& input,
            const TopologyLoopResult& loop,
            const TubeSectionPolicy& policy,
            const OperationContext& operationContext,
            int selectedCount,
            int candidateCount
        )
        {
            if (!loop.connectedLoop || loop.orderedPath.size() < 3U)
            {
                return failure<SectionCandidate>
                (
                    DiagnosticCode::TubeSectionLoopUnavailable,
                    QStringLiteral("候选图元无法形成严格闭合的方管截面。"),
                    QStringLiteral("PathTopology did not return a strict loop."),
                    operationContext,
                    contextValues(input.contentRevision, selectedCount, candidateCount, nullptr, 0.0)
                );
            }

            SectionCandidate candidate;
            candidate.model.contentRevision = input.contentRevision;
            candidate.model.outerBoundaryEntityIds = loop.usedEntityIds;
            std::vector<Vector3d> path;

            for (const Vector3d& point : loop.orderedPath)
            {
                if (path.empty() || distance3D(path.back(), point) > policy.numericalEpsilon)
                    path.push_back(point);
            }
            if (path.size() > 1U
                && distance3D(path.front(), path.back()) <= policy.numericalEpsilon)
                path.pop_back();
            if (path.size() < 3U)
            {
                return failure<SectionCandidate>
                (
                    DiagnosticCode::TubeSectionBoundaryInvalid,
                    QStringLiteral("方管截面边界有效点不足。"),
                    QStringLiteral("Normalized strict loop has fewer than three points."),
                    operationContext,
                    contextValues(input.contentRevision, selectedCount, candidateCount, nullptr, 0.0)
                );
            }

            const LocalFrame3d localFrame{ geometry::stableBoundsCenter(path) };
            for (Vector3d& point : path)
            {
                point = localFrame.toLocal(point);
            }

            double minimumX = path.front().x;
            double maximumX = minimumX;
            for (const Vector3d& point : path)
            {
                minimumX = std::min(minimumX, point.x);
                maximumX = std::max(maximumX, point.x);
            }
            const double localCenterX = minimumX + (maximumX - minimumX) * 0.5;
            candidate.model.centerX = localFrame.origin.x + localCenterX;
            for (const Vector3d& point : path)
                candidate.model.maximumPlaneDeviation = std::max
                    (candidate.model.maximumPlaneDeviation,
                     std::abs(point.x - localCenterX));
            std::vector<Vector2d> boundary;
            boundary.reserve(path.size());
            for (const Vector3d& point : path)
            {
                const Vector2d projected{ point.y, point.z };
                if (boundary.empty() || distance2D(boundary.back(), projected) > policy.numericalEpsilon)
                    boundary.push_back(projected);
            }
            if (boundary.size() > 1U
                && distance2D(boundary.front(), boundary.back()) <= policy.numericalEpsilon)
                boundary.pop_back();
            if (boundary.size() < 3U)
            {
                return failure<SectionCandidate>
                (
                    DiagnosticCode::TubeSectionBoundaryInvalid,
                    QStringLiteral("方管截面的 YZ 投影无效。"),
                    QStringLiteral("Projected boundary has fewer than three points."),
                    operationContext,
                    contextValues(input.contentRevision, selectedCount, candidateCount,
                        &candidate.model, 0.0)
                );
            }
            if (selfIntersects(boundary, policy.numericalEpsilon))
            {
                return failure<SectionCandidate>
                (
                    DiagnosticCode::TubeSectionSelfIntersection,
                    QStringLiteral("方管截面的 YZ 投影存在自相交。"),
                    QStringLiteral("Projected strict loop is not a simple polygon."),
                    operationContext,
                    contextValues(input.contentRevision, selectedCount, candidateCount,
                        &candidate.model, 0.0)
                );
            }

            candidate.area = std::abs(signedArea(boundary));
            if (candidate.area <= policy.minimumSectionArea)
            {
                return failure<SectionCandidate>
                (
                    DiagnosticCode::TubeSectionAreaInvalid,
                    QStringLiteral("方管截面的 YZ 投影面积无效。"),
                    QStringLiteral("Projected area is below policy minimum."),
                    operationContext,
                    contextValues(input.contentRevision, selectedCount, candidateCount,
                        &candidate.model, candidate.area)
                );
            }

            if (signedArea(boundary) > 0.0)
            {
                std::reverse(boundary.begin(), boundary.end());
                std::reverse(path.begin(), path.end());
            }
            const auto stable = std::min_element(boundary.begin(), boundary.end(), [](const auto& left, const auto& right)
            {
                return left.x < right.x || (left.x == right.x && left.y < right.y);
            });
            const std::size_t stableIndex = static_cast<std::size_t>
                (std::distance(boundary.begin(), stable));
            std::rotate(boundary.begin(), stable, boundary.end());
            std::rotate(path.begin(), path.begin() + stableIndex, path.end());
            candidate.model.orderedBoundary3D = path;
            for (Vector3d& point : candidate.model.orderedBoundary3D)
            {
                point = localFrame.toWorld(point);
            }

            double minimumY = boundary.front().x;
            double maximumY = minimumY;
            double minimumZ = boundary.front().y;
            double maximumZ = minimumZ;
            for (const Vector2d& point : boundary)
            {
                minimumY = std::min(minimumY, point.x);
                maximumY = std::max(maximumY, point.x);
                minimumZ = std::min(minimumZ, point.y);
                maximumZ = std::max(maximumZ, point.y);
            }

            TubeSectionGeometry geometry;
            geometry.boundary = std::move(boundary);
            geometry.centerY = minimumY + (maximumY - minimumY) * 0.5;
            geometry.centerZ = minimumZ + (maximumZ - minimumZ) * 0.5;
            geometry.yLength = maximumY - minimumY;
            geometry.zWidth = maximumZ - minimumZ;
            const auto prepared = TubeCutBoundaryClassifier::prepareSection
                (geometry, operationContext, policy.numericalEpsilon);
            if (!prepared.succeeded() || !prepared.value.has_value())
            {
                return failure<SectionCandidate>
                (
                    DiagnosticCode::TubeSectionPreparationFailed,
                    QStringLiteral("方管截面周向参数化失败。"),
                    QStringLiteral("TubeCutBoundaryClassifier::prepareSection failed."),
                    operationContext,
                    contextValues(input.contentRevision, selectedCount, candidateCount,
                        &candidate.model, candidate.area)
                );
            }
            const CornerAnalysis corners = analyzeCorners(prepared.value->boundary, policy);
            candidate.model.geometry = *prepared.value;
            for (Vector2d& point : candidate.model.geometry.boundary)
            {
                point.x += localFrame.origin.y;
                point.y += localFrame.origin.z;
            }
            candidate.model.geometry.centerY += localFrame.origin.y;
            candidate.model.geometry.centerZ += localFrame.origin.z;
            candidate.model.roundedCornerCount = corners.count;
            candidate.model.cornerRadii = corners.radii;
            candidate.model.cornerRadius = corners.radius;
            candidate.model.cornerConfidence = corners.confidence;
            if (corners.count >= 3 && corners.radius > policy.numericalEpsilon)
            {
                std::size_t radiusIndex = 0;
                for (const int yDirection : { -1, 1 })
                {
                    for (const int zDirection : { -1, 1 })
                    {
                        const double radius = radiusIndex < corners.radii.size()
                            ? corners.radii[radiusIndex] : 0.0;
                        ++radiusIndex;
                        if (radius <= policy.numericalEpsilon) continue;
                        const double outerY = (yDirection < 0 ? minimumY : maximumY)
                            + localFrame.origin.y;
                        const double outerZ = (zDirection < 0 ? minimumZ : maximumZ)
                            + localFrame.origin.z;
                        candidate.model.corners.push_back
                        ({
                            { outerY - yDirection * radius, outerZ - zDirection * radius },
                            radius,
                            yDirection,
                            zDirection
                        });
                    }
                }
            }

            std::map<EntityId, std::size_t> sourceIndices;
            for (const TopologyPathRecord& record : input.records)
                sourceIndices.emplace(record.entityId, record.sourceIndex);
            for (const EntityId entityId : loop.usedEntityIds)
            {
                const auto found = sourceIndices.find(entityId);
                if (found != sourceIndices.end())
                    candidate.minimumSourceIndex = std::min(candidate.minimumSourceIndex, found->second);
                candidate.minimumEntityId = std::min(candidate.minimumEntityId, entityId);
            }

            OperationResult<SectionCandidate> result;
            result.status = OperationStatus::Success;
            result.value = std::move(candidate);
            return result;
        }

        bool candidateContainedBy
        (
            const SectionCandidate& inner,
            const SectionCandidate& outer,
            const TubeSectionPolicy& policy
        )
        {
            if (inner.area >= outer.area - policy.minimumSectionArea
                || std::abs(inner.model.centerX - outer.model.centerX)
                    > policy.connectionTolerance)
                return false;

            bool hasStrictInteriorPoint = false;
            const auto& outerBoundary = outer.model.geometry.boundary;
            const auto& innerBoundary = inner.model.geometry.boundary;
            for (std::size_t index = 0; index < innerBoundary.size(); ++index)
            {
                const Vector2d& point = innerBoundary[index];
                const Vector2d& next = innerBoundary[(index + 1) % innerBoundary.size()];
                const Vector2d middle
                    { point.x + (next.x - point.x) * 0.5,
                      point.y + (next.y - point.y) * 0.5 };
                for (const Vector2d& sample : { point, middle })
                {
                    const double boundaryDistance = distanceToBoundary(sample, outerBoundary);
                    if (boundaryDistance <= policy.boundaryDistanceTolerance) continue;
                    if (!pointInside(sample, outerBoundary)) return false;
                    hasStrictInteriorPoint = true;
                }
            }
            return hasStrictInteriorPoint;
        }

        bool segmentEntersInterior
        (
            const Vector2d& start,
            const Vector2d& end,
            const std::vector<Vector2d>& boundary,
            double interiorTolerance,
            double epsilon
        )
        {
            std::vector<double> parameters{ 0.0, 1.0 };
            const Vector2d direction{ end.x - start.x, end.y - start.y };
            for (std::size_t index = 0; index < boundary.size(); ++index)
            {
                const Vector2d& edgeStart = boundary[index];
                const Vector2d& edgeEnd = boundary[(index + 1) % boundary.size()];
                const Vector2d edge{ edgeEnd.x - edgeStart.x, edgeEnd.y - edgeStart.y };
                const double denominator = direction.x * edge.y - direction.y * edge.x;
                if (std::abs(denominator) <= epsilon) continue;
                const Vector2d offset{ edgeStart.x - start.x, edgeStart.y - start.y };
                const double parameter = (offset.x * edge.y - offset.y * edge.x) / denominator;
                const double edgeParameter = (offset.x * direction.y - offset.y * direction.x) / denominator;
                if (parameter > epsilon && parameter < 1.0 - epsilon
                    && edgeParameter >= -epsilon && edgeParameter <= 1.0 + epsilon)
                    parameters.push_back(parameter);
            }
            std::sort(parameters.begin(), parameters.end());
            parameters.erase(std::unique(parameters.begin(), parameters.end(), [epsilon](double a, double b)
            {
                return std::abs(a - b) <= epsilon;
            }), parameters.end());
            for (std::size_t index = 0; index + 1 < parameters.size(); ++index)
            {
                const double parameter = (parameters[index] + parameters[index + 1]) * 0.5;
                const Vector2d sample
                {
                    start.x + direction.x * parameter,
                    start.y + direction.y * parameter
                };
                if (pointInside(sample, boundary)
                    && distanceToBoundary(sample, boundary) > interiorTolerance) return true;
            }
            return false;
        }

        double pointSegmentDistance3D
        (
            const Vector3d& point,
            const Vector3d& start,
            const Vector3d& end
        )
        {
            const Vector3d direction
            {
                end.x - start.x,
                end.y - start.y,
                end.z - start.z
            };
            const double lengthSquared = direction.x * direction.x
                + direction.y * direction.y + direction.z * direction.z;
            if (lengthSquared <= 1.0e-18) return distance3D(point, start);
            const double parameter = std::clamp
            (
                ((point.x - start.x) * direction.x
                    + (point.y - start.y) * direction.y
                    + (point.z - start.z) * direction.z) / lengthSquared,
                0.0,
                1.0
            );
            return distance3D
            (
                point,
                {
                    start.x + direction.x * parameter,
                    start.y + direction.y * parameter,
                    start.z + direction.z * parameter
                }
            );
        }

        bool pointOnLoop
        (
            const Vector3d& point,
            const std::vector<Vector3d>& loop,
            double epsilon
        )
        {
            for (std::size_t index = 0; index < loop.size(); ++index)
            {
                if (pointSegmentDistance3D
                (
                    point,
                    loop[index],
                    loop[(index + 1U) % loop.size()]
                ) <= epsilon) return true;
            }
            return false;
        }

        bool recordContainsNonBoundarySegment
        (
            const TopologyPathRecord& record,
            const std::vector<Vector3d>& loop,
            double epsilon
        )
        {
            if (record.points.size() < 2U || loop.size() < 3U) return false;
            for (std::size_t index = 0; index + 1U < record.points.size(); ++index)
            {
                const Vector3d midpoint
                {
                    (record.points[index].x + record.points[index + 1U].x) * 0.5,
                    (record.points[index].y + record.points[index + 1U].y) * 0.5,
                    (record.points[index].z + record.points[index + 1U].z) * 0.5
                };
                if (!pointOnLoop(midpoint, loop, epsilon)) return true;
            }
            return false;
        }

        std::vector<EntityId> stableIds
        (
            const std::set<EntityId>& ids,
            const TopologyInput& input
        )
        {
            std::vector<std::pair<std::size_t, EntityId>> ordered;
            for (const TopologyPathRecord& record : input.records)
                if (ids.count(record.entityId) != 0U)
                    ordered.emplace_back(record.sourceIndex, record.entityId);
            std::sort(ordered.begin(), ordered.end());
            std::vector<EntityId> result;
            for (const auto& [sourceIndex, entityId] : ordered)
            {
                Q_UNUSED(sourceIndex);
                result.push_back(entityId);
            }
            return result;
        }
    }

    OperationResult<TubeSectionModel> TubeSectionAnalyzer::buildFromSelection
    (
        const TopologyInput& input,
        const topology::PathTopology& topology,
        const std::vector<EntityId>& selectedEntityIds,
        const TubeSectionPolicy& policy,
        const OperationContext& context
    )
    {
        if (!validPolicy(policy) || input.records.empty() || selectedEntityIds.empty())
        {
            return failure<TubeSectionModel>
            (
                DiagnosticCode::TubeSectionInputInvalid,
                QStringLiteral("方管截面识别输入无效。"),
                QStringLiteral("Input, selection, or TubeSectionPolicy is invalid."),
                context,
                contextValues(input.contentRevision, static_cast<int>(selectedEntityIds.size()), 0, nullptr, 0.0)
            );
        }

        const auto loop = topology.extractSeededLoop(selectedEntityIds);
        if (!loop.succeeded() || !loop.value.has_value())
        {
            OperationResult<TubeSectionModel> result;
            result.status = OperationStatus::Failed;
            result.mergeDiagnostics(loop);
            result.addDiagnostic(diagnostic
            (
                DiagnosticCode::TubeSectionLoopUnavailable,
                DiagnosticSeverity::Error,
                QStringLiteral("所选图元无法扩展为严格闭合的方管截面。"),
                QStringLiteral("PathTopology::extractSeededLoop failed."),
                context,
                contextValues(input.contentRevision, static_cast<int>(selectedEntityIds.size()), 1, nullptr, 0.0)
            ));
            return result;
        }

        const auto candidate = buildCandidate
        (
            input,
            *loop.value,
            policy,
            context,
            static_cast<int>(selectedEntityIds.size()),
            1
        );
        if (!candidate.succeeded() || !candidate.value.has_value())
        {
            OperationResult<TubeSectionModel> result;
            result.status = candidate.status;
            result.mergeDiagnostics(candidate);
            return result;
        }
        OperationResult<TubeSectionModel> result;
        result.status = OperationStatus::Success;
        result.value = candidate.value->model;
        return result;
    }

    OperationResult<TubeSectionModel> TubeSectionAnalyzer::findBest
    (
        const TopologyInput& input,
        const topology::PathTopology& topology,
        const TubeSectionPolicy& policy,
        const OperationContext& context
    )
    {
        if (!validPolicy(policy) || input.records.empty())
        {
            return failure<TubeSectionModel>
            (
                DiagnosticCode::TubeSectionInputInvalid,
                QStringLiteral("方管截面自动识别输入无效。"),
                QStringLiteral("TopologyInput or TubeSectionPolicy is invalid."),
                context,
                contextValues(input.contentRevision, 0, 0, nullptr, 0.0)
            );
        }

        const std::vector<int> componentIds = topology.componentIds();
        std::map<int, std::vector<EntityId>> components;
        for (std::size_t index = 0; index < input.records.size() && index < componentIds.size(); ++index)
            components[componentIds[index]].push_back(input.records[index].entityId);

        std::vector<SectionCandidate> candidates;
        std::set<std::vector<EntityId>> candidateKeys;
        int inspectedCount = 0;

        for (const auto& [componentId, componentEntityIds] : components)
        {
            Q_UNUSED(componentId);
            std::vector<EntityId> remaining = componentEntityIds;

            while (!remaining.empty())
            {
                const auto loop = topology.extractBestLoop(remaining);
                ++inspectedCount;
                if (!loop.succeeded() || !loop.value.has_value()
                    || !loop.value->connectedLoop || loop.value->usedEntityIds.empty()) break;
                std::vector<EntityId> key = loop.value->usedEntityIds;
                std::sort(key.begin(), key.end());
                if (candidateKeys.insert(key).second)
                {
                    const auto candidate = buildCandidate
                        (input, *loop.value, policy, context, 0, inspectedCount);
                    if (candidate.succeeded() && candidate.value.has_value())
                        candidates.push_back(*candidate.value);
                }
                const std::set<EntityId> used
                    (loop.value->usedEntityIds.begin(), loop.value->usedEntityIds.end());
                const std::size_t previousSize = remaining.size();
                remaining.erase(std::remove_if(remaining.begin(), remaining.end(), [&used](EntityId id)
                {
                    return used.count(id) != 0U;
                }), remaining.end());
                if (remaining.size() == previousSize) break;
            }
        }

        if (candidates.empty())
        {
            return failure<TubeSectionModel>
            (
                DiagnosticCode::TubeSectionLoopUnavailable,
                QStringLiteral("未找到 YZ 投影有效的严格闭合方管截面。"),
                QStringLiteral("No strict-loop candidate with a valid YZ projection was found."),
                context,
                contextValues(input.contentRevision, 0, inspectedCount, nullptr, 0.0)
            );
        }

        std::vector<std::size_t> outerCandidates;
        for (std::size_t candidateIndex = 0; candidateIndex < candidates.size(); ++candidateIndex)
        {
            bool contained = false;
            for (std::size_t other = 0; other < candidates.size() && !contained; ++other)
                if (other != candidateIndex)
                    contained = candidateContainedBy(candidates[candidateIndex], candidates[other], policy);
            if (!contained) outerCandidates.push_back(candidateIndex);
        }

        std::sort(outerCandidates.begin(), outerCandidates.end(), [&candidates, &policy](std::size_t left, std::size_t right)
        {
            if (std::abs(candidates[left].area - candidates[right].area) > policy.minimumSectionArea)
                return candidates[left].area > candidates[right].area;
            if (candidates[left].minimumSourceIndex != candidates[right].minimumSourceIndex)
                return candidates[left].minimumSourceIndex < candidates[right].minimumSourceIndex;
            return candidates[left].minimumEntityId < candidates[right].minimumEntityId;
        });

        OperationResult<TubeSectionModel> result;
        result.status = outerCandidates.size() > 1U
            ? OperationStatus::PartialSuccess : OperationStatus::Success;
        result.value = candidates[outerCandidates.front()].model;
        QVariantMap values = contextValues
        (
            input.contentRevision,
            0,
            inspectedCount,
            &*result.value,
            candidates[outerCandidates.front()].area
        );
        values.insert(QStringLiteral("validCandidateCount"),
            static_cast<int>(candidates.size()));
        values.insert(QStringLiteral("roundedCandidateCount"),
            static_cast<int>(std::count_if(candidates.cbegin(), candidates.cend(), [](const SectionCandidate& candidate)
            {
                return candidate.model.roundedCornerCount >= 3
                    && candidate.model.cornerConfidence > 0.0;
            })));
        if (outerCandidates.size() > 1U)
        {
            result.addDiagnostic(diagnostic
            (
                DiagnosticCode::TubeSectionMultipleOuterLoops,
                DiagnosticSeverity::Warning,
                QStringLiteral("识别到多个独立外层截面，已稳定选择面积最大的截面。"),
                QStringLiteral("Multiple non-contained strict loops were found."),
                context,
                values
            ));
        }
        else
        {
            result.addDiagnostic(diagnostic
            (
                DiagnosticCode::None,
                DiagnosticSeverity::Info,
                QStringLiteral("方管截面自动识别完成。"),
                QStringLiteral("Best outer strict loop selected."),
                context,
                values
            ));
        }
        return result;
    }

    OperationResult<InternalPathClassification> TubeSectionAnalyzer::classifyInternalPaths
    (
        const TopologyInput& input,
        const topology::PathTopology& topology,
        const TubeSectionModel& section,
        const TubeSectionPolicy& policy,
        const OperationContext& context
    )
    {
        if (!validPolicy(policy) || section.geometry.boundary.size() < 3U)
        {
            return failure<InternalPathClassification>
            (
                DiagnosticCode::TubeSectionInteriorClassificationFailed,
                QStringLiteral("方管截面不可用，无法识别内部图元。"),
                QStringLiteral("TubeSectionModel or policy is invalid."),
                context,
                contextValues(input.contentRevision, 0, 0, &section, 0.0)
            );
        }

        OperationResult<InternalPathClassification> result =
            classifyTopologicalInteriorPaths(input, topology, context);
        if (!result.succeeded() || !result.value.has_value()) return result;

        const std::set<EntityId> outerIds
            (section.outerBoundaryEntityIds.begin(), section.outerBoundaryEntityIds.end());
        const std::set<EntityId> topologicalIds
        (
            result.value->topologicalInteriorEntityIds.begin(),
            result.value->topologicalInteriorEntityIds.end()
        );
        std::set<EntityId> physicalIds;
        const auto& boundary = section.geometry.boundary;

        for (const TopologyPathRecord& record : input.records)
        {
            if (outerIds.count(record.entityId) != 0U
                || topologicalIds.count(record.entityId) != 0U
                || record.points.empty()) continue;
            bool physicalInterior = false;
            for (const Vector3d& point : record.points)
            {
                const Vector2d projected{ point.y, point.z };
                if (pointInside(projected, boundary)
                    && distanceToBoundary(projected, boundary) > policy.numericalEpsilon)
                {
                    physicalInterior = true;
                    break;
                }
            }
            for (std::size_t index = 0; !physicalInterior && index + 1 < record.points.size(); ++index)
            {
                physicalInterior = segmentEntersInterior
                (
                    { record.points[index].y, record.points[index].z },
                    { record.points[index + 1].y, record.points[index + 1].z },
                    boundary,
                    policy.numericalEpsilon,
                    policy.numericalEpsilon
                );
            }
            if (physicalInterior) physicalIds.insert(record.entityId);
        }
        result.value->physicalInteriorEntityIds = stableIds(physicalIds, input);
        result.addDiagnostic(diagnostic
        (
            DiagnosticCode::None,
            DiagnosticSeverity::Info,
            QStringLiteral("方管内部图元识别完成。"),
            QStringLiteral("Physical and topological interior classifications completed."),
            context,
            contextValues
            (
                input.contentRevision,
                0,
                0,
                &section,
                std::abs(signedArea(section.geometry.boundary)),
                static_cast<int>(result.value->physicalInteriorEntityIds.size()),
                static_cast<int>(result.value->topologicalInteriorEntityIds.size())
            )
        ));
        return result;
    }

    OperationResult<InternalPathClassification>
    TubeSectionAnalyzer::classifyTopologicalInteriorPaths
    (
        const TopologyInput& input,
        const topology::PathTopology& topology,
        const OperationContext& context
    )
    {
        InternalPathClassification classification;
        OperationResult<InternalPathClassification> result;
        const std::vector<int> componentIds = topology.componentIds();
        if (componentIds.size() != input.records.size())
        {
            return failure<InternalPathClassification>
            (
                DiagnosticCode::TubeSectionInteriorClassificationFailed,
                QStringLiteral("内部线拓扑输入与连通分量不一致。"),
                QStringLiteral("Topology component count does not match input records."),
                context,
                contextValues(input.contentRevision, 0, 0, nullptr, 0.0)
            );
        }

        std::map<int, std::vector<EntityId>> components;
        std::map<EntityId, const TopologyPathRecord*> recordsById;
        for (std::size_t index = 0; index < input.records.size(); ++index)
        {
            components[componentIds[index]].push_back(input.records[index].entityId);
            recordsById[input.records[index].entityId] = &input.records[index];
        }

        std::set<EntityId> topologicalIds;
        bool hasWarning = false;
        for (const auto& [componentId, ids] : components)
        {
            const OperationResult<TopologyLoopResult> loop = topology.extractBestLoop(ids);
            if (!loop.succeeded() || !loop.value.has_value() || !loop.value->connectedLoop)
            {
                ++classification.skippedComponentCount;
                QVariantMap values = contextValues
                    (input.contentRevision, 0, static_cast<int>(ids.size()), nullptr, 0.0);
                values.insert(QStringLiteral("componentId"), componentId);
                values.insert(QStringLiteral("reason"), QStringLiteral("no-strict-outer-loop"));
                result.addDiagnostic(diagnostic
                (
                    DiagnosticCode::None,
                    DiagnosticSeverity::Info,
                    QStringLiteral("开放或无严格外轮廓的图元组已跳过。"),
                    QStringLiteral("Connected component has no strict outer loop."),
                    context,
                    values
                ));
                continue;
            }

            const std::set<EntityId> used
                (loop.value->usedEntityIds.begin(), loop.value->usedEntityIds.end());
            for (EntityId entityId : ids)
            {
                if (used.count(entityId) == 0U)
                {
                    topologicalIds.insert(entityId);
                    continue;
                }

                const auto record = recordsById.find(entityId);
                if (record != recordsById.end() && recordContainsNonBoundarySegment
                (
                    *record->second,
                    loop.value->orderedPath,
                    PathTopologyTolerance{}.numericalJoinEpsilon
                ))
                {
                    hasWarning = true;
                    QVariantMap values = contextValues
                        (input.contentRevision, 0, static_cast<int>(ids.size()), nullptr, 0.0);
                    values.insert(QStringLiteral("entityId"), QVariant::fromValue<qulonglong>(entityId));
                    values.insert(QStringLiteral("componentId"), componentId);
                    values.insert(QStringLiteral("reason"),
                        QStringLiteral("entity-contains-boundary-and-interior-segments"));
                    Diagnostic warning = diagnostic
                    (
                        DiagnosticCode::TubeSectionInteriorClassificationFailed,
                        DiagnosticSeverity::Warning,
                        QStringLiteral("一个图元同时包含外边界段和内部段，已保留该图元。"),
                        QStringLiteral("EntityId spans the selected outer loop and interior branches."),
                        context,
                        values
                    );
                    warning.entityId = entityId;
                    result.addDiagnostic(warning);
                }
            }
        }

        classification.topologicalInteriorEntityIds = stableIds(topologicalIds, input);
        result.status = hasWarning ? OperationStatus::PartialSuccess : OperationStatus::Success;
        result.value = std::move(classification);
        return result;
    }
}
