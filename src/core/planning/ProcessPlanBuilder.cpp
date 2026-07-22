#include "core/planning/ProcessPlanBuilder.h"

#include "core/machining/TubeCutBoundary.h"

#include <QStringList>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <unordered_map>
#include <unordered_set>

namespace cadcam::planning
{
    namespace
    {
        using geometry::EntityId;
        using geometry::Vector2d;
        using geometry::Vector3d;
        using machining::TubeCutAnalysis;
        using machining::TubeCutBoundaryClassifier;
        using machining::TubeCutResult;

        constexpr double kCalculationEpsilon = 1.0e-12;

        struct BoundaryData
        {
            int groupId = -1;
            int pairId = -1;
            BoundaryRole role = BoundaryRole::None;
            TubeCutAnalysis analysis;
        };

        struct BoundaryIdentity
        {
            int pairId = -1;
            BoundaryRole role = BoundaryRole::None;
            std::vector<EntityId> entityIds;
            std::size_t stableSourceIndex = 0;
            EntityId stableEntityId = 0;
        };

        struct XBounds
        {
            bool valid = false;
            double minimum = 0.0;
            double maximum = 0.0;
        };

        struct DirectedEntity
        {
            const PlanningEntity* entity = nullptr;
            bool reverseRelativeToInput = false;
            std::optional<double> selectedStartParameter;
            Vector3d start;
            Vector3d end;
            int entryAxisReversalCount = 0;
            double entryTangentCost = 0.0;
        };

        struct GroupTraversal
        {
            int groupId = -1;
            std::vector<DirectedEntity> entities;
            Vector3d start;
            Vector3d end;
            double movementDistance = 0.0;
            double rotationCost = 0.0;
            int surfaceCost = 0;
            int entryAxisReversalCount = 0;
            double entryTangentCost = 0.0;
            std::size_t stableSourceIndex = 0;
            EntityId stableEntityId = 0;
        };

        struct ClosedLoopTraversalReport
        {
            int groupId = -1;
            std::vector<EntityId> memberEntityIds;
            int memberCount = 0;
            int nodeCount = 0;
            int connectedComponentCount = 0;
            int branchNodeCount = 0;
            int invalidDegreeNodeCount = 0;
            int candidateCount = 0;
            std::vector<EntityId> selectedOrder;
            std::vector<bool> selectedReverse;
            bool simpleLoopValid = false;
            QString status = QStringLiteral("Failed");
            QString failureReason;
        };

        struct SectionProjection
        {
            bool valid = false;
            double perimeterPosition = 0.0;
            double distance = std::numeric_limits<double>::max();
        };

        double distance(const Vector3d& left, const Vector3d& right)
        {
            return std::sqrt
            (
                (left.x - right.x) * (left.x - right.x)
                + (left.y - right.y) * (left.y - right.y)
                + (left.z - right.z) * (left.z - right.z)
            );
        }

        QString strategyName(ProcessOrderingStrategy strategy)
        {
            return strategy == ProcessOrderingStrategy::LazyRotation
                ? QStringLiteral("LazyRotation")
                : QStringLiteral("NearestNext");
        }

        QString groupKindName(ProcessGroupKind kind)
        {
            switch (kind)
            {
            case ProcessGroupKind::SingleEntity: return QStringLiteral("SingleEntity");
            case ProcessGroupKind::ConnectedChain: return QStringLiteral("ConnectedChain");
            case ProcessGroupKind::ClosedLoop: return QStringLiteral("ClosedLoop");
            case ProcessGroupKind::BreakBoundary: return QStringLiteral("BreakBoundary");
            case ProcessGroupKind::WasteBoundary: return QStringLiteral("WasteBoundary");
            }
            return QStringLiteral("Unknown");
        }

        QVariantMap diagnosticValues
        (
            const ProcessPlanningInput& input,
            const ProcessPlanningPolicy& policy,
            EntityId entityId = 0,
            std::size_t sourceIndex = 0,
            int boundaryPairId = -1,
            int groupId = -1,
            int predecessorGroupId = -1,
            int successorGroupId = -1,
            int candidateCount = 0,
            int eligibleCount = 0,
            int groupCount = 0,
            int assignmentCount = 0,
            int excludedCount = 0,
            int processOrder = -1,
            int continuousGroupId = -1,
            bool initialSelection = false,
            const GroupTraversal* selected = nullptr,
            ProcessGroupKind selectedGroupKind = ProcessGroupKind::SingleEntity,
            int blockedNearestBoundaryGroupId = -1
        )
        {
            QVariantMap values;
            values.insert(QStringLiteral("contentRevision"), QVariant::fromValue<qulonglong>(input.contentRevision));
            values.insert(QStringLiteral("entityId"), QVariant::fromValue<qulonglong>(entityId));
            values.insert(QStringLiteral("sourceIndex"), QVariant::fromValue<qulonglong>(sourceIndex));
            values.insert(QStringLiteral("boundaryPairId"), boundaryPairId);
            values.insert(QStringLiteral("groupId"), groupId);
            values.insert(QStringLiteral("predecessorGroupId"), predecessorGroupId);
            values.insert(QStringLiteral("successorGroupId"), successorGroupId);
            values.insert(QStringLiteral("orderingStrategy"), strategyName(policy.orderingStrategy));
            values.insert(QStringLiteral("candidateCount"), candidateCount);
            values.insert(QStringLiteral("eligibleCount"), eligibleCount);
            values.insert(QStringLiteral("groupCount"), groupCount);
            values.insert(QStringLiteral("assignmentCount"), assignmentCount);
            values.insert(QStringLiteral("excludedCount"), excludedCount);
            values.insert(QStringLiteral("processOrder"), processOrder);
            values.insert(QStringLiteral("continuousGroupId"), continuousGroupId);
            values.insert(QStringLiteral("initialSelection"), initialSelection);
            values.insert(QStringLiteral("initialPositionX"), policy.initialPosition.x);
            values.insert(QStringLiteral("initialPositionY"), policy.initialPosition.y);
            values.insert(QStringLiteral("initialPositionZ"), policy.initialPosition.z);
            values.insert(QStringLiteral("selectedGroupId"), selected != nullptr ? selected->groupId : -1);
            values.insert(QStringLiteral("selectedGroupKind"), selected != nullptr
                ? groupKindName(selectedGroupKind) : QString());
            values.insert(QStringLiteral("selectedMovementDistance"), selected != nullptr
                ? selected->movementDistance : -1.0);
            values.insert(QStringLiteral("selectedRotationCost"), selected != nullptr
                ? selected->rotationCost : -1.0);
            values.insert(QStringLiteral("selectedSurfaceCost"), selected != nullptr
                ? selected->surfaceCost : -1);
            values.insert(QStringLiteral("blockedNearestBoundaryGroupId"), blockedNearestBoundaryGroupId);
            return values;
        }

        Diagnostic planningDiagnostic
        (
            const OperationContext& context,
            DiagnosticCode code,
            const QString& message,
            const QString& detail,
            const QVariantMap& values,
            DiagnosticSeverity severity = DiagnosticSeverity::Error
        )
        {
            Diagnostic diagnostic;
            diagnostic.code = code;
            diagnostic.severity = severity;
            diagnostic.component = QStringLiteral("ProcessPlanBuilder");
            diagnostic.operation = QStringLiteral("BuildProcessPlan");
            diagnostic.stage = QStringLiteral("ProcessPlanning");
            diagnostic.userMessage = message;
            diagnostic.technicalDetail = detail;
            diagnostic.correlationId = context.correlationId;
            diagnostic.context = values;
            const qulonglong entityId = values.value(QStringLiteral("entityId")).toULongLong();
            if (entityId != 0U) diagnostic.entityId = entityId;
            const int groupId = values.value(QStringLiteral("groupId"), -1).toInt();
            if (groupId >= 0) diagnostic.groupId = groupId;
            return diagnostic;
        }

        template<typename T>
        OperationResult<T> failure
        (
            OperationStatus status,
            const OperationContext& context,
            DiagnosticCode code,
            const QString& message,
            const QString& detail,
            const QVariantMap& values
        )
        {
            OperationResult<T> result;
            result.status = status;
            result.addDiagnostic(planningDiagnostic(context, code, message, detail, values));
            return result;
        }

        std::vector<double> cumulativeSectionLengths(const machining::TubeSectionGeometry& section)
        {
            std::vector<double> cumulative(section.boundary.size() + 1U, 0.0);
            for (std::size_t index = 0; index < section.boundary.size(); ++index)
            {
                const Vector2d& first = section.boundary[index];
                const Vector2d& second = section.boundary[(index + 1U) % section.boundary.size()];
                cumulative[index + 1U] = cumulative[index]
                    + std::hypot(second.x - first.x, second.y - first.y);
            }
            return cumulative;
        }

        SectionProjection projectToSection
        (
            const Vector2d& point,
            const machining::TubeSectionGeometry& section,
            const std::vector<double>& cumulative
        )
        {
            SectionProjection best;
            const Vector2d localPoint
                { point.x - section.centerY, point.y - section.centerZ };
            for (std::size_t index = 0; index < section.boundary.size(); ++index)
            {
                const Vector2d start
                    { section.boundary[index].x - section.centerY,
                      section.boundary[index].y - section.centerZ };
                const Vector2d& worldEnd = section.boundary[(index + 1U) % section.boundary.size()];
                const Vector2d end
                    { worldEnd.x - section.centerY, worldEnd.y - section.centerZ };
                const double dy = end.x - start.x;
                const double dz = end.y - start.y;
                const double lengthSquared = dy * dy + dz * dz;
                if (lengthSquared <= kCalculationEpsilon) continue;
                const double factor = std::clamp
                (
                    ((localPoint.x - start.x) * dy + (localPoint.y - start.y) * dz)
                        / lengthSquared,
                    0.0,
                    1.0
                );
                const double projectedY = start.x + dy * factor;
                const double projectedZ = start.y + dz * factor;
                const double candidateDistance = std::hypot
                    (localPoint.x - projectedY, localPoint.y - projectedZ);
                if (candidateDistance < best.distance)
                {
                    best.valid = true;
                    best.distance = candidateDistance;
                    best.perimeterPosition = cumulative[index] + std::sqrt(lengthSquared) * factor;
                }
            }
            return best;
        }

        BoundarySide classifyPoint
        (
            const Vector3d& point,
            const TubeCutAnalysis& boundary,
            const machining::TubeSectionGeometry& section,
            double tolerance
        )
        {
            if (boundary.unwrappedBoundary.size() < 2U || section.boundary.size() < 3U || section.perimeter <= 0.0)
            {
                return BoundarySide::Indeterminate;
            }

            const std::vector<double> cumulative = cumulativeSectionLengths(section);
            const SectionProjection projection = projectToSection({ point.y, point.z }, section, cumulative);
            const double safeTolerance = std::max(1.0e-6, std::abs(tolerance));
            if (!projection.valid || projection.distance > safeTolerance)
            {
                return BoundarySide::Indeterminate;
            }

            const double perimeter = section.perimeter;
            double minimumBoundaryX = boundary.unwrappedBoundary.front().x;
            double maximumBoundaryX = minimumBoundaryX;
            for (const machining::UnwrappedBoundaryPoint& sample : boundary.unwrappedBoundary)
            {
                minimumBoundaryX = std::min(minimumBoundaryX, sample.x);
                maximumBoundaryX = std::max(maximumBoundaryX, sample.x);
            }
            const double referenceX = minimumBoundaryX
                + (maximumBoundaryX - minimumBoundaryX) * 0.5;
            const double localPointX = point.x - referenceX;
            double queryPosition = projection.perimeterPosition;
            double minimumPosition = boundary.unwrappedBoundary.front().perimeterPosition;
            double maximumPosition = minimumPosition;
            for (const machining::UnwrappedBoundaryPoint& sample : boundary.unwrappedBoundary)
            {
                minimumPosition = std::min(minimumPosition, sample.perimeterPosition);
                maximumPosition = std::max(maximumPosition, sample.perimeterPosition);
            }
            const double intervalCenter = (minimumPosition + maximumPosition) * 0.5;
            while (queryPosition - intervalCenter > perimeter * 0.5) queryPosition -= perimeter;
            while (queryPosition - intervalCenter < -perimeter * 0.5) queryPosition += perimeter;

            std::vector<double> intersections;
            bool onBoundary = false;
            for (const double shift : { -perimeter, 0.0, perimeter })
            {
                for (std::size_t index = 0; index + 1U < boundary.unwrappedBoundary.size(); ++index)
                {
                    const auto& first = boundary.unwrappedBoundary[index];
                    const auto& second = boundary.unwrappedBoundary[index + 1U];
                    const double firstX = first.x - referenceX;
                    const double secondX = second.x - referenceX;
                    const double firstS = first.perimeterPosition + shift;
                    const double secondS = second.perimeterPosition + shift;
                    const double edgeX = secondX - firstX;
                    const double edgeS = secondS - firstS;
                    const double lengthSquared = edgeX * edgeX + edgeS * edgeS;
                    if (lengthSquared > kCalculationEpsilon)
                    {
                        const double factor = std::clamp
                        (
                            ((localPointX - firstX) * edgeX
                                + (queryPosition - firstS) * edgeS) / lengthSquared,
                            0.0,
                            1.0
                        );
                        const double nearestX = firstX + edgeX * factor;
                        const double nearestS = firstS + edgeS * factor;
                        onBoundary = onBoundary
                            || std::hypot(localPointX - nearestX,
                                queryPosition - nearestS) <= safeTolerance;
                    }
                    const bool crosses = (firstS <= queryPosition && queryPosition < secondS)
                        || (secondS <= queryPosition && queryPosition < firstS);
                    if (!crosses || std::abs(edgeS) <= kCalculationEpsilon) continue;
                    const double factor = (queryPosition - firstS) / edgeS;
                    const double intersectionX = firstX + edgeX * factor;
                    intersections.push_back(intersectionX);
                    onBoundary = onBoundary
                        || std::abs(intersectionX - localPointX) <= safeTolerance;
                }
            }
            if (onBoundary) return BoundarySide::OnBoundary;

            std::sort(intersections.begin(), intersections.end());
            std::vector<double> unique;
            for (const double x : intersections)
            {
                if (!unique.empty() && std::abs(x - unique.back()) <= safeTolerance)
                {
                    unique.back() = (unique.back() + x) * 0.5;
                }
                else
                {
                    unique.push_back(x);
                }
            }
            const int crossings = static_cast<int>(std::count_if
            (
                unique.cbegin(), unique.cend(),
                [localPointX, safeTolerance](double x)
                { return x < localPointX - safeTolerance; }
            ));
            return crossings % 2 == 0 ? BoundarySide::Left : BoundarySide::Right;
        }

        BoundarySide classifyGroup
        (
            const ProcessGroup& group,
            const std::unordered_map<EntityId, const PlanningEntity*>& entities,
            const BoundaryData& boundary,
            const machining::TubeSectionGeometry& section,
            double tolerance
        )
        {
            double groupMinimumX = std::numeric_limits<double>::max();
            double groupMaximumX = std::numeric_limits<double>::lowest();
            for (const EntityId entityId : group.entityIds)
            {
                const auto found = entities.find(entityId);
                if (found == entities.end() || found->second->path.vertices.empty())
                {
                    return BoundarySide::Indeterminate;
                }
                for (const geometry::PathVertex3D& vertex : found->second->path.vertices)
                {
                    groupMinimumX = std::min(groupMinimumX, vertex.position.x);
                    groupMaximumX = std::max(groupMaximumX, vertex.position.x);
                }
            }

            double boundaryMinimumX = std::numeric_limits<double>::max();
            double boundaryMaximumX = std::numeric_limits<double>::lowest();
            for (const machining::UnwrappedBoundaryPoint& point : boundary.analysis.unwrappedBoundary)
            {
                boundaryMinimumX = std::min(boundaryMinimumX, point.x);
                boundaryMaximumX = std::max(boundaryMaximumX, point.x);
            }
            const double safeTolerance = std::max(1.0e-6, std::abs(tolerance));
            if (groupMaximumX < boundaryMinimumX - safeTolerance)
            {
                return BoundarySide::Left;
            }
            if (groupMinimumX > boundaryMaximumX + safeTolerance)
            {
                return BoundarySide::Right;
            }

            bool left = false;
            bool right = false;
            bool onBoundary = false;
            for (const EntityId entityId : group.entityIds)
            {
                const auto found = entities.find(entityId);
                if (found == entities.end()) return BoundarySide::Indeterminate;
                const auto& vertices = found->second->path.vertices;
                if (vertices.empty()) return BoundarySide::Indeterminate;
                for (std::size_t index = 0; index < vertices.size(); ++index)
                {
                    const BoundarySide side = classifyPoint(vertices[index].position, boundary.analysis, section, tolerance);
                    if (side == BoundarySide::Indeterminate) return side;
                    left = left || side == BoundarySide::Left;
                    right = right || side == BoundarySide::Right;
                    onBoundary = onBoundary || side == BoundarySide::OnBoundary;
                    if (index + 1U < vertices.size())
                    {
                        const Vector3d& first = vertices[index].position;
                        const Vector3d& second = vertices[index + 1U].position;
                        const Vector3d midpoint
                        {
                            first.x + (second.x - first.x) * 0.5,
                            first.y + (second.y - first.y) * 0.5,
                            first.z + (second.z - first.z) * 0.5
                        };
                        const BoundarySide middleSide = classifyPoint(midpoint, boundary.analysis, section, tolerance);
                        if (middleSide == BoundarySide::Indeterminate) return middleSide;
                        left = left || middleSide == BoundarySide::Left;
                        right = right || middleSide == BoundarySide::Right;
                        onBoundary = onBoundary || middleSide == BoundarySide::OnBoundary;
                    }
                }
            }
            if (left && right) return BoundarySide::Mixed;
            if (left) return BoundarySide::Left;
            if (right) return BoundarySide::Right;
            return onBoundary ? BoundarySide::OnBoundary : BoundarySide::Indeterminate;
        }

        QString sideName(BoundarySide side)
        {
            switch (side)
            {
            case BoundarySide::Left: return QStringLiteral("Left");
            case BoundarySide::OnBoundary: return QStringLiteral("OnBoundary");
            case BoundarySide::Right: return QStringLiteral("Right");
            case BoundarySide::Mixed: return QStringLiteral("Mixed");
            case BoundarySide::Indeterminate: return QStringLiteral("Indeterminate");
            }
            return QStringLiteral("Indeterminate");
        }

        XBounds groupXBounds
        (
            const ProcessGroup& group,
            const std::unordered_map<EntityId, const PlanningEntity*>& entities
        )
        {
            XBounds bounds;
            for (const EntityId entityId : group.entityIds)
            {
                const auto found = entities.find(entityId);
                if (found == entities.end()) return {};
                for (const geometry::PathVertex3D& vertex : found->second->path.vertices)
                {
                    if (!bounds.valid)
                    {
                        bounds.valid = true;
                        bounds.minimum = bounds.maximum = vertex.position.x;
                    }
                    else
                    {
                        bounds.minimum = std::min(bounds.minimum, vertex.position.x);
                        bounds.maximum = std::max(bounds.maximum, vertex.position.x);
                    }
                }
            }
            return bounds;
        }

        XBounds boundaryXBounds(const BoundaryData& boundary)
        {
            XBounds bounds;
            for (const machining::UnwrappedBoundaryPoint& point : boundary.analysis.unwrappedBoundary)
            {
                if (!bounds.valid)
                {
                    bounds.valid = true;
                    bounds.minimum = bounds.maximum = point.x;
                }
                else
                {
                    bounds.minimum = std::min(bounds.minimum, point.x);
                    bounds.maximum = std::max(bounds.maximum, point.x);
                }
            }
            return bounds;
        }

        QVariantMap boundaryDiagnosticValues
        (
            const ProcessPlanningInput& input,
            const ProcessPlanningPolicy& policy,
            const BoundaryData& boundary,
            const ProcessGroup& otherGroup,
            const std::unordered_map<EntityId, const PlanningEntity*>& entities,
            int boundarySpatialRank,
            BoundarySide relativeSide,
            BoundarySide reverseRelativeSide = BoundarySide::Indeterminate
        )
        {
            QVariantMap values = diagnosticValues
                (input, policy, 0U, 0U, boundary.pairId, otherGroup.groupId);
            const XBounds boundaryBounds = boundaryXBounds(boundary);
            const XBounds otherBounds = groupXBounds(otherGroup, entities);
            values.insert(QStringLiteral("boundaryGroupId"), boundary.groupId);
            values.insert(QStringLiteral("otherGroupId"), otherGroup.groupId);
            values.insert(QStringLiteral("boundarySpatialRank"), boundarySpatialRank);
            values.insert(QStringLiteral("relativeSide"), sideName(relativeSide));
            values.insert(QStringLiteral("reverseRelativeSide"), sideName(reverseRelativeSide));
            values.insert(QStringLiteral("boundaryMinimumX"),
                boundaryBounds.valid ? boundaryBounds.minimum : 0.0);
            values.insert(QStringLiteral("boundaryMaximumX"),
                boundaryBounds.valid ? boundaryBounds.maximum : 0.0);
            values.insert(QStringLiteral("otherMinimumX"),
                otherBounds.valid ? otherBounds.minimum : 0.0);
            values.insert(QStringLiteral("otherMaximumX"),
                otherBounds.valid ? otherBounds.maximum : 0.0);
            return values;
        }

        std::vector<Vector3d> directedPoints(const PlanningEntity& entity, bool reverse)
        {
            std::vector<Vector3d> points;
            points.reserve(entity.path.vertices.size());
            for (const geometry::PathVertex3D& vertex : entity.path.vertices) points.push_back(vertex.position);
            if (!reverse || points.size() < 2U) return points;
            if (entity.path.closed)
            {
                std::reverse(points.begin() + 1, points.end());
            }
            else
            {
                std::reverse(points.begin(), points.end());
            }
            return points;
        }

        bool directionAllowed(const PlanningEntity& entity, bool reverse, bool allowReverse)
        {
            switch (entity.directionPreference)
            {
            case process::DirectionPreference::Forward: return !reverse;
            case process::DirectionPreference::Reverse: return reverse;
            case process::DirectionPreference::Auto: return !reverse || allowReverse;
            }
            return false;
        }

        std::optional<GroupTraversal> buildTraversal
        (
            const ProcessGroup& group,
            const std::unordered_map<EntityId, const PlanningEntity*>& entities,
            const Vector3d& currentPosition,
            bool allowReverse,
            double tolerance,
            std::optional<std::pair<EntityId, bool>> forcedStart = std::nullopt
        )
        {
            GroupTraversal traversal;
            traversal.groupId = group.groupId;
            std::unordered_set<EntityId> used;
            Vector3d cursor = currentPosition;

            while (used.size() < group.entityIds.size())
            {
                const PlanningEntity* selected = nullptr;
                bool selectedReverse = false;
                std::vector<Vector3d> selectedPoints;
                double bestDistance = std::numeric_limits<double>::max();

                for (const EntityId entityId : group.entityIds)
                {
                    if (used.find(entityId) != used.end()) continue;
                    const auto found = entities.find(entityId);
                    if (found == entities.end()) return std::nullopt;
                    const PlanningEntity& entity = *found->second;
                    for (const bool reverse : { false, true })
                    {
                        if (!directionAllowed(entity, reverse, allowReverse)) continue;
                        if (used.empty() && forcedStart.has_value()
                            && (forcedStart->first != entityId || forcedStart->second != reverse)) continue;
                        const std::vector<Vector3d> points = directedPoints(entity, reverse);
                        if (points.size() < 2U) continue;
                        const double candidateDistance = distance(cursor, points.front());
                        if (!used.empty() && candidateDistance > tolerance) continue;
                        const bool replace = selected == nullptr
                            || candidateDistance < bestDistance - kCalculationEpsilon
                            || (std::abs(candidateDistance - bestDistance) <= kCalculationEpsilon
                                && (entity.sourceIndex < selected->sourceIndex
                                    || (entity.sourceIndex == selected->sourceIndex && entity.entityId < selected->entityId)));
                        if (replace)
                        {
                            selected = &entity;
                            selectedReverse = reverse;
                            selectedPoints = points;
                            bestDistance = candidateDistance;
                        }
                    }
                }

                if (selected == nullptr) return std::nullopt;
                DirectedEntity directed;
                directed.entity = selected;
                directed.reverseRelativeToInput = selectedReverse;
                directed.selectedStartParameter = selected->startParameter;
                directed.start = selectedPoints.front();
                directed.end = selectedPoints.back();
                traversal.entities.push_back(directed);
                used.insert(selected->entityId);
                cursor = directed.end;
            }

            if (traversal.entities.empty()) return std::nullopt;
            traversal.start = traversal.entities.front().start;
            traversal.end = traversal.entities.back().end;
            if (group.closed && distance(traversal.end, traversal.start) > tolerance) return std::nullopt;
            if (group.closed) traversal.end = traversal.start;
            traversal.movementDistance = distance(currentPosition, traversal.start);
            traversal.entryAxisReversalCount =
                traversal.entities.front().entryAxisReversalCount;
            traversal.entryTangentCost = traversal.entities.front().entryTangentCost;
            traversal.stableSourceIndex = traversal.entities.front().entity->sourceIndex;
            traversal.stableEntityId = traversal.entities.front().entity->entityId;
            return traversal;
        }

        double angleDegrees(const Vector3d& point, const machining::TubeSectionModel& section)
        {
            return std::atan2
            (
                point.z - section.geometry.centerZ,
                point.y - section.geometry.centerY
            ) * 180.0 / 3.14159265358979323846;
        }

        int surfaceIndex(double degrees)
        {
            double normalized = std::fmod(degrees + 360.0, 360.0);
            return static_cast<int>(std::floor((normalized + 45.0) / 90.0)) % 4;
        }

        void scoreTraversal
        (
            GroupTraversal& traversal,
            const Vector3d& currentPosition,
            const std::optional<machining::TubeSectionModel>& section
        )
        {
            traversal.movementDistance = distance(currentPosition, traversal.start);
            if (!section.has_value()) return;
            const double currentAngle = angleDegrees(currentPosition, *section);
            const double startAngle = angleDegrees(traversal.start, *section);
            traversal.rotationCost = std::abs(std::remainder(startAngle - currentAngle, 360.0));
            const int currentSurface = surfaceIndex(currentAngle);
            const int targetSurface = surfaceIndex(startAngle);
            const int rawDifference = std::abs(currentSurface - targetSurface);
            traversal.surfaceCost = std::min(rawDifference, 4 - rawDifference);
        }

        double entryThreshold(double connectionTolerance)
        {
            return std::max(kCalculationEpsilon, connectionTolerance * 1.0e-6);
        }

        std::optional<double> rotaryTravelLength
        (
            const Vector3d& start,
            const Vector3d& end,
            const Vector2d& center,
            double threshold
        )
        {
            const double startRadius = std::hypot(start.y - center.x, start.z - center.y);
            const double endRadius = std::hypot(end.y - center.x, end.z - center.y);
            const double localRadius = (startRadius + endRadius) * 0.5;
            if (!std::isfinite(localRadius) || localRadius <= threshold) return std::nullopt;

            const double startAngle = std::atan2(start.z - center.y, start.y - center.x);
            const double endAngle = std::atan2(end.z - center.y, end.y - center.x);
            const double angleDelta = std::remainder
                (endAngle - startAngle, 2.0 * 3.14159265358979323846);
            const double travel = localRadius * angleDelta;
            return std::isfinite(travel) ? std::optional<double>(travel) : std::nullopt;
        }

        void scoreEntrySmoothness
        (
            DirectedEntity& directed,
            const Vector3d& currentPosition,
            const Vector3d& nextPoint,
            const std::optional<Vector2d>& tubeCenter,
            double connectionTolerance
        )
        {
            const double threshold = entryThreshold(connectionTolerance);
            const double approachDx = directed.start.x - currentPosition.x;
            const double cutDx = nextPoint.x - directed.start.x;
            if (std::abs(approachDx) > threshold && std::abs(cutDx) > threshold
                && approachDx * cutDx < 0.0)
            {
                ++directed.entryAxisReversalCount;
            }

            if (tubeCenter.has_value()
                && std::isfinite(tubeCenter->x) && std::isfinite(tubeCenter->y))
            {
                const std::optional<double> approachRotary = rotaryTravelLength
                    (currentPosition, directed.start, *tubeCenter, threshold);
                const std::optional<double> cutRotary = rotaryTravelLength
                    (directed.start, nextPoint, *tubeCenter, threshold);
                if (approachRotary.has_value() && cutRotary.has_value()
                    && std::abs(*approachRotary) > threshold
                    && std::abs(*cutRotary) > threshold
                    && *approachRotary * *cutRotary < 0.0)
                {
                    ++directed.entryAxisReversalCount;
                }

                const double approachA = approachRotary.value_or(0.0);
                const double cutA = cutRotary.value_or(0.0);
                const double approachLength = std::hypot(approachDx, approachA);
                const double cutLength = std::hypot(cutDx, cutA);
                if (approachLength > threshold && cutLength > threshold)
                {
                    const double dotValue = (approachDx * cutDx + approachA * cutA)
                        / (approachLength * cutLength);
                    directed.entryTangentCost = std::clamp(1.0 - dotValue, 0.0, 2.0);
                }
                return;
            }

            const Vector3d approach
            {
                directed.start.x - currentPosition.x,
                directed.start.y - currentPosition.y,
                directed.start.z - currentPosition.z
            };
            const Vector3d cut
            {
                nextPoint.x - directed.start.x,
                nextPoint.y - directed.start.y,
                nextPoint.z - directed.start.z
            };
            const double approachLength = std::sqrt
                (approach.x * approach.x + approach.y * approach.y + approach.z * approach.z);
            const double cutLength = std::sqrt
                (cut.x * cut.x + cut.y * cut.y + cut.z * cut.z);
            if (approachLength > threshold && cutLength > threshold)
            {
                const double dotValue = (approach.x * cut.x + approach.y * cut.y
                    + approach.z * cut.z) / (approachLength * cutLength);
                directed.entryTangentCost = std::clamp(1.0 - dotValue, 0.0, 2.0);
            }
        }

        bool isSingleClosedEntryOptimizedCurve
        (
            const ProcessGroup& group,
            const PlanningEntity& entity
        )
        {
            return group.entityIds.size() == 1U && entity.path.closed
                && (entity.sourceKind == geometry::SourceGeometryKind::Circle
                    || entity.sourceKind == geometry::SourceGeometryKind::Ellipse);
        }

        std::optional<GroupTraversal> buildSingleClosedCurveTraversal
        (
            const ProcessGroup& group,
            const PlanningEntity& entity,
            const Vector3d& currentPosition,
            bool reverse,
            std::size_t startIndex,
            double connectionTolerance,
            const std::optional<machining::TubeSectionModel>& section,
            const std::optional<Vector2d>& tubeCenter
        )
        {
            const std::size_t pointCount = entity.path.vertices.size();
            if (pointCount < 2U || startIndex >= pointCount) return std::nullopt;

            std::vector<Vector3d> points;
            points.reserve(pointCount);
            for (std::size_t offset = 0U; offset < pointCount; ++offset)
            {
                const std::size_t index = reverse
                    ? (startIndex + pointCount - offset) % pointCount
                    : (startIndex + offset) % pointCount;
                points.push_back(entity.path.vertices[index].position);
            }

            const double threshold = entryThreshold(connectionTolerance);
            const auto next = std::find_if
            (
                points.cbegin() + 1,
                points.cend(),
                [&points, threshold](const Vector3d& point)
                {
                    return distance(points.front(), point) > threshold;
                }
            );
            if (next == points.cend()) return std::nullopt;

            const std::optional<double> selectedStartParameter = entity.startParameter.has_value()
                ? entity.startParameter
                : std::optional<double>(entity.path.vertices[startIndex].sourceParameter);
            if (!selectedStartParameter.has_value()
                || !std::isfinite(*selectedStartParameter)) return std::nullopt;

            DirectedEntity directed;
            directed.entity = &entity;
            directed.reverseRelativeToInput = reverse;
            directed.selectedStartParameter = selectedStartParameter;
            directed.start = points.front();
            directed.end = points.front();
            scoreEntrySmoothness
                (directed, currentPosition, *next, tubeCenter, connectionTolerance);

            GroupTraversal traversal;
            traversal.groupId = group.groupId;
            traversal.entities.push_back(directed);
            traversal.start = directed.start;
            traversal.end = directed.end;
            traversal.entryAxisReversalCount = directed.entryAxisReversalCount;
            traversal.entryTangentCost = directed.entryTangentCost;
            traversal.stableSourceIndex = entity.sourceIndex;
            traversal.stableEntityId = entity.entityId;
            scoreTraversal(traversal, currentPosition, section);
            return traversal;
        }

        bool selectedStartParameterLess
        (
            const std::optional<double>& left,
            const std::optional<double>& right
        )
        {
            if (left.has_value() != right.has_value()) return !left.has_value();
            if (!left.has_value()) return false;
            if (std::abs(*left - *right) > kCalculationEpsilon) return *left < *right;
            return false;
        }

        bool traversalLess
        (
            const GroupTraversal& left,
            const GroupTraversal& right,
            ProcessOrderingStrategy strategy
        )
        {
            if (strategy == ProcessOrderingStrategy::LazyRotation)
            {
                if (std::abs(left.rotationCost - right.rotationCost) > kCalculationEpsilon)
                    return left.rotationCost < right.rotationCost;
                if (left.surfaceCost != right.surfaceCost) return left.surfaceCost < right.surfaceCost;
                if (left.entryAxisReversalCount != right.entryAxisReversalCount)
                    return left.entryAxisReversalCount < right.entryAxisReversalCount;
                if (std::abs(left.entryTangentCost - right.entryTangentCost) > kCalculationEpsilon)
                    return left.entryTangentCost < right.entryTangentCost;
                if (std::abs(left.movementDistance - right.movementDistance) > kCalculationEpsilon)
                    return left.movementDistance < right.movementDistance;
            }
            else
            {
                if (std::abs(left.movementDistance - right.movementDistance) > kCalculationEpsilon)
                    return left.movementDistance < right.movementDistance;
                if (left.entryAxisReversalCount != right.entryAxisReversalCount)
                    return left.entryAxisReversalCount < right.entryAxisReversalCount;
                if (std::abs(left.entryTangentCost - right.entryTangentCost) > kCalculationEpsilon)
                    return left.entryTangentCost < right.entryTangentCost;
            }
            if (left.stableSourceIndex != right.stableSourceIndex)
                return left.stableSourceIndex < right.stableSourceIndex;
            if (left.stableEntityId != right.stableEntityId)
                return left.stableEntityId < right.stableEntityId;
            const std::optional<double> leftStart = left.entities.empty()
                ? std::nullopt : left.entities.front().selectedStartParameter;
            const std::optional<double> rightStart = right.entities.empty()
                ? std::nullopt : right.entities.front().selectedStartParameter;
            if (selectedStartParameterLess(leftStart, rightStart)) return true;
            if (selectedStartParameterLess(rightStart, leftStart)) return false;
            const bool leftReverse = !left.entities.empty()
                && left.entities.front().reverseRelativeToInput;
            const bool rightReverse = !right.entities.empty()
                && right.entities.front().reverseRelativeToInput;
            return leftReverse < rightReverse;
        }

        QVariantMap closedLoopDiagnosticValues(const ClosedLoopTraversalReport& report)
        {
            auto entityIdsText = [](const std::vector<EntityId>& entityIds)
            {
                QStringList values;
                values.reserve(static_cast<qsizetype>(entityIds.size()));
                for (const EntityId entityId : entityIds)
                    values.push_back(QString::number(entityId));
                return values.join(QLatin1Char(','));
            };
            QStringList reverseValues;
            reverseValues.reserve(static_cast<qsizetype>(report.selectedReverse.size()));
            for (const bool reverse : report.selectedReverse)
                reverseValues.push_back(reverse ? QStringLiteral("1") : QStringLiteral("0"));

            QVariantMap values;
            values.insert(QStringLiteral("closedLoopSummary"), true);
            values.insert(QStringLiteral("groupId"), report.groupId);
            values.insert(QStringLiteral("memberCount"), report.memberCount);
            values.insert(QStringLiteral("memberEntityIds"), entityIdsText(report.memberEntityIds));
            values.insert(QStringLiteral("nodeCount"), report.nodeCount);
            values.insert(QStringLiteral("connectedComponentCount"), report.connectedComponentCount);
            values.insert(QStringLiteral("branchNodeCount"), report.branchNodeCount);
            values.insert(QStringLiteral("invalidDegreeNodeCount"), report.invalidDegreeNodeCount);
            values.insert(QStringLiteral("candidateCount"), report.candidateCount);
            values.insert(QStringLiteral("selectedOrder"), entityIdsText(report.selectedOrder));
            values.insert(QStringLiteral("selectedReverse"), reverseValues.join(QLatin1Char(',')));
            values.insert(QStringLiteral("status"), report.status);
            values.insert(QStringLiteral("failureReason"), report.failureReason);
            return values;
        }

        class ClosedLoopTraversalBuilder
        {
        public:
            struct Result
            {
                std::optional<GroupTraversal> traversal;
                ClosedLoopTraversalReport report;
            };

            static Result build
            (
                const ProcessGroup& group,
                const std::unordered_map<EntityId, const PlanningEntity*>& entities,
                const Vector3d& currentPosition,
                const ProcessPlanningPolicy& policy,
                const std::optional<machining::TubeSectionModel>& section,
                const std::optional<Vector2d>& tubeCenter,
                ProcessOrderingStrategy selectionStrategy
            )
            {
                Result result;
                result.report.groupId = group.groupId;
                result.report.memberEntityIds = group.entityIds;
                std::sort(result.report.memberEntityIds.begin(), result.report.memberEntityIds.end());
                result.report.memberCount = static_cast<int>(group.entityIds.size());

                struct Edge
                {
                    const PlanningEntity* entity = nullptr;
                    Vector3d sourceStart;
                    Vector3d sourceEnd;
                    int startNode = -1;
                    int endNode = -1;
                };
                std::vector<Edge> edges;
                edges.reserve(group.entityIds.size());
                std::set<EntityId> uniqueIds;
                for (const EntityId entityId : group.entityIds)
                {
                    const auto found = entities.find(entityId);
                    if (found == entities.end() || found->second == nullptr
                        || !uniqueIds.insert(entityId).second)
                    {
                        result.report.failureReason = QStringLiteral("Closed-loop member is missing or duplicated.");
                        return result;
                    }
                    const PlanningEntity& entity = *found->second;
                    if (entity.path.closed || entity.path.vertices.size() < 2U)
                    {
                        result.report.failureReason = entity.path.closed
                            ? QStringLiteral("Multi-entity closed loop contains a semantically closed member.")
                            : QStringLiteral("Closed-loop member has fewer than two path points.");
                        return result;
                    }
                    const Vector3d sourceStart = entity.path.vertices.front().position;
                    const Vector3d sourceEnd = entity.path.vertices.back().position;
                    if (!std::isfinite(sourceStart.x) || !std::isfinite(sourceStart.y)
                        || !std::isfinite(sourceStart.z) || !std::isfinite(sourceEnd.x)
                        || !std::isfinite(sourceEnd.y) || !std::isfinite(sourceEnd.z))
                    {
                        result.report.failureReason = QStringLiteral("Closed-loop member endpoint is not finite.");
                        return result;
                    }
                    edges.push_back({ &entity, sourceStart, sourceEnd });
                }
                std::sort(edges.begin(), edges.end(), [](const Edge& left, const Edge& right)
                {
                    if (left.entity->sourceIndex != right.entity->sourceIndex)
                        return left.entity->sourceIndex < right.entity->sourceIndex;
                    return left.entity->entityId < right.entity->entityId;
                });

                const std::size_t endpointCount = edges.size() * 2U;
                std::vector<std::size_t> parents(endpointCount);
                for (std::size_t index = 0; index < endpointCount; ++index) parents[index] = index;
                const auto findRoot = [&parents](std::size_t value)
                {
                    std::size_t root = value;
                    while (parents[root] != root) root = parents[root];
                    while (parents[value] != value)
                    {
                        const std::size_t next = parents[value];
                        parents[value] = root;
                        value = next;
                    }
                    return root;
                };
                const auto endpoint = [&edges](std::size_t index) -> const Vector3d&
                {
                    const Edge& edge = edges[index / 2U];
                    return index % 2U == 0U ? edge.sourceStart : edge.sourceEnd;
                };
                for (std::size_t left = 0; left < endpointCount; ++left)
                {
                    for (std::size_t right = left + 1U; right < endpointCount; ++right)
                    {
                        if (distance(endpoint(left), endpoint(right)) > policy.connectionTolerance) continue;
                        const std::size_t leftRoot = findRoot(left);
                        const std::size_t rightRoot = findRoot(right);
                        if (leftRoot != rightRoot) parents[rightRoot] = leftRoot;
                    }
                }

                std::map<std::size_t, int> nodeByRoot;
                for (std::size_t edgeIndex = 0; edgeIndex < edges.size(); ++edgeIndex)
                {
                    const std::size_t startRoot = findRoot(edgeIndex * 2U);
                    const std::size_t endRoot = findRoot(edgeIndex * 2U + 1U);
                    const auto nodeFor = [&nodeByRoot](std::size_t root)
                    {
                        const auto inserted = nodeByRoot.emplace
                            (root, static_cast<int>(nodeByRoot.size()));
                        return inserted.first->second;
                    };
                    edges[edgeIndex].startNode = nodeFor(startRoot);
                    edges[edgeIndex].endNode = nodeFor(endRoot);
                }
                result.report.nodeCount = static_cast<int>(nodeByRoot.size());

                std::vector<std::vector<std::size_t>> adjacency(nodeByRoot.size());
                for (std::size_t edgeIndex = 0; edgeIndex < edges.size(); ++edgeIndex)
                {
                    adjacency[static_cast<std::size_t>(edges[edgeIndex].startNode)].push_back(edgeIndex);
                    adjacency[static_cast<std::size_t>(edges[edgeIndex].endNode)].push_back(edgeIndex);
                }
                for (const auto& incidentEdges : adjacency)
                {
                    if (incidentEdges.size() > 2U) ++result.report.branchNodeCount;
                    if (incidentEdges.size() != 2U) ++result.report.invalidDegreeNodeCount;
                }

                std::vector<bool> visitedNodes(adjacency.size(), false);
                for (std::size_t node = 0; node < adjacency.size(); ++node)
                {
                    if (visitedNodes[node] || adjacency[node].empty()) continue;
                    ++result.report.connectedComponentCount;
                    std::vector<std::size_t> pending{ node };
                    visitedNodes[node] = true;
                    while (!pending.empty())
                    {
                        const std::size_t currentNode = pending.back();
                        pending.pop_back();
                        for (const std::size_t edgeIndex : adjacency[currentNode])
                        {
                            const Edge& edge = edges[edgeIndex];
                            const std::size_t nextNode = static_cast<std::size_t>
                                (edge.startNode == static_cast<int>(currentNode)
                                    ? edge.endNode : edge.startNode);
                            if (!visitedNodes[nextNode])
                            {
                                visitedNodes[nextNode] = true;
                                pending.push_back(nextNode);
                            }
                        }
                    }
                }

                result.report.simpleLoopValid = result.report.connectedComponentCount == 1
                    && result.report.branchNodeCount == 0
                    && result.report.invalidDegreeNodeCount == 0
                    && edges.size() == adjacency.size();
                if (!result.report.simpleLoopValid)
                {
                    result.report.failureReason = QStringLiteral("Closed-loop endpoint graph is not one simple cycle.");
                    return result;
                }

                std::optional<GroupTraversal> best;
                int bestStartEndpoint = -1;
                int bestLoopDirection = -1;
                for (std::size_t startEdgeIndex = 0; startEdgeIndex < edges.size(); ++startEdgeIndex)
                {
                    for (const bool startReverse : { false, true })
                    {
                        const Edge& startEdge = edges[startEdgeIndex];
                        if (!directionAllowed(*startEdge.entity, startReverse, policy.allowReverse)) continue;

                        GroupTraversal candidate;
                        candidate.groupId = group.groupId;
                        std::vector<bool> used(edges.size(), false);
                        int currentNode = startReverse ? startEdge.endNode : startEdge.startNode;
                        const int initialNode = currentNode;
                        Vector3d previousEnd;
                        bool hasPreviousEnd = false;
                        Vector3d firstNextPoint;
                        bool candidateValid = true;

                        for (std::size_t step = 0; step < edges.size(); ++step)
                        {
                            std::vector<std::size_t> unusedIncident;
                            for (const std::size_t edgeIndex : adjacency[static_cast<std::size_t>(currentNode)])
                            {
                                if (!used[edgeIndex]
                                    && std::find(unusedIncident.begin(), unusedIncident.end(), edgeIndex)
                                        == unusedIncident.end())
                                    unusedIncident.push_back(edgeIndex);
                            }
                            const std::size_t edgeIndex = step == 0U
                                ? startEdgeIndex
                                : unusedIncident.size() == 1U
                                    ? unusedIncident.front() : edges.size();
                            if (edgeIndex >= edges.size() || used[edgeIndex])
                            {
                                candidateValid = false;
                                break;
                            }

                            const Edge& edge = edges[edgeIndex];
                            const bool reverse = edge.endNode == currentNode;
                            if ((edge.startNode != currentNode && edge.endNode != currentNode)
                                || !directionAllowed(*edge.entity, reverse, policy.allowReverse))
                            {
                                candidateValid = false;
                                break;
                            }
                            std::vector<Vector3d> points = directedPoints(*edge.entity, reverse);
                            if (points.size() < 2U
                                || (hasPreviousEnd
                                    && distance(previousEnd, points.front()) > policy.connectionTolerance))
                            {
                                candidateValid = false;
                                break;
                            }
                            const double threshold = entryThreshold(policy.connectionTolerance);
                            const auto nextPoint = std::find_if
                            (
                                points.cbegin() + 1,
                                points.cend(),
                                [&points, threshold](const Vector3d& point)
                                { return distance(points.front(), point) > threshold; }
                            );
                            if (nextPoint == points.cend())
                            {
                                candidateValid = false;
                                break;
                            }

                            DirectedEntity directed;
                            directed.entity = edge.entity;
                            directed.reverseRelativeToInput = reverse;
                            directed.selectedStartParameter = edge.entity->startParameter;
                            directed.start = points.front();
                            directed.end = points.back();
                            candidate.entities.push_back(directed);
                            if (step == 0U) firstNextPoint = *nextPoint;
                            previousEnd = directed.end;
                            hasPreviousEnd = true;
                            used[edgeIndex] = true;
                            currentNode = reverse ? edge.startNode : edge.endNode;
                        }

                        if (!candidateValid || currentNode != initialNode
                            || candidate.entities.size() != edges.size()) continue;
                        candidate.start = candidate.entities.front().start;
                        if (distance(candidate.entities.back().end, candidate.start)
                            > policy.connectionTolerance) continue;
                        candidate.end = candidate.start;
                        scoreEntrySmoothness
                        (
                            candidate.entities.front(), currentPosition, firstNextPoint,
                            tubeCenter, policy.connectionTolerance
                        );
                        candidate.entryAxisReversalCount =
                            candidate.entities.front().entryAxisReversalCount;
                        candidate.entryTangentCost = candidate.entities.front().entryTangentCost;
                        candidate.stableSourceIndex = candidate.entities.front().entity->sourceIndex;
                        candidate.stableEntityId = candidate.entities.front().entity->entityId;
                        scoreTraversal(candidate, currentPosition, section);
                        ++result.report.candidateCount;

                        const int startEndpoint = startReverse ? 1 : 0;
                        const int loopDirection = startReverse ? 1 : 0;
                        const auto stableLess = [&candidate, startEndpoint, loopDirection,
                            &best, bestStartEndpoint, bestLoopDirection, selectionStrategy]()
                        {
                            if (!best.has_value()) return true;
                            if (traversalLess(candidate, *best, selectionStrategy)) return true;
                            if (traversalLess(*best, candidate, selectionStrategy)) return false;
                            if (startEndpoint != bestStartEndpoint)
                                return startEndpoint < bestStartEndpoint;
                            if (loopDirection != bestLoopDirection)
                                return loopDirection < bestLoopDirection;
                            std::vector<EntityId> candidateOrder;
                            std::vector<EntityId> bestOrder;
                            for (const DirectedEntity& directed : candidate.entities)
                                candidateOrder.push_back(directed.entity->entityId);
                            for (const DirectedEntity& directed : best->entities)
                                bestOrder.push_back(directed.entity->entityId);
                            return candidateOrder < bestOrder;
                        };
                        if (stableLess())
                        {
                            best = std::move(candidate);
                            bestStartEndpoint = startEndpoint;
                            bestLoopDirection = loopDirection;
                        }
                    }
                }

                if (!best.has_value())
                {
                    result.report.failureReason = QStringLiteral("No complete loop traversal satisfies member direction constraints.");
                    return result;
                }
                result.report.status = QStringLiteral("Success");
                for (const DirectedEntity& directed : best->entities)
                {
                    result.report.selectedOrder.push_back(directed.entity->entityId);
                    result.report.selectedReverse.push_back(directed.reverseRelativeToInput);
                }
                result.traversal = std::move(best);
                return result;
            }
        };

        std::optional<GroupTraversal> bestTraversal
        (
            const ProcessGroup& group,
            const std::unordered_map<EntityId, const PlanningEntity*>& entities,
            const Vector3d& currentPosition,
            const ProcessPlanningPolicy& policy,
            const std::optional<machining::TubeSectionModel>& section,
            const std::optional<Vector2d>& tubeCenter,
            ProcessOrderingStrategy selectionStrategy,
            ClosedLoopTraversalReport* closedLoopReport = nullptr
        )
        {
            if (closedLoopReport != nullptr) *closedLoopReport = ClosedLoopTraversalReport{};
            if (group.entityIds.size() == 1U)
            {
                const auto found = entities.find(group.entityIds.front());
                if (found == entities.end()) return std::nullopt;
                const PlanningEntity& entity = *found->second;
                if (isSingleClosedEntryOptimizedCurve(group, entity))
                {
                    std::optional<GroupTraversal> best;
                    const std::size_t startCandidateCount = entity.startParameter.has_value()
                        ? 1U : entity.path.vertices.size();
                    for (std::size_t startIndex = 0U;
                        startIndex < startCandidateCount; ++startIndex)
                    {
                        for (const bool reverse : { false, true })
                        {
                            if (!directionAllowed(entity, reverse, policy.allowReverse)) continue;
                            auto candidate = buildSingleClosedCurveTraversal
                            (
                                group, entity, currentPosition, reverse, startIndex,
                                policy.connectionTolerance, section, tubeCenter
                            );
                            if (!candidate.has_value()) continue;
                            if (!best.has_value()
                                || traversalLess(*candidate, *best, selectionStrategy))
                            {
                                best = std::move(candidate);
                            }
                        }
                    }
                    return best;
                }
            }

            if (group.kind == ProcessGroupKind::ClosedLoop && group.entityIds.size() > 1U)
            {
                auto closedLoop = ClosedLoopTraversalBuilder::build
                (
                    group, entities, currentPosition, policy, section,
                    tubeCenter, selectionStrategy
                );
                if (closedLoopReport != nullptr) *closedLoopReport = std::move(closedLoop.report);
                return std::move(closedLoop.traversal);
            }

            std::optional<GroupTraversal> best;
            for (const EntityId entityId : group.entityIds)
            {
                for (const bool reverse : { false, true })
                {
                    const auto found = entities.find(entityId);
                    if (found == entities.end()
                        || !directionAllowed(*found->second, reverse, policy.allowReverse)) continue;
                    auto candidate = buildTraversal
                    (
                        group, entities, currentPosition, policy.allowReverse,
                        policy.connectionTolerance, std::make_pair(entityId, reverse)
                    );
                    if (!candidate.has_value()) continue;
                    scoreTraversal(*candidate, currentPosition, section);
                    if (!best.has_value() || traversalLess(*candidate, *best, selectionStrategy))
                        best = std::move(candidate);
                }
            }
            return best;
        }

        bool sameEntitySet(std::vector<EntityId> left, std::vector<EntityId> right)
        {
            std::sort(left.begin(), left.end());
            std::sort(right.begin(), right.end());
            return left == right;
        }

        struct ClosedLoopValidationFailure
        {
            int groupId = -1;
            EntityId previousEntityId = 0;
            EntityId currentEntityId = 0;
            double joinGap = 0.0;
            QString reason;
        };

        bool validateMultiEntityClosedLoopUnits
        (
            const ProcessPlan& plan,
            const std::unordered_map<EntityId, const PlanningEntity*>& entities,
            double connectionTolerance,
            ClosedLoopValidationFailure& failure
        )
        {
            std::map<EntityId, const ProcessAssignment*> assignments;
            for (const ProcessAssignment& assignment : plan.assignments)
                assignments.emplace(assignment.entityId, &assignment);

            for (const ProcessGroup& group : plan.groups)
            {
                if (group.kind != ProcessGroupKind::ClosedLoop
                    || group.entityIds.size() <= 1U) continue;
                std::vector<EntityId> key = group.entityIds;
                std::sort(key.begin(), key.end());
                const auto unit = std::find_if
                (
                    plan.processUnits.cbegin(), plan.processUnits.cend(),
                    [&key](const ProcessUnit& candidate)
                    { return candidate.key.memberEntityIds == key; }
                );
                if (unit == plan.processUnits.cend()
                    || unit->orderedMemberEntityIds.size() != group.entityIds.size())
                {
                    failure.groupId = group.groupId;
                    failure.reason = QStringLiteral("Closed-loop ProcessUnit is missing or incomplete.");
                    return false;
                }

                Vector3d firstStart;
                Vector3d previousEnd;
                EntityId previousEntityId = 0;
                bool first = true;
                for (const EntityId entityId : unit->orderedMemberEntityIds)
                {
                    const auto entity = entities.find(entityId);
                    const auto assignment = assignments.find(entityId);
                    if (entity == entities.end() || assignment == assignments.end()
                        || entity->second == nullptr || entity->second->path.closed)
                    {
                        failure.groupId = group.groupId;
                        failure.currentEntityId = entityId;
                        failure.reason = QStringLiteral("Closed-loop member or assignment is invalid.");
                        return false;
                    }
                    const std::vector<Vector3d> points = directedPoints
                        (*entity->second, assignment->second->reverse);
                    if (points.size() < 2U)
                    {
                        failure.groupId = group.groupId;
                        failure.currentEntityId = entityId;
                        failure.reason = QStringLiteral("Closed-loop member has no physical endpoints.");
                        return false;
                    }
                    if (first)
                    {
                        firstStart = points.front();
                        first = false;
                    }
                    else
                    {
                        const double gap = distance(previousEnd, points.front());
                        if (gap > connectionTolerance)
                        {
                            failure = { group.groupId, previousEntityId, entityId, gap,
                                QStringLiteral("Adjacent closed-loop members are not physically connected.") };
                            return false;
                        }
                    }
                    previousEnd = points.back();
                    previousEntityId = entityId;
                }
                const double closureGap = distance(previousEnd, firstStart);
                if (closureGap > connectionTolerance)
                {
                    failure = { group.groupId, previousEntityId,
                        unit->orderedMemberEntityIds.front(), closureGap,
                        QStringLiteral("Closed-loop traversal does not return to its physical start.") };
                    return false;
                }
            }
            return true;
        }
    }

    OperationResult<ProcessPlan> ProcessPlanBuilder::build
    (
        const ProcessPlanningInput& input,
        const ProcessPlanningPolicy& policy,
        const OperationContext& context
    )
    {
        if (input.contentRevision == 0U || input.topology == nullptr
            || input.entities.empty() || policy.connectionTolerance <= 0.0
            || !std::isfinite(policy.connectionTolerance))
        {
            return failure<ProcessPlan>
            (
                OperationStatus::InvalidInput, context, DiagnosticCode::ProcessPlanningInputInvalid,
                QStringLiteral("加工计划输入无效。"), QStringLiteral("Revision, topology, entities, or policy is invalid."),
                diagnosticValues(input, policy)
            );
        }
        if (input.topologyInput.contentRevision != input.contentRevision)
        {
            return failure<ProcessPlan>
            (
                OperationStatus::Conflict, context, DiagnosticCode::ProcessPlanningRevisionMismatch,
                QStringLiteral("加工计划输入版本不一致。"), QStringLiteral("TopologyInput revision does not match planning input."),
                diagnosticValues(input, policy)
            );
        }
        if (input.tubeSection.has_value()
            && input.tubeSection->contentRevision != input.contentRevision)
        {
            return failure<ProcessPlan>
            (
                OperationStatus::Conflict, context, DiagnosticCode::ProcessPlanningRevisionMismatch,
                QStringLiteral("方管截面已过期，请重新识别后再排序。"),
                QStringLiteral("TubeSectionModel revision does not match planning input."),
                diagnosticValues(input, policy)
            );
        }
        if (policy.orderingStrategy == ProcessOrderingStrategy::LazyRotation && !input.tubeSection.has_value())
        {
            return failure<ProcessPlan>
            (
                OperationStatus::InvalidInput, context, DiagnosticCode::ProcessPlanningInputInvalid,
                QStringLiteral("启用懒旋转加工前需要先识别方管截面。"), QStringLiteral("LazyRotation requires TubeSectionModel."),
                diagnosticValues(input, policy)
            );
        }

        std::optional<machining::TubeSectionGeometry> planningSection;
        if (input.tubeSection.has_value())
        {
            auto preparedSection = TubeCutBoundaryClassifier::prepareSection
                (input.tubeSection->geometry, context);
            if (!preparedSection.succeeded() || !preparedSection.value.has_value())
            {
                auto result = failure<ProcessPlan>
                (
                    OperationStatus::InvalidInput, context,
                    DiagnosticCode::ProcessPlanningInputInvalid,
                    QStringLiteral("已识别的方管截面无法用于加工计划。"),
                    QStringLiteral("Tube section normalization failed before planning."),
                    diagnosticValues(input, policy)
                );
                result.mergeDiagnostics(preparedSection.diagnostics);
                return result;
            }
            planningSection = std::move(*preparedSection.value);
        }

        ProcessPlan plan;
        plan.contentRevision = input.contentRevision;
        plan.processStateRevision = input.processStateRevision;
        plan.mode = ProcessPlanMode::Rotary4Axis;
        plan.orderingStrategy = policy.orderingStrategy;
        std::unordered_map<EntityId, const PlanningEntity*> entities;
        std::unordered_set<EntityId> seen;

        std::vector<EntityId> ordinaryIds;
        std::map<std::pair<int, BoundaryRole>, std::vector<EntityId>> boundaryIds;
        for (const PlanningEntity& entity : input.entities)
        {
            if (entity.entityId == 0U || !seen.insert(entity.entityId).second)
            {
                return failure<ProcessPlan>
                (
                    OperationStatus::InvalidInput, context, DiagnosticCode::ProcessPlanningInputInvalid,
                    QStringLiteral("加工计划包含无效或重复图元编号。"), QStringLiteral("EntityId is zero or duplicated."),
                    diagnosticValues(input, policy, entity.entityId, entity.sourceIndex)
                );
            }
            entities.emplace(entity.entityId, &entity);
            const ProcessExclusionReason reason = !entity.visible
                ? ProcessExclusionReason::Hidden
                : !entity.processEnabled
                    ? ProcessExclusionReason::UserDisabled
                    : entity.excludedAsInternalGeometry && entity.boundaryRole == BoundaryRole::None
                        ? ProcessExclusionReason::InternalGeometry
                        : entity.sourceKind == geometry::SourceGeometryKind::Point
                            || entity.sourceKind == geometry::SourceGeometryKind::Unknown
                            ? ProcessExclusionReason::UnsupportedGeometry
                            : entity.path.vertices.size() < 2U
                                ? ProcessExclusionReason::InvalidPath
                                : ProcessExclusionReason::InvalidPath;
            const bool excluded = !entity.visible || !entity.processEnabled
                || (entity.excludedAsInternalGeometry && entity.boundaryRole == BoundaryRole::None)
                || entity.sourceKind == geometry::SourceGeometryKind::Point
                || entity.sourceKind == geometry::SourceGeometryKind::Unknown
                || entity.path.vertices.size() < 2U;
            if (excluded)
            {
                plan.exclusions.push_back({ entity.entityId, reason });
                continue;
            }
            if (entity.boundaryRole != BoundaryRole::None && entity.boundaryPairId >= 0)
                boundaryIds[{ entity.boundaryPairId, entity.boundaryRole }].push_back(entity.entityId);
            else
                ordinaryIds.push_back(entity.entityId);
        }

        std::sort(ordinaryIds.begin(), ordinaryIds.end(), [&entities](EntityId left, EntityId right)
        {
            const PlanningEntity* l = entities.at(left);
            const PlanningEntity* r = entities.at(right);
            return l->sourceIndex != r->sourceIndex ? l->sourceIndex < r->sourceIndex : left < right;
        });

        std::vector<BoundaryIdentity> boundaryIdentities;
        boundaryIdentities.reserve(boundaryIds.size());
        for (const auto& [key, ids] : boundaryIds)
        {
            BoundaryIdentity identity;
            identity.pairId = key.first;
            identity.role = key.second;
            identity.entityIds = ids;
            identity.stableSourceIndex = std::numeric_limits<std::size_t>::max();
            identity.stableEntityId = std::numeric_limits<EntityId>::max();
            for (const EntityId entityId : ids)
            {
                identity.stableSourceIndex = std::min
                    (identity.stableSourceIndex, entities.at(entityId)->sourceIndex);
                identity.stableEntityId = std::min(identity.stableEntityId, entityId);
            }
            boundaryIdentities.push_back(std::move(identity));
        }
        std::sort(boundaryIdentities.begin(), boundaryIdentities.end(),
            [](const BoundaryIdentity& left, const BoundaryIdentity& right)
            {
                if (left.stableSourceIndex != right.stableSourceIndex)
                    return left.stableSourceIndex < right.stableSourceIndex;
                if (left.stableEntityId != right.stableEntityId)
                    return left.stableEntityId < right.stableEntityId;
                return static_cast<int>(left.role) < static_cast<int>(right.role);
            });

        std::vector<BoundaryData> boundaries;
        for (const BoundaryIdentity& identity : boundaryIdentities)
        {
            ProcessGroup group;
            group.groupId = static_cast<int>(plan.groups.size());
            group.kind = identity.role == BoundaryRole::Break
                ? ProcessGroupKind::BreakBoundary
                : ProcessGroupKind::WasteBoundary;
            group.closed = true;
            group.entityIds = identity.entityIds;
            std::sort(group.entityIds.begin(), group.entityIds.end(), [&entities](EntityId left, EntityId right)
            {
                return entities.at(left)->sourceIndex < entities.at(right)->sourceIndex;
            });
            plan.groups.push_back(group);
            if (identity.role == BoundaryRole::Waste)
            {
                for (const EntityId entityId : identity.entityIds)
                    plan.exclusions.push_back({ entityId, ProcessExclusionReason::WasteRegion });
            }

            if (!input.tubeSection.has_value())
            {
                return failure<ProcessPlan>
                (
                    OperationStatus::InvalidInput, context, DiagnosticCode::ProcessPlanningBoundaryInvalid,
                    QStringLiteral("验证加工断面前需要先识别方管截面。"), QStringLiteral("Boundary validation requires TubeSectionModel."),
                    diagnosticValues(input, policy, 0U, 0U, identity.pairId, group.groupId)
                );
            }
            const auto loop = input.topology->extractBestLoop
                (identity.entityIds, identity.entityIds);
            if (!loop.succeeded() || !loop.value.has_value() || !loop.value->connectedLoop
                || !sameEntitySet(loop.value->usedEntityIds, identity.entityIds))
            {
                auto result = failure<ProcessPlan>
                (
                    OperationStatus::Failed, context, DiagnosticCode::ProcessPlanningBoundaryInvalid,
                    QStringLiteral("加工断面无法形成包含全部指定图元的严格闭环。"), QStringLiteral("Boundary strict-loop extraction failed."),
                    diagnosticValues(input, policy, 0U, 0U, identity.pairId, group.groupId)
                );
                result.mergeDiagnostics(loop.diagnostics);
                return result;
            }
            const double surfaceMappingTolerance = std::clamp
                (policy.connectionTolerance * 0.01, 1.0e-4, 0.01);
            const auto analysis = TubeCutBoundaryClassifier::analyze
            (
                loop.value->orderedPath,
                loop.value->usedEntityIds,
                loop.value->maximumJoinGap,
                *planningSection,
                context,
                surfaceMappingTolerance
            );
            if (!analysis.succeeded() || !analysis.value.has_value())
            {
                auto result = failure<ProcessPlan>
                (
                    OperationStatus::Failed, context, DiagnosticCode::ProcessPlanningBoundaryInvalid,
                    QStringLiteral("加工断面重新验证失败，无法建立安全加工计划。"), QStringLiteral("TubeCutBoundaryClassifier analysis failed."),
                    diagnosticValues(input, policy, 0U, 0U, identity.pairId, group.groupId)
                );
                result.mergeDiagnostics(analysis.diagnostics);
                return result;
            }
            if (analysis.value->result == TubeCutResult::Indeterminate)
            {
                return failure<ProcessPlan>
                (
                    OperationStatus::Failed, context, DiagnosticCode::ProcessPlanningBoundaryInvalid,
                    QStringLiteral("加工断面判定不确定，无法建立安全加工计划。"), QStringLiteral("TubeCutBoundaryClassifier returned Indeterminate."),
                    diagnosticValues(input, policy, 0U, 0U, identity.pairId, group.groupId)
                );
            }
            if (analysis.value->result == TubeCutResult::KeepsLeftAndRight)
            {
                return failure<ProcessPlan>
                (
                    OperationStatus::Failed, context, DiagnosticCode::ProcessPlanningBoundaryKeepsConnected,
                    QStringLiteral("加工断面仍会保留左右材料桥，不能作为中断切面。"), QStringLiteral("Boundary winding is zero."),
                    diagnosticValues(input, policy, 0U, 0U, identity.pairId, group.groupId)
                );
            }
            boundaries.push_back
                ({ group.groupId, identity.pairId, identity.role, *analysis.value });
        }

        if (!ordinaryIds.empty())
        {
            const std::vector<int> componentIds = input.topology->componentIds(ordinaryIds);
            if (componentIds.size() != ordinaryIds.size())
            {
                return failure<ProcessPlan>
                (
                    OperationStatus::Failed, context, DiagnosticCode::ProcessPlanningGroupBuildFailed,
                    QStringLiteral("普通加工图元的拓扑分组失败。"), QStringLiteral("componentIds size mismatch."),
                    diagnosticValues(input, policy)
                );
            }
            std::map<int, std::vector<EntityId>> components;
            int syntheticComponent = -1;
            for (std::size_t index = 0; index < ordinaryIds.size(); ++index)
            {
                const int componentId = componentIds[index] >= 0 ? componentIds[index] : syntheticComponent--;
                components[componentId].push_back(ordinaryIds[index]);
            }
            for (auto& [componentId, ids] : components)
            {
                (void)componentId;
                ProcessGroup group;
                group.groupId = static_cast<int>(plan.groups.size());
                group.entityIds = ids;
                if (ids.size() == 1U && entities.at(ids.front())->path.closed)
                {
                    group.kind = ProcessGroupKind::ClosedLoop;
                    group.closed = true;
                }
                else if (ids.size() == 1U)
                {
                    group.kind = ProcessGroupKind::SingleEntity;
                }
                else
                {
                    const auto loop = input.topology->extractBestLoop(ids, ids);
                    group.closed = policy.preserveClosedLoopsAsAtomicGroups
                        && loop.succeeded() && loop.value.has_value() && loop.value->connectedLoop
                        && sameEntitySet(loop.value->usedEntityIds, ids);
                    group.kind = group.closed ? ProcessGroupKind::ClosedLoop : ProcessGroupKind::ConnectedChain;
                }
                plan.groups.push_back(group);
            }
        }

        std::unordered_set<int> excludedGroups;
        for (const ProcessGroup& group : plan.groups)
            if (group.kind == ProcessGroupKind::WasteBoundary) excludedGroups.insert(group.groupId);

        std::vector<std::vector<int>> boundarySuccessors(boundaries.size());
        std::vector<int> boundaryIndegree(boundaries.size(), 0);
        for (std::size_t leftIndex = 0; leftIndex < boundaries.size(); ++leftIndex)
        {
            for (std::size_t rightIndex = leftIndex + 1U;
                rightIndex < boundaries.size(); ++rightIndex)
            {
                const BoundaryData& leftBoundary = boundaries[leftIndex];
                const BoundaryData& rightBoundary = boundaries[rightIndex];
                const ProcessGroup& leftGroup = plan.groups
                    [static_cast<std::size_t>(leftBoundary.groupId)];
                const ProcessGroup& rightGroup = plan.groups
                    [static_cast<std::size_t>(rightBoundary.groupId)];
                const BoundarySide leftRelativeToRight = classifyGroup
                (
                    leftGroup, entities, rightBoundary,
                    *planningSection, policy.connectionTolerance
                );
                const BoundarySide rightRelativeToLeft = classifyGroup
                (
                    rightGroup, entities, leftBoundary,
                    *planningSection, policy.connectionTolerance
                );
                const bool leftBeforeRight = leftRelativeToRight == BoundarySide::Left
                    && rightRelativeToLeft == BoundarySide::Right;
                const bool rightBeforeLeft = leftRelativeToRight == BoundarySide::Right
                    && rightRelativeToLeft == BoundarySide::Left;
                if (!leftBeforeRight && !rightBeforeLeft)
                {
                    const bool crossing = leftRelativeToRight == BoundarySide::Mixed
                        || rightRelativeToLeft == BoundarySide::Mixed;
                    const bool overlapping = leftRelativeToRight == BoundarySide::OnBoundary
                        || rightRelativeToLeft == BoundarySide::OnBoundary;
                    return failure<ProcessPlan>
                    (
                        OperationStatus::Failed, context,
                        DiagnosticCode::ProcessPlanningBoundaryClassificationFailed,
                        crossing
                            ? QStringLiteral("两个加工断面相互交叉，无法确定空间顺序。")
                            : overlapping
                                ? QStringLiteral("两个加工断面重合，无法建立独立工艺屏障。")
                                : QStringLiteral("两个加工断面无法严格区分左右空间关系。"),
                        QStringLiteral("Bidirectional boundary classification is not a strict Left/Right pair."),
                        boundaryDiagnosticValues
                        (
                            input, policy, rightBoundary, leftGroup, entities, -1,
                            leftRelativeToRight, rightRelativeToLeft
                        )
                    );
                }

                const std::size_t predecessor = leftBeforeRight ? leftIndex : rightIndex;
                const std::size_t successor = leftBeforeRight ? rightIndex : leftIndex;
                boundarySuccessors[predecessor].push_back(static_cast<int>(successor));
                ++boundaryIndegree[successor];
            }
        }

        std::vector<int> boundaryOrder;
        boundaryOrder.reserve(boundaries.size());
        std::vector<bool> boundaryScheduled(boundaries.size(), false);
        while (boundaryOrder.size() < boundaries.size())
        {
            std::vector<int> eligibleBoundaries;
            for (std::size_t index = 0; index < boundaries.size(); ++index)
                if (!boundaryScheduled[index] && boundaryIndegree[index] == 0)
                    eligibleBoundaries.push_back(static_cast<int>(index));
            if (eligibleBoundaries.size() != 1U)
            {
                return failure<ProcessPlan>
                (
                    OperationStatus::Failed, context,
                    DiagnosticCode::ProcessPlanningBoundaryClassificationFailed,
                    eligibleBoundaries.empty()
                        ? QStringLiteral("多断面空间关系形成循环，无法生成加工计划。")
                        : QStringLiteral("多个加工断面缺少唯一左右顺序，无法生成加工计划。"),
                    QStringLiteral("Boundary spatial relation graph is cyclic or not uniquely ordered."),
                    diagnosticValues(input, policy, 0U, 0U, -1, -1, -1, -1,
                        static_cast<int>(boundaries.size()),
                        static_cast<int>(eligibleBoundaries.size()))
                );
            }
            const int selectedBoundary = eligibleBoundaries.front();
            boundaryScheduled[static_cast<std::size_t>(selectedBoundary)] = true;
            boundaryOrder.push_back(selectedBoundary);
            for (const int successor : boundarySuccessors[static_cast<std::size_t>(selectedBoundary)])
                --boundaryIndegree[static_cast<std::size_t>(successor)];
        }

        std::unordered_map<int, int> boundaryRankByGroup;
        for (std::size_t rank = 0; rank < boundaryOrder.size(); ++rank)
        {
            const std::size_t boundaryIndex = static_cast<std::size_t>(boundaryOrder[rank]);
            boundaryRankByGroup[boundaries[boundaryIndex].groupId] = static_cast<int>(rank);
        }

        std::unordered_map<int, std::vector<BoundarySide>> groupBoundarySides;
        for (const ProcessGroup& group : plan.groups)
        {
            if (group.kind == ProcessGroupKind::BreakBoundary
                || group.kind == ProcessGroupKind::WasteBoundary) continue;
            std::vector<BoundarySide> sides(boundaries.size(), BoundarySide::Indeterminate);
            for (std::size_t boundaryIndex = 0; boundaryIndex < boundaries.size(); ++boundaryIndex)
            {
                const BoundaryData& boundary = boundaries[boundaryIndex];
                const BoundarySide side = classifyGroup
                (
                    group, entities, boundary, *planningSection,
                    policy.connectionTolerance
                );
                if (side == BoundarySide::Mixed || side == BoundarySide::Indeterminate
                    || side == BoundarySide::OnBoundary)
                {
                    return failure<ProcessPlan>
                    (
                        OperationStatus::Failed, context,
                        DiagnosticCode::ProcessPlanningBoundaryClassificationFailed,
                        side == BoundarySide::Mixed
                            ? QStringLiteral("加工组跨越加工断面，无法建立安全加工计划。")
                            : side == BoundarySide::OnBoundary
                                ? QStringLiteral("普通加工组落在加工断面上，无法建立安全加工计划。")
                                : QStringLiteral("加工组相对加工断面的侧别无法确定。"),
                        QStringLiteral("Ordinary group classification is not strictly Left or Right."),
                        boundaryDiagnosticValues
                        (
                            input, policy, boundary, group, entities,
                            boundaryRankByGroup[boundary.groupId], side
                        )
                    );
                }
                sides[boundaryIndex] = side;
            }
            bool enteredLeftSide = false;
            for (const int orderedIndex : boundaryOrder)
            {
                const BoundarySide side = sides[static_cast<std::size_t>(orderedIndex)];
                if (side == BoundarySide::Left) enteredLeftSide = true;
                else if (enteredLeftSide)
                {
                    const BoundaryData& boundary = boundaries
                        [static_cast<std::size_t>(orderedIndex)];
                    return failure<ProcessPlan>
                    (
                        OperationStatus::Failed, context,
                        DiagnosticCode::ProcessPlanningBoundaryClassificationFailed,
                        QStringLiteral("加工组相对多个断面的侧别不满足连续空间分区。"),
                        QStringLiteral("Boundary side pattern is not monotonic Right* then Left*."),
                        boundaryDiagnosticValues
                        (
                            input, policy, boundary, group, entities,
                            boundaryRankByGroup[boundary.groupId], side
                        )
                    );
                }
            }
            groupBoundarySides.emplace(group.groupId, std::move(sides));
        }

        // Waste intervals reuse the same strict spatial order as production barriers.
        if (boundaries.size() >= 2U)
        {
            for (const ProcessGroup& group : plan.groups)
            {
                if (group.kind == ProcessGroupKind::BreakBoundary
                    || group.kind == ProcessGroupKind::WasteBoundary) continue;
                const auto sides = groupBoundarySides.find(group.groupId);
                if (sides == groupBoundarySides.end()) continue;
                for (std::size_t rank = 0; rank + 1U < boundaryOrder.size(); ++rank)
                {
                    const std::size_t leftIndex = static_cast<std::size_t>(boundaryOrder[rank]);
                    const std::size_t rightIndex = static_cast<std::size_t>(boundaryOrder[rank + 1U]);
                    const BoundaryData& leftBoundary = boundaries[leftIndex];
                    const BoundaryData& rightBoundary = boundaries[rightIndex];
                    if (leftBoundary.role != BoundaryRole::Waste
                        && rightBoundary.role != BoundaryRole::Waste) continue;
                    if (sides->second[leftIndex] == BoundarySide::Right
                        && sides->second[rightIndex] == BoundarySide::Left)
                    {
                        excludedGroups.insert(group.groupId);
                        for (const EntityId entityId : group.entityIds)
                            plan.exclusions.push_back
                                ({ entityId, ProcessExclusionReason::WasteRegion });
                        break;
                    }
                }
            }
        }

        std::set<std::pair<int, int>> precedencePairs;
        for (std::size_t boundaryIndex = 0; boundaryIndex < boundaries.size(); ++boundaryIndex)
        {
            const BoundaryData& boundary = boundaries[boundaryIndex];
            if (boundary.role != BoundaryRole::Break) continue;
            for (const ProcessGroup& group : plan.groups)
            {
                if (group.groupId == boundary.groupId || excludedGroups.find(group.groupId) != excludedGroups.end()) continue;
                BoundarySide side = BoundarySide::Indeterminate;
                if (group.kind == ProcessGroupKind::BreakBoundary
                    || group.kind == ProcessGroupKind::WasteBoundary)
                {
                    side = boundaryRankByGroup.at(group.groupId)
                        < boundaryRankByGroup.at(boundary.groupId)
                        ? BoundarySide::Left : BoundarySide::Right;
                }
                else
                {
                    const auto sides = groupBoundarySides.find(group.groupId);
                    if (sides != groupBoundarySides.end()) side = sides->second[boundaryIndex];
                }
                const int predecessor = side == BoundarySide::Left
                    ? group.groupId : boundary.groupId;
                const int successor = side == BoundarySide::Left
                    ? boundary.groupId : group.groupId;
                if (precedencePairs.insert({ predecessor, successor }).second)
                    plan.precedenceConstraints.push_back
                        ({ predecessor, successor, boundary.pairId });
            }
        }

        std::unordered_map<int, int> indegree;
        std::unordered_map<int, std::vector<int>> successors;
        std::unordered_set<int> schedulable;
        for (const ProcessGroup& group : plan.groups)
        {
            if (excludedGroups.find(group.groupId) == excludedGroups.end())
            {
                schedulable.insert(group.groupId);
                indegree[group.groupId] = 0;
            }
        }
        for (const ProcessPrecedence& precedence : plan.precedenceConstraints)
        {
            ++indegree[precedence.successorGroupId];
            successors[precedence.predecessorGroupId].push_back(precedence.successorGroupId);
        }
        if (schedulable.empty())
        {
            return failure<ProcessPlan>
            (
                OperationStatus::Failed, context, DiagnosticCode::ProcessPlanningNoProcessableEntities,
                QStringLiteral("当前文档没有可加工图元。"), QStringLiteral("Every entity is excluded."),
                diagnosticValues(input, policy, 0U, 0U, -1, -1, -1, -1, 0, 0,
                    static_cast<int>(plan.groups.size()), 0, static_cast<int>(plan.exclusions.size()))
            );
        }

        std::unordered_set<int> scheduled;
        QVector<Diagnostic> closedLoopDiagnostics;
        Vector3d currentPosition = policy.initialPosition;
        int processOrder = 0;
        while (scheduled.size() < schedulable.size())
        {
            std::vector<int> eligible;
            for (const int groupId : schedulable)
                if (scheduled.find(groupId) == scheduled.end() && indegree[groupId] == 0) eligible.push_back(groupId);
            std::sort(eligible.begin(), eligible.end());
            if (eligible.empty())
            {
                return failure<ProcessPlan>
                (
                    OperationStatus::Failed, context, DiagnosticCode::ProcessPlanningPrecedenceCycle,
                    QStringLiteral("中断切面前置约束形成循环，无法生成加工计划。"), QStringLiteral("No eligible group remains while unscheduled groups exist."),
                    diagnosticValues(input, policy, 0U, 0U, -1, -1, -1, -1,
                        static_cast<int>(schedulable.size() - scheduled.size()), 0,
                        static_cast<int>(plan.groups.size()), static_cast<int>(plan.assignments.size()),
                        static_cast<int>(plan.exclusions.size()), -1, -1, scheduled.empty())
                );
            }

            std::optional<GroupTraversal> selected;
            std::optional<ClosedLoopTraversalReport> selectedClosedLoopReport;
            const bool initialSelection = scheduled.empty();
            const ProcessOrderingStrategy selectionStrategy = initialSelection
                ? ProcessOrderingStrategy::NearestNext
                : policy.orderingStrategy;
            for (const int groupId : eligible)
            {
                const ProcessGroup& group = plan.groups[static_cast<std::size_t>(groupId)];
                ClosedLoopTraversalReport candidateClosedLoopReport;
                auto candidate = bestTraversal
                    (group, entities, currentPosition, policy, input.tubeSection,
                        input.tubeSectionCenter, selectionStrategy, &candidateClosedLoopReport);
                if (!candidate.has_value())
                {
                    QVariantMap values = diagnosticValues
                    (
                        input, policy, 0U, 0U, -1, groupId, -1, -1,
                        static_cast<int>(schedulable.size() - scheduled.size()),
                        static_cast<int>(eligible.size()), static_cast<int>(plan.groups.size()),
                        static_cast<int>(plan.assignments.size()),
                        static_cast<int>(plan.exclusions.size()), -1, -1, initialSelection,
                        selected.has_value() ? &*selected : nullptr,
                        selected.has_value()
                            ? plan.groups[static_cast<std::size_t>(selected->groupId)].kind
                            : group.kind
                    );
                    if (candidateClosedLoopReport.groupId >= 0)
                    {
                        const QVariantMap closedLoopValues =
                            closedLoopDiagnosticValues(candidateClosedLoopReport);
                        for (auto iterator = closedLoopValues.cbegin();
                            iterator != closedLoopValues.cend(); ++iterator)
                            values.insert(iterator.key(), iterator.value());
                    }
                    return failure<ProcessPlan>
                    (
                        OperationStatus::Failed, context,
                        candidateClosedLoopReport.groupId >= 0
                            && !candidateClosedLoopReport.simpleLoopValid
                            ? DiagnosticCode::ProcessPlanningGroupBuildFailed
                            : DiagnosticCode::ProcessPlanningDirectionFailed,
                        candidateClosedLoopReport.groupId >= 0
                            && !candidateClosedLoopReport.simpleLoopValid
                            ? QStringLiteral("多图元闭合加工单元不是唯一简单环，无法生成加工计划。")
                            : QStringLiteral("连续加工组无法建立有效入口和加工方向。"),
                        candidateClosedLoopReport.groupId >= 0
                            ? candidateClosedLoopReport.failureReason
                            : QStringLiteral("No connected traversal covers every group entity."),
                        values
                    );
                }
                if (!selected.has_value() || traversalLess(*candidate, *selected, selectionStrategy))
                {
                    selected = std::move(candidate);
                    selectedClosedLoopReport = candidateClosedLoopReport.groupId >= 0
                        ? std::optional<ClosedLoopTraversalReport>
                            (std::move(candidateClosedLoopReport))
                        : std::nullopt;
                }
            }
            if (!selected.has_value())
            {
                return failure<ProcessPlan>
                (
                    OperationStatus::Failed, context, DiagnosticCode::ProcessPlanningOrderingFailed,
                    QStringLiteral("无法从可调度加工组中选择下一组。"), QStringLiteral("eligibleGroups produced no candidate."),
                    diagnosticValues(input, policy, 0U, 0U, -1, -1, -1, -1,
                        static_cast<int>(schedulable.size() - scheduled.size()),
                        static_cast<int>(eligible.size()), static_cast<int>(plan.groups.size()),
                        static_cast<int>(plan.assignments.size()), static_cast<int>(plan.exclusions.size()),
                        -1, -1, initialSelection)
                );
            }

            const ProcessGroup& selectedGroup = plan.groups[static_cast<std::size_t>(selected->groupId)];
            const bool continuous = selectedGroup.kind == ProcessGroupKind::ConnectedChain
                || selectedGroup.kind == ProcessGroupKind::ClosedLoop
                || selectedGroup.kind == ProcessGroupKind::BreakBoundary;
            const int continuousGroupId = continuous ? selectedGroup.groupId : -1;
            ProcessUnit processUnit;
            processUnit.key.memberEntityIds = selectedGroup.entityIds;
            std::sort(processUnit.key.memberEntityIds.begin(), processUnit.key.memberEntityIds.end());
            processUnit.closed = selectedGroup.closed;
            processUnit.orderedMemberEntityIds.reserve(selected->entities.size());
            for (const DirectedEntity& directed : selected->entities)
                processUnit.orderedMemberEntityIds.push_back(directed.entity->entityId);
            const int processUnitIndex = static_cast<int>(plan.processUnits.size());
            plan.processUnits.push_back(processUnit);
            plan.processUnitSequence.units.push_back(processUnit.key);
            for (const DirectedEntity& directed : selected->entities)
            {
                ProcessAssignment assignment;
                assignment.entityId = directed.entity->entityId;
                assignment.processOrder = processOrder++;
                assignment.processUnitIndex = processUnitIndex;
                assignment.continuousGroupId = continuousGroupId;
                assignment.reverse = directed.reverseRelativeToInput;
                assignment.startParameter = directed.selectedStartParameter;
                plan.assignments.push_back(assignment);
            }
            currentPosition = selected->end;
            if (selectedClosedLoopReport.has_value())
            {
                closedLoopDiagnostics.push_back(planningDiagnostic
                (
                    context,
                    DiagnosticCode::ProcessPlanningClosedLoopSummary,
                    QStringLiteral("多图元闭合加工单元已建立确定遍历。"),
                    QStringLiteral("Closed-loop traversal selected from complete loop candidates."),
                    closedLoopDiagnosticValues(*selectedClosedLoopReport),
                    DiagnosticSeverity::Info
                ));
            }
            scheduled.insert(selected->groupId);
            for (const int successor : successors[selected->groupId]) --indegree[successor];
        }

        std::unordered_set<EntityId> assignedIds;
        std::unordered_set<EntityId> excludedIds;
        std::unordered_map<EntityId, int> groupByEntity;
        std::unordered_map<int, int> firstOrderByGroup;
        std::unordered_map<int, int> lastOrderByGroup;
        for (const ProcessGroup& group : plan.groups)
        {
            for (const EntityId entityId : group.entityIds)
            {
                if (!groupByEntity.emplace(entityId, group.groupId).second)
                {
                    return failure<ProcessPlan>
                    (
                        OperationStatus::InternalError, context, DiagnosticCode::ProcessPlanningInvariantViolation,
                        QStringLiteral("加工图元被重复分配到多个加工组。"),
                        QStringLiteral("EntityId belongs to more than one ProcessGroup."),
                        diagnosticValues(input, policy, entityId, 0U, -1, group.groupId)
                    );
                }
            }
        }
        for (std::size_t index = 0; index < plan.assignments.size(); ++index)
        {
            const ProcessAssignment& assignment = plan.assignments[index];
            if (!assignedIds.insert(assignment.entityId).second
                || assignment.processOrder != static_cast<int>(index)
                || assignment.processUnitIndex < 0
                || static_cast<std::size_t>(assignment.processUnitIndex) >= plan.processUnits.size()
                || groupByEntity.find(assignment.entityId) == groupByEntity.end())
            {
                return failure<ProcessPlan>
                (
                    OperationStatus::InternalError, context, DiagnosticCode::ProcessPlanningInvariantViolation,
                    QStringLiteral("加工计划完整性校验失败。"), QStringLiteral("Duplicate assignment, non-contiguous order, or missing group."),
                    diagnosticValues(input, policy, assignment.entityId, 0U, -1,
                        groupByEntity.find(assignment.entityId) != groupByEntity.end() ? groupByEntity[assignment.entityId] : -1,
                        -1, -1, 0, 0, static_cast<int>(plan.groups.size()),
                        static_cast<int>(plan.assignments.size()), static_cast<int>(plan.exclusions.size()),
                        assignment.processOrder, assignment.continuousGroupId)
                );
            }
            const int groupId = groupByEntity[assignment.entityId];
            if (firstOrderByGroup.find(groupId) == firstOrderByGroup.end()) firstOrderByGroup[groupId] = assignment.processOrder;
            lastOrderByGroup[groupId] = assignment.processOrder;
        }
        if (!validateProcessUnitStructure(plan))
        {
            return failure<ProcessPlan>
            (
                OperationStatus::InternalError, context, DiagnosticCode::ProcessPlanningInvariantViolation,
                QStringLiteral("加工单元成员、顺序或分配关系校验失败。"),
                QStringLiteral("ProcessUnit structure is inconsistent with assignments or sequence."),
                diagnosticValues(input, policy)
            );
        }
        ClosedLoopValidationFailure closedLoopFailure;
        if (!validateMultiEntityClosedLoopUnits
            (plan, entities, policy.connectionTolerance, closedLoopFailure))
        {
            QVariantMap values = diagnosticValues
                (input, policy, closedLoopFailure.currentEntityId, 0U, -1,
                    closedLoopFailure.groupId);
            values.insert(QStringLiteral("previousEntityId"),
                QVariant::fromValue<qulonglong>(closedLoopFailure.previousEntityId));
            values.insert(QStringLiteral("joinGap"), closedLoopFailure.joinGap);
            values.insert(QStringLiteral("connectionTolerance"), policy.connectionTolerance);
            return failure<ProcessPlan>
            (
                OperationStatus::InternalError, context,
                DiagnosticCode::ProcessPlanningInvariantViolation,
                QStringLiteral("多图元闭合加工单元的最终遍历不连续。"),
                closedLoopFailure.reason, values
            );
        }
        for (const ProcessExclusion& exclusion : plan.exclusions)
        {
            if (!excludedIds.insert(exclusion.entityId).second || assignedIds.find(exclusion.entityId) != assignedIds.end())
            {
                return failure<ProcessPlan>
                (
                    OperationStatus::InternalError, context, DiagnosticCode::ProcessPlanningInvariantViolation,
                    QStringLiteral("加工计划排除项校验失败。"), QStringLiteral("Exclusions overlap assignments or contain duplicates."),
                    diagnosticValues(input, policy, exclusion.entityId)
                );
            }
        }
        for (const ProcessPrecedence& precedence : plan.precedenceConstraints)
        {
            if (lastOrderByGroup.find(precedence.predecessorGroupId) == lastOrderByGroup.end()
                || firstOrderByGroup.find(precedence.successorGroupId) == firstOrderByGroup.end()
                || lastOrderByGroup[precedence.predecessorGroupId] >= firstOrderByGroup[precedence.successorGroupId])
            {
                return failure<ProcessPlan>
                (
                    OperationStatus::InternalError, context, DiagnosticCode::ProcessPlanningInvariantViolation,
                    QStringLiteral("加工计划违反加工断面屏障约束。"), QStringLiteral("Break boundary precedence was not satisfied."),
                    diagnosticValues(input, policy, 0U, 0U, precedence.boundaryPairId, -1,
                        precedence.predecessorGroupId, precedence.successorGroupId)
                );
            }
        }
        for (std::size_t rank = 0; rank < boundaryOrder.size(); ++rank)
        {
            const std::size_t boundaryIndex = static_cast<std::size_t>(boundaryOrder[rank]);
            const BoundaryData& boundary = boundaries[boundaryIndex];
            if (boundary.role != BoundaryRole::Break) continue;
            const ProcessGroup& boundaryGroup = plan.groups
                [static_cast<std::size_t>(boundary.groupId)];
            const auto boundaryFirst = firstOrderByGroup.find(boundary.groupId);
            const auto boundaryLast = lastOrderByGroup.find(boundary.groupId);
            if (boundaryFirst == firstOrderByGroup.end()
                || boundaryLast == lastOrderByGroup.end()
                || boundaryLast->second - boundaryFirst->second + 1
                    != static_cast<int>(boundaryGroup.entityIds.size()))
            {
                return failure<ProcessPlan>
                (
                    OperationStatus::InternalError, context,
                    DiagnosticCode::ProcessPlanningInvariantViolation,
                    QStringLiteral("加工断面组在最终计划中不连续。"),
                    QStringLiteral("Break boundary assignments are missing or not contiguous."),
                    boundaryDiagnosticValues
                    (
                        input, policy, boundary, boundaryGroup, entities,
                        static_cast<int>(rank), BoundarySide::OnBoundary
                    )
                );
            }

            int maximumLeftLastOrder = -1;
            for (const ProcessGroup& otherGroup : plan.groups)
            {
                if (otherGroup.groupId == boundary.groupId
                    || excludedGroups.find(otherGroup.groupId) != excludedGroups.end()) continue;
                BoundarySide side = BoundarySide::Indeterminate;
                if (otherGroup.kind == ProcessGroupKind::BreakBoundary
                    || otherGroup.kind == ProcessGroupKind::WasteBoundary)
                {
                    side = boundaryRankByGroup.at(otherGroup.groupId)
                        < static_cast<int>(rank)
                        ? BoundarySide::Left : BoundarySide::Right;
                }
                else
                {
                    side = groupBoundarySides.at(otherGroup.groupId)[boundaryIndex];
                }
                const auto otherFirst = firstOrderByGroup.find(otherGroup.groupId);
                const auto otherLast = lastOrderByGroup.find(otherGroup.groupId);
                const bool validSideOrder = otherFirst != firstOrderByGroup.end()
                    && otherLast != lastOrderByGroup.end()
                    && (side == BoundarySide::Left
                        ? otherLast->second < boundaryFirst->second
                        : side == BoundarySide::Right
                            && otherFirst->second > boundaryLast->second);
                if (!validSideOrder)
                {
                    return failure<ProcessPlan>
                    (
                        OperationStatus::InternalError, context,
                        DiagnosticCode::ProcessPlanningInvariantViolation,
                        QStringLiteral("最终加工计划违反断面左右工艺屏障。"),
                        QStringLiteral("A group was scheduled on the wrong side of a Break boundary."),
                        boundaryDiagnosticValues
                        (
                            input, policy, boundary, otherGroup, entities,
                            static_cast<int>(rank), side
                        )
                    );
                }
                if (side == BoundarySide::Left)
                    maximumLeftLastOrder = std::max
                        (maximumLeftLastOrder, otherLast->second);
            }
            if (maximumLeftLastOrder + 1 != boundaryFirst->second)
            {
                return failure<ProcessPlan>
                (
                    OperationStatus::InternalError, context,
                    DiagnosticCode::ProcessPlanningInvariantViolation,
                    QStringLiteral("左侧加工组完成后未立即加工对应断面。"),
                    QStringLiteral("Break boundary does not immediately follow its final left-side group."),
                    boundaryDiagnosticValues
                    (
                        input, policy, boundary, boundaryGroup, entities,
                        static_cast<int>(rank), BoundarySide::OnBoundary
                    )
                );
            }
        }
        if (assignedIds.size() + excludedIds.size() != seen.size())
        {
            return failure<ProcessPlan>
            (
                OperationStatus::InternalError, context, DiagnosticCode::ProcessPlanningInvariantViolation,
                QStringLiteral("加工计划未完整覆盖文档图元。"),
                QStringLiteral("Assignments and exclusions do not cover every PlanningEntity exactly once."),
                diagnosticValues(input, policy, 0U, 0U, -1, -1, -1, -1, 0, 0,
                    static_cast<int>(plan.groups.size()), static_cast<int>(plan.assignments.size()),
                    static_cast<int>(plan.exclusions.size()))
            );
        }

        plan.assignments.shrink_to_fit();
        OperationResult<ProcessPlan> result;
        result.status = OperationStatus::Success;
        result.value = std::move(plan);
        result.mergeDiagnostics(closedLoopDiagnostics);
        return result;
    }
}
