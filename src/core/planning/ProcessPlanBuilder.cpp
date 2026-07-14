#include "core/planning/ProcessPlanBuilder.h"

#include "core/machining/TubeCutBoundary.h"

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

        struct DirectedEntity
        {
            const PlanningEntity* entity = nullptr;
            bool reverseRelativeToInput = false;
            Vector3d start;
            Vector3d end;
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
            std::size_t stableSourceIndex = 0;
            EntityId stableEntityId = 0;
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
            int continuousGroupId = -1
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
            for (std::size_t index = 0; index < section.boundary.size(); ++index)
            {
                const Vector2d& start = section.boundary[index];
                const Vector2d& end = section.boundary[(index + 1U) % section.boundary.size()];
                const double dy = end.x - start.x;
                const double dz = end.y - start.y;
                const double lengthSquared = dy * dy + dz * dz;
                if (lengthSquared <= kCalculationEpsilon) continue;
                const double factor = std::clamp
                (
                    ((point.x - start.x) * dy + (point.y - start.y) * dz) / lengthSquared,
                    0.0,
                    1.0
                );
                const double projectedY = start.x + dy * factor;
                const double projectedZ = start.y + dz * factor;
                const double candidateDistance = std::hypot(point.x - projectedY, point.y - projectedZ);
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
                    const double firstS = first.perimeterPosition + shift;
                    const double secondS = second.perimeterPosition + shift;
                    const double edgeX = second.x - first.x;
                    const double edgeS = secondS - firstS;
                    const double lengthSquared = edgeX * edgeX + edgeS * edgeS;
                    if (lengthSquared > kCalculationEpsilon)
                    {
                        const double factor = std::clamp
                        (
                            ((point.x - first.x) * edgeX + (queryPosition - firstS) * edgeS) / lengthSquared,
                            0.0,
                            1.0
                        );
                        const double nearestX = first.x + edgeX * factor;
                        const double nearestS = firstS + edgeS * factor;
                        onBoundary = onBoundary
                            || std::hypot(point.x - nearestX, queryPosition - nearestS) <= safeTolerance;
                    }
                    const bool crosses = (firstS <= queryPosition && queryPosition < secondS)
                        || (secondS <= queryPosition && queryPosition < firstS);
                    if (!crosses || std::abs(edgeS) <= kCalculationEpsilon) continue;
                    const double factor = (queryPosition - firstS) / edgeS;
                    const double intersectionX = first.x + edgeX * factor;
                    intersections.push_back(intersectionX);
                    onBoundary = onBoundary || std::abs(intersectionX - point.x) <= safeTolerance;
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
                [&point, safeTolerance](double x) { return x < point.x - safeTolerance; }
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
                            (first.x + second.x) * 0.5,
                            (first.y + second.y) * 0.5,
                            (first.z + second.z) * 0.5
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
                        if (reverse && !allowReverse) continue;
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
            }
            if (std::abs(left.movementDistance - right.movementDistance) > kCalculationEpsilon)
                return left.movementDistance < right.movementDistance;
            if (left.stableSourceIndex != right.stableSourceIndex)
                return left.stableSourceIndex < right.stableSourceIndex;
            return left.stableEntityId < right.stableEntityId;
        }

        std::optional<GroupTraversal> bestTraversal
        (
            const ProcessGroup& group,
            const std::unordered_map<EntityId, const PlanningEntity*>& entities,
            const Vector3d& currentPosition,
            const ProcessPlanningPolicy& policy,
            const std::optional<machining::TubeSectionModel>& section
        )
        {
            std::optional<GroupTraversal> best;
            for (const EntityId entityId : group.entityIds)
            {
                for (const bool reverse : { false, true })
                {
                    if (reverse && !policy.allowReverse) continue;
                    auto candidate = buildTraversal
                    (
                        group, entities, currentPosition, policy.allowReverse,
                        policy.connectionTolerance, std::make_pair(entityId, reverse)
                    );
                    if (!candidate.has_value()) continue;
                    scoreTraversal(*candidate, currentPosition, section);
                    if (!best.has_value() || traversalLess(*candidate, *best, policy.orderingStrategy))
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

        ProcessPlan plan;
        plan.contentRevision = input.contentRevision;
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

        std::vector<BoundaryData> boundaries;
        for (const auto& [key, ids] : boundaryIds)
        {
            ProcessGroup group;
            group.groupId = static_cast<int>(plan.groups.size());
            group.kind = key.second == BoundaryRole::Break
                ? ProcessGroupKind::BreakBoundary
                : ProcessGroupKind::WasteBoundary;
            group.closed = true;
            group.entityIds = ids;
            std::sort(group.entityIds.begin(), group.entityIds.end(), [&entities](EntityId left, EntityId right)
            {
                return entities.at(left)->sourceIndex < entities.at(right)->sourceIndex;
            });
            plan.groups.push_back(group);
            if (key.second == BoundaryRole::Waste)
            {
                for (const EntityId entityId : ids)
                    plan.exclusions.push_back({ entityId, ProcessExclusionReason::WasteRegion });
            }

            if (!input.tubeSection.has_value())
            {
                return failure<ProcessPlan>
                (
                    OperationStatus::InvalidInput, context, DiagnosticCode::ProcessPlanningBoundaryInvalid,
                    QStringLiteral("验证加工断面前需要先识别方管截面。"), QStringLiteral("Boundary validation requires TubeSectionModel."),
                    diagnosticValues(input, policy, 0U, 0U, key.first, group.groupId)
                );
            }
            const auto loop = input.topology->extractBestLoop(ids, ids);
            if (!loop.succeeded() || !loop.value.has_value() || !loop.value->connectedLoop
                || !sameEntitySet(loop.value->usedEntityIds, ids))
            {
                auto result = failure<ProcessPlan>
                (
                    OperationStatus::Failed, context, DiagnosticCode::ProcessPlanningBoundaryInvalid,
                    QStringLiteral("加工断面无法形成包含全部指定图元的严格闭环。"), QStringLiteral("Boundary strict-loop extraction failed."),
                    diagnosticValues(input, policy, 0U, 0U, key.first, group.groupId)
                );
                result.mergeDiagnostics(loop.diagnostics);
                return result;
            }
            const auto analysis = TubeCutBoundaryClassifier::analyze
            (
                loop.value->orderedPath,
                loop.value->usedEntityIds,
                loop.value->maximumJoinGap,
                input.tubeSection->geometry,
                context
            );
            if (!analysis.succeeded() || !analysis.value.has_value()
                || analysis.value->result == TubeCutResult::Indeterminate)
            {
                auto result = failure<ProcessPlan>
                (
                    OperationStatus::Failed, context, DiagnosticCode::ProcessPlanningBoundaryInvalid,
                    QStringLiteral("加工断面判定不确定，无法建立安全加工计划。"), QStringLiteral("TubeCutBoundaryClassifier returned Indeterminate."),
                    diagnosticValues(input, policy, 0U, 0U, key.first, group.groupId)
                );
                result.mergeDiagnostics(analysis.diagnostics);
                return result;
            }
            if (analysis.value->result == TubeCutResult::KeepsLeftAndRight)
            {
                return failure<ProcessPlan>
                (
                    OperationStatus::Failed, context, DiagnosticCode::ProcessPlanningBoundaryKeepsConnected,
                    QStringLiteral("加工断面仍会保留左右材料桥，不能作为中断切面。"), QStringLiteral("Boundary winding is zero."),
                    diagnosticValues(input, policy, 0U, 0U, key.first, group.groupId)
                );
            }
            boundaries.push_back({ group.groupId, key.first, key.second, *analysis.value });
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

        // Waste intervals are recomputed from spatial boundary order, never from stale CadItem state.
        if (boundaries.size() >= 2U)
        {
            std::vector<int> boundaryOrder(boundaries.size());
            for (std::size_t index = 0; index < boundaries.size(); ++index) boundaryOrder[index] = static_cast<int>(index);
            std::stable_sort(boundaryOrder.begin(), boundaryOrder.end(), [&](int leftIndex, int rightIndex)
            {
                const ProcessGroup& left = plan.groups[boundaries[static_cast<std::size_t>(leftIndex)].groupId];
                const BoundarySide side = classifyGroup
                (
                    left, entities, boundaries[static_cast<std::size_t>(rightIndex)],
                    input.tubeSection->geometry, policy.connectionTolerance
                );
                if (side == BoundarySide::Left) return true;
                if (side == BoundarySide::Right) return false;
                return left.groupId < plan.groups[boundaries[static_cast<std::size_t>(rightIndex)].groupId].groupId;
            });
            for (const ProcessGroup& group : plan.groups)
            {
                if (group.kind == ProcessGroupKind::BreakBoundary || group.kind == ProcessGroupKind::WasteBoundary) continue;
                for (std::size_t index = 0; index + 1U < boundaryOrder.size(); ++index)
                {
                    const BoundaryData& leftBoundary = boundaries[static_cast<std::size_t>(boundaryOrder[index])];
                    const BoundaryData& rightBoundary = boundaries[static_cast<std::size_t>(boundaryOrder[index + 1U])];
                    if (leftBoundary.role != BoundaryRole::Waste && rightBoundary.role != BoundaryRole::Waste) continue;
                    const BoundarySide relativeLeft = classifyGroup
                        (group, entities, leftBoundary, input.tubeSection->geometry, policy.connectionTolerance);
                    const BoundarySide relativeRight = classifyGroup
                        (group, entities, rightBoundary, input.tubeSection->geometry, policy.connectionTolerance);
                    if (relativeLeft == BoundarySide::Right && relativeRight == BoundarySide::Left)
                    {
                        excludedGroups.insert(group.groupId);
                        for (const EntityId entityId : group.entityIds)
                            plan.exclusions.push_back({ entityId, ProcessExclusionReason::WasteRegion });
                        break;
                    }
                }
            }
        }

        std::set<std::pair<int, int>> precedencePairs;
        for (const BoundaryData& boundary : boundaries)
        {
            if (boundary.role != BoundaryRole::Break) continue;
            for (const ProcessGroup& group : plan.groups)
            {
                if (group.groupId == boundary.groupId || excludedGroups.find(group.groupId) != excludedGroups.end()) continue;
                const BoundarySide side = classifyGroup
                (
                    group, entities, boundary, input.tubeSection->geometry,
                    policy.connectionTolerance
                );
                if (side == BoundarySide::Mixed || side == BoundarySide::Indeterminate)
                {
                    return failure<ProcessPlan>
                    (
                        OperationStatus::Failed, context, DiagnosticCode::ProcessPlanningBoundaryClassificationFailed,
                        QStringLiteral("加工组跨越中断切面或侧别无法确定。"), QStringLiteral("Group classified as Mixed or Indeterminate."),
                        diagnosticValues(input, policy, 0U, 0U, boundary.pairId, group.groupId)
                    );
                }
                if (side == BoundarySide::Left && precedencePairs.insert({ group.groupId, boundary.groupId }).second)
                    plan.precedenceConstraints.push_back({ group.groupId, boundary.groupId, boundary.pairId });
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
                        static_cast<int>(plan.exclusions.size()))
                );
            }

            std::optional<GroupTraversal> selected;
            for (const int groupId : eligible)
            {
                const ProcessGroup& group = plan.groups[static_cast<std::size_t>(groupId)];
                auto candidate = bestTraversal(group, entities, currentPosition, policy, input.tubeSection);
                if (!candidate.has_value())
                {
                    return failure<ProcessPlan>
                    (
                        OperationStatus::Failed, context, DiagnosticCode::ProcessPlanningDirectionFailed,
                        QStringLiteral("连续加工组无法建立有效入口和加工方向。"), QStringLiteral("No connected traversal covers every group entity."),
                        diagnosticValues(input, policy, 0U, 0U, -1, groupId, -1, -1,
                            static_cast<int>(schedulable.size() - scheduled.size()), static_cast<int>(eligible.size()),
                            static_cast<int>(plan.groups.size()), static_cast<int>(plan.assignments.size()),
                            static_cast<int>(plan.exclusions.size()))
                    );
                }
                if (!selected.has_value() || traversalLess(*candidate, *selected, policy.orderingStrategy))
                    selected = std::move(candidate);
            }
            if (!selected.has_value())
            {
                return failure<ProcessPlan>
                (
                    OperationStatus::Failed, context, DiagnosticCode::ProcessPlanningOrderingFailed,
                    QStringLiteral("无法从可调度加工组中选择下一组。"), QStringLiteral("eligibleGroups produced no candidate."),
                    diagnosticValues(input, policy)
                );
            }

            const ProcessGroup& selectedGroup = plan.groups[static_cast<std::size_t>(selected->groupId)];
            const bool continuous = selectedGroup.kind == ProcessGroupKind::ConnectedChain
                || selectedGroup.kind == ProcessGroupKind::ClosedLoop
                || selectedGroup.kind == ProcessGroupKind::BreakBoundary;
            const int continuousGroupId = continuous ? selectedGroup.groupId : -1;
            for (const DirectedEntity& directed : selected->entities)
            {
                ProcessAssignment assignment;
                assignment.entityId = directed.entity->entityId;
                assignment.processOrder = processOrder++;
                assignment.continuousGroupId = continuousGroupId;
                assignment.reverse = directed.entity->currentReverse ^ directed.reverseRelativeToInput;
                assignment.startParameter = directed.entity->currentStartParameter;
                plan.assignments.push_back(assignment);
            }
            currentPosition = selected->end;
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
                    QStringLiteral("加工计划违反中断切面前置约束。"), QStringLiteral("Left -> Break precedence was not satisfied."),
                    diagnosticValues(input, policy, 0U, 0U, precedence.boundaryPairId, -1,
                        precedence.predecessorGroupId, precedence.successorGroupId)
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
        return result;
    }
}
