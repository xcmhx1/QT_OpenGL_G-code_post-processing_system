#include "core/planning/PlanarProcessPlanBuilder.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>

namespace cadcam::planning
{
    namespace
    {
        constexpr double kHalfPi = 1.57079632679489661923;

        bool processableKind(geometry::SourceGeometryKind kind)
        {
            using Kind = geometry::SourceGeometryKind;
            return kind == Kind::Line || kind == Kind::Arc || kind == Kind::Circle
                || kind == Kind::Ellipse || kind == Kind::Polyline || kind == Kind::Spline;
        }

        double distance(const geometry::Vector3d& left, const geometry::Vector3d& right)
        {
            return std::sqrt
            (
                (left.x - right.x) * (left.x - right.x)
                + (left.y - right.y) * (left.y - right.y)
                + (left.z - right.z) * (left.z - right.z)
            );
        }

        bool validPath(const geometry::Path3D& path, double epsilon)
        {
            if (path.vertices.size() < 2U) return false;
            for (std::size_t index = 1; index < path.vertices.size(); ++index)
            {
                if (distance(path.vertices[index - 1U].position,
                    path.vertices[index].position) > epsilon) return true;
            }
            return path.closed && distance(path.vertices.back().position,
                path.vertices.front().position) > epsilon;
        }

        QString modeName(ProcessPlanMode mode)
        {
            return mode == ProcessPlanMode::Planar3Axis
                ? QStringLiteral("Planar3Axis") : QStringLiteral("Rotary4Axis");
        }

        Diagnostic planningDiagnostic
        (
            DiagnosticCode code,
            DiagnosticSeverity severity,
            const QString& message,
            const OperationContext& context,
            std::uint64_t revision,
            const PlanarPlanningEntity* entity = nullptr,
            double entryDistance = 0.0,
            int candidateCount = 0,
            int assignmentCount = 0,
            int excludedCount = 0
        )
        {
            Diagnostic diagnostic;
            diagnostic.code = code;
            diagnostic.severity = severity;
            diagnostic.component = QStringLiteral("PlanarProcessPlanBuilder");
            diagnostic.operation = context.operationName;
            diagnostic.stage = QStringLiteral("build-planar-process-plan");
            diagnostic.userMessage = message;
            diagnostic.correlationId = context.correlationId;
            diagnostic.context =
            {
                { QStringLiteral("contentRevision"), QVariant::fromValue<qulonglong>(revision) },
                { QStringLiteral("planMode"), modeName(ProcessPlanMode::Planar3Axis) },
                { QStringLiteral("entityId"), QVariant::fromValue<qulonglong>
                    (entity != nullptr ? entity->entityId : 0U) },
                { QStringLiteral("sourceIndex"), QVariant::fromValue<qulonglong>
                    (entity != nullptr ? entity->sourceIndex : 0U) },
                { QStringLiteral("processOrder"), assignmentCount },
                { QStringLiteral("entryDistance"), entryDistance },
                { QStringLiteral("directionPreference"), entity != nullptr
                    ? static_cast<int>(entity->directionPreference) : 0 },
                { QStringLiteral("startParameter"), entity != nullptr
                    ? entity->startParameter.value_or(0.0) : 0.0 },
                { QStringLiteral("candidateCount"), candidateCount },
                { QStringLiteral("assignmentCount"), assignmentCount },
                { QStringLiteral("excludedCount"), excludedCount }
            };
            if (entity != nullptr && entity->entityId != 0U) diagnostic.entityId = entity->entityId;
            return diagnostic;
        }

        std::optional<double> closedStartParameter(const PlanarPlanningEntity& entity)
        {
            if (entity.startParameter.has_value()) return entity.startParameter;
            if (entity.sourceKind == geometry::SourceGeometryKind::Circle
                || entity.sourceKind == geometry::SourceGeometryKind::Ellipse)
            {
                return kHalfPi;
            }
            return std::nullopt;
        }

        geometry::Vector3d closedEntryPosition(const PlanarPlanningEntity& entity)
        {
            const std::optional<double> start = closedStartParameter(entity);
            if (!start.has_value() || entity.path.vertices.empty())
                return entity.path.vertices.front().position;

            const geometry::PathVertex3D* closest = &entity.path.vertices.front();
            double closestDifference = std::abs(closest->sourceParameter - *start);
            for (const geometry::PathVertex3D& vertex : entity.path.vertices)
            {
                const double difference = std::abs(vertex.sourceParameter - *start);
                if (difference < closestDifference)
                {
                    closest = &vertex;
                    closestDifference = difference;
                }
            }
            return closest->position;
        }

        struct PlanarCandidateScore
        {
            double entryDistance = 0.0;
            std::size_t sourceIndex = 0U;
            geometry::EntityId entityId = 0U;
        };

        struct Candidate
        {
            const PlanarPlanningEntity* entity = nullptr;
            bool reverse = false;
            geometry::Vector3d entry;
            geometry::Vector3d exit;
            PlanarCandidateScore score;
        };

        bool candidateLess(const Candidate& left, const Candidate& right, double epsilon)
        {
            if (std::abs(left.score.entryDistance - right.score.entryDistance) > epsilon)
                return left.score.entryDistance < right.score.entryDistance;
            if (left.score.sourceIndex != right.score.sourceIndex)
                return left.score.sourceIndex < right.score.sourceIndex;
            return left.score.entityId < right.score.entityId;
        }

        Candidate candidateFor
        (
            const PlanarPlanningEntity& entity,
            const geometry::Vector3d& current,
            const PlanarProcessPlanningPolicy& policy
        )
        {
            Candidate candidate;
            candidate.entity = &entity;
            candidate.entry = entity.path.closed
                ? closedEntryPosition(entity) : entity.path.vertices.front().position;
            candidate.exit = entity.path.closed
                ? candidate.entry : entity.path.vertices.back().position;
            candidate.reverse = entity.directionPreference == process::DirectionPreference::Reverse;
            if (!entity.path.closed
                && entity.directionPreference == process::DirectionPreference::Auto
                && policy.allowReverse)
            {
                const double forwardDistance = distance(current, candidate.entry);
                const double reverseDistance = distance(current, candidate.exit);
                bool chooseReverse = reverseDistance < forwardDistance - policy.numericalEpsilon;
                if (std::abs(reverseDistance - forwardDistance) <= policy.numericalEpsilon)
                    chooseReverse = false;
                candidate.reverse = chooseReverse;
                if (chooseReverse) std::swap(candidate.entry, candidate.exit);
            }
            candidate.score =
                { distance(current, candidate.entry), entity.sourceIndex, entity.entityId };
            return candidate;
        }

        std::vector<bool> allowedDirections
        (
            const PlanarPlanningEntity& entity,
            const PlanarProcessPlanningPolicy& policy
        )
        {
            if (entity.directionPreference == process::DirectionPreference::Forward)
                return { false };
            if (entity.directionPreference == process::DirectionPreference::Reverse)
                return { true };
            return policy.allowReverse ? std::vector<bool>{ false, true }
                : std::vector<bool>{ false };
        }

        Candidate directedCandidate
        (
            const PlanarPlanningEntity& entity,
            bool reverse,
            const geometry::Vector3d& current
        )
        {
            Candidate candidate;
            candidate.entity = &entity;
            candidate.reverse = reverse;
            candidate.entry = entity.path.closed
                ? closedEntryPosition(entity) : entity.path.vertices.front().position;
            candidate.exit = entity.path.closed
                ? candidate.entry : entity.path.vertices.back().position;
            if (reverse && !entity.path.closed) std::swap(candidate.entry, candidate.exit);
            candidate.score =
                { distance(current, candidate.entry), entity.sourceIndex, entity.entityId };
            return candidate;
        }

        bool directionAllowed
        (
            const PlanarPlanningEntity& entity,
            bool reverse,
            const PlanarProcessPlanningPolicy& policy
        )
        {
            const std::vector<bool> directions = allowedDirections(entity, policy);
            return std::find(directions.begin(), directions.end(), reverse) != directions.end();
        }

        struct UnitTraversal
        {
            std::vector<Candidate> entities;
            bool closed = false;
        };

        std::optional<UnitTraversal> tryBuildTraversal
        (
            const std::vector<const PlanarPlanningEntity*>& component,
            const Candidate& start,
            bool requireClosed,
            const PlanarProcessPlanningPolicy& policy
        )
        {
            if (component.empty() || start.entity == nullptr
                || !directionAllowed(*start.entity, start.reverse, policy))
            {
                return std::nullopt;
            }
            if (component.size() > 1U
                && std::any_of(component.begin(), component.end(), [](const auto* entity)
                    { return entity->path.closed; }))
            {
                return std::nullopt;
            }

            UnitTraversal traversal;
            traversal.entities.reserve(component.size());
            traversal.entities.push_back(start);
            std::set<geometry::EntityId> used{ start.entity->entityId };
            geometry::Vector3d current = start.exit;
            const double joinTolerance = requireClosed
                ? policy.numericalEpsilon : policy.connectionTolerance;

            while (traversal.entities.size() < component.size())
            {
                std::optional<Candidate> next;
                for (const PlanarPlanningEntity* entity : component)
                {
                    if (entity == nullptr || used.count(entity->entityId) != 0U) continue;
                    for (const bool reverse : allowedDirections(*entity, policy))
                    {
                        Candidate candidate = directedCandidate(*entity, reverse, current);
                        if (candidate.score.entryDistance > joinTolerance) continue;
                        if (!next.has_value()
                            || candidateLess(candidate, *next, policy.numericalEpsilon))
                        {
                            next = candidate;
                        }
                    }
                }
                if (!next.has_value()) return std::nullopt;
                used.insert(next->entity->entityId);
                current = next->exit;
                traversal.entities.push_back(*next);
            }

            traversal.closed = component.size() == 1U && start.entity->path.closed;
            if (!traversal.closed)
            {
                traversal.closed = distance
                    (traversal.entities.back().exit, traversal.entities.front().entry)
                    <= policy.numericalEpsilon;
            }
            if (requireClosed != traversal.closed) return std::nullopt;
            return traversal;
        }

        std::optional<UnitTraversal> buildComponentTraversal
        (
            const std::vector<const PlanarPlanningEntity*>& component,
            const Candidate& preferred,
            bool requireClosed,
            const geometry::Vector3d& current,
            const PlanarProcessPlanningPolicy& policy
        )
        {
            if (auto traversal = tryBuildTraversal
                (component, preferred, requireClosed, policy))
            {
                return traversal;
            }

            std::optional<UnitTraversal> bestTraversal;
            std::optional<Candidate> bestStart;
            for (const PlanarPlanningEntity* entity : component)
            {
                if (entity == nullptr) continue;
                for (const bool reverse : allowedDirections(*entity, policy))
                {
                    Candidate start = directedCandidate(*entity, reverse, current);
                    auto traversal = tryBuildTraversal
                        (component, start, requireClosed, policy);
                    if (!traversal.has_value()) continue;
                    if (!bestStart.has_value()
                        || candidateLess(start, *bestStart, policy.numericalEpsilon))
                    {
                        bestStart = start;
                        bestTraversal = std::move(traversal);
                    }
                }
            }
            return bestTraversal;
        }

        bool sameEntityIds
        (
            std::vector<geometry::EntityId> left,
            std::vector<geometry::EntityId> right
        )
        {
            std::sort(left.begin(), left.end());
            std::sort(right.begin(), right.end());
            return left == right;
        }
    }

    OperationResult<ProcessPlan> PlanarProcessPlanBuilder::build
    (
        const PlanarProcessPlanningInput& input,
        const PlanarProcessPlanningPolicy& policy,
        const OperationContext& context
    )
    {
        OperationResult<ProcessPlan> result;
        if (input.contentRevision == 0U || input.entities.empty()
            || !std::isfinite(policy.connectionTolerance) || policy.connectionTolerance <= 0.0
            || !std::isfinite(policy.numericalEpsilon) || policy.numericalEpsilon <= 0.0)
        {
            result.status = OperationStatus::InvalidInput;
            result.addDiagnostic(planningDiagnostic(DiagnosticCode::PlanarPlanningInputInvalid,
                DiagnosticSeverity::Error, QStringLiteral("三轴加工计划输入无效。"),
                context, input.contentRevision));
            return result;
        }

        ProcessPlan plan;
        plan.contentRevision = input.contentRevision;
        plan.processStateRevision = input.processStateRevision;
        plan.mode = ProcessPlanMode::Planar3Axis;
        plan.orderingStrategy = ProcessOrderingStrategy::NearestNext;
        std::set<geometry::EntityId> ids;
        std::vector<const PlanarPlanningEntity*> remaining;
        remaining.reserve(input.entities.size());
        for (const PlanarPlanningEntity& entity : input.entities)
        {
            if (entity.entityId == 0U || !ids.insert(entity.entityId).second)
            {
                result.status = OperationStatus::InvalidInput;
                result.addDiagnostic(planningDiagnostic(DiagnosticCode::PlanarPlanningInputInvalid,
                    DiagnosticSeverity::Error, QStringLiteral("三轴加工计划包含无效或重复图元编号。"),
                    context, input.contentRevision, &entity));
                return result;
            }

            std::optional<ProcessExclusionReason> exclusion;
            if (!entity.visible) exclusion = ProcessExclusionReason::Hidden;
            else if (!entity.processEnabled) exclusion = ProcessExclusionReason::UserDisabled;
            else if (entity.excludedAsInternalGeometry) exclusion = ProcessExclusionReason::InternalGeometry;
            else if (!processableKind(entity.sourceKind)) exclusion = ProcessExclusionReason::UnsupportedGeometry;
            else if (!validPath(entity.path, policy.numericalEpsilon)) exclusion = ProcessExclusionReason::InvalidPath;
            if (exclusion.has_value())
            {
                plan.exclusions.push_back({ entity.entityId, *exclusion });
                continue;
            }
            remaining.push_back(&entity);
        }

        if (remaining.empty())
        {
            result.status = OperationStatus::InvalidInput;
            result.addDiagnostic(planningDiagnostic(DiagnosticCode::PlanarPlanningNoProcessableEntities,
                DiagnosticSeverity::Error, QStringLiteral("文档中没有可用于三轴加工的有效路径。"),
                context, input.contentRevision, nullptr, 0.0, 0, 0,
                static_cast<int>(plan.exclusions.size())));
            return result;
        }

        topology::TopologyInput topologyInput;
        topologyInput.contentRevision = input.contentRevision;
        topologyInput.records.reserve(remaining.size());
        for (const PlanarPlanningEntity* entity : remaining)
        {
            topology::TopologyPathRecord record;
            record.sourceIndex = entity->sourceIndex;
            record.entityId = entity->entityId;
            record.sourceKind = entity->sourceKind;
            record.semanticallyClosed = entity->path.closed;
            record.points.reserve(entity->path.vertices.size());
            for (const geometry::PathVertex3D& vertex : entity->path.vertices)
                record.points.push_back(vertex.position);
            topologyInput.records.push_back(std::move(record));
        }
        topology::PathTopologyTolerance topologyTolerance =
            topology::PathTopologyTolerance::fromConnectionTolerance(policy.connectionTolerance);
        topologyTolerance.numericalJoinEpsilon = policy.numericalEpsilon;
        TaskContext taskContext;
        taskContext.operationContext = context;
        topology::PathTopologyBuilder topologyBuilder;
        auto topologyResult = topologyBuilder.build
            (topologyInput, topologyTolerance, taskContext);
        result.mergeDiagnostics(topologyResult);
        if (!topologyResult.succeeded() || !topologyResult.value.has_value())
        {
            result.status = topologyResult.status;
            return result;
        }
        const topology::PathTopology& pathTopology = *topologyResult.value;
        const std::vector<int> componentIds = pathTopology.componentIds();
        if (componentIds.size() != remaining.size())
        {
            result.status = OperationStatus::InternalError;
            result.addDiagnostic(planningDiagnostic
            (
                DiagnosticCode::ProcessPlanningInvariantViolation,
                DiagnosticSeverity::Error,
                QStringLiteral("三轴拓扑分量与加工图元数量不一致。"),
                context, input.contentRevision
            ));
            return result;
        }
        std::map<geometry::EntityId, int> componentByEntity;
        for (std::size_t index = 0; index < remaining.size(); ++index)
            componentByEntity.emplace(remaining[index]->entityId, componentIds[index]);

        geometry::Vector3d current = policy.hasInitialPosition
            ? policy.initialPosition : geometry::Vector3d{};
        while (!remaining.empty())
        {
            std::optional<Candidate> best;
            for (std::size_t index = 0; index < remaining.size(); ++index)
            {
                Candidate candidate = candidateFor(*remaining[index], current, policy);
                if (!best.has_value() || candidateLess(candidate, *best, policy.numericalEpsilon))
                {
                    best = candidate;
                }
            }
            if (!best.has_value())
            {
                result.status = OperationStatus::Failed;
                result.addDiagnostic(planningDiagnostic(DiagnosticCode::PlanarPlanningOrderingFailed,
                    DiagnosticSeverity::Error, QStringLiteral("三轴最近距离排序无法选出下一个图元。"),
                    context, input.contentRevision, nullptr, 0.0,
                    static_cast<int>(remaining.size()), static_cast<int>(plan.assignments.size()),
                    static_cast<int>(plan.exclusions.size())));
                return result;
            }

            std::vector<const PlanarPlanningEntity*> component;
            const int componentId = componentByEntity.at(best->entity->entityId);
            for (const PlanarPlanningEntity* entity : remaining)
            {
                if (componentByEntity.at(entity->entityId) == componentId)
                    component.push_back(entity);
            }

            std::vector<geometry::EntityId> componentEntityIds;
            componentEntityIds.reserve(component.size());
            for (const PlanarPlanningEntity* entity : component)
                componentEntityIds.push_back(entity->entityId);
            bool componentClosed = false;
            auto loopResult = pathTopology.extractBestLoop
                (componentEntityIds, componentEntityIds);
            if (loopResult.succeeded() && loopResult.value.has_value()
                && loopResult.value->connectedLoop
                && sameEntityIds(loopResult.value->usedEntityIds, componentEntityIds))
            {
                componentClosed = true;
            }

            std::optional<UnitTraversal> traversal = buildComponentTraversal
                (component, *best, componentClosed, current, policy);
            if (!traversal.has_value())
            {
                traversal = UnitTraversal{ { *best }, best->entity->path.closed };
            }

            const int firstProcessOrder = static_cast<int>(plan.assignments.size());
            const int processUnitIndex = static_cast<int>(plan.processUnits.size());
            ProcessUnit unit;
            unit.closed = traversal->closed;
            for (const Candidate& directed : traversal->entities)
            {
                unit.key.memberEntityIds.push_back(directed.entity->entityId);
                unit.orderedMemberEntityIds.push_back(directed.entity->entityId);

                ProcessAssignment assignment;
                assignment.entityId = directed.entity->entityId;
                assignment.processOrder = static_cast<int>(plan.assignments.size());
                assignment.processUnitIndex = processUnitIndex;
                assignment.continuousGroupId = -1;
                assignment.reverse = directed.reverse;
                if (directed.entity->path.closed)
                    assignment.startParameter = closedStartParameter(*directed.entity);
                plan.assignments.push_back(assignment);
            }
            std::sort(unit.key.memberEntityIds.begin(), unit.key.memberEntityIds.end());
            plan.processUnits.push_back(unit);
            plan.processUnitSequence.units.push_back(unit.key);
            const ProcessGroupKind groupKind = traversal->closed
                ? ProcessGroupKind::ClosedLoop
                : traversal->entities.size() == 1U
                    ? ProcessGroupKind::SingleEntity : ProcessGroupKind::ConnectedChain;
            plan.groups.push_back
                ({ firstProcessOrder, groupKind, traversal->closed,
                    unit.orderedMemberEntityIds });
            current = traversal->entities.back().exit;

            std::set<geometry::EntityId> scheduledIds;
            for (const Candidate& directed : traversal->entities)
                scheduledIds.insert(directed.entity->entityId);
            remaining.erase
            (
                std::remove_if
                (
                    remaining.begin(), remaining.end(),
                    [&scheduledIds](const PlanarPlanningEntity* entity)
                    { return scheduledIds.count(entity->entityId) != 0U; }
                ),
                remaining.end()
            );
        }

        if (!validateProcessUnitStructure(plan))
        {
            result.status = OperationStatus::InternalError;
            result.addDiagnostic(planningDiagnostic
            (
                DiagnosticCode::ProcessPlanningInvariantViolation,
                DiagnosticSeverity::Error,
                QStringLiteral("三轴加工单元完整性校验失败。"),
                context, input.contentRevision, nullptr, 0.0, 0,
                static_cast<int>(plan.assignments.size()),
                static_cast<int>(plan.exclusions.size())
            ));
            return result;
        }

        result.status = OperationStatus::Success;
        result.value = std::move(plan);
        return result;
    }
}
