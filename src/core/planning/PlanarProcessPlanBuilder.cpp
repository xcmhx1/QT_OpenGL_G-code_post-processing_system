#include "core/planning/PlanarProcessPlanBuilder.h"

#include <algorithm>
#include <cmath>
#include <limits>
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
                { QStringLiteral("reverse"), entity != nullptr && entity->currentReversePreference },
                { QStringLiteral("startParameter"), entity != nullptr
                    ? entity->customStartParameter.value_or(0.0) : 0.0 },
                { QStringLiteral("candidateCount"), candidateCount },
                { QStringLiteral("assignmentCount"), assignmentCount },
                { QStringLiteral("excludedCount"), excludedCount }
            };
            if (entity != nullptr && entity->entityId != 0U) diagnostic.entityId = entity->entityId;
            return diagnostic;
        }

        std::optional<double> closedStartParameter(const PlanarPlanningEntity& entity)
        {
            if (entity.customStartParameter.has_value()) return entity.customStartParameter;
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
            candidate.reverse = entity.currentReversePreference;
            if (!entity.path.closed && policy.allowReverse)
            {
                const double forwardDistance = distance(current, candidate.entry);
                const double reverseDistance = distance(current, candidate.exit);
                bool chooseReverse = reverseDistance < forwardDistance - policy.numericalEpsilon;
                if (std::abs(reverseDistance - forwardDistance) <= policy.numericalEpsilon)
                    chooseReverse = policy.preserveUserDirection
                        ? entity.currentReversePreference : false;
                candidate.reverse = chooseReverse;
                if (chooseReverse) std::swap(candidate.entry, candidate.exit);
            }
            candidate.score =
                { distance(current, candidate.entry), entity.sourceIndex, entity.entityId };
            return candidate;
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

        geometry::Vector3d current = policy.hasInitialPosition
            ? policy.initialPosition : geometry::Vector3d{};
        while (!remaining.empty())
        {
            std::optional<Candidate> best;
            std::size_t bestIndex = 0U;
            for (std::size_t index = 0; index < remaining.size(); ++index)
            {
                Candidate candidate = candidateFor(*remaining[index], current, policy);
                if (!best.has_value() || candidateLess(candidate, *best, policy.numericalEpsilon))
                {
                    best = candidate;
                    bestIndex = index;
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

            ProcessAssignment assignment;
            assignment.entityId = best->entity->entityId;
            assignment.processOrder = static_cast<int>(plan.assignments.size());
            assignment.continuousGroupId = -1;
            assignment.reverse = best->reverse;
            if (best->entity->path.closed)
                assignment.startParameter = closedStartParameter(*best->entity);
            plan.assignments.push_back(assignment);
            plan.groups.push_back
            ({ assignment.processOrder, ProcessGroupKind::SingleEntity,
                best->entity->path.closed, { assignment.entityId } });
            current = best->exit;
            remaining.erase(remaining.begin() + static_cast<std::ptrdiff_t>(bestIndex));
        }

        result.status = OperationStatus::Success;
        result.value = std::move(plan);
        return result;
    }
}
