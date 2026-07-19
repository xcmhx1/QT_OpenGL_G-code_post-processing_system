#include "application/planning/ProcessPlanningService.h"

#include "cad/document/CadDocument.h"
#include "application/planning/DocumentProcessPlanningAdapter.h"
#include "core/planning/PlanarProcessPlanBuilder.h"
#include "core/planning/ProcessPlanBuilder.h"

#include <limits>
#include <map>
#include <set>

namespace
{
    using cadcam::geometry::EntityId;
    using cadcam::planning::ProcessAssignment;
    using cadcam::planning::ProcessGroup;
    using cadcam::planning::ProcessPlan;
    using cadcam::planning::ProcessUnit;
    using cadcam::planning::ProcessUnitKey;
    using cadcam::planning::ProcessUnitSequence;

    bool satisfiesPrecedenceConstraints(const ProcessPlan& plan)
    {
        std::map<EntityId, int> orderByEntity;
        for (const ProcessAssignment& assignment : plan.assignments)
            orderByEntity.emplace(assignment.entityId, assignment.processOrder);

        std::map<int, const ProcessGroup*> groupsById;
        for (const ProcessGroup& group : plan.groups)
            groupsById.emplace(group.groupId, &group);

        for (const auto& precedence : plan.precedenceConstraints)
        {
            const auto predecessor = groupsById.find(precedence.predecessorGroupId);
            const auto successor = groupsById.find(precedence.successorGroupId);
            if (predecessor == groupsById.end() || successor == groupsById.end()) return false;

            int predecessorLast = -1;
            int successorFirst = std::numeric_limits<int>::max();
            for (const EntityId entityId : predecessor->second->entityIds)
            {
                const auto found = orderByEntity.find(entityId);
                if (found == orderByEntity.end()) return false;
                predecessorLast = std::max(predecessorLast, found->second);
            }
            for (const EntityId entityId : successor->second->entityIds)
            {
                const auto found = orderByEntity.find(entityId);
                if (found == orderByEntity.end()) return false;
                successorFirst = std::min(successorFirst, found->second);
            }
            if (predecessorLast >= successorFirst) return false;
        }
        return true;
    }

    bool reorderProcessUnits
    (
        ProcessPlan& plan,
        const std::vector<ProcessUnitKey>& orderedKeys
    )
    {
        if (!cadcam::planning::validateProcessUnitStructure(plan)
            || orderedKeys.size() != plan.processUnits.size()) return false;

        std::map<std::vector<EntityId>, std::size_t> automaticUnitIndices;
        for (std::size_t index = 0; index < plan.processUnits.size(); ++index)
            automaticUnitIndices.emplace
                (plan.processUnits[index].key.memberEntityIds, index);

        std::vector<std::size_t> orderedIndices;
        orderedIndices.reserve(plan.processUnits.size());
        std::set<std::size_t> usedIndices;
        for (const ProcessUnitKey& key : orderedKeys)
        {
            const auto found = automaticUnitIndices.find(key.memberEntityIds);
            if (found == automaticUnitIndices.end()
                || !usedIndices.insert(found->second).second) return false;
            orderedIndices.push_back(found->second);
        }

        std::map<EntityId, ProcessAssignment> assignmentsByEntity;
        for (const ProcessAssignment& assignment : plan.assignments)
            assignmentsByEntity.emplace(assignment.entityId, assignment);

        ProcessPlan reordered = plan;
        reordered.processUnits.clear();
        reordered.processUnitSequence.units.clear();
        reordered.assignments.clear();
        reordered.processUnits.reserve(plan.processUnits.size());
        reordered.processUnitSequence.units.reserve(plan.processUnits.size());
        reordered.assignments.reserve(plan.assignments.size());

        for (const std::size_t sourceUnitIndex : orderedIndices)
        {
            const ProcessUnit& unit = plan.processUnits[sourceUnitIndex];
            const int processUnitIndex = static_cast<int>(reordered.processUnits.size());
            reordered.processUnits.push_back(unit);
            reordered.processUnitSequence.units.push_back(unit.key);
            for (const EntityId entityId : unit.orderedMemberEntityIds)
            {
                const auto found = assignmentsByEntity.find(entityId);
                if (found == assignmentsByEntity.end()) return false;
                ProcessAssignment assignment = found->second;
                assignment.processUnitIndex = processUnitIndex;
                assignment.processOrder = static_cast<int>(reordered.assignments.size());
                reordered.assignments.push_back(std::move(assignment));
            }
        }

        if (!cadcam::planning::validateProcessUnitStructure(reordered)
            || !satisfiesPrecedenceConstraints(reordered))
        {
            return false;
        }
        plan = std::move(reordered);
        return true;
    }

    bool preserveCurrentUnitSequence
    (
        ProcessPlan& plan,
        const ProcessUnitSequence& currentSequence
    )
    {
        if (!cadcam::planning::validateProcessUnitStructure(plan)) return false;

        std::map<std::vector<EntityId>, ProcessUnitKey> automaticUnits;
        for (const ProcessUnit& unit : plan.processUnits)
            automaticUnits.emplace(unit.key.memberEntityIds, unit.key);

        std::vector<ProcessUnitKey> orderedKeys;
        orderedKeys.reserve(plan.processUnits.size());
        std::set<std::vector<EntityId>> usedKeys;
        for (const ProcessUnitKey& key : currentSequence.units)
        {
            const auto found = automaticUnits.find(key.memberEntityIds);
            if (found != automaticUnits.end()
                && usedKeys.insert(found->first).second)
                orderedKeys.push_back(found->second);
        }
        for (const ProcessUnit& unit : plan.processUnits)
        {
            if (usedKeys.insert(unit.key.memberEntityIds).second)
                orderedKeys.push_back(unit.key);
        }
        return reorderProcessUnits(plan, orderedKeys);
    }

    Diagnostic sequenceDiagnostic
    (
        const OperationContext& context,
        const QString& stage,
        const QString& message
    )
    {
        Diagnostic diagnostic;
        diagnostic.code = DiagnosticCode::ProcessPlanningInvariantViolation;
        diagnostic.severity = DiagnosticSeverity::Error;
        diagnostic.component = QStringLiteral("ProcessPlanningService");
        diagnostic.operation = context.operationName;
        diagnostic.stage = stage;
        diagnostic.userMessage = message;
        diagnostic.correlationId = context.correlationId;
        return diagnostic;
    }
}

OperationResult<cadcam::planning::ProcessPlan> ProcessPlanningService::buildPlanarPlan
(
    CadDocument& document,
    const cadcam::process::DocumentProcessState& processState,
    const cadcam::planning::PlanarProcessPlanningPolicy& policy,
    const OperationContext& context
) const
{
    OperationResult<cadcam::planning::ProcessPlan> result;
    DocumentProcessPlanningAdapter adapter;
    auto capture = adapter.capturePlanar
        (document, processState, policy.sortIntent, context);
    result.mergeDiagnostics(capture);
    if (!capture.succeeded() || !capture.value.has_value())
    {
        result.status = capture.status;
        return result;
    }
    auto plan = cadcam::planning::PlanarProcessPlanBuilder::build
        (*capture.value, policy, context);
    result.mergeDiagnostics(plan);
    if (!plan.succeeded() || !plan.value.has_value())
    {
        result.status = plan.status;
        return result;
    }
    if (policy.sortIntent == cadcam::planning::ProcessSortIntent::PreserveCurrentSequence
        && !preserveCurrentUnitSequence
            (*plan.value, processState.processUnitSequence()))
    {
        result.status = OperationStatus::Failed;
        result.addDiagnostic(sequenceDiagnostic
        (
            context,
            QStringLiteral("preserve-planar-process-unit-sequence"),
            QStringLiteral("当前加工单元序列与三轴加工约束不一致，无法执行普通排序。")
        ));
        return result;
    }
    if (document.contentRevision() != plan.value->contentRevision
        || processState.revision() != plan.value->processStateRevision)
    {
        result.status = OperationStatus::Conflict;
        Diagnostic diagnostic;
        diagnostic.code = DiagnosticCode::ProcessStateRevisionMismatch;
        diagnostic.severity = DiagnosticSeverity::Error;
        diagnostic.component = QStringLiteral("ProcessPlanningService");
        diagnostic.operation = context.operationName;
        diagnostic.stage = QStringLiteral("validate-planar-plan-revision");
        diagnostic.userMessage = QStringLiteral("文档或加工状态在三轴加工计划构建期间已变更。");
        diagnostic.correlationId = context.correlationId;
        result.addDiagnostic(diagnostic);
        return result;
    }
    result.status = plan.status;
    result.value = std::move(*plan.value);
    return result;
}

OperationResult<cadcam::planning::ProcessPlan> ProcessPlanningService::buildRotaryPlan
(
    CadDocument& document,
    const cadcam::process::DocumentProcessState& processState,
    const std::optional<cadcam::machining::TubeSectionModel>& tubeSection,
    const cadcam::planning::ProcessPlanningPolicy& policy,
    const OperationContext& context
) const
{
    cadcam::topology::PathTopology topology;
    DocumentProcessPlanningAdapter adapter;
    auto capture = adapter.captureRotary
        (document, processState, tubeSection, policy.sortIntent,
            policy.connectionTolerance, topology, context);
    if (!capture.succeeded() || !capture.value.has_value())
    {
        OperationResult<cadcam::planning::ProcessPlan> result;
        result.status = capture.status;
        result.mergeDiagnostics(capture.diagnostics);
        return result;
    }
    auto result = cadcam::planning::ProcessPlanBuilder::build
        (*capture.value, policy, context);
    result.mergeDiagnostics(capture.diagnostics);
    if (result.succeeded() && result.value.has_value()
        && policy.sortIntent == cadcam::planning::ProcessSortIntent::PreserveCurrentSequence
        && !preserveCurrentUnitSequence
            (*result.value, processState.processUnitSequence()))
    {
        result.status = OperationStatus::Failed;
        result.value.reset();
        result.addDiagnostic(sequenceDiagnostic
        (
            context,
            QStringLiteral("preserve-rotary-process-unit-sequence"),
            QStringLiteral("当前加工单元序列与四轴加工断面约束不一致，无法执行普通排序。")
        ));
        return result;
    }
    if (result.succeeded() && result.value.has_value()
        && (document.contentRevision() != result.value->contentRevision
            || processState.revision() != result.value->processStateRevision))
    {
        result.status = OperationStatus::Conflict;
        result.value.reset();
        Diagnostic diagnostic;
        diagnostic.code = DiagnosticCode::ProcessStateRevisionMismatch;
        diagnostic.severity = DiagnosticSeverity::Error;
        diagnostic.component = QStringLiteral("ProcessPlanningService");
        diagnostic.operation = context.operationName;
        diagnostic.stage = QStringLiteral("validate-rotary-plan-revision");
        diagnostic.userMessage = QStringLiteral("文档或加工状态在四轴加工计划构建期间已变更。");
        diagnostic.correlationId = context.correlationId;
        result.addDiagnostic(diagnostic);
    }
    return result;
}

OperationResult<cadcam::planning::ProcessPlan>
ProcessPlanningService::reorderPlanByUnitSequence
(
    const cadcam::planning::ProcessPlan& plan,
    const cadcam::planning::ProcessUnitSequence& sequence,
    const OperationContext& context
) const
{
    OperationResult<cadcam::planning::ProcessPlan> result;
    cadcam::planning::ProcessPlan reordered = plan;
    if (!reorderProcessUnits(reordered, sequence.units))
    {
        result.status = OperationStatus::Failed;
        result.addDiagnostic(sequenceDiagnostic
        (
            context,
            QStringLiteral("reorder-process-unit-sequence"),
            QStringLiteral("加工单元序列不符合当前加工计划或加工断面约束。")
        ));
        return result;
    }

    reordered.processUnitSequence.revision = sequence.revision;
    result.status = OperationStatus::Success;
    result.value = std::move(reordered);
    return result;
}
