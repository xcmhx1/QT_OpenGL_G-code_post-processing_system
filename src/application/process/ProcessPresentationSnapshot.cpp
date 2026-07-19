#include "application/process/ProcessPresentationSnapshot.h"

#include <algorithm>
#include <map>
#include <set>

namespace cadcam::process
{
    const ProcessPresentationEntry* ProcessPresentationSnapshot::find
    (geometry::EntityId entityId) const
    {
        const auto found = std::lower_bound(entries.cbegin(), entries.cend(), entityId,
            [](const ProcessPresentationEntry& entry, geometry::EntityId id)
            { return entry.entityId < id; });
        return found != entries.cend() && found->entityId == entityId ? &*found : nullptr;
    }

    OperationResult<ProcessPresentationSnapshot> ProcessPresentationSnapshot::build
    (const planning::ProcessPlan& plan, const OperationContext& context)
    {
        OperationResult<ProcessPresentationSnapshot> result;
        const auto failInvalidPresentation = [&result, &context](const QString& userMessage)
        {
            result.status = OperationStatus::InvalidInput;
            Diagnostic diagnostic;
            diagnostic.code = DiagnosticCode::ProcessPresentationInvalid;
            diagnostic.severity = DiagnosticSeverity::Error;
            diagnostic.component = QStringLiteral("ProcessPresentationSnapshot");
            diagnostic.operation = context.operationName;
            diagnostic.userMessage = userMessage;
            diagnostic.correlationId = context.correlationId;
            result.addDiagnostic(diagnostic);
        };

        if (!planning::validateProcessUnitStructure(plan))
        {
            failInvalidPresentation(QStringLiteral("加工显示快照的加工单元与计划序列不一致。"));
            return result;
        }

        ProcessPresentationSnapshot snapshot;
        snapshot.contentRevision = plan.contentRevision;
        snapshot.processStateRevision = plan.processStateRevision;
        snapshot.mode = plan.mode;
        snapshot.processUnits.reserve(plan.processUnits.size());

        std::map<geometry::EntityId, const planning::ProcessAssignment*> assignmentsByEntity;
        for (const planning::ProcessAssignment& assignment : plan.assignments)
        {
            assignmentsByEntity.emplace(assignment.entityId, &assignment);
        }

        for (std::size_t index = 0; index < plan.processUnits.size(); ++index)
        {
            const planning::ProcessUnit& unit = plan.processUnits[index];
            if (unit.orderedMemberEntityIds.empty())
            {
                failInvalidPresentation(QStringLiteral("加工显示快照包含空加工单元。"));
                return result;
            }

            const auto anchorAssignment = assignmentsByEntity.find
                (unit.orderedMemberEntityIds.front());
            if (anchorAssignment == assignmentsByEntity.end())
            {
                failInvalidPresentation(QStringLiteral("加工单元起点缺少对应的计划分配。"));
                return result;
            }

            snapshot.processUnits.push_back
            ({
                unit.key,
                static_cast<int>(index),
                unit.orderedMemberEntityIds,
                unit.orderedMemberEntityIds.front(),
                anchorAssignment->second->reverse,
                anchorAssignment->second->startParameter
            });
        }

        std::set<geometry::EntityId> ids;
        for (const planning::ProcessAssignment& assignment : plan.assignments)
        {
            if (assignment.entityId == 0 || !ids.insert(assignment.entityId).second)
            {
                failInvalidPresentation(QStringLiteral("加工显示快照包含无效或重复图元。"));
                return result;
            }
            snapshot.entries.push_back({ assignment.entityId, assignment.processOrder,
                assignment.continuousGroupId, assignment.reverse, assignment.startParameter,
                false, std::nullopt });
        }
        for (const planning::ProcessExclusion& exclusion : plan.exclusions)
        {
            if (exclusion.entityId == 0 || !ids.insert(exclusion.entityId).second)
            {
                failInvalidPresentation(QStringLiteral("加工显示快照的计划和排除项发生重叠。"));
                return result;
            }
            snapshot.entries.push_back({ exclusion.entityId, -1, -1, false, std::nullopt,
                true, exclusion.reason });
        }
        std::sort(snapshot.entries.begin(), snapshot.entries.end(),
            [](const auto& left, const auto& right) { return left.entityId < right.entityId; });
        result.status = OperationStatus::Success;
        result.value = std::move(snapshot);
        return result;
    }
}
