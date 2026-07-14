#include "application/process/ProcessPresentationSnapshot.h"

#include <algorithm>
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
        ProcessPresentationSnapshot snapshot;
        snapshot.contentRevision = plan.contentRevision;
        snapshot.processStateRevision = plan.processStateRevision;
        snapshot.mode = plan.mode;
        std::set<geometry::EntityId> ids;
        for (const planning::ProcessAssignment& assignment : plan.assignments)
        {
            if (assignment.entityId == 0 || !ids.insert(assignment.entityId).second)
            {
                result.status = OperationStatus::InvalidInput;
                Diagnostic diagnostic;
                diagnostic.code = DiagnosticCode::ProcessPresentationInvalid;
                diagnostic.severity = DiagnosticSeverity::Error;
                diagnostic.component = QStringLiteral("ProcessPresentationSnapshot");
                diagnostic.operation = context.operationName;
                diagnostic.userMessage = QStringLiteral("加工显示快照包含无效或重复图元。");
                diagnostic.correlationId = context.correlationId;
                result.addDiagnostic(diagnostic);
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
                result.status = OperationStatus::InvalidInput;
                Diagnostic diagnostic;
                diagnostic.code = DiagnosticCode::ProcessPresentationInvalid;
                diagnostic.severity = DiagnosticSeverity::Error;
                diagnostic.component = QStringLiteral("ProcessPresentationSnapshot");
                diagnostic.operation = context.operationName;
                diagnostic.userMessage = QStringLiteral("加工显示快照的计划和排除项发生重叠。");
                diagnostic.correlationId = context.correlationId;
                result.addDiagnostic(diagnostic);
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
