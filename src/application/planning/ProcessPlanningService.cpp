#include "application/planning/ProcessPlanningService.h"

#include "cad/document/CadDocument.h"
#include "application/planning/DocumentProcessPlanningAdapter.h"
#include "core/machining/TubeSectionProjector.h"
#include "core/planning/PlanarProcessPlanBuilder.h"
#include "core/planning/ProcessPlanBuilder.h"
#include "core/planning/SingleClosedEntryRefiner.h"

#include <QDebug>
#include <QStringList>

#include <limits>
#include <cmath>
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

    struct Zone16SummaryReport
    {
        int unitCount = 0;
        int singleZoneUnitCount = 0;
        int multiZoneUnitCount = 0;
        int uncertainUnitCount = 0;
        int zeroMaskUnitCount = 0;
        QString status = QStringLiteral("Success");
    };

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

    bool rebuildProcessUnitExecution(ProcessPlan& plan)
    {
        std::map<EntityId, ProcessAssignment> assignmentsByEntity;
        for (const ProcessAssignment& assignment : plan.assignments)
            assignmentsByEntity.emplace(assignment.entityId, assignment);

        plan.processUnitSequence.units.clear();
        plan.assignments.clear();
        plan.processUnitSequence.units.reserve(plan.processUnits.size());
        plan.assignments.reserve(assignmentsByEntity.size());
        for (std::size_t unitIndex = 0; unitIndex < plan.processUnits.size(); ++unitIndex)
        {
            const ProcessUnit& unit = plan.processUnits[unitIndex];
            plan.processUnitSequence.units.push_back(unit.key);
            for (const EntityId entityId : unit.orderedMemberEntityIds)
            {
                const auto found = assignmentsByEntity.find(entityId);
                if (found == assignmentsByEntity.end()) return false;
                ProcessAssignment assignment = found->second;
                assignment.processUnitIndex = static_cast<int>(unitIndex);
                assignment.processOrder = static_cast<int>(plan.assignments.size());
                plan.assignments.push_back(std::move(assignment));
            }
        }
        std::map<EntityId, int> unitIndexByEntity;
        for (const ProcessAssignment& assignment : plan.assignments)
            unitIndexByEntity.emplace
                (assignment.entityId, assignment.processUnitIndex);
        for (cadcam::planning::ProcessPathFragment& fragment :
            plan.plannedFragments)
        {
            const auto unitIndex = unitIndexByEntity.find
                (fragment.entityId);
            if (unitIndex == unitIndexByEntity.end()) return false;
            fragment.processUnitIndex = unitIndex->second;
        }
        return cadcam::planning::validateProcessUnitStructure(plan)
            && satisfiesPrecedenceConstraints(plan);
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

        std::vector<ProcessUnit> reorderedUnits;
        reorderedUnits.reserve(plan.processUnits.size());
        for (const std::size_t sourceUnitIndex : orderedIndices)
        {
            reorderedUnits.push_back(plan.processUnits[sourceUnitIndex]);
        }
        plan.processUnits = std::move(reorderedUnits);
        return rebuildProcessUnitExecution(plan);
    }

    bool setProcessUnitTraversal
    (
        ProcessPlan& plan,
        const ProcessUnitKey& key,
        const cadcam::process::ProcessUnitTraversalOverride& traversal
    )
    {
        if (!cadcam::planning::validProcessUnitKey(key)
            || traversal.members.size() != key.memberEntityIds.size()) return false;

        std::vector<EntityId> memberIds;
        memberIds.reserve(traversal.members.size());
        for (const auto& member : traversal.members)
        {
            if (member.entityId == 0U
                || (member.startParameter.has_value()
                    && !std::isfinite(*member.startParameter))) return false;
            memberIds.push_back(member.entityId);
        }
        std::sort(memberIds.begin(), memberIds.end());
        if (memberIds != key.memberEntityIds
            || std::adjacent_find(memberIds.begin(), memberIds.end()) != memberIds.end())
            return false;

        auto unit = std::find_if
        (
            plan.processUnits.begin(), plan.processUnits.end(),
            [&key](const ProcessUnit& candidate) { return candidate.key == key; }
        );
        if (unit == plan.processUnits.end()) return false;
        const int processUnitIndex = static_cast<int>
            (std::distance(plan.processUnits.begin(), unit));

        std::map<EntityId, ProcessAssignment*> assignmentsByEntity;
        for (ProcessAssignment& assignment : plan.assignments)
            assignmentsByEntity.emplace(assignment.entityId, &assignment);

        unit->orderedMemberEntityIds.clear();
        unit->orderedMemberEntityIds.reserve(traversal.members.size());
        for (const auto& member : traversal.members)
        {
            const auto assignment = assignmentsByEntity.find(member.entityId);
            if (assignment == assignmentsByEntity.end()) return false;
            assignment->second->reverse = member.reverse;
            assignment->second->startParameter = member.startParameter;
            unit->orderedMemberEntityIds.push_back(member.entityId);
        }
        plan.plannedFragments.erase
        (
            std::remove_if
            (
                plan.plannedFragments.begin(), plan.plannedFragments.end(),
                [processUnitIndex]
                (const cadcam::planning::ProcessPathFragment& fragment)
                {
                    return fragment.processUnitIndex == processUnitIndex;
                }
            ),
            plan.plannedFragments.end()
        );
        return true;
    }

    bool applyStoredUnitTraversalOverrides
    (
        ProcessPlan& plan,
        const cadcam::process::DocumentProcessState& processState
    )
    {
        bool changed = false;
        for (const ProcessUnit& unit : plan.processUnits)
        {
            const auto* traversal = processState.findProcessUnitTraversalOverride(unit.key);
            if (traversal == nullptr) continue;
            if (!setProcessUnitTraversal(plan, unit.key, *traversal)) return false;
            changed = true;
        }
        return !changed || rebuildProcessUnitExecution(plan);
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

    cadcam::geometry::Path3D finalProfilePath
    (
        const cadcam::planning::PlanningEntity& entity,
        const ProcessAssignment& assignment
    )
    {
        cadcam::geometry::Path3D path = entity.path;
        if (path.closed && assignment.startParameter.has_value()
            && path.vertices.size() > 1U)
        {
            const double parameter = *assignment.startParameter;
            const auto start = std::min_element
            (
                path.vertices.begin(), path.vertices.end(),
                [parameter](const cadcam::geometry::PathVertex3D& left,
                    const cadcam::geometry::PathVertex3D& right)
                {
                    return std::abs(left.sourceParameter - parameter)
                        < std::abs(right.sourceParameter - parameter);
                }
            );
            std::rotate(path.vertices.begin(), start, path.vertices.end());
        }
        if (assignment.reverse && path.vertices.size() > 1U)
        {
            if (path.closed)
                std::reverse(path.vertices.begin() + 1, path.vertices.end());
            else
                std::reverse(path.vertices.begin(), path.vertices.end());
        }
        return path;
    }

    QString processUnitKeyText(const ProcessUnitKey& key)
    {
        QStringList values;
        values.reserve(static_cast<qsizetype>(key.memberEntityIds.size()));
        for (const EntityId entityId : key.memberEntityIds)
            values.push_back(QString::number(entityId));
        return values.join(QLatin1Char('+'));
    }

    int setBitCount(cadcam::machining::TubeZoneMask mask)
    {
        int count = 0;
        while (mask != 0U)
        {
            count += static_cast<int>(mask & 1U);
            mask = static_cast<cadcam::machining::TubeZoneMask>(mask >> 1U);
        }
        return count;
    }

    Diagnostic zone16ProfileDiagnostic
    (
        const OperationContext& context,
        const ProcessUnit& unit,
        int groupId,
        const cadcam::machining::ProcessUnitZoneProfile* profile,
        const QString& status,
        const QString& detail
    )
    {
        QStringList zones;
        QStringList xSpans;
        const cadcam::machining::TubeZoneMask mask = profile != nullptr
            ? profile->occupancyMask : 0U;
        const cadcam::machining::TubeZoneMask certainMask = profile != nullptr
            ? profile->certainMask : 0U;
        const cadcam::machining::TubeZoneMask possibleMask = profile != nullptr
            ? profile->possibleMask : 0U;
        if (profile != nullptr)
        {
            for (std::size_t index = 0U;
                index < cadcam::machining::kTubeZone16Count; ++index)
            {
                const auto zone = static_cast<cadcam::machining::TubeZone16>(index);
                const auto& span = profile->zoneSpans[index];
                if (!span.occupied) continue;
                const QString name = cadcam::machining::tubeZoneName(zone);
                zones.push_back(name);
                xSpans.push_back(QStringLiteral("%1:[%2,%3]")
                    .arg(name)
                    .arg(span.minimumX, 0, 'f', 6)
                    .arg(span.maximumX, 0, 'f', 6));
            }
        }

        const QString maskDigits = QStringLiteral("%1")
            .arg(mask, 4, 16, QLatin1Char('0')).toUpper();
        const QString certainMaskDigits = QStringLiteral("%1")
            .arg(certainMask, 4, 16, QLatin1Char('0')).toUpper();
        const QString possibleMaskDigits = QStringLiteral("%1")
            .arg(possibleMask, 4, 16, QLatin1Char('0')).toUpper();
        Diagnostic diagnostic;
        diagnostic.code = DiagnosticCode::ProcessPlanningZone16Profile;
        diagnostic.severity = profile != nullptr && !profile->uncertain
            ? DiagnosticSeverity::Info : DiagnosticSeverity::Warning;
        diagnostic.component = QStringLiteral("ProcessPlanningService");
        diagnostic.operation = context.operationName;
        diagnostic.stage = QStringLiteral("build-zone16-profile");
        diagnostic.userMessage = profile != nullptr
            ? QStringLiteral("加工单元 16 区位画像已生成。")
            : QStringLiteral("加工单元 16 区位画像生成失败。");
        diagnostic.technicalDetail = detail;
        diagnostic.correlationId = context.correlationId;
        diagnostic.groupId = groupId;
        diagnostic.context.insert(QStringLiteral("zone16Profile"), true);
        diagnostic.context.insert(QStringLiteral("unitKey"), processUnitKeyText(unit.key));
        diagnostic.context.insert(QStringLiteral("groupId"), groupId);
        diagnostic.context.insert(QStringLiteral("mask"),
            QStringLiteral("0x%1").arg(maskDigits));
        diagnostic.context.insert(QStringLiteral("certainMask"),
            QStringLiteral("0x%1").arg(certainMaskDigits));
        diagnostic.context.insert(QStringLiteral("possibleMask"),
            QStringLiteral("0x%1").arg(possibleMaskDigits));
        diagnostic.context.insert(QStringLiteral("zones"), zones.join(QLatin1Char(',')));
        diagnostic.context.insert(QStringLiteral("entryZone"), profile != nullptr
            ? cadcam::machining::tubeZoneName(profile->entryZone)
            : QStringLiteral("Unknown"));
        diagnostic.context.insert(QStringLiteral("exitZone"), profile != nullptr
            ? cadcam::machining::tubeZoneName(profile->exitZone)
            : QStringLiteral("Unknown"));
        diagnostic.context.insert(QStringLiteral("entryPerimeter"), profile != nullptr
            ? profile->entryPerimeterPosition : 0.0);
        diagnostic.context.insert(QStringLiteral("exitPerimeter"), profile != nullptr
            ? profile->exitPerimeterPosition : 0.0);
        diagnostic.context.insert(QStringLiteral("xSpans"),
            xSpans.join(QLatin1Char(';')));
        diagnostic.context.insert(QStringLiteral("maximumShellDeviation"),
            profile != nullptr ? profile->maximumShellDeviation : 0.0);
        diagnostic.context.insert(QStringLiteral("averageShellDeviation"),
            profile != nullptr ? profile->averageShellDeviation : 0.0);
        diagnostic.context.insert(QStringLiteral("uncertain"),
            profile == nullptr || profile->uncertain);
        diagnostic.context.insert(QStringLiteral("status"), status);
        return diagnostic;
    }

    Diagnostic zone16SummaryDiagnostic
    (
        const OperationContext& context,
        const Zone16SummaryReport& report
    )
    {
        Diagnostic diagnostic;
        diagnostic.code = DiagnosticCode::ProcessPlanningZone16Summary;
        diagnostic.severity = report.uncertainUnitCount == 0
                && report.zeroMaskUnitCount == 0
            ? DiagnosticSeverity::Info : DiagnosticSeverity::Warning;
        diagnostic.component = QStringLiteral("ProcessPlanningService");
        diagnostic.operation = context.operationName;
        diagnostic.stage = QStringLiteral("summarize-zone16-profiles");
        diagnostic.userMessage = QStringLiteral("四轴加工单元 16 区位画像汇总已完成。");
        diagnostic.technicalDetail =
            QStringLiteral("Zone16 profile summary reflects final ProcessUnit traversal.");
        diagnostic.correlationId = context.correlationId;
        diagnostic.context.insert(QStringLiteral("zone16Summary"), true);
        diagnostic.context.insert(QStringLiteral("unitCount"), report.unitCount);
        diagnostic.context.insert(QStringLiteral("singleZoneUnitCount"),
            report.singleZoneUnitCount);
        diagnostic.context.insert(QStringLiteral("multiZoneUnitCount"),
            report.multiZoneUnitCount);
        diagnostic.context.insert(QStringLiteral("uncertainUnitCount"),
            report.uncertainUnitCount);
        diagnostic.context.insert(QStringLiteral("zeroMaskUnitCount"),
            report.zeroMaskUnitCount);
        diagnostic.context.insert(QStringLiteral("status"), report.status);
        return diagnostic;
    }

    void appendZone16PlanningDiagnostics
    (
        OperationResult<ProcessPlan>& result,
        const cadcam::planning::ProcessPlanningInput& input,
        const OperationContext& context
    )
    {
        if (!result.succeeded() || !result.value.has_value()) return;

        Zone16SummaryReport summary;
        if (!input.tubeSection.has_value())
        {
            summary.status = QStringLiteral("SkippedNoSection");
            result.addDiagnostic(zone16SummaryDiagnostic(context, summary));
            return;
        }

        const auto& section = *input.tubeSection;
        const double projectionTolerance = std::max(1.0e-5,
            std::max(section.geometry.yLength, section.geometry.zWidth) * 1.0e-6);
        std::map<EntityId, const cadcam::planning::PlanningEntity*> entities;
        for (const auto& entity : input.entities)
            entities.emplace(entity.entityId, &entity);
        std::map<EntityId, const ProcessAssignment*> assignments;
        for (const ProcessAssignment& assignment : result.value->assignments)
            assignments.emplace(assignment.entityId, &assignment);
        std::map<EntityId, int> groupIds;
        for (const ProcessGroup& group : result.value->groups)
            for (const EntityId entityId : group.entityIds)
                groupIds.emplace(entityId, group.groupId);

        for (const ProcessUnit& unit : result.value->processUnits)
        {
            ++summary.unitCount;
            std::vector<cadcam::geometry::Path3D> paths;
            paths.reserve(unit.orderedMemberEntityIds.size());
            bool complete = true;
            int groupId = -1;
            for (const EntityId entityId : unit.orderedMemberEntityIds)
            {
                const auto entity = entities.find(entityId);
                const auto assignment = assignments.find(entityId);
                const auto group = groupIds.find(entityId);
                if (entity == entities.end() || assignment == assignments.end()
                    || group == groupIds.end())
                {
                    complete = false;
                    break;
                }
                if (groupId < 0) groupId = group->second;
                else if (groupId != group->second)
                {
                    complete = false;
                    break;
                }
                paths.push_back(finalProfilePath(*entity->second, *assignment->second));
            }

            OperationResult<cadcam::machining::ProcessUnitZoneProfile> profileResult;
            if (complete && !paths.empty())
            {
                profileResult = cadcam::machining::TubeSectionProjector::buildProfile
                    (section, paths, unit.closed, projectionTolerance, context);
            }
            const auto* profile = profileResult.value.has_value()
                ? &*profileResult.value : nullptr;
            QString status = QStringLiteral("Success");
            QString detail = QStringLiteral("Zone16 profile completed.");
            if (profile == nullptr)
            {
                status = QStringLiteral("Failed");
                detail = !complete
                    ? QStringLiteral("Final ProcessUnit members could not be mapped to one planning group.")
                    : profileResult.diagnostics.isEmpty()
                        ? QStringLiteral("Zone16 profile did not return a value.")
                        : profileResult.diagnostics.front().technicalDetail;
                ++summary.zeroMaskUnitCount;
                ++summary.uncertainUnitCount;
                summary.status = QStringLiteral("PartialSuccess");
            }
            else
            {
                const int occupiedZoneCount = setBitCount(profile->certainMask);
                if (occupiedZoneCount == 0) ++summary.zeroMaskUnitCount;
                else if (occupiedZoneCount == 1) ++summary.singleZoneUnitCount;
                else ++summary.multiZoneUnitCount;
                if (profile->uncertain) ++summary.uncertainUnitCount;
                if (profile->uncertain || occupiedZoneCount == 0)
                {
                    status = QStringLiteral("PartialSuccess");
                    summary.status = QStringLiteral("PartialSuccess");
                }
            }
            result.addDiagnostic(zone16ProfileDiagnostic
                (context, unit, groupId, profile, status, detail));
        }
        result.addDiagnostic(zone16SummaryDiagnostic(context, summary));
    }

    void logClosedLoopPlanningSummaries(const QVector<Diagnostic>& diagnostics)
    {
        for (const Diagnostic& diagnostic : diagnostics)
        {
            if (!diagnostic.context.value(QStringLiteral("closedLoopSummary")).toBool()) continue;
            const QVariantMap& values = diagnostic.context;
            qInfo().noquote()
                << QStringLiteral("[ProcessPlanning][ClosedLoop] groupId=%1 memberCount=%2 memberEntityIds=%3 nodeCount=%4 branchNodeCount=%5 candidateCount=%6 selectedOrder=%7 selectedReverse=%8 status=%9")
                    .arg(values.value(QStringLiteral("groupId"), -1).toInt())
                    .arg(values.value(QStringLiteral("memberCount"), 0).toInt())
                    .arg(values.value(QStringLiteral("memberEntityIds")).toString())
                    .arg(values.value(QStringLiteral("nodeCount"), 0).toInt())
                    .arg(values.value(QStringLiteral("branchNodeCount"), 0).toInt())
                    .arg(values.value(QStringLiteral("candidateCount"), 0).toInt())
                    .arg(values.value(QStringLiteral("selectedOrder")).toString())
                    .arg(values.value(QStringLiteral("selectedReverse")).toString())
                    .arg(values.value(QStringLiteral("status"), QStringLiteral("Failed")).toString());
        }
    }

    void logProcessUnitContinuity(const QVector<Diagnostic>& diagnostics)
    {
        for (const Diagnostic& diagnostic : diagnostics)
        {
            if (diagnostic.code
                    != DiagnosticCode::MachineTrajectoryContinuityFailure
                && diagnostic.stage
                    != QStringLiteral("split-discontinuous-process-unit"))
            {
                continue;
            }
            const QVariantMap& values = diagnostic.context;
            qInfo().noquote()
                << QStringLiteral(
                    "[ProcessUnitContinuity] processUnitIndex=%1 "
                    "previousEntityId=%2 nextEntityId=%3 "
                    "sourceConnectionDistance=%4 machineConnectionDistance=%5 "
                    "previousSurface=%6 nextSurface=%7 previousA=%8 nextA=%9 "
                    "failure=%10 splitApplied=%11")
                    .arg(values.value(QStringLiteral("processUnitIndex")).toInt())
                    .arg(values.value(QStringLiteral("previousEntityId")).toULongLong())
                    .arg(values.value(QStringLiteral("nextEntityId")).toULongLong())
                    .arg(values.value(QStringLiteral("sourceConnectionDistance"))
                        .toDouble(), 0, 'g', 15)
                    .arg(values.value(QStringLiteral("machineConnectionDistance"))
                        .toDouble(), 0, 'g', 15)
                    .arg(values.value(QStringLiteral("previousSurface")).toString())
                    .arg(values.value(QStringLiteral("nextSurface")).toString())
                    .arg(values.value(QStringLiteral("previousA")).toDouble(),
                        0, 'g', 15)
                    .arg(values.value(QStringLiteral("nextA")).toDouble(),
                        0, 'g', 15)
                    .arg(values.value(QStringLiteral("failure")).toString())
                    .arg(values.value(QStringLiteral("splitApplied")).toBool()
                        ? QStringLiteral("true") : QStringLiteral("false"));
        }
    }

    void logBreakStartPlanningSummaries(const QVector<Diagnostic>& diagnostics)
    {
        for (const Diagnostic& diagnostic : diagnostics)
        {
            if (!diagnostic.context.value
                (QStringLiteral("breakStartSummary")).toBool())
            {
                continue;
            }
            const QVariantMap& values = diagnostic.context;
            qInfo().noquote()
                << QStringLiteral("[ProcessPlanning][BreakStart] groupId=%1 boundaryRank=%2 strategy=%3 preferredStartZone=%4 startZone=%5 startEntityId=%6 startParameter=%7 startPosition=%8 runLength=%9 direction=%10 exitZone=%11 exitConfidence=%12 fragmentCount=%13 midpointFragmentUsed=%14 status=%15")
                    .arg(values.value(QStringLiteral("groupId"), -1).toInt())
                    .arg(values.value(QStringLiteral("boundaryRank"), -1).toInt())
                    .arg(values.value(QStringLiteral("strategy")).toString())
                    .arg(values.value(QStringLiteral("preferredStartZone"),
                        QStringLiteral("Unknown")).toString())
                    .arg(values.value(QStringLiteral("selectedZone"),
                        QStringLiteral("Unknown")).toString())
                    .arg(values.value(QStringLiteral("selectedEntityId")).toULongLong())
                    .arg(values.value(QStringLiteral("selectedSourceParameter"))
                        .toDouble(), 0, 'g', 15)
                    .arg(values.value(QStringLiteral("selectedMidpoint")).toString())
                    .arg(values.value(QStringLiteral("selectedRunLength"))
                        .toDouble(), 0, 'g', 15)
                    .arg(values.value(QStringLiteral("direction")).toString())
                    .arg(values.value(QStringLiteral("exitZone"),
                        QStringLiteral("Unknown")).toString())
                    .arg(values.value(QStringLiteral("exitConfidence"))
                        .toDouble(), 0, 'g', 15)
                    .arg(values.value(QStringLiteral("fragmentCount")).toInt())
                    .arg(values.value(QStringLiteral("midpointFragmentUsed"))
                        .toBool() ? 1 : 0)
                    .arg(values.value(QStringLiteral("status"),
                        QStringLiteral("Unknown")).toString());
        }
    }

    void logEntrySelectionSummaries(const QVector<Diagnostic>& diagnostics)
    {
        for (const Diagnostic& diagnostic : diagnostics)
        {
            if (diagnostic.context.value
                (QStringLiteral("singleClosedEntryRefinement")).toBool())
            {
                const QVariantMap& values = diagnostic.context;
                const int transferKind =
                    values.value(QStringLiteral("transferKind")).toInt();
                const QString transferKindName =
                    transferKind == 0
                    ? QStringLiteral("InitialApproach")
                    : transferKind == 1
                        ? QStringLiteral("SameZoneSurfaceTransfer")
                        : transferKind == 2
                            ? QStringLiteral("SameZoneClearanceTransfer")
                            : QStringLiteral("CrossZoneRotaryTransfer");
                qInfo().noquote()
                    << QStringLiteral("[ProcessPlanning][SingleClosedEntryRefinement] processUnitIndex=%1 unitKey=%2 entityId=%3 sourceKind=%4 ownerZone=%5 previousProcessUnitIndex=%6 fromProcessUnit=%7 toProcessUnit=%8 previousCutEnd=%9 transferKind=%10 searchIntervalCount=%11 rootCandidateCount=%12 validTangentCount=%13 mode=%14 selectedSourceParameter=%15 selectedReverse=%16 finalApproachOrigin=%17 selectedCutStart=%18 approachCutDot=%19 approachCutAngle=%20 tangentResidual=%21 previewSegmentCount=%22")
                        .arg(values.value(QStringLiteral("processUnitIndex")).toInt())
                        .arg(values.value(QStringLiteral("unitKey")).toString())
                        .arg(values.value(QStringLiteral("entityId")).toULongLong())
                        .arg(values.value(QStringLiteral("sourceKind")).toString())
                        .arg(values.value(QStringLiteral("ownerZone")).toString())
                        .arg(values.value(QStringLiteral("previousProcessUnitIndex")).toInt())
                        .arg(values.value(QStringLiteral("fromProcessUnit")).toInt())
                        .arg(values.value(QStringLiteral("toProcessUnit")).toInt())
                        .arg(values.value(QStringLiteral("previousCutEnd")).toString())
                        .arg(transferKindName)
                        .arg(values.value(QStringLiteral("searchIntervalCount")).toInt())
                        .arg(values.value(QStringLiteral("rootCandidateCount")).toInt())
                        .arg(values.value(QStringLiteral("validTangentCount")).toInt())
                        .arg(values.value(QStringLiteral("mode")).toString())
                        .arg(values.value(QStringLiteral("selectedSourceParameter")).toDouble(), 0, 'g', 15)
                        .arg(values.value(QStringLiteral("selectedReverse")).toBool()
                            ? QStringLiteral("true") : QStringLiteral("false"))
                        .arg(values.value(QStringLiteral("finalApproachOrigin")).toString())
                        .arg(values.value(QStringLiteral("selectedCutStart")).toString())
                        .arg(values.value(QStringLiteral("approachCutDot")).toDouble(), 0, 'g', 15)
                        .arg(values.value(QStringLiteral("approachCutAngle")).toDouble(), 0, 'g', 15)
                        .arg(values.value(QStringLiteral("tangentResidual")).toDouble(), 0, 'g', 15)
                        .arg(values.value(QStringLiteral("previewSegmentCount")).toInt());
                continue;
            }
            if (diagnostic.context.value
                (QStringLiteral("entryZoneProfile")).toBool())
            {
                const QVariantMap& values = diagnostic.context;
                qInfo().noquote()
                    << QStringLiteral("[ProcessPlanning][EntryZoneProfile] unitKey=%1 groupKind=%2 memberSourceKinds=%3 certainMask=%4 possibleMask=%5 connectionEntryMask=%6 curveInteriorEntryMask=%7 zoneRunMidpointEntryMask=%8 legalEntryMask=%9 candidateCountsByZone=%10 arcMemberIdsByZone=%11 ellipseMemberIdsByZone=%12")
                        .arg(values.value(QStringLiteral("unitKey")).toString())
                        .arg(values.value(QStringLiteral("groupKind")).toString())
                        .arg(values.value(QStringLiteral("memberSourceKinds")).toString())
                        .arg(values.value(QStringLiteral("certainMask")).toString())
                        .arg(values.value(QStringLiteral("possibleMask")).toString())
                        .arg(values.value(QStringLiteral("connectionEntryMask")).toString())
                        .arg(values.value(QStringLiteral("curveInteriorEntryMask")).toString())
                        .arg(values.value(QStringLiteral("zoneRunMidpointEntryMask")).toString())
                        .arg(values.value(QStringLiteral("legalEntryMask")).toString())
                        .arg(values.value(QStringLiteral("candidateCountsByZone")).toString())
                        .arg(values.value(QStringLiteral("arcMemberIdsByZone")).toString())
                        .arg(values.value(QStringLiteral("ellipseMemberIdsByZone")).toString());
                continue;
            }
            if (!diagnostic.context.value
                (QStringLiteral("entrySelectionSummary")).toBool())
            {
                continue;
            }
            const QVariantMap& values = diagnostic.context;
            if (values.value
                (QStringLiteral("entryRefinementSummary")).toBool())
            {
                qInfo().noquote()
        << QStringLiteral("[ProcessPlanning][EntryRefinement] unitKey=%1 ownerZone=%2 previousCutEnd=%3 previousTransferAnchor=%4 mode=%5 curveMemberCount=%6 arcTangentRootCount=%7 ellipseTangentRootCount=%8 validTangentCount=%9 connectionCandidateCount=%10 selectedEntityId=%11 selectedSourceKind=%12 selectedSourceParameter=%13 travelDistance=%14 approachCutAngle=%15 selectedReverse=%16 fragmentCount=%17 nearestConnectionDistance=%18 forwardAngle=%19 reverseAngle=%20 tangentResidual=%21 approachCutDot=%22")
                        .arg(values.value(QStringLiteral("unitKey")).toString())
                        .arg(values.value(QStringLiteral("ownerZone")).toString())
                        .arg(values.value(QStringLiteral("previousCutEnd")).toString())
            .arg(values.value(QStringLiteral("previousTransferAnchor")).toString())
                        .arg(values.value(QStringLiteral("selectionMode")).toString())
                        .arg(values.value(QStringLiteral("curveMemberCount")).toInt())
                        .arg(values.value(QStringLiteral("arcTangentRootCount")).toInt())
                        .arg(values.value(QStringLiteral("ellipseTangentRootCount")).toInt())
                        .arg(values.value(QStringLiteral("validTangentCount")).toInt())
                        .arg(values.value(QStringLiteral("connectionCandidateCount")).toInt())
                        .arg(values.value(QStringLiteral("selectedEntityId")).toULongLong())
                        .arg(values.value(QStringLiteral("selectedSourceKind")).toString())
                        .arg(values.value(QStringLiteral("selectedSourceParameter")).toDouble(), 0, 'g', 15)
                        .arg(values.value(QStringLiteral("travelDistance")).toDouble(), 0, 'g', 15)
                        .arg(values.value(QStringLiteral("approachCutAngle")).toDouble(), 0, 'g', 15)
                        .arg(values.value(QStringLiteral("selectedReverse")).toBool()
                            ? QStringLiteral("true") : QStringLiteral("false"))
                        .arg(values.value(QStringLiteral("fragmentCount")).toInt())
                        .arg(values.value(QStringLiteral("nearestConnectionDistance")).toDouble(), 0, 'g', 15)
                        .arg(values.value(QStringLiteral("forwardAngle")).toDouble(), 0, 'g', 15)
                        .arg(values.value(QStringLiteral("reverseAngle")).toDouble(), 0, 'g', 15)
                        .arg(values.value(QStringLiteral("tangentResidual")).toDouble(), 0, 'g', 15)
                        .arg(values.value(QStringLiteral("approachCutDot")).toDouble(), 0, 'g', 15);
                continue;
            }
            qInfo().noquote()
                << QStringLiteral("[ProcessPlanning][EntrySelection] unitKey=%1 ownerZone=%2 scheduledZone=%3 selectedEntryZone=%4 selectionMode=%5 candidateKind=%6 connectionCandidateCount=%7 arcInteriorCandidateCount=%8 ellipseInteriorCandidateCount=%9 zoneRunMidpointCandidateCount=%10 curveCandidateRejectedCount=%11 candidateCount=%12 wrongZoneRejectedCount=%13 selectedEntityId=%14 selectedSourceKind=%15 selectedSourceParameter=%16 selectedReverse=%17 entryPosition=%18 firstCutTangent=%19 distanceToMemberEndpoint=%20 distanceToZoneBoundary=%21 axisReversalCount=%22 tangentCost=%23 rotationCost=%24 movementDistance=%25 fragmentCount=%26")
                    .arg(values.value(QStringLiteral("unitKey")).toString())
                    .arg(values.value(QStringLiteral("ownerZone")).toString())
                    .arg(values.value(QStringLiteral("scheduledZone")).toString())
                    .arg(values.value(QStringLiteral("selectedEntryZone")).toString())
                    .arg(values.value(QStringLiteral("selectionMode")).toString())
                    .arg(values.value(QStringLiteral("candidateKind")).toString())
                    .arg(values.value(QStringLiteral("connectionCandidateCount")).toInt())
                    .arg(values.value(QStringLiteral("arcInteriorCandidateCount")).toInt())
                    .arg(values.value(QStringLiteral("ellipseInteriorCandidateCount")).toInt())
                    .arg(values.value(QStringLiteral("zoneRunMidpointCandidateCount")).toInt())
                    .arg(values.value(QStringLiteral("curveCandidateRejectedCount")).toInt())
                    .arg(values.value(QStringLiteral("candidateCount")).toInt())
                    .arg(values.value(QStringLiteral("wrongZoneRejectedCount")).toInt())
                    .arg(values.value(QStringLiteral("selectedEntityId")).toULongLong())
                    .arg(values.value(QStringLiteral("selectedSourceKind")).toString())
                    .arg(values.value(QStringLiteral("selectedSourceParameter"))
                        .toDouble(), 0, 'g', 15)
                    .arg(values.value(QStringLiteral("selectedReverse")).toBool()
                        ? QStringLiteral("true") : QStringLiteral("false"))
                    .arg(values.value(QStringLiteral("entryPosition")).toString())
                    .arg(values.value(QStringLiteral("firstCutTangent")).toString())
                    .arg(values.value(QStringLiteral("distanceToMemberEndpoint"))
                        .toDouble(), 0, 'g', 15)
                    .arg(values.value(QStringLiteral("distanceToZoneBoundary"))
                        .toDouble(), 0, 'g', 15)
                    .arg(values.value(QStringLiteral("axisReversalCount")).toInt())
                    .arg(values.value(QStringLiteral("tangentCost"))
                        .toDouble(), 0, 'g', 15)
                    .arg(values.value(QStringLiteral("rotationCost"))
                        .toDouble(), 0, 'g', 15)
                    .arg(values.value(QStringLiteral("movementDistance"))
                        .toDouble(), 0, 'g', 15)
                    .arg(values.value(QStringLiteral("fragmentCount")).toInt());
        }
    }

    void logZone16SweepPlanningSummaries(const QVector<Diagnostic>& diagnostics)
    {
        for (const Diagnostic& diagnostic : diagnostics)
        {
            const QVariantMap& values = diagnostic.context;
            if (values.value(QStringLiteral("zoneOwnership")).toBool())
            {
                qInfo().noquote()
                    << QStringLiteral("[ProcessPlanning][ZoneOwnership] partitionId=%1 unitKey=%2 certainMask=%3 possibleMask=%4 ownerCandidateMask=%5 ownerZone=%6 ownerBasis=%7 legalEntryMaskBefore=%8 usedBoundaryFallback=%9")
                        .arg(values.value(QStringLiteral("partitionId"), -1).toInt())
                        .arg(values.value(QStringLiteral("unitKey")).toString())
                        .arg(values.value(QStringLiteral("certainMask")).toString())
                        .arg(values.value(QStringLiteral("possibleMask")).toString())
                        .arg(values.value(QStringLiteral("ownerCandidateMask")).toString())
                        .arg(values.value(QStringLiteral("ownerZone"),
                            QStringLiteral("Unknown")).toString())
                        .arg(values.value(QStringLiteral("ownerBasis")).toString())
                        .arg(values.value(QStringLiteral("legalEntryMaskBefore")).toString())
                        .arg(values.value(QStringLiteral("usedBoundaryFallback")).toBool()
                            ? QStringLiteral("true") : QStringLiteral("false"));
                continue;
            }
            if (values.value(QStringLiteral("zonePhase")).toBool())
            {
                qInfo().noquote()
                    << QStringLiteral("[ProcessPlanning][ZonePhase] partitionId=%1 zone=%2 event=%3 ownedUnitCount=%4 processedUnitCount=%5 remainingUnitCount=%6")
                        .arg(values.value(QStringLiteral("partitionId"), -1).toInt())
                        .arg(values.value(QStringLiteral("zone"),
                            QStringLiteral("Unknown")).toString())
                        .arg(values.value(QStringLiteral("event")).toString())
                        .arg(values.value(QStringLiteral("ownedUnitCount"), 0).toInt())
                        .arg(values.value(QStringLiteral("processedUnitCount"), 0).toInt())
                        .arg(values.value(QStringLiteral("remainingUnitCount"), 0).toInt());
                continue;
            }
            if (values.value(QStringLiteral("zoneBlocked")).toBool())
            {
                qInfo().noquote()
                    << QStringLiteral("[ProcessPlanning][ZoneBlocked] partitionId=%1 zone=%2 frontierX=%3 unfinishedUnitKeys=%4 blockedUnitKeys=%5 remainingPredecessors=%6")
                        .arg(values.value(QStringLiteral("partitionId"), -1).toInt())
                        .arg(values.value(QStringLiteral("zone"),
                            QStringLiteral("Unknown")).toString())
                        .arg(values.value(QStringLiteral("frontierX"), 0.0)
                            .toDouble(), 0, 'f', 6)
                        .arg(values.value(QStringLiteral("unfinishedUnitKeys")).toString())
                        .arg(values.value(QStringLiteral("blockedUnitKeys")).toString())
                        .arg(values.value(QStringLiteral("remainingPredecessors")).toString());
                continue;
            }
            if (!values.value(QStringLiteral("zone16SweepSummary")).toBool())
                continue;
            qInfo().noquote()
                << QStringLiteral("[ProcessPlanning][Zone16Sweep] partitionId=%1 initialZone=%2 perimeterDirection=%3 longitudinalDirection=%4 partitionMinimumX=%5 partitionMaximumX=%6 processedUnitCount=%7 zoneTransitions=%8 backtrackCount=%9 status=%10")
                    .arg(values.value(QStringLiteral("partitionId"), -1).toInt())
                    .arg(values.value(QStringLiteral("initialZone"),
                        QStringLiteral("Unknown")).toString())
                    .arg(values.value(QStringLiteral("perimeterDirection"),
                        QStringLiteral("Clockwise")).toString())
                    .arg(values.value(QStringLiteral("longitudinalDirection"), 1).toInt())
                    .arg(values.value(QStringLiteral("partitionMinimumX"), 0.0)
                        .toDouble(), 0, 'f', 6)
                    .arg(values.value(QStringLiteral("partitionMaximumX"), 0.0)
                        .toDouble(), 0, 'f', 6)
                    .arg(values.value(QStringLiteral("processedUnitCount"), 0).toInt())
                    .arg(values.value(QStringLiteral("zoneTransitions"), 0).toInt())
                    .arg(values.value(QStringLiteral("backtrackCount"), 0).toInt())
                    .arg(values.value(QStringLiteral("status"),
                        QStringLiteral("Unknown")).toString());
            const QStringList selectedUnits =
                values.value(QStringLiteral("selectedUnits")).toStringList();
            for (const QString& selectedUnit : selectedUnits)
            {
                qInfo().noquote()
                    << QStringLiteral("[ProcessPlanning][Zone16SweepUnit] %1")
                        .arg(selectedUnit);
            }
        }
    }

    void logZone16PlanningSummaries(const QVector<Diagnostic>& diagnostics)
    {
        for (const Diagnostic& diagnostic : diagnostics)
        {
            const QVariantMap& values = diagnostic.context;
            if (values.value(QStringLiteral("zone16Profile")).toBool())
            {
                qInfo().noquote()
                    << QStringLiteral("[ProcessPlanning][Zone16Profile] unitKey=%1 groupId=%2 mask=%3 certainMask=%4 possibleMask=%5 zones=%6 entryZone=%7 exitZone=%8 entryPerimeter=%9 exitPerimeter=%10 xSpans=%11 maximumShellDeviation=%12 averageShellDeviation=%13 uncertain=%14 status=%15")
                        .arg(values.value(QStringLiteral("unitKey")).toString())
                        .arg(values.value(QStringLiteral("groupId"), -1).toInt())
                        .arg(values.value(QStringLiteral("mask"),
                            QStringLiteral("0x0000")).toString())
                        .arg(values.value(QStringLiteral("certainMask"),
                            QStringLiteral("0x0000")).toString())
                        .arg(values.value(QStringLiteral("possibleMask"),
                            QStringLiteral("0x0000")).toString())
                        .arg(values.value(QStringLiteral("zones")).toString())
                        .arg(values.value(QStringLiteral("entryZone"),
                            QStringLiteral("Unknown")).toString())
                        .arg(values.value(QStringLiteral("exitZone"),
                            QStringLiteral("Unknown")).toString())
                        .arg(values.value(QStringLiteral("entryPerimeter"), 0.0)
                            .toDouble(), 0, 'f', 6)
                        .arg(values.value(QStringLiteral("exitPerimeter"), 0.0)
                            .toDouble(), 0, 'f', 6)
                        .arg(values.value(QStringLiteral("xSpans")).toString())
                        .arg(values.value(QStringLiteral("maximumShellDeviation"),
                            0.0).toDouble(), 0, 'f', 9)
                        .arg(values.value(QStringLiteral("averageShellDeviation"),
                            0.0).toDouble(), 0, 'f', 9)
                        .arg(values.value(QStringLiteral("uncertain")).toBool()
                            ? QStringLiteral("true") : QStringLiteral("false"))
                        .arg(values.value(QStringLiteral("status"),
                            QStringLiteral("Unknown")).toString());
            }
            else if (values.value(QStringLiteral("zone16Summary")).toBool())
            {
                qInfo().noquote()
                    << QStringLiteral("[ProcessPlanning][Zone16Summary] unitCount=%1 singleZoneUnitCount=%2 multiZoneUnitCount=%3 uncertainUnitCount=%4 zeroMaskUnitCount=%5 status=%6")
                        .arg(values.value(QStringLiteral("unitCount"), 0).toInt())
                        .arg(values.value(QStringLiteral("singleZoneUnitCount"), 0).toInt())
                        .arg(values.value(QStringLiteral("multiZoneUnitCount"), 0).toInt())
                        .arg(values.value(QStringLiteral("uncertainUnitCount"), 0).toInt())
                        .arg(values.value(QStringLiteral("zeroMaskUnitCount"), 0).toInt())
                        .arg(values.value(QStringLiteral("status"),
                            QStringLiteral("Unknown")).toString());
            }
        }
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
        && (!preserveCurrentUnitSequence(*plan.value, processState.processUnitSequence())
            || !applyStoredUnitTraversalOverrides(*plan.value, processState)))
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
    qInfo().noquote()
        << QStringLiteral("[ProcessPlanning][Zone16Policy] initialZone=%1 perimeterDirection=%2 longitudinalDirection=%3")
            .arg(cadcam::machining::tubeZoneName
                (policy.zone16Sweep.initialZone))
            .arg(policy.zone16Sweep.perimeterDirection
                == cadcam::planning::PerimeterSweepDirection::Clockwise
                ? QStringLiteral("Clockwise")
                : QStringLiteral("CounterClockwise"))
            .arg(policy.zone16Sweep.longitudinalDirection
                == cadcam::planning::LongitudinalSweepDirection::PositiveX
                ? QStringLiteral("PositiveX")
                : QStringLiteral("NegativeX"));
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
    logProcessUnitContinuity(result.diagnostics);
    logClosedLoopPlanningSummaries(result.diagnostics);
    logBreakStartPlanningSummaries(result.diagnostics);
    logEntrySelectionSummaries(result.diagnostics);
    logZone16SweepPlanningSummaries(result.diagnostics);
    result.mergeDiagnostics(capture.diagnostics);
    if (result.succeeded() && result.value.has_value()
        && policy.sortIntent == cadcam::planning::ProcessSortIntent::PreserveCurrentSequence
        && (!preserveCurrentUnitSequence(*result.value, processState.processUnitSequence())
            || !applyStoredUnitTraversalOverrides(*result.value, processState)))
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
        && policy.sortIntent
            == cadcam::planning::ProcessSortIntent::PreserveCurrentSequence)
    {
        auto continuity = cadcam::planning::SingleClosedEntryRefiner::refine
            (*result.value, *capture.value, policy, context);
        logProcessUnitContinuity(continuity.diagnostics);
        result.mergeDiagnostics(continuity);
        if (!continuity.succeeded())
        {
            result.status = continuity.status;
            result.value.reset();
            return result;
        }
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
    appendZone16PlanningDiagnostics(result, *capture.value, context);
    logZone16PlanningSummaries(result.diagnostics);
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

OperationResult<cadcam::planning::ProcessPlan>
ProcessPlanningService::applyPlanUnitTraversal
(
    const cadcam::planning::ProcessPlan& plan,
    const cadcam::planning::ProcessUnitKey& key,
    const cadcam::process::ProcessUnitTraversalOverride& traversal,
    const OperationContext& context
) const
{
    OperationResult<cadcam::planning::ProcessPlan> result;
    cadcam::planning::ProcessPlan updated = plan;
    if (!cadcam::planning::validateProcessUnitStructure(updated)
        || !setProcessUnitTraversal(updated, key, traversal)
        || !rebuildProcessUnitExecution(updated))
    {
        result.status = OperationStatus::Failed;
        result.addDiagnostic(sequenceDiagnostic
        (
            context,
            QStringLiteral("apply-process-unit-traversal"),
            QStringLiteral("加工单元整组反向不符合当前计划或加工断面约束。")
        ));
        return result;
    }
    result.status = OperationStatus::Success;
    result.value = std::move(updated);
    return result;
}
