#include "core/planning/ProcessPlanBuilder.h"

#include "core/machining/TubeCutBoundary.h"
#include "core/machining/TubeSectionProjector.h"
#include "core/machine/RotaryKinematics.h"
#include "core/planning/SingleClosedEntryRefiner.h"

#include <QStringList>

#include <algorithm>
#include <array>
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
#include "internal/ProcessPlanBuilder_InternalTypesAndCurves.inl"
#include "internal/ProcessPlanBuilder_SectionAndSurface.inl"
#include "internal/ProcessPlanBuilder_TraversalAndZones.inl"
#include "internal/ProcessPlanBuilder_ZoneProfilesAndDiagnostics.inl"
#include "internal/ProcessPlanBuilder_ClosedLoopTraversal.inl"
#include "internal/ProcessPlanBuilder_ClosedLoopZoneRun.inl"
#include "internal/ProcessPlanBuilder_EntrySelectionAndValidation.inl"

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
            || !std::isfinite(policy.connectionTolerance)
            || !std::isfinite(policy.rotationSafetyClearance)
            || policy.rotationSafetyClearance <= 0.0
            || !std::isfinite(policy.sameZoneTransferClearance)
            || policy.sameZoneTransferClearance < 0.0
            || !std::isfinite(policy.connectionDistanceTieTolerance)
            || policy.connectionDistanceTieTolerance <= 0.0)
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
        std::optional<machining::TubeSectionModel> surfaceSweepSection;
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
            surfaceSweepSection = *input.tubeSection;
            surfaceSweepSection->geometry = *planningSection;
        }
        const bool zone16SweepEnabled = policy.sortIntent
                == ProcessSortIntent::RebuildSequence
            && policy.orderingStrategy == ProcessOrderingStrategy::LazyRotation
            && surfaceSweepSection.has_value();
        const double zoneProjectionTolerance = surfaceSweepSection.has_value()
            ? std::max(1.0e-5, std::max(surfaceSweepSection->geometry.yLength,
                surfaceSweepSection->geometry.zWidth) * 1.0e-6)
            : 0.0;

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
                    : entity.sourceKind == geometry::SourceGeometryKind::Point
                        || entity.sourceKind == geometry::SourceGeometryKind::Unknown
                        ? ProcessExclusionReason::UnsupportedGeometry
                        : ProcessExclusionReason::InvalidPath;
            const bool excluded = !entity.visible || !entity.processEnabled
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
            const auto appendOrdinaryGroup =
                [&plan, &entities]
                (const std::vector<EntityId>& ids, bool closed)
            {
                if (ids.empty()) return;
                ProcessGroup group;
                group.groupId = static_cast<int>(plan.groups.size());
                group.entityIds = ids;
                group.closed = closed
                    || (ids.size() == 1U
                        && entities.at(ids.front())->path.closed);
                group.kind = group.closed
                    ? ProcessGroupKind::ClosedLoop
                    : ids.size() == 1U
                        ? ProcessGroupKind::SingleEntity
                        : ProcessGroupKind::ConnectedChain;
                plan.groups.push_back(std::move(group));
            };
            const auto appendOpenGroups =
                [&appendOrdinaryGroup, &entities, &policy]
                (const std::vector<EntityId>& ids)
            {
                for (const std::vector<EntityId>& chain :
                    partitionOpenComponentIntoTraversableChains
                    (
                        ids, entities, policy.allowReverse,
                        policy.connectionTolerance
                    ))
                {
                    appendOrdinaryGroup(chain, false);
                }
            };
            for (auto& [componentId, ids] : components)
            {
                (void)componentId;
                if (!policy.preserveClosedLoopsAsAtomicGroups)
                {
                    appendOpenGroups(ids);
                    continue;
                }

                std::vector<std::vector<EntityId>> pendingComponents
                    { std::move(ids) };
                for (std::size_t pendingIndex = 0U;
                    pendingIndex < pendingComponents.size(); ++pendingIndex)
                {
                    std::vector<EntityId> currentIds =
                        std::move(pendingComponents[pendingIndex]);
                    const auto loop = input.topology->extractBestLoop
                        (currentIds, currentIds);
                    if (!loop.succeeded() || !loop.value.has_value()
                        || !loop.value->connectedLoop
                        || loop.value->usedEntityIds.empty())
                    {
                        appendOpenGroups(currentIds);
                        continue;
                    }

                    const std::unordered_set<EntityId> loopIds
                        (loop.value->usedEntityIds.cbegin(),
                            loop.value->usedEntityIds.cend());
                    std::vector<EntityId> closedIds;
                    std::vector<EntityId> remainingIds;
                    closedIds.reserve(loopIds.size());
                    remainingIds.reserve(currentIds.size() - loopIds.size());
                    for (const EntityId entityId : currentIds)
                    {
                        (loopIds.count(entityId) != 0U
                            ? closedIds : remainingIds).push_back(entityId);
                    }
                    if (closedIds.empty())
                    {
                        appendOpenGroups(currentIds);
                        continue;
                    }
                    appendOrdinaryGroup(closedIds, true);
                    if (remainingIds.empty()) continue;

                    const std::vector<int> remainingComponentIds =
                        input.topology->componentIds(remainingIds);
                    if (remainingComponentIds.size() != remainingIds.size())
                    {
                        appendOpenGroups(remainingIds);
                        continue;
                    }
                    std::map<int, std::vector<EntityId>> remainingComponents;
                    int remainingSyntheticComponent = -1;
                    for (std::size_t index = 0U;
                        index < remainingIds.size(); ++index)
                    {
                        const int remainingComponentId =
                            remainingComponentIds[index] >= 0
                            ? remainingComponentIds[index]
                            : remainingSyntheticComponent--;
                        remainingComponents[remainingComponentId].push_back
                            (remainingIds[index]);
                    }
                    for (auto& [remainingComponentId, remaining] :
                        remainingComponents)
                    {
                        (void)remainingComponentId;
                        pendingComponents.push_back(std::move(remaining));
                    }
                }
            }
        }

        std::unordered_set<int> excludedGroups;
        for (const ProcessGroup& group : plan.groups)
            if (group.kind == ProcessGroupKind::WasteBoundary) excludedGroups.insert(group.groupId);

        QVector<Diagnostic> zoneSweepDiagnostics;
        std::unordered_map<int, ProcessGroupZoneProfile> groupZoneProfiles;
        if (zone16SweepEnabled)
        {
            groupZoneProfiles.reserve(plan.groups.size());
            for (const ProcessGroup& group : plan.groups)
            {
                if (group.kind == ProcessGroupKind::BreakBoundary
                    || group.kind == ProcessGroupKind::WasteBoundary)
                    continue;

                std::vector<geometry::Path3D> paths;
                paths.reserve(group.entityIds.size());
                for (const EntityId entityId : group.entityIds)
                {
                    paths.push_back(entities.at(entityId)->path);
                }
                auto profileResult = machining::TubeSectionProjector::buildProfile
                    (*surfaceSweepSection, paths, group.closed,
                        zoneProjectionTolerance, context);
                if (!profileResult.succeeded() || !profileResult.value.has_value())
                {
                    auto failed = failure<ProcessPlan>
                    (
                        OperationStatus::Failed, context,
                        DiagnosticCode::ProcessPlanningZoneSweepProfileInvalid,
                        QStringLiteral("加工单元无法建立可靠的方管 16 区位画像。"),
                        QStringLiteral("Static ProcessGroup zone profile construction failed."),
                        diagnosticValues(input, policy, 0U, 0U, -1,
                            group.groupId)
                    );
                    failed.mergeDiagnostics(profileResult.diagnostics);
                    return failed;
                }
                zoneSweepDiagnostics += profileResult.diagnostics;

                const machining::ProcessUnitZoneProfile& sourceProfile =
                    *profileResult.value;
                ProcessGroupZoneProfile profile;
                profile.certainMask = sourceProfile.certainMask;
                profile.possibleMask = sourceProfile.possibleMask;
                profile.zoneSpans = sourceProfile.zoneSpans;
                profile.closed = group.closed;
                profile.uncertain = sourceProfile.uncertain;
                for (std::size_t zoneIndex = 0U;
                    zoneIndex < machining::kTubeZone16Count;
                    ++zoneIndex)
                {
                    TraversalSelectionContext selection;
                    selection.requiredEntryZone =
                        static_cast<machining::TubeZone16>(zoneIndex);
                    selection.longitudinalDirection =
                        policy.zone16Sweep.longitudinalDirection
                            == LongitudinalSweepDirection::PositiveX
                        ? 1 : -1;
                    selection.zoneHitX = selection.longitudinalDirection >= 0
                        ? profile.zoneSpans[zoneIndex].minimumX
                        : profile.zoneSpans[zoneIndex].maximumX;
                    selection.frontierX = selection.zoneHitX;
                    selection.projectionTolerance =
                        zoneProjectionTolerance;
                    selection.previousEnd = policy.initialPosition;
                    selection.hardZoneConstraint = true;
                    ClosedLoopTraversalReport ignoredLoopReport;
                    auto entryTraversal = bestTraversal
                    (
                        group, entities, policy.initialPosition, policy,
                        input.tubeSection, input.tubeSectionCenter,
                        ProcessOrderingStrategy::LazyRotation,
                        &ignoredLoopReport, &selection
                    );
                    if (!entryTraversal.has_value()
                        || !entryTraversal->selectedEntry.has_value())
                    {
                        continue;
                    }
                    profile.entryCandidates[zoneIndex].push_back
                        (*entryTraversal->selectedEntry);
                    profile.entryCandidateCounts[zoneIndex] =
                        entryTraversal->entryCandidateCount;
                    const machining::TubeZone16 zone =
                        static_cast<machining::TubeZone16>(zoneIndex);
                    const machining::TubeZoneMask zoneBit =
                        machining::tubeZoneBit(zone);
                    if (group.kind == ProcessGroupKind::ClosedLoop
                        && group.entityIds.size() > 1U)
                    {
                        if (entryTraversal->connectionCandidateCount > 0)
                            profile.connectionEntryMask |= zoneBit;
                        if (entryTraversal->arcInteriorCandidateCount > 0
                            || entryTraversal
                                ->ellipseInteriorCandidateCount > 0)
                        {
                            profile.curveInteriorEntryMask |= zoneBit;
                        }
                        profile.arcMemberIdsByZone[zoneIndex] =
                            entryTraversal
                                ->arcInteriorCandidateEntityIds;
                        profile.ellipseMemberIdsByZone[zoneIndex] =
                            entryTraversal
                                ->ellipseInteriorCandidateEntityIds;
                        profile.legalEntryMask =
                            profile.connectionEntryMask
                            | profile.curveInteriorEntryMask
                            | profile.zoneRunMidpointEntryMask;
                    }
                    else
                    {
                        profile.legalEntryMask |= zoneBit;
                    }
                }
                if ((profile.certainMask & ~profile.possibleMask) != 0U
                    || profile.possibleMask == 0U)
                {
                    QVariantMap values = diagnosticValues
                        (input, policy, 0U, 0U, -1, group.groupId);
                    values.insert(QStringLiteral("unitKey"),
                        processGroupKeyText(group));
                    values.insert(QStringLiteral("certainMask"),
                        zoneMaskText(profile.certainMask));
                    values.insert(QStringLiteral("possibleMask"),
                        zoneMaskText(profile.possibleMask));
                    values.insert(QStringLiteral("legalEntryMask"),
                        zoneMaskText(profile.legalEntryMask));
                    return failure<ProcessPlan>
                    (
                        OperationStatus::Failed,
                        context,
                        DiagnosticCode::ProcessPlanningZoneSweepProfileInvalid,
                        QStringLiteral("加工单元没有可用于 16 区位调度的可靠投影。"),
                        QStringLiteral("Zone occupancy masks are inconsistent or empty."),
                        values
                    );
                }
                profile.schedulableMask = profile.certainMask != 0U
                    ? profile.certainMask : profile.possibleMask;
                QVariantMap entryValues;
                entryValues.insert(QStringLiteral("entryZoneProfile"), true);
                entryValues.insert(QStringLiteral("unitKey"),
                    processGroupKeyText(group));
                entryValues.insert(QStringLiteral("groupKind"),
                    groupKindName(group.kind));
                entryValues.insert(QStringLiteral("certainMask"),
                    zoneMaskText(profile.certainMask));
                entryValues.insert(QStringLiteral("possibleMask"),
                    zoneMaskText(profile.possibleMask));
                entryValues.insert(QStringLiteral("legalEntryMask"),
                    zoneMaskText(profile.legalEntryMask));
                entryValues.insert(QStringLiteral("connectionEntryMask"),
                    zoneMaskText(profile.connectionEntryMask));
                entryValues.insert(QStringLiteral("curveInteriorEntryMask"),
                    zoneMaskText(profile.curveInteriorEntryMask));
                entryValues.insert(QStringLiteral("zoneRunMidpointEntryMask"),
                    zoneMaskText(profile.zoneRunMidpointEntryMask));
                QStringList memberSourceKinds;
                for (const EntityId entityId : group.entityIds)
                {
                    const auto entity = entities.find(entityId);
                    memberSourceKinds.push_back(QStringLiteral("%1:%2")
                        .arg(entityId)
                        .arg(entity != entities.end()
                            && entity->second != nullptr
                            ? planningSourceKindName
                                (entity->second->sourceKind)
                            : QStringLiteral("Missing")));
                }
                entryValues.insert(QStringLiteral("memberSourceKinds"),
                    memberSourceKinds.join(QLatin1Char(',')));
                QStringList counts;
                QStringList arcMembers;
                QStringList ellipseMembers;
                for (std::size_t zoneIndex = 0U;
                    zoneIndex < machining::kTubeZone16Count;
                    ++zoneIndex)
                {
                    const QString zoneName = machining::tubeZoneName
                        (static_cast<machining::TubeZone16>(zoneIndex));
                    counts.push_back(QStringLiteral("%1:%2")
                        .arg(zoneName)
                        .arg(profile.entryCandidateCounts[zoneIndex]));
                    auto idsText = [](const std::vector<EntityId>& ids)
                    {
                        QStringList values;
                        for (const EntityId id : ids)
                            values.push_back(QString::number(id));
                        return values.join(QLatin1Char(','));
                    };
                    arcMembers.push_back(QStringLiteral("%1:%2")
                        .arg(zoneName)
                        .arg(idsText
                            (profile.arcMemberIdsByZone[zoneIndex])));
                    ellipseMembers.push_back(QStringLiteral("%1:%2")
                        .arg(zoneName)
                        .arg(idsText
                            (profile.ellipseMemberIdsByZone[zoneIndex])));
                }
                entryValues.insert(QStringLiteral("candidateCountsByZone"),
                    counts.join(QLatin1Char(',')));
                entryValues.insert(QStringLiteral("arcMemberIdsByZone"),
                    arcMembers.join(QLatin1Char(';')));
                entryValues.insert(QStringLiteral("ellipseMemberIdsByZone"),
                    ellipseMembers.join(QLatin1Char(';')));
                zoneSweepDiagnostics.push_back(planningDiagnostic
                (
                    context,
                    DiagnosticCode::ProcessPlanningEntrySelectionSummary,
                    QStringLiteral("加工单元已建立合法入口区位画像。"),
                    QStringLiteral("Executable entry candidates were classified independently from occupancy."),
                    entryValues,
                    DiagnosticSeverity::Info
                ));
                groupZoneProfiles.emplace(group.groupId, std::move(profile));
            }
        }

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

        std::vector<TubeZoneSweepPartition> zoneSweepPartitions;
        std::unordered_map<int, int> partitionAfterBreakGroup;
        if (zone16SweepEnabled)
        {
            std::vector<std::size_t> orderedBreakBoundaryIndices;
            for (const int orderedIndex : boundaryOrder)
            {
                const std::size_t boundaryIndex =
                    static_cast<std::size_t>(orderedIndex);
                if (boundaries[boundaryIndex].role == BoundaryRole::Break)
                    orderedBreakBoundaryIndices.push_back(boundaryIndex);
            }
            zoneSweepPartitions.resize(orderedBreakBoundaryIndices.size() + 1U);
            std::vector<bool> partitionBoundsInitialized
                (zoneSweepPartitions.size(), false);
            for (std::size_t partitionIndex = 0U;
                partitionIndex < zoneSweepPartitions.size(); ++partitionIndex)
            {
                zoneSweepPartitions[partitionIndex].partitionId =
                    static_cast<int>(partitionIndex);
                zoneSweepPartitions[partitionIndex].initialZone =
                    policy.zone16Sweep.initialZone;
                zoneSweepPartitions[partitionIndex].perimeterDirection =
                    policy.zone16Sweep.perimeterDirection
                        == PerimeterSweepDirection::Clockwise ? 1 : -1;
                zoneSweepPartitions[partitionIndex].longitudinalDirection =
                    policy.zone16Sweep.longitudinalDirection
                        == LongitudinalSweepDirection::PositiveX ? 1 : -1;
            }
            for (std::size_t breakIndex = 0U;
                breakIndex < orderedBreakBoundaryIndices.size(); ++breakIndex)
            {
                partitionAfterBreakGroup.emplace
                (
                    boundaries[orderedBreakBoundaryIndices[breakIndex]].groupId,
                    static_cast<int>(breakIndex + 1U)
                );
            }

            for (const ProcessGroup& group : plan.groups)
            {
                if (group.kind == ProcessGroupKind::BreakBoundary
                    || group.kind == ProcessGroupKind::WasteBoundary
                    || excludedGroups.find(group.groupId) != excludedGroups.end())
                    continue;
                const auto sides = groupBoundarySides.find(group.groupId);
                if (sides == groupBoundarySides.end()
                    || groupZoneProfiles.find(group.groupId)
                        == groupZoneProfiles.end())
                {
                    return failure<ProcessPlan>
                    (
                        OperationStatus::Failed, context,
                        DiagnosticCode::ProcessPlanningZoneSweepProfileInvalid,
                        QStringLiteral("加工单元缺少 16 区位加工段数据。"),
                        QStringLiteral("Zone16 partition input is missing group side or profile data."),
                        diagnosticValues(input, policy, 0U, 0U, -1,
                            group.groupId)
                    );
                }

                std::size_t partitionIndex = 0U;
                for (const std::size_t boundaryIndex :
                    orderedBreakBoundaryIndices)
                {
                    if (sides->second[boundaryIndex] == BoundarySide::Right)
                        ++partitionIndex;
                }
                if (partitionIndex >= zoneSweepPartitions.size())
                {
                    return failure<ProcessPlan>
                    (
                        OperationStatus::InternalError, context,
                        DiagnosticCode::ProcessPlanningInvariantViolation,
                        QStringLiteral("加工单元无法映射到有效的 16 区位加工段。"),
                        QStringLiteral("Computed partition index exceeds partition count."),
                        diagnosticValues(input, policy, 0U, 0U, -1,
                            group.groupId)
                    );
                }

                TubeZoneSweepPartition& partition =
                    zoneSweepPartitions[partitionIndex];
                partition.groupIds.insert(group.groupId);
                const ProcessGroupZoneProfile& profile =
                    groupZoneProfiles.at(group.groupId);
                const machining::TubeZoneMask boundsMask =
                    profile.schedulableMask;
                for (std::size_t zoneIndex = 0U;
                    zoneIndex < machining::kTubeZone16Count; ++zoneIndex)
                {
                    const auto zone =
                        static_cast<machining::TubeZone16>(zoneIndex);
                    if ((boundsMask & machining::tubeZoneBit(zone)) == 0U)
                        continue;
                    const machining::TubeZoneSpan& span =
                        profile.zoneSpans[zoneIndex];
                    if (!partitionBoundsInitialized[partitionIndex])
                    {
                        partition.minimumX = span.minimumX;
                        partition.maximumX = span.maximumX;
                        partitionBoundsInitialized[partitionIndex] = true;
                    }
                    else
                    {
                        partition.minimumX = std::min
                            (partition.minimumX, span.minimumX);
                        partition.maximumX = std::max
                            (partition.maximumX, span.maximumX);
                    }
                }
            }
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
        std::unordered_map<int, int> schedulingOccurrences;
        QVector<Diagnostic> closedLoopDiagnostics;
        Zone16SweepState zoneSweepState;
        Zone16SweepReport zoneSweepReport;
        machining::TubeZone16 currentSweepZone =
            policy.zone16Sweep.initialZone;
        Vector3d currentPosition = policy.initialPosition;
        std::optional<machining::TubeZone16> previousTransferZone;
        int processOrder = 0;
        std::vector<bool> startedPartitions(zoneSweepPartitions.size(), false);
        std::vector<bool> finishedPartitions(zoneSweepPartitions.size(), false);
        std::optional<OperationResult<ProcessPlan>>
            zoneSweepLifecycleFailure;
        const auto setZoneSweepLifecycleFailure =
            [&](DiagnosticCode code, const QString& userMessage,
                const QString& technicalDetail, QVariantMap values)
        {
            zoneSweepLifecycleFailure = failure<ProcessPlan>
            (
                OperationStatus::InternalError, context, code,
                userMessage, technicalDetail, std::move(values)
            );
        };
        const auto finishZoneSweepPartition = [&]() -> bool
        {
            if (!zoneSweepState.active) return true;
            if (zoneSweepState.partitionId < 0
                || static_cast<std::size_t>(zoneSweepState.partitionId)
                    >= zoneSweepPartitions.size())
            {
                setZoneSweepLifecycleFailure
                (
                    DiagnosticCode::ProcessPlanningInvariantViolation,
                    QStringLiteral("16 区位加工段状态无效。"),
                    QStringLiteral("Active zone sweep references an invalid partition."),
                    diagnosticValues(input, policy)
                );
                return false;
            }
            const std::size_t partitionIndex =
                static_cast<std::size_t>(zoneSweepState.partitionId);
            const TubeZoneSweepPartition& partition =
                zoneSweepPartitions[partitionIndex];
            const bool allGroupsScheduled = std::all_of
            (
                partition.groupIds.cbegin(), partition.groupIds.cend(),
                [&scheduled](int groupId)
                {
                    return scheduled.find(groupId) != scheduled.end();
                }
            );
            bool pairedPhases = true;
            for (std::size_t zoneIndex = 0U;
                zoneIndex < machining::kTubeZone16Count; ++zoneIndex)
            {
                if (zoneSweepState.enteredZones[zoneIndex]
                    != zoneSweepState.completedZones[zoneIndex])
                {
                    pairedPhases = false;
                    break;
                }
            }
            if (!allGroupsScheduled || !pairedPhases
                || zoneSweepReport.processedUnitCount
                    != static_cast<int>(partition.groupIds.size())
                || zoneSweepReport.backtrackCount != 0)
            {
                QVariantMap values = diagnosticValues(input, policy);
                values.insert(QStringLiteral("partitionId"),
                    zoneSweepState.partitionId);
                values.insert(QStringLiteral("allGroupsScheduled"),
                    allGroupsScheduled);
                values.insert(QStringLiteral("pairedPhases"), pairedPhases);
                values.insert(QStringLiteral("processedUnitCount"),
                    zoneSweepReport.processedUnitCount);
                values.insert(QStringLiteral("expectedUnitCount"),
                    static_cast<int>(partition.groupIds.size()));
                values.insert(QStringLiteral("backtrackCount"),
                    zoneSweepReport.backtrackCount);
                setZoneSweepLifecycleFailure
                (
                    DiagnosticCode::ProcessPlanningZoneIncomplete,
                    QStringLiteral("16 区位加工段尚未完整完成，不能结束该加工段。"),
                    QStringLiteral("Partition completion requires every owner group and every entered zone phase to be complete."),
                    std::move(values)
                );
                return false;
            }
            if (finishedPartitions[partitionIndex])
            {
                QVariantMap values = diagnosticValues(input, policy);
                values.insert(QStringLiteral("partitionId"),
                    zoneSweepState.partitionId);
                setZoneSweepLifecycleFailure
                (
                    DiagnosticCode::ProcessPlanningInvariantViolation,
                    QStringLiteral("16 区位加工段被重复结束。"),
                    QStringLiteral("A zone sweep partition may finish only once."),
                    std::move(values)
                );
                return false;
            }
            finishedPartitions[partitionIndex] = true;
            zoneSweepDiagnostics.push_back
                (zone16SweepDiagnostic(context, zoneSweepReport));
            zoneSweepState = Zone16SweepState{};
            zoneSweepReport = Zone16SweepReport{};
            return true;
        };
        const auto startZoneSweepPartition =
            [&](int partitionId, machining::TubeZone16 initialZone) -> bool
        {
            zoneSweepLifecycleFailure.reset();
            if (partitionId < 0
                || static_cast<std::size_t>(partitionId)
                    >= zoneSweepPartitions.size())
                return false;
            const std::size_t partitionIndex =
                static_cast<std::size_t>(partitionId);
            if (zoneSweepState.active || startedPartitions[partitionIndex])
            {
                QVariantMap values = diagnosticValues(input, policy);
                values.insert(QStringLiteral("partitionId"), partitionId);
                values.insert(QStringLiteral("partitionActive"),
                    zoneSweepState.active);
                values.insert(QStringLiteral("partitionStarted"),
                    static_cast<bool>(startedPartitions[partitionIndex]));
                values.insert(QStringLiteral("partitionFinished"),
                    static_cast<bool>(finishedPartitions[partitionIndex]));
                setZoneSweepLifecycleFailure
                (
                    zoneSweepState.active
                        ? DiagnosticCode::ProcessPlanningZoneIncomplete
                        : DiagnosticCode::ProcessPlanningInvariantViolation,
                    zoneSweepState.active
                        ? QStringLiteral("当前 16 区位加工段尚未完成，不能启动其他加工段。")
                        : QStringLiteral("同一 16 区位加工段不能重复启动。"),
                    QStringLiteral("A zone sweep partition cannot restart or replace an active partition."),
                    std::move(values)
                );
                return false;
            }

            TubeZoneSweepPartition& partition =
                zoneSweepPartitions[partitionIndex];
            partition.initialZone = initialZone;
            std::vector<int> orderedGroupIds
                (partition.groupIds.cbegin(), partition.groupIds.cend());
            std::sort(orderedGroupIds.begin(), orderedGroupIds.end(),
                [&plan](int left, int right)
            {
                return processGroupStableLess
                (
                    plan.groups[static_cast<std::size_t>(left)],
                    plan.groups[static_cast<std::size_t>(right)]
                );
            });
            for (const int groupId : orderedGroupIds)
            {
                const ProcessGroup& group =
                    plan.groups[static_cast<std::size_t>(groupId)];
                ProcessGroupZoneProfile& profile =
                    groupZoneProfiles.at(groupId);
                const machining::TubeZoneMask strongZones =
                    strongTubeZoneMask();
                const bool canCreateOwnerZoneEntry =
                    group.kind == ProcessGroupKind::ClosedLoop
                    && group.entityIds.size() > 1U;
                const machining::TubeZoneMask requiredEntryMask =
                    canCreateOwnerZoneEntry
                    ? std::numeric_limits<machining::TubeZoneMask>::max()
                    : profile.legalEntryMask;
                machining::TubeZoneMask ownerCandidateMask =
                    profile.certainMask & strongZones & requiredEntryMask;
                bool usedPossibleFallback = false;
                bool usedBoundaryFallback = false;
                if (ownerCandidateMask == 0U)
                {
                    ownerCandidateMask =
                        profile.possibleMask & strongZones
                        & requiredEntryMask;
                    usedPossibleFallback = ownerCandidateMask != 0U;
                }
                if (ownerCandidateMask == 0U
                    && profile.certainMask != 0U)
                {
                    ownerCandidateMask =
                        profile.certainMask & requiredEntryMask;
                    usedBoundaryFallback = true;
                }
                if (ownerCandidateMask == 0U
                    && profile.possibleMask != 0U)
                {
                    ownerCandidateMask =
                        profile.possibleMask & requiredEntryMask;
                    usedPossibleFallback = true;
                    usedBoundaryFallback = true;
                }
                std::optional<machining::TubeZone16> ownerZone =
                    firstZoneInSweep(ownerCandidateMask, initialZone,
                        partition.perimeterDirection);
                if (!ownerZone.has_value())
                {
                    QVariantMap values = diagnosticValues
                        (input, policy, 0U, 0U, -1, groupId);
                    values.insert(QStringLiteral("partitionId"), partitionId);
                    values.insert(QStringLiteral("unitKey"),
                        processGroupKeyText(plan.groups
                            [static_cast<std::size_t>(groupId)]));
                    values.insert(QStringLiteral("groupKind"),
                        groupKindName(group.kind));
                    values.insert(QStringLiteral("memberCount"),
                        static_cast<int>(group.entityIds.size()));
                    values.insert(QStringLiteral("certainMask"),
                        zoneMaskText(profile.certainMask));
                    values.insert(QStringLiteral("possibleMask"),
                        zoneMaskText(profile.possibleMask));
                    values.insert(QStringLiteral("legalEntryMask"),
                        zoneMaskText(profile.legalEntryMask));
                    values.insert(QStringLiteral("requiredEntryMask"),
                        zoneMaskText(requiredEntryMask));
                    values.insert(QStringLiteral("ownerCandidateMask"),
                        zoneMaskText(ownerCandidateMask));
                    values.insert(QStringLiteral("zoneOwnershipFailure"), true);
                    setZoneSweepLifecycleFailure
                    (
                        DiagnosticCode::ProcessPlanningZoneSweepProfileInvalid,
                        QStringLiteral("加工单元没有可用的唯一生产区位。"),
                        QStringLiteral("Neither certain nor possible occupancy contains a zone in the partition sweep."),
                        std::move(values)
                    );
                    return false;
                }
                ZoneSweepOwnership ownership;
                ownership.groupId = groupId;
                ownership.ownerZone = *ownerZone;
                ownership.usedPossibleFallback = usedPossibleFallback;
                ownership.usedBoundaryFallback = usedBoundaryFallback;
                ownership.ownerCandidateMask = ownerCandidateMask;
                ownership.legalEntryMaskBefore = profile.legalEntryMask;
                const machining::TubeZoneMask ownerBit =
                    machining::tubeZoneBit(*ownerZone);
                if ((profile.legalEntryMask & ownerBit) == 0U)
                {
                    if (group.kind == ProcessGroupKind::ClosedLoop
                        && group.entityIds.size() > 1U
                        && surfaceSweepSection.has_value())
                    {
                        TraversalSelectionContext selection;
                        selection.requiredEntryZone = *ownerZone;
                        selection.longitudinalDirection =
                            partition.longitudinalDirection;
                        const machining::TubeZoneSpan& ownerSpan =
                            profile.zoneSpans[machining::tubeZoneIndex
                                (*ownerZone)];
                        selection.zoneHitX =
                            selection.longitudinalDirection >= 0
                            ? ownerSpan.minimumX : ownerSpan.maximumX;
                        selection.frontierX = selection.zoneHitX;
                        selection.projectionTolerance =
                            zoneProjectionTolerance;
                        selection.previousEnd = currentPosition;
                        selection.hardZoneConstraint = true;
                        selection.allowZoneRunMidpointFallback = true;
                        ClosedLoopTraversalReport ignoredLoopReport;
                        auto ownerEntry = bestTraversal
                        (
                            group, entities, currentPosition, policy,
                            input.tubeSection, input.tubeSectionCenter,
                            ProcessOrderingStrategy::LazyRotation,
                            &ignoredLoopReport, &selection
                        );
                        if (ownerEntry.has_value()
                            && ownerEntry->selectedEntry.has_value()
                            && ownerEntry->selectedEntry->zone == *ownerZone)
                        {
                            const std::size_t ownerIndex =
                                machining::tubeZoneIndex(*ownerZone);
                            profile.entryCandidates[ownerIndex].push_back
                                (*ownerEntry->selectedEntry);
                            profile.entryCandidateCounts[ownerIndex] =
                                ownerEntry->entryCandidateCount;
                            switch (ownerEntry->selectedEntry->kind)
                            {
                            case ZoneEntryCandidateKind::
                                ClosedLoopArcInterior:
                            case ZoneEntryCandidateKind::
                                ClosedLoopEllipseInterior:
                                profile.curveInteriorEntryMask |= ownerBit;
                                break;
                            case ZoneEntryCandidateKind::
                                ClosedLoopConnection:
                                profile.connectionEntryMask |= ownerBit;
                                break;
                            case ZoneEntryCandidateKind::
                                ClosedLoopZoneRunMidpoint:
                                profile.zoneRunMidpointEntryMask |= ownerBit;
                                break;
                            default:
                                break;
                            }
                            profile.legalEntryMask =
                                profile.connectionEntryMask
                                | profile.curveInteriorEntryMask
                                | profile.zoneRunMidpointEntryMask;
                            const QString unitKey =
                                processGroupKeyText(group);
                            for (Diagnostic& diagnostic :
                                zoneSweepDiagnostics)
                            {
                                if (!diagnostic.context.value
                                        (QStringLiteral("entryZoneProfile"))
                                        .toBool()
                                    || diagnostic.context.value
                                        (QStringLiteral("unitKey"))
                                        .toString() != unitKey)
                                {
                                    continue;
                                }
                                diagnostic.context.insert
                                (
                                    QStringLiteral(
                                        "zoneRunMidpointEntryMask"),
                                    zoneMaskText(profile
                                        .zoneRunMidpointEntryMask)
                                );
                                diagnostic.context.insert
                                (
                                    QStringLiteral("legalEntryMask"),
                                    zoneMaskText(profile.legalEntryMask)
                                );
                                break;
                            }
                        }
                    }
                    if ((profile.legalEntryMask & ownerBit) == 0U)
                    {
                        QVariantMap values = diagnosticValues
                            (input, policy, 0U, 0U, -1, groupId);
                        values.insert(QStringLiteral("partitionId"),
                            partitionId);
                        values.insert(QStringLiteral("unitKey"),
                            processGroupKeyText(plan.groups
                                [static_cast<std::size_t>(groupId)]));
                        values.insert(QStringLiteral("ownerZone"),
                            machining::tubeZoneName(*ownerZone));
                        values.insert(QStringLiteral("certainMask"),
                            zoneMaskText(profile.certainMask));
                        values.insert(QStringLiteral("possibleMask"),
                            zoneMaskText(profile.possibleMask));
                        values.insert(QStringLiteral("legalEntryMaskBefore"),
                            zoneMaskText(ownership.legalEntryMaskBefore));
                        setZoneSweepLifecycleFailure
                        (
                            DiagnosticCode::
                                ProcessPlanningOwnerZoneEntryUnavailable,
                            QStringLiteral("加工单元无法在所属区位建立合法起刀点。"),
                            QStringLiteral("The immutable owner zone has no legal connection, curve-interior, or reliable run-midpoint entry."),
                            std::move(values)
                        );
                        return false;
                    }
                }
                partition.ownerships.emplace(groupId, ownership);
                partition.zoneBuckets[machining::tubeZoneIndex(*ownerZone)]
                    .push_back(groupId);
                zoneSweepDiagnostics.push_back(zoneOwnershipDiagnostic
                (
                    context, partition,
                    plan.groups[static_cast<std::size_t>(groupId)],
                    profile, ownership
                ));
            }
            std::unordered_map<int, int> bucketOccurrences;
            for (auto& bucket : partition.zoneBuckets)
            {
                std::sort(bucket.begin(), bucket.end(),
                    [&plan](int left, int right)
                {
                    return processGroupStableLess
                    (
                        plan.groups[static_cast<std::size_t>(left)],
                        plan.groups[static_cast<std::size_t>(right)]
                    );
                });
                for (const int groupId : bucket)
                    ++bucketOccurrences[groupId];
            }
            for (const int groupId : orderedGroupIds)
            {
                if (partition.ownerships.count(groupId) != 1U
                    || bucketOccurrences[groupId] != 1)
                {
                    QVariantMap values = diagnosticValues
                        (input, policy, 0U, 0U, -1, groupId);
                    values.insert(QStringLiteral("partitionId"), partitionId);
                    values.insert(QStringLiteral("ownershipCount"),
                        static_cast<int>(partition.ownerships.count(groupId)));
                    values.insert(QStringLiteral("bucketOccurrenceCount"),
                        bucketOccurrences[groupId]);
                    setZoneSweepLifecycleFailure
                    (
                        DiagnosticCode::ProcessPlanningInvariantViolation,
                        QStringLiteral("加工单元的 16 区位生产归属不唯一。"),
                        QStringLiteral("Every ordinary group must have one owner and occur in exactly one production bucket."),
                        std::move(values)
                    );
                    return false;
                }
            }

            startedPartitions[partitionIndex] = true;
            zoneSweepState.partitionId = partitionId;
            zoneSweepState.initialZone = initialZone;
            zoneSweepState.currentZoneOffset = 0;
            zoneSweepState.longitudinalDirection =
                partition.longitudinalDirection;
            zoneSweepState.frontierX =
                partition.longitudinalDirection >= 0
                ? partition.minimumX : partition.maximumX;
            zoneSweepState.zoneEntered = false;
            zoneSweepState.active = !partition.groupIds.empty();

            zoneSweepReport.partitionId = partitionId;
            zoneSweepReport.initialZone = initialZone;
            zoneSweepReport.perimeterDirection =
                partition.perimeterDirection;
            zoneSweepReport.longitudinalDirection =
                partition.longitudinalDirection;
            zoneSweepReport.partitionMinimumX = partition.minimumX;
            zoneSweepReport.partitionMaximumX = partition.maximumX;
            zoneSweepReport.active = true;
            if (!zoneSweepState.active)
            {
                finishedPartitions[partitionIndex] = true;
                zoneSweepDiagnostics.push_back
                    (zone16SweepDiagnostic(context, zoneSweepReport));
                zoneSweepState = Zone16SweepState{};
                zoneSweepReport = Zone16SweepReport{};
            }
            return true;
        };
        const auto zoneBlockedFailure =
            [&](const TubeZoneSweepPartition& partition,
                machining::TubeZone16 zone)
                -> OperationResult<ProcessPlan>
        {
            QStringList ownedUnitKeys;
            QStringList unfinishedUnitKeys;
            QStringList eligibleUnitKeys;
            QStringList blockedUnitKeys;
            QStringList remainingPredecessors;
            const auto& bucket =
                partition.zoneBuckets[machining::tubeZoneIndex(zone)];
            for (const int groupId : bucket)
            {
                const QString unitKey = processGroupKeyText
                    (plan.groups[static_cast<std::size_t>(groupId)]);
                ownedUnitKeys.push_back(unitKey);
                if (scheduled.find(groupId) != scheduled.end())
                    continue;
                unfinishedUnitKeys.push_back(unitKey);
                if (indegree[groupId] == 0)
                    eligibleUnitKeys.push_back(unitKey);
                else
                    blockedUnitKeys.push_back(unitKey);
                remainingPredecessors.push_back
                    (QStringLiteral("%1:%2").arg(unitKey)
                        .arg(indegree[groupId]));
            }
            QVariantMap values = diagnosticValues(input, policy);
            values.insert(QStringLiteral("zoneBlocked"), true);
            values.insert(QStringLiteral("partitionId"),
                partition.partitionId);
            values.insert(QStringLiteral("zone"),
                machining::tubeZoneName(zone));
            values.insert(QStringLiteral("frontierX"),
                zoneSweepState.frontierX);
            values.insert(QStringLiteral("ownedUnitKeys"),
                ownedUnitKeys.join(QLatin1Char(',')));
            values.insert(QStringLiteral("unfinishedUnitKeys"),
                unfinishedUnitKeys.join(QLatin1Char(',')));
            values.insert(QStringLiteral("eligibleUnitKeys"),
                eligibleUnitKeys.join(QLatin1Char(',')));
            values.insert(QStringLiteral("blockedUnitKeys"),
                blockedUnitKeys.join(QLatin1Char(',')));
            values.insert(QStringLiteral("remainingPredecessors"),
                remainingPredecessors.join(QLatin1Char(',')));
            OperationResult<ProcessPlan> blocked = failure<ProcessPlan>
            (
                OperationStatus::Failed, context,
                DiagnosticCode::ProcessPlanningZoneBlockedByPrecedence,
                QStringLiteral("当前区位仍有未完成加工单元，但其前置约束尚未满足，不能提前切换区位。"),
                QStringLiteral("The current owner-zone bucket is unfinished and has no eligible unit."),
                values
            );
            blocked.mergeDiagnostics(zoneSweepDiagnostics);
            return blocked;
        };
        while (scheduled.size() < schedulable.size())
        {
            std::vector<int> eligible;
            for (const int groupId : schedulable)
                if (scheduled.find(groupId) == scheduled.end() && indegree[groupId] == 0) eligible.push_back(groupId);
            std::sort(eligible.begin(), eligible.end());
            if (eligible.empty())
            {
                if (zone16SweepEnabled && zoneSweepState.active)
                {
                    const TubeZoneSweepPartition& partition =
                        zoneSweepPartitions[static_cast<std::size_t>
                            (zoneSweepState.partitionId)];
                    const machining::TubeZone16 zone = zoneAtOffset
                    (
                        zoneSweepState.initialZone,
                        zoneSweepState.currentZoneOffset,
                        partition.perimeterDirection
                    );
                    if (!zoneCompleted(partition, zone, scheduled))
                        return zoneBlockedFailure(partition, zone);
                }
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

            if (zone16SweepEnabled && zoneSweepState.active)
            {
                const TubeZoneSweepPartition& partition =
                    zoneSweepPartitions[static_cast<std::size_t>
                        (zoneSweepState.partitionId)];
                const bool complete = std::all_of
                (
                    partition.groupIds.cbegin(), partition.groupIds.cend(),
                    [&scheduled](int groupId)
                    { return scheduled.find(groupId) != scheduled.end(); }
                );
                if (complete)
                {
                    if (zoneSweepReport.processedUnitCount
                        != static_cast<int>(partition.groupIds.size()))
                    {
                        QVariantMap values = diagnosticValues
                            (input, policy, 0U, 0U, -1, -1);
                        values.insert(QStringLiteral("partitionId"),
                            partition.partitionId);
                        values.insert(QStringLiteral("processedUnitCount"),
                            zoneSweepReport.processedUnitCount);
                        values.insert(QStringLiteral("expectedUnitCount"),
                            static_cast<int>(partition.groupIds.size()));
                        return failure<ProcessPlan>
                        (
                            OperationStatus::InternalError, context,
                            DiagnosticCode::ProcessPlanningInvariantViolation,
                            QStringLiteral("16 区位加工段计数与实际加工单元不一致。"),
                            QStringLiteral("Zone sweep report did not account for every partition group exactly once."),
                            values
                        );
                    }
                    if (!finishZoneSweepPartition())
                        return std::move(*zoneSweepLifecycleFailure);
                }
            }

            const bool hasEligibleBreak = std::any_of
            (
                eligible.cbegin(), eligible.cend(),
                [&plan](int groupId)
                {
                    return plan.groups[static_cast<std::size_t>(groupId)].kind
                        == ProcessGroupKind::BreakBoundary;
                }
            );
            if (zone16SweepEnabled && !zoneSweepState.active
                && !hasEligibleBreak)
            {
                for (const TubeZoneSweepPartition& partition :
                    zoneSweepPartitions)
                {
                    const bool hasEligibleUnit = std::any_of
                    (
                        eligible.cbegin(), eligible.cend(),
                        [&partition](int groupId)
                        {
                            return partition.groupIds.find(groupId)
                                != partition.groupIds.end();
                        }
                    );
                    if (hasEligibleUnit)
                    {
                        if (!startZoneSweepPartition
                            (partition.partitionId, currentSweepZone))
                        {
                            if (zoneSweepLifecycleFailure.has_value())
                                return std::move(*zoneSweepLifecycleFailure);
                            return failure<ProcessPlan>
                            (
                                OperationStatus::Failed, context,
                                DiagnosticCode::ProcessPlanningZoneSweepProfileInvalid,
                                QStringLiteral("无法以当前工艺区位启动 16 区位扫描。"),
                                QStringLiteral("The eligible partition rejected the current sweep zone."),
                                diagnosticValues(input, policy)
                            );
                        }
                        break;
                    }
                }
            }

            const bool initialSelection = scheduled.empty();
            const ProcessOrderingStrategy selectionStrategy =
                policy.sortIntent == ProcessSortIntent::RebuildSequence
                    && policy.orderingStrategy
                        == ProcessOrderingStrategy::LazyRotation
                ? ProcessOrderingStrategy::LazyRotation
                : ProcessOrderingStrategy::NearestNext;
            std::vector<int> candidateGroupIds = eligible;
            std::unordered_map<int, ZoneSweepSelection> zoneSelections;
            if (zone16SweepEnabled && zoneSweepState.active)
            {
                candidateGroupIds.clear();
                const TubeZoneSweepPartition& partition =
                    zoneSweepPartitions[static_cast<std::size_t>
                        (zoneSweepState.partitionId)];
                const std::unordered_set<int> eligibleSet
                    (eligible.cbegin(), eligible.cend());
                while (zoneSweepState.currentZoneOffset
                    < static_cast<int>(machining::kTubeZone16Count))
                {
                    const machining::TubeZone16 zone = zoneAtOffset
                        (zoneSweepState.initialZone,
                            zoneSweepState.currentZoneOffset,
                            partition.perimeterDirection);
                    const std::size_t zoneIndex =
                        machining::tubeZoneIndex(zone);
                    if (!zoneSweepState.zoneEntered)
                    {
                        if (zoneSweepState.enteredZones[zoneIndex])
                        {
                            QVariantMap values = diagnosticValues
                                (input, policy);
                            values.insert(QStringLiteral("partitionId"),
                                zoneSweepState.partitionId);
                            values.insert(QStringLiteral("zone"),
                                machining::tubeZoneName(zone));
                            values.insert(QStringLiteral("currentZoneOffset"),
                                zoneSweepState.currentZoneOffset);
                            return failure<ProcessPlan>
                            (
                                OperationStatus::InternalError, context,
                                DiagnosticCode::ProcessPlanningZoneReentered,
                                QStringLiteral("16 区位加工阶段被重复进入。"),
                                QStringLiteral("A completed or previously entered zone cannot be entered again."),
                                values
                            );
                        }
                        zoneSweepState.frontierX =
                            zoneSweepState.longitudinalDirection >= 0
                            ? partition.minimumX : partition.maximumX;
                        zoneSweepState.zoneEntered = true;
                        zoneSweepState.enteredZones[zoneIndex] = true;
                        zoneSweepDiagnostics.push_back(zonePhaseDiagnostic
                        (
                            context, partition, zone, scheduled,
                            QStringLiteral("Enter")
                        ));
                    }

                    if (zoneCompleted(partition, zone, scheduled))
                    {
                        if (!zoneSweepState.zoneEntered
                            || zoneSweepState.completedZones[zoneIndex])
                        {
                            QVariantMap values = diagnosticValues
                                (input, policy);
                            values.insert(QStringLiteral("partitionId"),
                                zoneSweepState.partitionId);
                            values.insert(QStringLiteral("zone"),
                                machining::tubeZoneName(zone));
                            return failure<ProcessPlan>
                            (
                                OperationStatus::InternalError, context,
                                DiagnosticCode::ProcessPlanningZoneReentered,
                                QStringLiteral("16 区位加工阶段完成状态重复。"),
                                QStringLiteral("A zone phase may complete only once after entering."),
                                values
                            );
                        }
                        zoneSweepState.completedZones[zoneIndex] = true;
                        zoneSweepDiagnostics.push_back(zonePhaseDiagnostic
                        (
                            context, partition, zone, scheduled,
                            QStringLiteral("Complete")
                        ));
                        ++zoneSweepState.currentZoneOffset;
                        ++zoneSweepReport.zoneTransitionCount;
                        zoneSweepState.zoneEntered = false;
                        continue;
                    }

                    std::vector<ZoneSweepSelection> available;
                    for (const int groupId :
                        partition.zoneBuckets[zoneIndex])
                    {
                        if (scheduled.find(groupId) != scheduled.end()
                            || eligibleSet.find(groupId) == eligibleSet.end())
                            continue;
                        const ProcessGroupZoneProfile& profile =
                            groupZoneProfiles.at(groupId);
                        const machining::TubeZoneSpan& span =
                            profile.zoneSpans[zoneIndex];
                        const bool behind =
                            zoneSweepState.longitudinalDirection >= 0
                            ? span.maximumX < zoneSweepState.frontierX
                                - zoneProjectionTolerance
                            : span.minimumX > zoneSweepState.frontierX
                                + zoneProjectionTolerance;
                        if (behind)
                        {
                            QVariantMap values = diagnosticValues
                                (input, policy, 0U, 0U, -1, groupId);
                            values.insert(QStringLiteral("partitionId"),
                                zoneSweepState.partitionId);
                            values.insert(QStringLiteral("zone"),
                                machining::tubeZoneName(zone));
                            values.insert(QStringLiteral("unitKey"),
                                processGroupKeyText(plan.groups
                                    [static_cast<std::size_t>(groupId)]));
                            values.insert(QStringLiteral("spanMinimumX"),
                                span.minimumX);
                            values.insert(QStringLiteral("spanMaximumX"),
                                span.maximumX);
                            values.insert(QStringLiteral("frontierX"),
                                zoneSweepState.frontierX);
                            values.insert(QStringLiteral("remainingPredecessors"),
                                indegree[groupId]);
                            return failure<ProcessPlan>
                            (
                                OperationStatus::Failed, context,
                                DiagnosticCode::ProcessPlanningZoneSweepBacktrackRequired,
                                QStringLiteral("16 区位扫描发现必须回头的加工单元，已停止排序。"),
                                QStringLiteral("An eligible unit lies completely behind the monotonic zone frontier."),
                                values
                            );
                        }

                        ZoneSweepSelection selection;
                        selection.groupId = groupId;
                        selection.zone = zone;
                        selection.span = span;
                        selection.hitX =
                            zoneSweepState.longitudinalDirection >= 0
                            ? span.minimumX : span.maximumX;
                        selection.frontierBefore =
                            zoneSweepState.frontierX;
                        selection.fallbackOwner = partition.ownerships.at
                            (groupId).usedPossibleFallback;
                        available.push_back(selection);
                    }

                    if (!available.empty())
                    {
                        std::sort(available.begin(), available.end(),
                            [&](const ZoneSweepSelection& left,
                                const ZoneSweepSelection& right)
                        {
                            if (std::abs(left.hitX - right.hitX)
                                > zoneProjectionTolerance)
                            {
                                return zoneSweepState.longitudinalDirection >= 0
                                    ? left.hitX < right.hitX
                                    : left.hitX > right.hitX;
                            }
                            return processGroupStableLess
                            (
                                plan.groups[static_cast<std::size_t>(left.groupId)],
                                plan.groups[static_cast<std::size_t>(right.groupId)]
                            );
                        });
                        const ZoneSweepSelection& selection =
                            available.front();
                        candidateGroupIds.push_back(selection.groupId);
                        zoneSelections.emplace
                            (selection.groupId, selection);
                        break;
                    }

                    return zoneBlockedFailure(partition, zone);
                }

                if (candidateGroupIds.empty())
                {
                    QVariantMap values = diagnosticValues(input, policy);
                    values.insert(QStringLiteral("partitionId"),
                        zoneSweepState.partitionId);
                    values.insert(QStringLiteral("currentZoneOffset"),
                        zoneSweepState.currentZoneOffset);
                    return failure<ProcessPlan>
                    (
                        OperationStatus::InternalError, context,
                        DiagnosticCode::ProcessPlanningZoneIncomplete,
                        QStringLiteral("16 区位加工段未生成下一加工单元。"),
                        QStringLiteral("The sweep exhausted its zone phases without completing the active partition."),
                        values
                    );
                }
            }

            std::vector<SchedulingCandidate> candidates;
            candidates.reserve(candidateGroupIds.size());
            for (const int groupId : candidateGroupIds)
            {
                const ProcessGroup& group = plan.groups[static_cast<std::size_t>(groupId)];
                ClosedLoopTraversalReport candidateClosedLoopReport;
                std::optional<GroupTraversal> candidate;
                std::optional<BreakBoundaryTraversalReport>
                    candidateBreakReport;
                if (group.kind == ProcessGroupKind::BreakBoundary)
                {
                    const auto boundaryRank =
                        boundaryRankByGroup.find(group.groupId);
                    if (!surfaceSweepSection.has_value()
                        || boundaryRank == boundaryRankByGroup.end())
                    {
                        QVariantMap values = diagnosticValues
                            (input, policy, 0U, 0U, -1, groupId);
                        values.insert(QStringLiteral("boundaryRank"),
                            boundaryRank == boundaryRankByGroup.end()
                                ? -1 : boundaryRank->second);
                        return failure<ProcessPlan>
                        (
                            OperationStatus::Failed, context,
                            DiagnosticCode::ProcessPlanningBreakMidpointCandidateMissing,
                            QStringLiteral("加工断面缺少有效截面或空间排序，无法选择强区位中点。"),
                            QStringLiteral("Break midpoint selection requires a prepared TubeSectionModel and boundary rank."),
                            values
                        );
                    }
                    auto breakTraversal =
                        ClosedLoopZoneRunBuilder::buildBreak
                        (
                            group, entities, currentPosition, policy,
                            *surfaceSweepSection, input.tubeSectionCenter,
                            selectionStrategy, zoneProjectionTolerance,
                            boundaryRank->second,
                            currentSweepZone
                        );
                    candidateClosedLoopReport =
                        std::move(breakTraversal.closedLoopReport);
                    candidateBreakReport =
                        std::move(breakTraversal.report);
                    candidate =
                        std::move(breakTraversal.traversal);
                }
                else
                {
                    std::optional<TraversalSelectionContext>
                        traversalSelection;
                    const auto zoneSelection =
                        zoneSelections.find(groupId);
                    if (zoneSelection != zoneSelections.end())
                    {
                        traversalSelection.emplace();
                        traversalSelection->requiredEntryZone =
                            zoneSelection->second.zone;
                        traversalSelection->longitudinalDirection =
                            zoneSweepState.longitudinalDirection;
                        traversalSelection->zoneHitX =
                            zoneSelection->second.hitX;
                        traversalSelection->frontierX =
                            zoneSelection->second.frontierBefore;
                        traversalSelection->projectionTolerance =
                            zoneProjectionTolerance;
                        traversalSelection->previousCutEnd =
                            currentPosition;
                        machine::ToolTransferPolicy transferPolicy;
                        transferPolicy.rotationSafetyClearance =
                            policy.rotationSafetyClearance;
                        transferPolicy.sameZoneTransferClearance =
                            policy.sameZoneTransferClearance;
                        transferPolicy.coordinatedTransferEnabled =
                            policy.coordinatedTransferEnabled;
                        traversalSelection->previousTransferAnchor =
                            initialSelection
                            ? currentPosition
                            : machine::RotaryKinematics
                                ::sourceTransferAnchor
                                (
                                    currentPosition,
                                    previousTransferZone,
                                    zoneSelection->second.zone,
                                    transferPolicy,
                                    input.tubeSection,
                                    input.tubeSectionCenter.has_value()
                                        ? input.tubeSectionCenter->x
                                        : surfaceSweepSection
                                            ->geometry.centerY,
                                    input.tubeSectionCenter.has_value()
                                        ? input.tubeSectionCenter->y
                                        : surfaceSweepSection
                                            ->geometry.centerZ,
                                    zoneProjectionTolerance
                                );
                        traversalSelection->previousEnd =
                            traversalSelection->previousTransferAnchor;
                        traversalSelection->hardZoneConstraint = true;
                        const ProcessGroupZoneProfile& profile =
                            groupZoneProfiles.at(groupId);
                        traversalSelection
                            ->allowZoneRunMidpointFallback =
                            (profile.zoneRunMidpointEntryMask
                                & machining::tubeZoneBit
                                    (zoneSelection->second.zone)) != 0U;
                    }
                    candidate = bestTraversal
                        (group, entities, currentPosition, policy,
                            input.tubeSection, input.tubeSectionCenter,
                            selectionStrategy,
                            &candidateClosedLoopReport,
                            traversalSelection.has_value()
                                ? &*traversalSelection : nullptr);
                }
                if (!candidate.has_value())
                {
                    const bool manualEntryConstraint =
                        hasManualEntryConstraint(group, entities);
                    const bool hardZoneConstraint =
                        zoneSelections.find(groupId)
                            != zoneSelections.end();
                    QVariantMap values = diagnosticValues
                    (
                        input, policy, 0U, 0U, -1, groupId, -1, -1,
                        static_cast<int>(schedulable.size() - scheduled.size()),
                        static_cast<int>(eligible.size()), static_cast<int>(plan.groups.size()),
                        static_cast<int>(plan.assignments.size()),
                        static_cast<int>(plan.exclusions.size()), -1, -1, initialSelection,
                        nullptr, group.kind
                    );
                    values.insert(QStringLiteral("manualEntryConstraint"),
                        manualEntryConstraint);
                    values.insert(QStringLiteral("hardZoneConstraint"),
                        hardZoneConstraint);
                    const auto requiredZone =
                        zoneSelections.find(groupId);
                    if (requiredZone != zoneSelections.end())
                    {
                        values.insert(QStringLiteral("requiredEntryZone"),
                            machining::tubeZoneName
                                (requiredZone->second.zone));
                    }
                    if (candidateClosedLoopReport.groupId >= 0)
                    {
                        const QVariantMap closedLoopValues =
                            closedLoopDiagnosticValues(candidateClosedLoopReport);
                        for (auto iterator = closedLoopValues.cbegin();
                            iterator != closedLoopValues.cend(); ++iterator)
                            values.insert(iterator.key(), iterator.value());
                    }
                    if (candidateBreakReport.has_value())
                    {
                        const QVariantMap breakValues =
                            breakDiagnosticValues(*candidateBreakReport);
                        for (auto iterator = breakValues.cbegin();
                            iterator != breakValues.cend(); ++iterator)
                        {
                            values.insert(iterator.key(),
                                iterator.value());
                        }
                    }
                    return failure<ProcessPlan>
                    (
                        manualEntryConstraint && hardZoneConstraint
                            ? OperationStatus::Conflict
                            : OperationStatus::Failed, context,
                        candidateBreakReport.has_value()
                            ? candidateBreakReport->failureCode
                            : candidateClosedLoopReport.groupId >= 0
                            && !candidateClosedLoopReport.simpleLoopValid
                            ? DiagnosticCode::ProcessPlanningGroupBuildFailed
                            : DiagnosticCode::ProcessPlanningDirectionFailed,
                        manualEntryConstraint && hardZoneConstraint
                            ? QStringLiteral("人工起点或方向无法在当前扫描区位形成合法入口。")
                            : candidateBreakReport.has_value()
                            ? QStringLiteral("加工断面无法从可靠强区位边段中点建立闭环遍历。")
                            : candidateClosedLoopReport.groupId >= 0
                            && !candidateClosedLoopReport.simpleLoopValid
                            ? QStringLiteral("多图元闭合加工单元不是唯一简单环，无法生成加工计划。")
                            : QStringLiteral("连续加工组无法建立有效入口和加工方向。"),
                        manualEntryConstraint && hardZoneConstraint
                            ? QStringLiteral("Manual entry constraints conflict with requiredEntryZone.")
                            : candidateBreakReport.has_value()
                            ? candidateBreakReport->failureReason
                            : candidateClosedLoopReport.groupId >= 0
                            ? candidateClosedLoopReport.failureReason
                            : QStringLiteral("No connected traversal covers every group entity."),
                        values
                    );
                }
                SchedulingCandidate schedulingCandidate;
                schedulingCandidate.traversal = std::move(*candidate);
                if (candidateClosedLoopReport.groupId >= 0)
                    schedulingCandidate.closedLoopReport =
                        std::move(candidateClosedLoopReport);
                if (candidateBreakReport.has_value())
                    schedulingCandidate.breakReport =
                        std::move(candidateBreakReport);
                candidates.push_back(std::move(schedulingCandidate));
            }
            if (candidates.empty())
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

            std::size_t selectedIndex = 0U;
            for (std::size_t candidateIndex = 1U;
                candidateIndex < candidates.size(); ++candidateIndex)
            {
                bool replace = false;
                const auto leftZone = zoneSelections.find
                    (candidates[candidateIndex].traversal.groupId);
                const auto rightZone = zoneSelections.find
                    (candidates[selectedIndex].traversal.groupId);
                if (leftZone != zoneSelections.end()
                    && rightZone != zoneSelections.end())
                {
                    const ZoneSweepSelection& left = leftZone->second;
                    const ZoneSweepSelection& right = rightZone->second;
                    if (std::abs(left.hitX - right.hitX)
                        > zoneProjectionTolerance)
                    {
                        replace = zoneSweepState.longitudinalDirection >= 0
                            ? left.hitX < right.hitX
                            : left.hitX > right.hitX;
                    }
                    else
                    {
                        const auto intersectsFrontier =
                            [&](const ZoneSweepSelection& selection)
                        {
                            return selection.span.minimumX
                                    <= selection.frontierBefore
                                        + zoneProjectionTolerance
                                && selection.span.maximumX
                                    >= selection.frontierBefore
                                        - zoneProjectionTolerance;
                        };
                        const bool leftIntersects = intersectsFrontier(left);
                        const bool rightIntersects = intersectsFrontier(right);
                        if (leftIntersects != rightIntersects)
                        {
                            replace = leftIntersects;
                        }
                        else
                        {
                            replace = processGroupStableLess
                            (
                                plan.groups[static_cast<std::size_t>
                                    (candidates[candidateIndex]
                                        .traversal.groupId)],
                                plan.groups[static_cast<std::size_t>
                                    (candidates[selectedIndex]
                                        .traversal.groupId)]
                            );
                        }
                    }
                }
                else
                {
                    replace = traversalLess(candidates[candidateIndex].traversal,
                        candidates[selectedIndex].traversal, selectionStrategy);
                }
                if (replace) selectedIndex = candidateIndex;
            }
            SchedulingCandidate selectedCandidate =
                std::move(candidates[selectedIndex]);
            GroupTraversal& selected = selectedCandidate.traversal;
            std::optional<ClosedLoopTraversalReport>& selectedClosedLoopReport =
                selectedCandidate.closedLoopReport;
            std::optional<BreakBoundaryTraversalReport>& selectedBreakReport =
                selectedCandidate.breakReport;

            const ProcessGroup& selectedGroup = plan.groups[static_cast<std::size_t>(selected.groupId)];
            const auto scheduledZoneSelection =
                zoneSelections.find(selected.groupId);
            if (zone16SweepEnabled
                && selectedGroup.kind != ProcessGroupKind::BreakBoundary
                && scheduledZoneSelection == zoneSelections.end())
            {
                QVariantMap values = diagnosticValues
                    (input, policy, 0U, 0U, -1, selected.groupId);
                values.insert(QStringLiteral("unitKey"),
                    processGroupKeyText(selectedGroup));
                values.insert(QStringLiteral("activePartitionId"),
                    zoneSweepState.partitionId);
                return failure<ProcessPlan>
                (
                    OperationStatus::InternalError, context,
                    DiagnosticCode::ProcessPlanningInvariantViolation,
                    QStringLiteral("普通加工单元未经过唯一生产区位桶调度。"),
                    QStringLiteral("Every ordinary group in a Zone16 sweep must be selected from its owner bucket."),
                    values
                );
            }
            if (scheduledZoneSelection != zoneSelections.end())
            {
                if (!selected.selectedEntry.has_value()
                    || selected.selectedEntry->zone
                        != scheduledZoneSelection->second.zone)
                {
                    QVariantMap values = diagnosticValues
                        (input, policy, 0U, 0U, -1,
                            selected.groupId);
                    values.insert(QStringLiteral("unitKey"),
                        processGroupKeyText(selectedGroup));
                    values.insert(QStringLiteral("scheduledZone"),
                        machining::tubeZoneName
                            (scheduledZoneSelection->second.zone));
                    values.insert(QStringLiteral("selectedEntryZone"),
                        selected.selectedEntry.has_value()
                        ? machining::tubeZoneName
                            (selected.selectedEntry->zone)
                        : QStringLiteral("None"));
                    return failure<ProcessPlan>
                    (
                        OperationStatus::Failed, context,
                        DiagnosticCode::
                            ProcessPlanningInvariantViolation,
                        QStringLiteral("加工单元实际起刀区位与当前扫描区位不一致。"),
                        QStringLiteral("scheduledZone must equal selectedEntryZone before plan publication."),
                        values
                    );
                }
            }
            const bool continuous = selectedGroup.kind == ProcessGroupKind::ConnectedChain
                || selectedGroup.kind == ProcessGroupKind::ClosedLoop
                || selectedGroup.kind == ProcessGroupKind::BreakBoundary;
            const int continuousGroupId = continuous ? selectedGroup.groupId : -1;
            ProcessUnit processUnit;
            processUnit.key.memberEntityIds = selectedGroup.entityIds;
            std::sort(processUnit.key.memberEntityIds.begin(), processUnit.key.memberEntityIds.end());
            processUnit.closed = selectedGroup.closed;
            processUnit.ownerZone =
                scheduledZoneSelection != zoneSelections.end()
                ? std::optional<machining::TubeZone16>
                    (scheduledZoneSelection->second.zone)
                : selected.selectedEntry.has_value()
                    ? std::optional<machining::TubeZone16>
                        (selected.selectedEntry->zone)
                    : std::nullopt;
            processUnit.orderedMemberEntityIds.reserve(selected.entities.size());
            for (const DirectedEntity& directed : selected.entities)
                processUnit.orderedMemberEntityIds.push_back(directed.entity->entityId);
            const int processUnitIndex = static_cast<int>(plan.processUnits.size());
            plan.processUnits.push_back(processUnit);
            plan.processUnitSequence.units.push_back(processUnit.key);
            for (const DirectedEntity& directed : selected.entities)
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
            const std::vector<ProcessPathFragment>* selectedFragments =
                selectedBreakReport.has_value()
                ? &selectedBreakReport->fragments
                : &selected.fragments;
            if (selectedFragments != nullptr
                && !selectedFragments->empty())
            {
                for (ProcessPathFragment fragment :
                    *selectedFragments)
                {
                    fragment.processUnitIndex = processUnitIndex;
                    plan.plannedFragments.push_back(std::move(fragment));
                }
            }
            currentPosition = selected.end;
            previousTransferZone = processUnit.ownerZone;
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
            if (policy.sortIntent == ProcessSortIntent::RebuildSequence
                && policy.orderingStrategy
                    == ProcessOrderingStrategy::LazyRotation
                && selectedGroup.kind != ProcessGroupKind::BreakBoundary)
            {
                closedLoopDiagnostics.push_back
                    (entrySelectionDiagnostic
                    (
                        context, selectedGroup, selected,
                        scheduledZoneSelection != zoneSelections.end()
                        ? std::optional<machining::TubeZone16>
                            (scheduledZoneSelection->second.zone)
                        : std::nullopt
                    ));
            }
            if (!scheduled.insert(selected.groupId).second)
            {
                return failure<ProcessPlan>
                (
                    OperationStatus::InternalError, context,
                    DiagnosticCode::ProcessPlanningInvariantViolation,
                    QStringLiteral("加工单元被重复加入加工计划。"),
                    QStringLiteral("A ProcessGroup may be scheduled exactly once."),
                    diagnosticValues(input, policy, 0U, 0U, -1,
                        selected.groupId)
                );
            }
            ++schedulingOccurrences[selected.groupId];
            for (const int successor : successors[selected.groupId]) --indegree[successor];

            if (zone16SweepEnabled && zoneSweepState.active
                && selectedGroup.kind != ProcessGroupKind::BreakBoundary)
            {
                const TubeZoneSweepPartition& partition =
                    zoneSweepPartitions[static_cast<std::size_t>
                        (zoneSweepState.partitionId)];
                const machining::TubeZone16 zone = zoneAtOffset
                (
                    zoneSweepState.initialZone,
                    zoneSweepState.currentZoneOffset,
                    partition.perimeterDirection
                );
                const std::size_t zoneIndex =
                    machining::tubeZoneIndex(zone);
                if (zoneCompleted(partition, zone, scheduled))
                {
                    if (!zoneSweepState.zoneEntered
                        || !zoneSweepState.enteredZones[zoneIndex]
                        || zoneSweepState.completedZones[zoneIndex])
                    {
                        QVariantMap values = diagnosticValues
                            (input, policy, 0U, 0U, -1,
                                selected.groupId);
                        values.insert(QStringLiteral("partitionId"),
                            zoneSweepState.partitionId);
                        values.insert(QStringLiteral("zone"),
                            machining::tubeZoneName(zone));
                        return failure<ProcessPlan>
                        (
                            OperationStatus::InternalError, context,
                            DiagnosticCode::ProcessPlanningZoneIncomplete,
                            QStringLiteral("16 区位加工阶段完成状态无效。"),
                            QStringLiteral("The owner-zone bucket completed outside a valid entered phase."),
                            values
                        );
                    }
                    zoneSweepState.completedZones[zoneIndex] = true;
                    zoneSweepDiagnostics.push_back(zonePhaseDiagnostic
                    (
                        context, partition, zone, scheduled,
                        QStringLiteral("Complete")
                    ));
                    ++zoneSweepState.currentZoneOffset;
                    ++zoneSweepReport.zoneTransitionCount;
                    zoneSweepState.zoneEntered = false;
                }
            }

            if (zone16SweepEnabled)
            {
                if (selectedGroup.kind == ProcessGroupKind::BreakBoundary)
                {
                    const auto partition = partitionAfterBreakGroup.find
                        (selected.groupId);
                    if (!selectedBreakReport.has_value()
                        || !selectedBreakReport->exitZone.has_value())
                    {
                        QVariantMap values = diagnosticValues
                            (input, policy, 0U, 0U, -1, selected.groupId);
                        values.insert(QStringLiteral("unitKey"),
                            processGroupKeyText(selectedGroup));
                        if (selectedBreakReport.has_value())
                        {
                            const QVariantMap breakValues =
                                breakDiagnosticValues(*selectedBreakReport);
                            for (auto iterator = breakValues.cbegin();
                                iterator != breakValues.cend(); ++iterator)
                            {
                                values.insert(iterator.key(),
                                    iterator.value());
                            }
                        }
                        return failure<ProcessPlan>
                        (
                            OperationStatus::Failed, context,
                            DiagnosticCode::ProcessPlanningBreakExitZoneUnresolved,
                            QStringLiteral("加工断面的最终计划片段无法解析可靠出口区位。"),
                            QStringLiteral("Break exit zone was not resolved from the final planned fragment."),
                            values
                        );
                    }
                    currentSweepZone =
                        *selectedBreakReport->exitZone;
                    if (partition == partitionAfterBreakGroup.end())
                    {
                        QVariantMap values =
                            breakDiagnosticValues(*selectedBreakReport);
                        values.insert(QStringLiteral("nextPartitionId"), -1);
                        values.insert(QStringLiteral("partitionMappingFound"),
                            false);
                        values.insert(QStringLiteral("partitionStartSucceeded"),
                            false);
                        return failure<ProcessPlan>
                        (
                            OperationStatus::Failed, context,
                            DiagnosticCode::ProcessPlanningBreakPartitionMappingMissing,
                            QStringLiteral("加工断面缺少下一加工段的分区映射。"),
                            QStringLiteral("No zone-sweep partition is mapped after the selected Break boundary."),
                            values
                        );
                    }
                    if (!startZoneSweepPartition
                        (partition->second,
                            *selectedBreakReport->exitZone))
                    {
                        if (zoneSweepLifecycleFailure.has_value())
                            return std::move(*zoneSweepLifecycleFailure);
                        QVariantMap values =
                            breakDiagnosticValues(*selectedBreakReport);
                        values.insert(QStringLiteral("nextPartitionId"),
                            partition->second);
                        values.insert(QStringLiteral("partitionMappingFound"),
                            true);
                        values.insert(QStringLiteral("partitionStartSucceeded"),
                            false);
                        return failure<ProcessPlan>
                        (
                            OperationStatus::Failed, context,
                            DiagnosticCode::ProcessPlanningBreakPartitionStartFailed,
                            QStringLiteral("加工断面出口区位无法启动下一加工段。"),
                            QStringLiteral("The mapped partition rejected the resolved Break exit zone."),
                            values
                        );
                    }
                    selectedBreakReport->nextPartitionId =
                        partition->second;
                    selectedBreakReport->partitionMappingFound = true;
                    selectedBreakReport->partitionStartSucceeded = true;
                }
                else
                {
                    const auto activeSelection =
                        zoneSelections.find(selected.groupId);
                    if (!zoneSweepState.active)
                    {
                        if (!startZoneSweepPartition
                            (0, currentSweepZone))
                        {
                            if (zoneSweepLifecycleFailure.has_value())
                                return std::move(*zoneSweepLifecycleFailure);
                            return failure<ProcessPlan>
                            (
                                OperationStatus::Failed, context,
                                DiagnosticCode::ProcessPlanningZoneSweepProfileInvalid,
                                QStringLiteral("首个加工单元无法初始化 16 区位扫描。"),
                                QStringLiteral("The configured initial sweep zone could not start partition zero."),
                                diagnosticValues(input, policy, 0U, 0U, -1,
                                    selected.groupId)
                            );
                        }
                    }

                    ZoneSweepSelection selection;
                    bool selectionAvailable = false;
                    if (activeSelection != zoneSelections.end())
                    {
                        selection = activeSelection->second;
                        selectionAvailable = true;
                    }
                    else if (zoneSweepState.active)
                    {
                        const ProcessGroupZoneProfile& profile =
                            groupZoneProfiles.at(selected.groupId);
                        const TubeZoneSweepPartition& partition =
                            zoneSweepPartitions[static_cast<std::size_t>
                                (zoneSweepState.partitionId)];
                        const auto ownership =
                            partition.ownerships.find(selected.groupId);
                        if (ownership == partition.ownerships.end())
                        {
                            QVariantMap values = diagnosticValues
                                (input, policy, 0U, 0U, -1, selected.groupId);
                            values.insert(QStringLiteral("unitKey"),
                                processGroupKeyText(selectedGroup));
                            values.insert(QStringLiteral("partitionId"),
                                zoneSweepState.partitionId);
                            values.insert(QStringLiteral("certainMask"),
                                zoneMaskText(profile.certainMask));
                            values.insert(QStringLiteral("possibleMask"),
                                zoneMaskText(profile.possibleMask));
                            values.insert(QStringLiteral("legalEntryMask"),
                                zoneMaskText(profile.legalEntryMask));
                            return failure<ProcessPlan>
                            (
                                OperationStatus::Failed, context,
                                DiagnosticCode::ProcessPlanningInvariantViolation,
                                QStringLiteral("加工单元缺少唯一生产区位归属。"),
                                QStringLiteral("An ordinary selected group is absent from the partition ownership table."),
                                values
                            );
                        }
                        const machining::TubeZone16 zone =
                            ownership->second.ownerZone;
                        const machining::TubeZone16 activeZone = zoneAtOffset
                        (
                            zoneSweepState.initialZone,
                            zoneSweepState.currentZoneOffset,
                            partition.perimeterDirection
                        );
                        if (zone != activeZone
                            || !selected.selectedEntry.has_value()
                            || selected.selectedEntry->zone != zone)
                        {
                            QVariantMap values = diagnosticValues
                                (input, policy, 0U, 0U, -1,
                                    selected.groupId);
                            values.insert(QStringLiteral("partitionId"),
                                zoneSweepState.partitionId);
                            values.insert(QStringLiteral("ownerZone"),
                                machining::tubeZoneName(zone));
                            values.insert(QStringLiteral("activeZone"),
                                machining::tubeZoneName(activeZone));
                            values.insert(QStringLiteral("selectedEntryZone"),
                                selected.selectedEntry.has_value()
                                ? machining::tubeZoneName
                                    (selected.selectedEntry->zone)
                                : QStringLiteral("None"));
                            return failure<ProcessPlan>
                            (
                                OperationStatus::InternalError, context,
                                DiagnosticCode::ProcessPlanningInvariantViolation,
                                QStringLiteral("加工单元未从当前唯一生产区位进入。"),
                                QStringLiteral("Owner zone, active phase, and selected entry zone must match."),
                                values
                            );
                        }
                        selection.groupId = selected.groupId;
                        selection.zone = zone;
                        selection.span = profile.zoneSpans
                            [machining::tubeZoneIndex(zone)];
                        selection.hitX =
                            zoneSweepState.longitudinalDirection >= 0
                            ? selection.span.minimumX
                            : selection.span.maximumX;
                        selection.frontierBefore =
                            zoneSweepState.frontierX;
                        selection.fallbackOwner =
                            ownership->second.usedPossibleFallback
                            || ownership->second.usedBoundaryFallback;
                        selectionAvailable = true;
                    }

                    if (selectionAvailable && zoneSweepReport.active)
                    {
                        currentSweepZone = selection.zone;
                        const double frontierAfter =
                            zoneSweepState.longitudinalDirection >= 0
                            ? std::max(zoneSweepState.frontierX,
                                selection.span.maximumX)
                            : std::min(zoneSweepState.frontierX,
                                selection.span.minimumX);
                        const ProcessGroupZoneProfile& profile =
                            groupZoneProfiles.at(selected.groupId);
                        zoneSweepReport.selectedUnits.push_back
                        (
                            QStringLiteral("unitKey=%1|zone=%2|certainMask=%3|possibleMask=%4|span=[%5,%6]|hitX=%7|frontierBefore=%8|frontierAfter=%9|startX=%10|endX=%11|fallbackOwner=%12|order=%13")
                                .arg(processGroupKeyText(selectedGroup))
                                .arg(machining::tubeZoneName(selection.zone))
                                .arg(zoneMaskText(profile.certainMask))
                                .arg(zoneMaskText(profile.possibleMask))
                                .arg(selection.span.minimumX, 0, 'f', 6)
                                .arg(selection.span.maximumX, 0, 'f', 6)
                                .arg(selection.hitX, 0, 'f', 6)
                                .arg(selection.frontierBefore, 0, 'f', 6)
                                .arg(frontierAfter, 0, 'f', 6)
                                .arg(selected.start.x, 0, 'f', 6)
                                .arg(selected.end.x, 0, 'f', 6)
                                .arg(selection.fallbackOwner
                                    ? QStringLiteral("true")
                                    : QStringLiteral("false"))
                                .arg(processUnitIndex + 1)
                        );
                        zoneSweepState.frontierX = frontierAfter;
                        ++zoneSweepReport.processedUnitCount;
                    }
                }
            }
            if (selectedBreakReport.has_value())
            {
                closedLoopDiagnostics.push_back
                    (breakStartDiagnostic(context, *selectedBreakReport));
            }
        }

        if (zone16SweepEnabled && zoneSweepState.active)
        {
            const TubeZoneSweepPartition& partition =
                zoneSweepPartitions[static_cast<std::size_t>
                    (zoneSweepState.partitionId)];
            if (zoneSweepReport.processedUnitCount
                != static_cast<int>(partition.groupIds.size()))
            {
                return failure<ProcessPlan>
                (
                    OperationStatus::InternalError, context,
                    DiagnosticCode::ProcessPlanningInvariantViolation,
                    QStringLiteral("最终 16 区位加工段未完整记录全部加工单元。"),
                    QStringLiteral("Final zone sweep report count does not match its partition."),
                    diagnosticValues(input, policy)
                );
            }
        }
        if (zone16SweepEnabled && !finishZoneSweepPartition())
            return std::move(*zoneSweepLifecycleFailure);
        if (zone16SweepEnabled)
        {
            for (std::size_t partitionIndex = 0U;
                partitionIndex < zoneSweepPartitions.size();
                ++partitionIndex)
            {
                const TubeZoneSweepPartition& partition =
                    zoneSweepPartitions[partitionIndex];
                if (partition.groupIds.empty()) continue;
                std::unordered_map<int, int> bucketOccurrences;
                for (const auto& bucket : partition.zoneBuckets)
                {
                    for (const int groupId : bucket)
                        ++bucketOccurrences[groupId];
                }
                for (const int groupId : partition.groupIds)
                {
                    if (partition.ownerships.count(groupId) != 1U
                        || bucketOccurrences[groupId] != 1
                        || schedulingOccurrences[groupId] != 1)
                    {
                        QVariantMap values = diagnosticValues
                            (input, policy, 0U, 0U, -1, groupId);
                        values.insert(QStringLiteral("partitionId"),
                            static_cast<int>(partitionIndex));
                        values.insert(QStringLiteral("ownershipCount"),
                            static_cast<int>
                                (partition.ownerships.count(groupId)));
                        values.insert(QStringLiteral("bucketOccurrenceCount"),
                            bucketOccurrences[groupId]);
                        values.insert(QStringLiteral("scheduledOccurrenceCount"),
                            schedulingOccurrences[groupId]);
                        return failure<ProcessPlan>
                        (
                            OperationStatus::InternalError, context,
                            DiagnosticCode::ProcessPlanningInvariantViolation,
                            QStringLiteral("16 区位加工单元的归属或调度次数不唯一。"),
                            QStringLiteral("Every ordinary group must have one owner, one production bucket, and one scheduling occurrence."),
                            values
                        );
                    }
                }
                if (!startedPartitions[partitionIndex]
                    || !finishedPartitions[partitionIndex])
                {
                    QVariantMap values = diagnosticValues(input, policy);
                    values.insert(QStringLiteral("partitionId"),
                        static_cast<int>(partitionIndex));
                    values.insert(QStringLiteral("started"),
                        static_cast<bool>(startedPartitions[partitionIndex]));
                    values.insert(QStringLiteral("finished"),
                        static_cast<bool>(finishedPartitions[partitionIndex]));
                    return failure<ProcessPlan>
                    (
                        OperationStatus::InternalError, context,
                        DiagnosticCode::ProcessPlanningZoneIncomplete,
                        QStringLiteral("16 区位加工段生命周期未完整结束。"),
                        QStringLiteral("Every non-empty partition must start and finish exactly once."),
                        values
                    );
                }
            }
        }

        OperationReport entryRefinement =
            SingleClosedEntryRefiner::refine
            (plan, input, policy, context);
        if (!entryRefinement.succeeded())
        {
            OperationResult<ProcessPlan> result;
            result.status = entryRefinement.status;
            result.mergeDiagnostics(entryRefinement.diagnostics);
            return result;
        }
        closedLoopDiagnostics += entryRefinement.diagnostics;

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
        result.mergeDiagnostics(zoneSweepDiagnostics);
        return result;
    }
}
