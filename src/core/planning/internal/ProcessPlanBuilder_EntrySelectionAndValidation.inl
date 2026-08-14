        QString entryCandidateKindName(ZoneEntryCandidateKind kind)
        {
            switch (kind)
            {
            case ZoneEntryCandidateKind::OpenEndpoint:
                return QStringLiteral("OpenEndpoint");
            case ZoneEntryCandidateKind::ClosedCurveParameter:
                return QStringLiteral("ClosedCurveParameter");
            case ZoneEntryCandidateKind::ClosedLoopConnection:
                return QStringLiteral("ClosedLoopConnection");
            case ZoneEntryCandidateKind::ClosedLoopArcInterior:
                return QStringLiteral("ClosedLoopArcInterior");
            case ZoneEntryCandidateKind::ClosedLoopEllipseInterior:
                return QStringLiteral("ClosedLoopEllipseInterior");
            case ZoneEntryCandidateKind::ClosedLoopZoneRunMidpoint:
                return QStringLiteral("ClosedLoopZoneRunMidpoint");
            case ZoneEntryCandidateKind::BreakZoneMidpoint:
                return QStringLiteral("BreakZoneMidpoint");
            }
            return QStringLiteral("Unknown");
        }

        QString planningSourceKindName(geometry::SourceGeometryKind kind)
        {
            switch (kind)
            {
            case geometry::SourceGeometryKind::Line:
                return QStringLiteral("Line");
            case geometry::SourceGeometryKind::Arc:
                return QStringLiteral("Arc");
            case geometry::SourceGeometryKind::Circle:
                return QStringLiteral("Circle");
            case geometry::SourceGeometryKind::Ellipse:
                return QStringLiteral("Ellipse");
            case geometry::SourceGeometryKind::Polyline:
                return QStringLiteral("Polyline");
            case geometry::SourceGeometryKind::Spline:
                return QStringLiteral("Spline");
            case geometry::SourceGeometryKind::Point:
                return QStringLiteral("Point");
            case geometry::SourceGeometryKind::Unknown:
                return QStringLiteral("Unknown");
            }
            return QStringLiteral("Unknown");
        }

        QString vectorText(const Vector3d& value)
        {
            return QStringLiteral("%1,%2,%3")
                .arg(value.x, 0, 'g', 15)
                .arg(value.y, 0, 'g', 15)
                .arg(value.z, 0, 'g', 15);
        }

        Diagnostic entrySelectionDiagnostic
        (
            const OperationContext& context,
            const ProcessGroup& group,
            const GroupTraversal& traversal,
            std::optional<machining::TubeZone16> scheduledZone
        )
        {
            QVariantMap values;
            values.insert(QStringLiteral("entrySelectionSummary"), true);
            values.insert(QStringLiteral("entryRefinementSummary"),
                !traversal.entryRefinementMode.isEmpty());
            values.insert(QStringLiteral("unitKey"), processGroupKeyText(group));
            values.insert(QStringLiteral("groupKind"), groupKindName(group.kind));
            values.insert(QStringLiteral("candidateCount"),
                traversal.entryCandidateCount);
            values.insert(QStringLiteral("connectionCandidateCount"),
                traversal.connectionCandidateCount);
            values.insert(QStringLiteral("arcInteriorCandidateCount"),
                traversal.arcInteriorCandidateCount);
            values.insert(QStringLiteral("ellipseInteriorCandidateCount"),
                traversal.ellipseInteriorCandidateCount);
            values.insert(QStringLiteral("zoneRunMidpointCandidateCount"),
                traversal.zoneRunMidpointCandidateCount);
            values.insert(QStringLiteral("curveCandidateRejectedCount"),
                traversal.curveCandidateRejectedCount);
            values.insert(QStringLiteral("wrongZoneRejectedCount"),
                traversal.wrongZoneRejectedCount);
            values.insert(QStringLiteral("selectedStart"),
                vectorText(traversal.start));
            values.insert(QStringLiteral("selectedReverse"),
                !traversal.entities.empty()
                    && traversal.entities.front().reverseRelativeToInput);
            values.insert(QStringLiteral("axisReversalCount"),
                traversal.entryAxisReversalCount);
            values.insert(QStringLiteral("tangentCost"),
                traversal.entryTangentCost);
            values.insert(QStringLiteral("rotationCost"),
                traversal.rotationCost);
            values.insert(QStringLiteral("movementDistance"),
                traversal.movementDistance);
            values.insert(QStringLiteral("scheduledZone"),
                scheduledZone.has_value()
                ? machining::tubeZoneName(*scheduledZone)
                : QStringLiteral("None"));
            values.insert(QStringLiteral("ownerZone"),
                scheduledZone.has_value()
                ? machining::tubeZoneName(*scheduledZone)
                : QStringLiteral("None"));
            values.insert(QStringLiteral("selectedEntryZone"),
                traversal.selectedEntry.has_value()
                ? machining::tubeZoneName
                    (traversal.selectedEntry->zone)
                : QStringLiteral("None"));
            values.insert(QStringLiteral("candidateKind"),
                traversal.selectedEntry.has_value()
                ? entryCandidateKindName
                    (traversal.selectedEntry->kind)
                : QStringLiteral("Unconstrained"));
            const bool curveInteriorSelected =
                traversal.selectedEntry.has_value()
                && (traversal.selectedEntry->kind
                        == ZoneEntryCandidateKind::ClosedLoopArcInterior
                    || traversal.selectedEntry->kind
                        == ZoneEntryCandidateKind::
                            ClosedLoopEllipseInterior);
            const bool zoneRunMidpointSelected =
                traversal.selectedEntry.has_value()
                && traversal.selectedEntry->kind
                    == ZoneEntryCandidateKind::
                        ClosedLoopZoneRunMidpoint;
            values.insert(QStringLiteral("selectionMode"),
                !traversal.entryRefinementMode.isEmpty()
                ? traversal.entryRefinementMode
                : curveInteriorSelected
                    ? QStringLiteral("ExactCurveTangent")
                    : zoneRunMidpointSelected
                        ? QStringLiteral("ZoneRunMidpointFallback")
                        : QStringLiteral("NearestConnection"));
            values.insert(QStringLiteral("selectedEntityId"),
                QVariant::fromValue<qulonglong>
                    (traversal.selectedEntry.has_value()
                    ? traversal.selectedEntry->entityId : 0U));
            values.insert(QStringLiteral("selectedSourceKind"),
                traversal.selectedEntry.has_value()
                ? planningSourceKindName
                    (traversal.selectedEntry->sourceKind)
                : QStringLiteral("Unknown"));
            values.insert(QStringLiteral("selectedSourceParameter"),
                traversal.selectedEntry.has_value()
                    && traversal.selectedEntry->sourceParameter.has_value()
                ? *traversal.selectedEntry->sourceParameter : 0.0);
            values.insert(QStringLiteral("entryPosition"),
                traversal.selectedEntry.has_value()
                ? vectorText(traversal.selectedEntry->entryPosition)
                : vectorText(traversal.start));
            values.insert(QStringLiteral("firstCutTangent"),
                traversal.selectedEntry.has_value()
                ? vectorText(traversal.selectedEntry->firstCutTangent)
                : QStringLiteral("0,0,0"));
            values.insert(QStringLiteral("distanceToZoneBoundary"),
                traversal.selectedEntry.has_value()
                ? traversal.selectedEntry->distanceToZoneBoundary
                : 0.0);
            values.insert(QStringLiteral("distanceToMemberEndpoint"),
                traversal.selectedEntry.has_value()
                ? traversal.selectedEntry->distanceToMemberEndpoint
                : 0.0);
            values.insert(QStringLiteral("fragmentCount"),
                static_cast<int>(traversal.fragments.size()));
            values.insert(QStringLiteral("midpointFragmentUsed"),
                !traversal.fragments.empty());
            values.insert(QStringLiteral("previousCutEnd"),
                vectorText(traversal.previousCutEnd));
            values.insert(QStringLiteral("previousTransferAnchor"),
                vectorText(traversal.previousTransferAnchor));
            values.insert(QStringLiteral("curveMemberCount"),
                traversal.curveMemberCount);
            values.insert(QStringLiteral("arcTangentRootCount"),
                traversal.arcTangentRootCount);
            values.insert(QStringLiteral("ellipseTangentRootCount"),
                traversal.ellipseTangentRootCount);
            values.insert(QStringLiteral("validTangentCount"),
                traversal.validTangentCount);
            values.insert(QStringLiteral("travelDistance"),
                traversal.entryTravelDistance);
            values.insert(QStringLiteral("approachCutAngle"),
                traversal.approachCutAngle);
            values.insert(QStringLiteral("nearestConnectionDistance"),
                traversal.nearestConnectionDistance);
            values.insert(QStringLiteral("forwardAngle"),
                traversal.forwardAngle);
            values.insert(QStringLiteral("reverseAngle"),
                traversal.reverseAngle);
            values.insert(QStringLiteral("tangentResidual"),
                traversal.tangentResidual);
            values.insert(QStringLiteral("approachCutDot"),
                traversal.approachCutDot);
            return planningDiagnostic
            (
                context,
                DiagnosticCode::ProcessPlanningEntrySelectionSummary,
                QStringLiteral("普通四轴加工单元已选择合法平滑入口。"),
                QStringLiteral("Ordinary rotary unit selected a legal entry after zone and longitudinal ordering."),
                values,
                DiagnosticSeverity::Info
            );
        }

        std::optional<GroupTraversal> bestTraversal
        (
            const ProcessGroup& group,
            const std::unordered_map<EntityId, const PlanningEntity*>& entities,
            const Vector3d& currentPosition,
            const ProcessPlanningPolicy& policy,
            const std::optional<machining::TubeSectionModel>& section,
            const std::optional<Vector2d>& tubeCenter,
            ProcessOrderingStrategy selectionStrategy,
            ClosedLoopTraversalReport* closedLoopReport = nullptr,
            const TraversalSelectionContext* selection = nullptr
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
                    int candidateCount = 0;
                    int wrongZoneRejectedCount = 0;
                    const std::size_t startCandidateCount =
                        entity.startParameter.has_value()
                        ? 1U : entity.path.vertices.size();
                    for (std::size_t startIndex = 0U;
                        startIndex < startCandidateCount; ++startIndex)
                    {
                        for (const bool reverse : { false, true })
                        {
                            if (!directionAllowed(entity, reverse,
                                policy.allowReverse)
                                || !manualDirectionAllowed(entity, reverse,
                                    selection))
                            {
                                continue;
                            }
                            const double candidateParameter =
                                entity.path.vertices[startIndex]
                                    .sourceParameter;
                            if (selection != nullptr
                                && selection->hardZoneConstraint
                                && entity.manualStartParameter.has_value()
                                && std::abs(candidateParameter
                                    - *entity.manualStartParameter)
                                    > 1.0e-10)
                            {
                                continue;
                            }
                            auto candidate = buildSingleClosedCurveTraversal
                            (
                                group, entity, currentPosition, reverse, startIndex,
                                policy.connectionTolerance, section, tubeCenter
                            );
                            if (!candidate.has_value()) continue;
                            if (selection != nullptr
                                && selection->hardZoneConstraint
                                && selection->requiredEntryZone.has_value()
                                && section.has_value())
                            {
                                const std::size_t pointCount =
                                    entity.path.vertices.size();
                                const double threshold = entryThreshold
                                    (policy.connectionTolerance);
                                std::optional<Vector3d> firstCutPoint;
                                for (std::size_t offset = 1U;
                                    offset < pointCount; ++offset)
                                {
                                    const std::size_t index = reverse
                                        ? (startIndex + pointCount - offset)
                                            % pointCount
                                        : (startIndex + offset) % pointCount;
                                    const Vector3d& point =
                                        entity.path.vertices[index].position;
                                    if (distance(candidate->start, point)
                                        > threshold)
                                    {
                                        firstCutPoint = point;
                                        break;
                                    }
                                }
                                if (!firstCutPoint.has_value())
                                    continue;
                                auto entry = classifyZoneEntry
                                (
                                    ZoneEntryCandidateKind::
                                        ClosedCurveParameter,
                                    entity.entityId, candidateParameter,
                                    reverse, candidate->start,
                                    *firstCutPoint, entity.sourceKind, *section,
                                    selection->projectionTolerance
                                );
                                if (!entry.has_value()
                                    || entry->zone
                                        != *selection->requiredEntryZone)
                                {
                                    ++wrongZoneRejectedCount;
                                    continue;
                                }
                                candidate->selectedEntry =
                                    std::move(entry);
                                scoreUnwrappedEntry
                                (
                                    candidate->entities.front(),
                                    selection->previousEnd,
                                    *firstCutPoint, *section,
                                    policy.connectionTolerance,
                                    selection->projectionTolerance
                                );
                                candidate->entryAxisReversalCount =
                                    candidate->entities.front()
                                        .entryAxisReversalCount;
                                candidate->entryTangentCost =
                                    candidate->entities.front()
                                        .entryTangentCost;
                            }
                            ++candidateCount;
                            if (!best.has_value()
                                || (selection != nullptr
                                    ? zoneConstrainedTraversalLess
                                        (*candidate, *best, *selection,
                                            selectionStrategy)
                                    : traversalLess(*candidate, *best,
                                        selectionStrategy)))
                            {
                                best = std::move(candidate);
                            }
                        }
                    }
                    if (best.has_value())
                    {
                        best->entryCandidateCount = candidateCount;
                        best->wrongZoneRejectedCount =
                            wrongZoneRejectedCount;
                    }
                    return best;
                }
            }

            if (group.kind == ProcessGroupKind::ClosedLoop && group.entityIds.size() > 1U)
            {
                auto canonicalLoop = ClosedLoopTraversalBuilder::build
                (
                    group, entities, currentPosition, policy, section,
                    tubeCenter, selectionStrategy
                );
                if (!canonicalLoop.traversal.has_value())
                {
                    if (closedLoopReport != nullptr)
                        *closedLoopReport =
                            std::move(canonicalLoop.report);
                    return std::nullopt;
                }
                if (selection == nullptr
                    || !selection->hardZoneConstraint
                    || !section.has_value())
                {
                    canonicalLoop.traversal->entryCandidateCount =
                        canonicalLoop.report.candidateCount;
                    if (closedLoopReport != nullptr)
                        *closedLoopReport =
                            std::move(canonicalLoop.report);
                    return std::move(canonicalLoop.traversal);
                }

                auto connectionLoop = ClosedLoopTraversalBuilder::build
                (
                    group, entities, currentPosition, policy, section,
                    tubeCenter, selectionStrategy, selection
                );
                auto interior = ClosedLoopZoneRunBuilder::buildOrdinary
                (
                    group, *canonicalLoop.traversal, entities,
                    currentPosition, policy, *section, tubeCenter,
                    selectionStrategy, *selection
                );
                const bool curveInteriorMode =
                    interior.traversal.has_value();
                const bool connectionMode =
                    !curveInteriorMode
                    && connectionLoop.traversal.has_value();
                std::optional<GroupTraversal> best;
                if (curveInteriorMode)
                    best = std::move(interior.traversal);
                else if (connectionMode)
                    best = std::move(connectionLoop.traversal);
                else if (selection->allowZoneRunMidpointFallback)
                    best = std::move(interior.zoneRunMidpointTraversal);
                const int connectionCandidateCount =
                    connectionLoop.report.candidateCount;
                const int totalCandidateCount =
                    connectionCandidateCount + interior.candidateCount
                    + interior.zoneRunMidpointCandidateCount;
                const int wrongZoneRejectedCount =
                    connectionLoop.report.wrongZoneRejectedCount
                    + interior.wrongZoneRejectedCount;
                if (best.has_value())
                {
                    best->entryCandidateCount = totalCandidateCount;
                    best->connectionCandidateCount =
                        connectionCandidateCount;
                    best->arcInteriorCandidateCount =
                        interior.arcInteriorCandidateCount;
                    best->ellipseInteriorCandidateCount =
                        interior.ellipseInteriorCandidateCount;
                    best->zoneRunMidpointCandidateCount =
                        interior.zoneRunMidpointCandidateCount;
                    best->curveCandidateRejectedCount =
                        interior.curveCandidateRejectedCount;
                    best->curveMemberCount =
                        interior.curveMemberCount;
                    best->arcTangentRootCount =
                        interior.arcTangentRootCount;
                    best->ellipseTangentRootCount =
                        interior.ellipseTangentRootCount;
                    best->validTangentCount =
                        interior.validTangentCount;
                    best->arcInteriorCandidateEntityIds =
                        std::move(interior.arcCandidateEntityIds);
                    best->ellipseInteriorCandidateEntityIds =
                        std::move(interior.ellipseCandidateEntityIds);
                    best->wrongZoneRejectedCount =
                        wrongZoneRejectedCount;
                }
                canonicalLoop.report.candidateCount =
                    totalCandidateCount;
                if (best.has_value())
                {
                    canonicalLoop.report.selectedOrder.clear();
                    canonicalLoop.report.selectedReverse.clear();
                    for (const DirectedEntity& directed :
                        best->entities)
                    {
                        canonicalLoop.report.selectedOrder.push_back
                            (directed.entity->entityId);
                        canonicalLoop.report.selectedReverse.push_back
                            (directed.reverseRelativeToInput);
                    }
                }
                if (closedLoopReport != nullptr)
                    *closedLoopReport =
                        std::move(canonicalLoop.report);
                return best;
            }

            std::optional<GroupTraversal> best;
            int candidateCount = 0;
            int wrongZoneRejectedCount = 0;
            for (const EntityId entityId : group.entityIds)
            {
                for (const bool reverse : { false, true })
                {
                    const auto found = entities.find(entityId);
                    if (found == entities.end()
                        || !directionAllowed(*found->second, reverse,
                            policy.allowReverse)
                        || !manualDirectionAllowed(*found->second,
                            reverse, selection))
                    {
                        continue;
                    }
                    auto candidate = buildTraversal
                    (
                        group, entities, currentPosition, policy.allowReverse,
                        policy.connectionTolerance, std::make_pair(entityId, reverse)
                    );
                    if (!candidate.has_value()) continue;
                    if (selection != nullptr
                        && selection->hardZoneConstraint
                        && selection->requiredEntryZone.has_value()
                        && section.has_value())
                    {
                        const DirectedEntity& entryDirected =
                            candidate->entities.front();
                        const bool manualDirectionsMatch = std::all_of
                        (
                            candidate->entities.cbegin(),
                            candidate->entities.cend(),
                            [&selection](const DirectedEntity& directed)
                            {
                                return directed.entity != nullptr
                                    && manualDirectionAllowed
                                    (
                                        *directed.entity,
                                        directed.reverseRelativeToInput,
                                        selection
                                    );
                            }
                        );
                        if (!manualDirectionsMatch) continue;
                        const std::vector<Vector3d> entryPoints =
                            directedPoints(*entryDirected.entity,
                                entryDirected.reverseRelativeToInput);
                        const double threshold = entryThreshold
                            (policy.connectionTolerance);
                        const auto next = std::find_if
                        (
                            entryPoints.cbegin() + 1,
                            entryPoints.cend(),
                            [&entryPoints, threshold]
                            (const Vector3d& point)
                            {
                                return distance(entryPoints.front(), point)
                                    > threshold;
                            }
                        );
                        if (next == entryPoints.cend()) continue;
                        const auto entryVertices =
                            entryDirected.reverseRelativeToInput
                            ? std::vector<geometry::PathVertex3D>
                                (entryDirected.entity->path.vertices.rbegin(),
                                    entryDirected.entity->path.vertices.rend())
                            : entryDirected.entity->path.vertices;
                        const std::optional<double> entryParameter =
                            entryVertices.empty() ? std::nullopt
                                : std::optional<double>
                                    (entryVertices.front().sourceParameter);
                        auto entry = classifyZoneEntry
                        (
                            ZoneEntryCandidateKind::OpenEndpoint,
                            entryDirected.entity->entityId,
                            entryParameter,
                            entryDirected.reverseRelativeToInput,
                            candidate->start, *next,
                            entryDirected.entity->sourceKind, *section,
                            selection->projectionTolerance
                        );
                        const bool manualStartMatches =
                            !entryDirected.entity
                                ->manualStartParameter.has_value()
                            || (entryParameter.has_value()
                                && std::abs(*entryParameter
                                    - *entryDirected.entity
                                        ->manualStartParameter)
                                    <= 1.0e-10);
                        const bool otherManualStartExists = std::any_of
                        (
                            candidate->entities.cbegin() + 1,
                            candidate->entities.cend(),
                            [](const DirectedEntity& directed)
                            {
                                return directed.entity != nullptr
                                    && directed.entity
                                        ->manualStartParameter.has_value();
                            }
                        );
                        if (!entry.has_value()
                            || entry->zone
                                != *selection->requiredEntryZone
                            || !manualStartMatches
                            || otherManualStartExists)
                        {
                            ++wrongZoneRejectedCount;
                            continue;
                        }
                        candidate->selectedEntry = std::move(entry);
                        scoreUnwrappedEntry
                        (
                            candidate->entities.front(),
                            selection->previousEnd, *next, *section,
                            policy.connectionTolerance,
                            selection->projectionTolerance
                        );
                        candidate->entryAxisReversalCount =
                            candidate->entities.front()
                                .entryAxisReversalCount;
                        candidate->entryTangentCost =
                            candidate->entities.front()
                                .entryTangentCost;
                    }
                    ++candidateCount;
                    scoreTraversal(*candidate, currentPosition, section);
                    if (!best.has_value()
                        || (selection != nullptr
                            ? zoneConstrainedTraversalLess
                                (*candidate, *best, *selection,
                                    selectionStrategy)
                            : traversalLess(*candidate, *best,
                                selectionStrategy)))
                        best = std::move(candidate);
                }
            }
            if (best.has_value())
            {
                best->entryCandidateCount = candidateCount;
                best->wrongZoneRejectedCount =
                    wrongZoneRejectedCount;
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
            std::map<int, std::vector<const ProcessPathFragment*>>
                fragmentsByUnit;
            for (const ProcessPathFragment& fragment :
                plan.plannedFragments)
            {
                fragmentsByUnit[fragment.processUnitIndex]
                    .push_back(&fragment);
            }

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
                const int processUnitIndex = static_cast<int>
                    (std::distance(plan.processUnits.cbegin(), unit));
                const auto fragmented = fragmentsByUnit.find
                    (processUnitIndex);
                if (fragmented != fragmentsByUnit.end())
                {
                    auto fragments = fragmented->second;
                    std::sort(fragments.begin(), fragments.end(),
                        [](const ProcessPathFragment* left,
                            const ProcessPathFragment* right)
                        {
                            return left->fragmentOrder
                                < right->fragmentOrder;
                        });
                    std::map<EntityId, int> fragmentCounts;
                    std::map<EntityId, double> fragmentLengths;
                    std::vector<std::vector<geometry::PathVertex3D>>
                        fragmentPaths;
                    for (std::size_t index = 0U;
                        index < fragments.size(); ++index)
                    {
                        const ProcessPathFragment* fragment =
                            fragments[index];
                        const auto entity = fragment != nullptr
                            ? entities.find(fragment->entityId)
                            : entities.end();
                        if (fragment == nullptr
                            || fragment->fragmentOrder
                                != static_cast<int>(index)
                            || entity == entities.end()
                            || entity->second == nullptr)
                        {
                            failure.groupId = group.groupId;
                            failure.reason = QStringLiteral(
                                "Closed-loop fragment metadata is incomplete.");
                            return false;
                        }
                        auto path = ClosedLoopZoneRunBuilder::
                            fragmentVertices
                            (
                                *entity->second,
                                fragment->sourceParameterBegin,
                                fragment->sourceParameterEnd,
                                fragment->reverse
                            );
                        if (!path.has_value())
                        {
                            failure.groupId = group.groupId;
                            failure.currentEntityId =
                                fragment->entityId;
                            failure.reason = QStringLiteral(
                                "Closed-loop fragment parameter interval is invalid.");
                            return false;
                        }
                        ++fragmentCounts[fragment->entityId];
                        fragmentLengths[fragment->entityId] +=
                            ClosedLoopZoneRunBuilder::pathLength(*path);
                        fragmentPaths.push_back(std::move(*path));
                    }
                    int splitMemberCount = 0;
                    for (const EntityId entityId :
                        unit->key.memberEntityIds)
                    {
                        const auto entity = entities.find(entityId);
                        if (entity == entities.end()
                            || entity->second == nullptr)
                        {
                            failure.groupId = group.groupId;
                            failure.currentEntityId = entityId;
                            failure.reason = QStringLiteral(
                                "Closed-loop fragment source entity is missing.");
                            return false;
                        }
                        const int count = fragmentCounts[entityId];
                        if (count == 2) ++splitMemberCount;
                        else if (count != 1)
                        {
                            failure.groupId = group.groupId;
                            failure.currentEntityId = entityId;
                            failure.reason = QStringLiteral(
                                "Closed-loop fragments do not cover each member exactly once.");
                            return false;
                        }
                        const double fullLength =
                            ClosedLoopZoneRunBuilder::pathLength
                                (entity->second->path.vertices);
                        const double lengthTolerance =
                            std::max(1.0e-8, fullLength * 1.0e-9);
                        if (std::abs(fragmentLengths[entityId]
                            - fullLength) > lengthTolerance)
                        {
                            failure.groupId = group.groupId;
                            failure.currentEntityId = entityId;
                            failure.reason = QStringLiteral(
                                "Closed-loop fragment coverage has a gap or overlap.");
                            return false;
                        }
                    }
                    if (splitMemberCount != 1
                        || fragmentCounts.size()
                            != unit->key.memberEntityIds.size())
                    {
                        failure.groupId = group.groupId;
                        failure.reason = QStringLiteral(
                            "Closed-loop internal entry must split exactly one member.");
                        return false;
                    }
                    for (std::size_t index = 1U;
                        index < fragmentPaths.size(); ++index)
                    {
                        const double gap = distance
                            (fragmentPaths[index - 1U].back().position,
                                fragmentPaths[index].front().position);
                        if (gap > connectionTolerance)
                        {
                            failure = { group.groupId,
                                fragments[index - 1U]->entityId,
                                fragments[index]->entityId, gap,
                                QStringLiteral(
                                    "Adjacent closed-loop fragments are not physically connected.") };
                            return false;
                        }
                    }
                    if (fragmentPaths.empty()
                        || distance(fragmentPaths.back().back().position,
                            fragmentPaths.front().front().position)
                            > connectionTolerance)
                    {
                        failure.groupId = group.groupId;
                        failure.reason = QStringLiteral(
                            "Closed-loop fragment traversal does not return to its entry.");
                        return false;
                    }
                    continue;
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
