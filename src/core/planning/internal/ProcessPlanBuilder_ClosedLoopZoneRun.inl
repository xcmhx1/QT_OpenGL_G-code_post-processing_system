        class ClosedLoopZoneRunBuilder
        {
        public:
            struct BreakSegment;
            struct BreakZoneRun;

            struct Result
            {
                std::optional<GroupTraversal> traversal;
                ClosedLoopTraversalReport closedLoopReport;
                BreakBoundaryTraversalReport report;
            };

            static Result buildBreak
            (
                const ProcessGroup& group,
                const std::unordered_map<EntityId, const PlanningEntity*>& entities,
                const Vector3d& currentPosition,
                const ProcessPlanningPolicy& policy,
                const machining::TubeSectionModel& section,
                const std::optional<Vector2d>& tubeCenter,
                ProcessOrderingStrategy selectionStrategy,
                double projectionTolerance,
                int boundaryRank,
                machining::TubeZone16 preferredStartZone
            )
            {
                Result result;
                result.report.groupId = group.groupId;
                result.report.boundaryRank = boundaryRank;
                result.report.preferredStartZone = preferredStartZone;
                result.report.forcedTopMidpoint =
                    preferredStartZone == machining::TubeZone16::TopFace;
                if (!group.entityIds.empty())
                {
                    const auto found = entities.find(group.entityIds.front());
                    if (found != entities.end() && found->second != nullptr)
                        result.report.boundaryPairId =
                            found->second->boundaryPairId;
                }

                auto loop = ClosedLoopTraversalBuilder::build
                (
                    group, entities, currentPosition, policy, section,
                    tubeCenter, selectionStrategy
                );
                result.closedLoopReport = std::move(loop.report);
                if (!loop.traversal.has_value())
                {
                    result.report.failureReason =
                        result.closedLoopReport.failureReason;
                    result.report.failureCode =
                        DiagnosticCode::ProcessPlanningBreakFragmentTraversalInvalid;
                    return result;
                }

                std::vector<std::vector<DirectedEntity>> directions;
                directions.push_back(loop.traversal->entities);
                std::vector<DirectedEntity> reverseDirection;
                reverseDirection.reserve(loop.traversal->entities.size());
                bool reverseDirectionAllowed = true;
                for (auto iterator = loop.traversal->entities.rbegin();
                    iterator != loop.traversal->entities.rend(); ++iterator)
                {
                    DirectedEntity directed = *iterator;
                    directed.reverseRelativeToInput =
                        !directed.reverseRelativeToInput;
                    if (directed.entity == nullptr
                        || !directionAllowed(*directed.entity,
                            directed.reverseRelativeToInput,
                            policy.allowReverse))
                    {
                        reverseDirectionAllowed = false;
                        break;
                    }
                    const auto vertices = orientedVertices(directed);
                    if (vertices.size() < 2U)
                    {
                        reverseDirectionAllowed = false;
                        break;
                    }
                    directed.start = vertices.front().position;
                    directed.end = vertices.back().position;
                    reverseDirection.push_back(std::move(directed));
                }
                if (reverseDirectionAllowed)
                    directions.push_back(std::move(reverseDirection));

                std::optional<Candidate> best;
                QStringList runDescriptions;
                int candidateRunCount = 0;
                bool midpointLocated = false;
                bool fragmentTraversalBuilt = false;
                bool exitResolved = false;
                for (std::size_t directionIndex = 0U;
                    directionIndex < directions.size(); ++directionIndex)
                {
                    const std::vector<DirectedEntity>& cycle =
                        directions[directionIndex];
                    const std::vector<BreakSegment> segments =
                        buildSegments(cycle, section, projectionTolerance);
                    const std::vector<BreakZoneRun> runs =
                        buildRuns(segments);
                    for (const BreakZoneRun& run : runs)
                    {
                        runDescriptions.push_back(describeRun(run));
                        if (!run.strongZone
                            || run.zone != result.report.preferredStartZone)
                        {
                            continue;
                        }
                        ++candidateRunCount;
                        auto candidate = buildCandidate
                        (
                            group.groupId, cycle, segments, run,
                            entities, currentPosition, policy,
                            section, tubeCenter, projectionTolerance,
                            0.5, std::nullopt, std::nullopt, true,
                            directionIndex == 0U
                                ? QStringLiteral("Forward")
                                : QStringLiteral("Reverse"),
                            midpointLocated, fragmentTraversalBuilt,
                            exitResolved
                        );
                        if (!candidate.has_value()) continue;
                        if (!best.has_value()
                            || candidateLess(*candidate, *best,
                                result.report.forcedTopMidpoint,
                                projectionTolerance, selectionStrategy))
                        {
                            best = std::move(candidate);
                        }
                    }
                }
                result.report.candidateRunCount = candidateRunCount;
                result.report.candidateRuns =
                    runDescriptions.join(QLatin1Char(';'));
                if (!best.has_value())
                {
                    if (!midpointLocated)
                    {
                        result.report.failureReason =
                            QStringLiteral("No reliable run in the preferred sweep zone has an interior arc-length midpoint.");
                        result.report.failureCode =
                            DiagnosticCode::ProcessPlanningBreakMidpointCandidateMissing;
                    }
                    else if (!fragmentTraversalBuilt)
                    {
                        result.report.failureReason =
                            QStringLiteral("Break midpoint could not be represented as one continuous complementary fragment traversal.");
                        result.report.failureCode =
                            DiagnosticCode::ProcessPlanningBreakFragmentTraversalInvalid;
                    }
                    else if (!exitResolved)
                    {
                        result.report.failureReason =
                            QStringLiteral("Break final fragment did not resolve to the selected strong start zone.");
                        result.report.failureCode =
                            DiagnosticCode::ProcessPlanningBreakExitZoneUnresolved;
                    }
                    else
                    {
                        result.report.failureReason =
                            QStringLiteral("Break candidate selection failed after all traversal invariants passed.");
                        result.report.failureCode =
                            DiagnosticCode::ProcessPlanningBreakFragmentTraversalInvalid;
                    }
                    return result;
                }

                result.report.startZone = best->zone;
                result.report.selectedRunLength = best->runLength;
                result.report.selectedMaximumShellDeviation =
                    best->maximumShellDeviation;
                result.report.selectedConfidence = best->confidence;
                result.report.selectedMidpoint = best->midpoint;
                result.report.selectedEntityId = best->midpointEntityId;
                result.report.selectedSourceParameter =
                    best->midpointSourceParameter;
                result.report.exitZone = best->zone;
                result.report.exitConfidence = best->exitConfidence;
                result.report.exitReliableLength = best->exitReliableLength;
                result.report.finalEntityId = best->finalEntityId;
                result.report.finalParameterBegin =
                    best->finalParameterBegin;
                result.report.finalParameterEnd =
                    best->finalParameterEnd;
                result.report.exitUsedFallback = best->exitUsedFallback;
                result.report.fragmentCount =
                    static_cast<int>(best->fragments.size());
                result.report.direction = best->direction;
                result.report.fragments = best->fragments;
                result.report.status = QStringLiteral("Success");
                result.report.failureReason.clear();
                result.traversal = std::move(best->traversal);
                result.closedLoopReport.status = QStringLiteral("Success");
                result.closedLoopReport.selectedOrder.clear();
                result.closedLoopReport.selectedReverse.clear();
                for (const DirectedEntity& directed :
                    result.traversal->entities)
                {
                    result.closedLoopReport.selectedOrder.push_back
                        (directed.entity->entityId);
                    result.closedLoopReport.selectedReverse.push_back
                        (directed.reverseRelativeToInput);
                }
                return result;
            }

            struct OrdinaryResult
            {
                std::optional<GroupTraversal> traversal;
                std::optional<GroupTraversal> zoneRunMidpointTraversal;
                int candidateCount = 0;
                int arcInteriorCandidateCount = 0;
                int ellipseInteriorCandidateCount = 0;
                int zoneRunMidpointCandidateCount = 0;
                int curveCandidateRejectedCount = 0;
                int wrongZoneRejectedCount = 0;
                int curveMemberCount = 0;
                int arcTangentRootCount = 0;
                int ellipseTangentRootCount = 0;
                int validTangentCount = 0;
                std::vector<EntityId> arcCandidateEntityIds;
                std::vector<EntityId> ellipseCandidateEntityIds;
            };

            static bool ordinaryCurveInteriorEligible
            (
                const PlanningEntity& entity
            )
            {
                return !entity.path.closed
                    && (entity.sourceKind
                            == geometry::SourceGeometryKind::Arc
                        || entity.sourceKind
                            == geometry::SourceGeometryKind::Ellipse);
            }

            static bool ordinaryZoneRunMidpointEligible
            (
                const PlanningEntity& entity
            )
            {
                if (entity.path.closed) return false;
                switch (entity.sourceKind)
                {
                case geometry::SourceGeometryKind::Line:
                case geometry::SourceGeometryKind::Polyline:
                case geometry::SourceGeometryKind::Spline:
                case geometry::SourceGeometryKind::Arc:
                case geometry::SourceGeometryKind::Ellipse:
                    return true;
                default:
                    return false;
                }
            }

            static std::optional<double> runFactorForParameter
            (
                const BreakZoneRun& run,
                const std::vector<BreakSegment>& segments,
                EntityId entityId,
                double parameter
            )
            {
                double accumulated = 0.0;
                for (const std::size_t segmentIndex : run.segmentIndices)
                {
                    const BreakSegment& segment = segments[segmentIndex];
                    if (segment.entityId == entityId)
                    {
                        const double minimum = std::min
                            (segment.parameterBegin, segment.parameterEnd);
                        const double maximum = std::max
                            (segment.parameterBegin, segment.parameterEnd);
                        if (parameter >= minimum - 1.0e-10
                            && parameter <= maximum + 1.0e-10)
                        {
                            const double denominator =
                                segment.parameterEnd - segment.parameterBegin;
                            if (std::abs(denominator) <= 1.0e-12)
                                return std::nullopt;
                            const double localFactor = std::clamp
                            (
                                (parameter - segment.parameterBegin)
                                    / denominator,
                                0.0,
                                1.0
                            );
                            return std::clamp
                            (
                                (accumulated + segment.length * localFactor)
                                    / run.length,
                                0.0,
                                1.0
                            );
                        }
                    }
                    accumulated += segment.length;
                }
                return std::nullopt;
            }

            static OrdinaryResult buildOrdinary
            (
                const ProcessGroup& group,
                const GroupTraversal& canonicalTraversal,
                const std::unordered_map<EntityId,
                    const PlanningEntity*>& entities,
                const Vector3d& currentPosition,
                const ProcessPlanningPolicy& policy,
                const machining::TubeSectionModel& section,
                const std::optional<Vector2d>& tubeCenter,
                ProcessOrderingStrategy selectionStrategy,
                const TraversalSelectionContext& selection
            )
            {
                OrdinaryResult result;
                if (!selection.hardZoneConstraint
                    || !selection.requiredEntryZone.has_value())
                {
                    return result;
                }

                std::vector<std::vector<DirectedEntity>> directions;
                directions.push_back(canonicalTraversal.entities);
                std::vector<DirectedEntity> reverseDirection;
                reverseDirection.reserve(canonicalTraversal.entities.size());
                bool reverseAllowed = true;
                for (auto iterator = canonicalTraversal.entities.rbegin();
                    iterator != canonicalTraversal.entities.rend(); ++iterator)
                {
                    DirectedEntity directed = *iterator;
                    directed.reverseRelativeToInput =
                        !directed.reverseRelativeToInput;
                    if (directed.entity == nullptr
                        || !directionAllowed(*directed.entity,
                            directed.reverseRelativeToInput,
                            policy.allowReverse)
                        || !manualDirectionAllowed(*directed.entity,
                            directed.reverseRelativeToInput, &selection))
                    {
                        reverseAllowed = false;
                        break;
                    }
                    const auto vertices = orientedVertices(directed);
                    if (vertices.size() < 2U)
                    {
                        reverseAllowed = false;
                        break;
                    }
                    directed.start = vertices.front().position;
                    directed.end = vertices.back().position;
                    reverseDirection.push_back(std::move(directed));
                }
                if (reverseAllowed)
                    directions.push_back(std::move(reverseDirection));

                std::set<EntityId> curveMemberIds;
                for (const DirectedEntity& directed :
                    canonicalTraversal.entities)
                {
                    if (directed.entity != nullptr
                        && ordinaryCurveInteriorEligible(*directed.entity)
                        && directed.entity->sourceEntity.has_value())
                    {
                        curveMemberIds.insert(directed.entity->entityId);
                    }
                }
                result.curveMemberCount =
                    static_cast<int>(curveMemberIds.size());

                for (std::size_t directionIndex = 0U;
                    directionIndex < directions.size(); ++directionIndex)
                {
                    const auto& cycle = directions[directionIndex];
                    if (!std::all_of(cycle.cbegin(), cycle.cend(),
                        [&selection](const DirectedEntity& directed)
                        {
                            return directed.entity != nullptr
                                && manualDirectionAllowed
                                (
                                    *directed.entity,
                                    directed.reverseRelativeToInput,
                                    &selection
                                );
                        }))
                    {
                        continue;
                    }
                    const std::vector<BreakSegment> segments =
                        buildSegments(cycle, section,
                            selection.projectionTolerance);
                    const std::vector<BreakZoneRun> runs =
                        buildRuns(segments);
                    for (const BreakZoneRun& run : runs)
                    {
                        if (!run.strongZone
                            || run.zone != *selection.requiredEntryZone
                            || run.length <= kCalculationEpsilon)
                        {
                            continue;
                        }

                        struct RootCandidate
                        {
                            EntityId entityId = 0;
                            geometry::SourceGeometryKind sourceKind =
                                geometry::SourceGeometryKind::Unknown;
                            double parameter = 0.0;
                            Vector3d point;
                            double residual = 0.0;
                            double runFactor = 0.0;
                        };
                        std::vector<RootCandidate> roots;
                        std::set<EntityId> examinedMembers;
                        for (const std::size_t segmentIndex :
                            run.segmentIndices)
                        {
                            const BreakSegment& segment =
                                segments[segmentIndex];
                            const auto entity = entities.find
                                (segment.entityId);
                            if (entity == entities.end()
                                || entity->second == nullptr
                                || !ordinaryCurveInteriorEligible
                                    (*entity->second)
                                || !entity->second->sourceEntity.has_value()
                                || !examinedMembers.insert
                                    (segment.entityId).second)
                            {
                                continue;
                            }
                            const PlanningEntity& curveEntity =
                                *entity->second;
                            std::vector<ExactCurveTangentRoot>
                                memberRoots;
                            if (curveEntity.sourceKind
                                == geometry::SourceGeometryKind::Arc)
                            {
                                if (const auto* arc =
                                    std::get_if<geometry::ArcGeometry>
                                    (&curveEntity.sourceEntity->geometry))
                                {
                                    memberRoots = arcTangentRoots
                                        (*arc,
                                            selection.previousTransferAnchor,
                                            std::max(1.0e-10,
                                                selection
                                                    .projectionTolerance
                                                    * 1.0e-6));
                                }
                            }
                            else if (curveEntity.sourceKind
                                == geometry::SourceGeometryKind::Ellipse)
                            {
                                if (const auto* ellipse =
                                    std::get_if<geometry::EllipseGeometry>
                                    (&curveEntity.sourceEntity->geometry))
                                {
                                    memberRoots = ellipseTangentRoots
                                        (*ellipse,
                                            selection.previousTransferAnchor,
                                            std::max(1.0e-10,
                                                selection
                                                    .projectionTolerance
                                                    * 1.0e-6));
                                }
                            }
                            if (directionIndex == 0U)
                            {
                                if (curveEntity.sourceKind
                                    == geometry::SourceGeometryKind::Arc)
                                {
                                    result.arcTangentRootCount +=
                                        static_cast<int>(memberRoots.size());
                                }
                                else
                                {
                                    result.ellipseTangentRootCount +=
                                        static_cast<int>(memberRoots.size());
                                }
                            }
                            for (const auto& root : memberRoots)
                            {
                                const auto runFactor =
                                    runFactorForParameter
                                    (
                                        run, segments,
                                        curveEntity.entityId,
                                        root.parameter
                                    );
                                if (!runFactor.has_value()
                                    || *runFactor <= kCalculationEpsilon
                                    || *runFactor
                                        >= 1.0 - kCalculationEpsilon)
                                {
                                    continue;
                                }
                                roots.push_back
                                ({
                                    curveEntity.entityId,
                                    curveEntity.sourceKind,
                                    root.parameter,
                                    root.point,
                                    root.residual,
                                    *runFactor
                                });
                            }
                        }

                        for (const RootCandidate& root : roots)
                        {
                            bool midpointLocated = false;
                            bool fragmentsBuilt = false;
                            bool exitResolved = false;
                            auto candidate = buildCandidate
                            (
                                group.groupId, cycle, segments, run,
                                entities, currentPosition, policy, section,
                                tubeCenter, selection.projectionTolerance,
                                root.runFactor, root.point, root.parameter,
                                false,
                                directionIndex == 0U
                                    ? QStringLiteral("Forward")
                                    : QStringLiteral("Reverse"),
                                midpointLocated, fragmentsBuilt, exitResolved
                            );
                            if (!candidate.has_value()
                                || !fragmentsBuilt)
                            {
                                ++result.curveCandidateRejectedCount;
                                continue;
                            }
                            const auto entryEntityFound = entities.find
                                (candidate->midpointEntityId);
                            if (entryEntityFound == entities.end()
                                || entryEntityFound->second == nullptr
                                || !ordinaryCurveInteriorEligible
                                    (*entryEntityFound->second))
                            {
                                ++result.curveCandidateRejectedCount;
                                continue;
                            }
                            const PlanningEntity& entryEntity =
                                *entryEntityFound->second;
                            const double endpointTolerance = std::max
                            (
                                1.0e-8,
                                selection.projectionTolerance * 1.0e-3
                            );
                            const double distanceToMemberEndpoint = std::min
                            (
                                distance(candidate->midpoint,
                                    entryEntity.path.vertices.front()
                                        .position),
                                distance(candidate->midpoint,
                                    entryEntity.path.vertices.back()
                                        .position)
                            );
                            if (distanceToMemberEndpoint
                                <= endpointTolerance)
                            {
                                ++result.curveCandidateRejectedCount;
                                continue;
                            }
                            const ZoneEntryCandidateKind candidateKind =
                                entryEntity.sourceKind
                                    == geometry::SourceGeometryKind::Arc
                                ? ZoneEntryCandidateKind::
                                    ClosedLoopArcInterior
                                : ZoneEntryCandidateKind::
                                    ClosedLoopEllipseInterior;
                            auto entry = classifyZoneEntry
                            (
                                candidateKind,
                                candidate->midpointEntityId,
                                candidate->midpointSourceParameter,
                                candidate->traversal.entities.front()
                                    .reverseRelativeToInput,
                                candidate->midpoint,
                                candidate->firstCutPoint,
                                entryEntity.sourceKind,
                                section,
                                selection.projectionTolerance
                            );
                            if (!entry.has_value()
                                || entry->zone
                                    != *selection.requiredEntryZone)
                            {
                                ++result.wrongZoneRejectedCount;
                                ++result.curveCandidateRejectedCount;
                                continue;
                            }
                            entry->distanceToMemberEndpoint =
                                distanceToMemberEndpoint;
                            if (entry->distanceToZoneBoundary
                                <= endpointTolerance)
                            {
                                ++result.curveCandidateRejectedCount;
                                continue;
                            }

                            const PlanningEntity* traversalEntryEntity =
                                candidate->traversal.entities.front().entity;
                            if (traversalEntryEntity == nullptr)
                            {
                                ++result.curveCandidateRejectedCount;
                                continue;
                            }
                            const bool manualStartMatches =
                                !traversalEntryEntity
                                    ->manualStartParameter.has_value()
                                || std::abs(*traversalEntryEntity
                                    ->manualStartParameter
                                    - candidate->midpointSourceParameter)
                                    <= 1.0e-10;
                            const bool otherManualStartExists = std::any_of
                            (
                                candidate->traversal.entities.cbegin() + 1,
                                candidate->traversal.entities.cend(),
                                [](const DirectedEntity& directed)
                                {
                                    return directed.entity != nullptr
                                        && directed.entity
                                            ->manualStartParameter.has_value();
                                }
                            );
                            if (!manualStartMatches
                                || otherManualStartExists)
                            {
                                ++result.wrongZoneRejectedCount;
                                ++result.curveCandidateRejectedCount;
                                continue;
                            }

                            candidate->traversal.selectedEntry =
                                std::move(entry);
                            candidate->traversal.fragments =
                                candidate->fragments;
                            scoreUnwrappedEntry
                            (
                                candidate->traversal.entities.front(),
                                selection.previousTransferAnchor,
                                candidate->firstCutPoint, section,
                                policy.connectionTolerance,
                                selection.projectionTolerance
                            );
                            candidate->traversal.entryAxisReversalCount =
                                candidate->traversal.entities.front()
                                    .entryAxisReversalCount;
                            candidate->traversal.entryTangentCost =
                                candidate->traversal.entities.front()
                                    .entryTangentCost;
                            candidate->traversal.entryRefinementMode =
                                QStringLiteral("ExactCurveTangent");
                            candidate->traversal.previousCutEnd =
                                selection.previousCutEnd;
                            candidate->traversal.previousTransferAnchor =
                                selection.previousTransferAnchor;
                            candidate->traversal.entryTravelDistance =
                                distance(selection.previousTransferAnchor,
                                    candidate->midpoint);
                            candidate->traversal.approachCutDot =
                                std::clamp
                                (
                                    1.0 - candidate->traversal
                                        .entryTangentCost,
                                    -1.0,
                                    1.0
                                );
                            candidate->traversal.approachCutAngle =
                                std::acos(candidate->traversal
                                    .approachCutDot)
                                * 180.0 / 3.14159265358979323846;
                            candidate->traversal.tangentResidual =
                                root.residual;
                            scoreTraversal(candidate->traversal,
                                currentPosition, section);
                            ++result.candidateCount;
                            ++result.validTangentCount;
                            if (entryEntity.sourceKind
                                == geometry::SourceGeometryKind::Arc)
                            {
                                ++result.arcInteriorCandidateCount;
                                result.arcCandidateEntityIds.push_back
                                    (entryEntity.entityId);
                            }
                            else
                            {
                                ++result.ellipseInteriorCandidateCount;
                                result.ellipseCandidateEntityIds.push_back
                                    (entryEntity.entityId);
                            }
                            const auto tangentLess =
                                [](const GroupTraversal& left,
                                    const GroupTraversal& right)
                            {
                                const bool leftSameDirection =
                                    left.approachCutDot > 0.0;
                                const bool rightSameDirection =
                                    right.approachCutDot > 0.0;
                                if (leftSameDirection != rightSameDirection)
                                    return leftSameDirection;
                                if (std::abs(left.entryTravelDistance
                                    - right.entryTravelDistance)
                                    > kCalculationEpsilon)
                                {
                                    return left.entryTravelDistance
                                        < right.entryTravelDistance;
                                }
                                if (left.entryAxisReversalCount
                                    != right.entryAxisReversalCount)
                                {
                                    return left.entryAxisReversalCount
                                        < right.entryAxisReversalCount;
                                }
                                if (std::abs(left.rotationCost
                                    - right.rotationCost)
                                    > kCalculationEpsilon)
                                {
                                    return left.rotationCost
                                        < right.rotationCost;
                                }
                                if (left.stableEntityId
                                    != right.stableEntityId)
                                {
                                    return left.stableEntityId
                                        < right.stableEntityId;
                                }
                                const double leftParameter =
                                    left.selectedEntry
                                        ->sourceParameter.value_or(0.0);
                                const double rightParameter =
                                    right.selectedEntry
                                        ->sourceParameter.value_or(0.0);
                                if (std::abs(leftParameter - rightParameter)
                                    > 1.0e-12)
                                {
                                    return leftParameter < rightParameter;
                                }
                                return left.entities.front()
                                    .reverseRelativeToInput
                                    < right.entities.front()
                                        .reverseRelativeToInput;
                            };
                            if (!result.traversal.has_value()
                                || tangentLess(candidate->traversal,
                                    *result.traversal))
                            {
                                result.traversal =
                                    std::move(candidate->traversal);
                            }
                        }
                    }
                }
                if (selection.allowZoneRunMidpointFallback)
                {
                    std::optional<Candidate> bestMidpoint;
                    for (std::size_t directionIndex = 0U;
                        directionIndex < directions.size(); ++directionIndex)
                    {
                        const auto& cycle = directions[directionIndex];
                        const std::vector<BreakSegment> segments =
                            buildSegments(cycle, section,
                                selection.projectionTolerance);
                        const std::vector<BreakZoneRun> runs =
                            buildRuns(segments);
                        for (const BreakZoneRun& run : runs)
                        {
                            if (!run.strongZone || run.touchesBoundary
                                || run.zone
                                    != *selection.requiredEntryZone
                                || run.length <= kCalculationEpsilon)
                            {
                                continue;
                            }
                            bool midpointLocated = false;
                            bool fragmentsBuilt = false;
                            bool exitResolved = false;
                            auto candidate = buildCandidate
                            (
                                group.groupId, cycle, segments, run,
                                entities, currentPosition, policy, section,
                                tubeCenter, selection.projectionTolerance,
                                0.5, std::nullopt, std::nullopt, false,
                                directionIndex == 0U
                                    ? QStringLiteral("Forward")
                                    : QStringLiteral("Reverse"),
                                midpointLocated, fragmentsBuilt, exitResolved
                            );
                            if (!candidate.has_value() || !midpointLocated
                                || !fragmentsBuilt)
                            {
                                continue;
                            }
                            const auto entryEntityFound = entities.find
                                (candidate->midpointEntityId);
                            if (entryEntityFound == entities.end()
                                || entryEntityFound->second == nullptr
                                || !ordinaryZoneRunMidpointEligible
                                    (*entryEntityFound->second))
                            {
                                continue;
                            }
                            const PlanningEntity& entryEntity =
                                *entryEntityFound->second;
                            const double endpointTolerance = std::max
                            (
                                1.0e-8,
                                selection.projectionTolerance * 1.0e-3
                            );
                            const double distanceToMemberEndpoint = std::min
                            (
                                distance(candidate->midpoint,
                                    entryEntity.path.vertices.front()
                                        .position),
                                distance(candidate->midpoint,
                                    entryEntity.path.vertices.back()
                                        .position)
                            );
                            if (distanceToMemberEndpoint
                                <= endpointTolerance)
                            {
                                continue;
                            }
                            auto entry = classifyZoneEntry
                            (
                                ZoneEntryCandidateKind::
                                    ClosedLoopZoneRunMidpoint,
                                candidate->midpointEntityId,
                                candidate->midpointSourceParameter,
                                candidate->traversal.entities.front()
                                    .reverseRelativeToInput,
                                candidate->midpoint,
                                candidate->firstCutPoint,
                                entryEntity.sourceKind,
                                section,
                                selection.projectionTolerance
                            );
                            if (!entry.has_value() || entry->ambiguous
                                || entry->zone
                                    != *selection.requiredEntryZone
                                || entry->distanceToZoneBoundary
                                    <= endpointTolerance)
                            {
                                continue;
                            }
                            const PlanningEntity* traversalEntryEntity =
                                candidate->traversal.entities.front().entity;
                            if (traversalEntryEntity == nullptr)
                                continue;
                            const bool manualStartMatches =
                                !traversalEntryEntity
                                    ->manualStartParameter.has_value()
                                || std::abs(*traversalEntryEntity
                                    ->manualStartParameter
                                    - candidate->midpointSourceParameter)
                                    <= 1.0e-10;
                            const bool otherManualStartExists = std::any_of
                            (
                                candidate->traversal.entities.cbegin() + 1,
                                candidate->traversal.entities.cend(),
                                [](const DirectedEntity& directed)
                                {
                                    return directed.entity != nullptr
                                        && directed.entity
                                            ->manualStartParameter.has_value();
                                }
                            );
                            if (!manualStartMatches
                                || otherManualStartExists)
                            {
                                continue;
                            }
                            entry->distanceToMemberEndpoint =
                                distanceToMemberEndpoint;
                            candidate->traversal.selectedEntry =
                                std::move(entry);
                            candidate->traversal.fragments =
                                candidate->fragments;
                            scoreUnwrappedEntry
                            (
                                candidate->traversal.entities.front(),
                                selection.previousTransferAnchor,
                                candidate->firstCutPoint, section,
                                policy.connectionTolerance,
                                selection.projectionTolerance
                            );
                            candidate->traversal.entryAxisReversalCount =
                                candidate->traversal.entities.front()
                                    .entryAxisReversalCount;
                            candidate->traversal.entryTangentCost =
                                candidate->traversal.entities.front()
                                    .entryTangentCost;
                            candidate->traversal.entryRefinementMode =
                                QStringLiteral("ZoneRunMidpointFallback");
                            candidate->traversal.previousCutEnd =
                                selection.previousCutEnd;
                            candidate->traversal.previousTransferAnchor =
                                selection.previousTransferAnchor;
                            candidate->traversal.entryTravelDistance =
                                distance(selection.previousTransferAnchor,
                                    candidate->midpoint);
                            candidate->traversal.approachCutDot =
                                std::clamp
                                (
                                    1.0 - candidate->traversal
                                        .entryTangentCost,
                                    -1.0,
                                    1.0
                                );
                            candidate->traversal.approachCutAngle =
                                std::acos(candidate->traversal
                                    .approachCutDot)
                                * 180.0 / 3.14159265358979323846;
                            scoreTraversal(candidate->traversal,
                                currentPosition, section);
                            ++result.zoneRunMidpointCandidateCount;
                            const double lengthTolerance = std::max
                                (1.0e-9,
                                    selection.projectionTolerance * 1.0e-3);
                            const auto midpointLess =
                                [lengthTolerance](const Candidate& left,
                                    const Candidate& right)
                            {
                                if (std::abs(left.runLength
                                    - right.runLength) > lengthTolerance)
                                {
                                    return left.runLength > right.runLength;
                                }
                                if (std::abs(left.maximumShellDeviation
                                    - right.maximumShellDeviation)
                                    > kCalculationEpsilon)
                                {
                                    return left.maximumShellDeviation
                                        < right.maximumShellDeviation;
                                }
                                if (left.midpointEntityId
                                    != right.midpointEntityId)
                                {
                                    return left.midpointEntityId
                                        < right.midpointEntityId;
                                }
                                if (left.midpointSourceParameter
                                    != right.midpointSourceParameter)
                                {
                                    return left.midpointSourceParameter
                                        < right.midpointSourceParameter;
                                }
                                return left.direction < right.direction;
                            };
                            if (!bestMidpoint.has_value()
                                || midpointLess(*candidate, *bestMidpoint))
                            {
                                bestMidpoint = std::move(candidate);
                            }
                        }
                    }
                    if (bestMidpoint.has_value())
                    {
                        bestMidpoint->traversal.entryCandidateCount =
                            result.zoneRunMidpointCandidateCount;
                        bestMidpoint->traversal
                            .zoneRunMidpointCandidateCount =
                            result.zoneRunMidpointCandidateCount;
                        result.zoneRunMidpointTraversal =
                            std::move(bestMidpoint->traversal);
                    }
                }
                if (result.traversal.has_value())
                {
                    std::sort(result.arcCandidateEntityIds.begin(),
                        result.arcCandidateEntityIds.end());
                    result.arcCandidateEntityIds.erase(std::unique
                    (
                        result.arcCandidateEntityIds.begin(),
                        result.arcCandidateEntityIds.end()
                    ), result.arcCandidateEntityIds.end());
                    std::sort(result.ellipseCandidateEntityIds.begin(),
                        result.ellipseCandidateEntityIds.end());
                    result.ellipseCandidateEntityIds.erase(std::unique
                    (
                        result.ellipseCandidateEntityIds.begin(),
                        result.ellipseCandidateEntityIds.end()
                    ), result.ellipseCandidateEntityIds.end());
                    result.traversal->entryCandidateCount =
                        result.candidateCount;
                    result.traversal->arcInteriorCandidateCount =
                        result.arcInteriorCandidateCount;
                    result.traversal->ellipseInteriorCandidateCount =
                        result.ellipseInteriorCandidateCount;
                    result.traversal->curveCandidateRejectedCount =
                        result.curveCandidateRejectedCount;
                    result.traversal->arcInteriorCandidateEntityIds =
                        result.arcCandidateEntityIds;
                    result.traversal->ellipseInteriorCandidateEntityIds =
                        result.ellipseCandidateEntityIds;
                    result.traversal->wrongZoneRejectedCount =
                        result.wrongZoneRejectedCount;
                    result.traversal->curveMemberCount =
                        result.curveMemberCount;
                    result.traversal->arcTangentRootCount =
                        result.arcTangentRootCount;
                    result.traversal->ellipseTangentRootCount =
                        result.ellipseTangentRootCount;
                    result.traversal->validTangentCount =
                        result.validTangentCount;
                }
                if (result.zoneRunMidpointTraversal.has_value())
                {
                    result.zoneRunMidpointTraversal->curveMemberCount =
                        result.curveMemberCount;
                    result.zoneRunMidpointTraversal->arcTangentRootCount =
                        result.arcTangentRootCount;
                    result.zoneRunMidpointTraversal
                        ->ellipseTangentRootCount =
                        result.ellipseTangentRootCount;
                    result.zoneRunMidpointTraversal->validTangentCount =
                        result.validTangentCount;
                }
                return result;
            }

        public:
            struct BreakSegment
            {
                std::size_t entityOrder = 0U;
                std::size_t segmentOrder = 0U;
                EntityId entityId = 0;
                std::size_t sourceIndex = 0U;
                Vector3d start;
                Vector3d end;
                double parameterBegin = 0.0;
                double parameterEnd = 0.0;
                double length = 0.0;
                std::optional<machining::TubeZone16> strongZone;
                std::optional<machining::TubeZone16> possibleZone;
                double maximumShellDeviation = 0.0;
                double confidence = 0.0;
                bool touchesBoundary = false;
            };

            struct BreakZoneRun
            {
                machining::TubeZone16 zone =
                    machining::TubeZone16::TopFace;
                std::vector<std::size_t> segmentIndices;
                double length = 0.0;
                double maximumShellDeviation = 0.0;
                double confidence = 0.0;
                bool strongZone = false;
                bool touchesBoundary = false;
            };

            struct FragmentLocation
            {
                std::size_t segmentIndex = 0U;
                geometry::PathVertex3D vertex;
            };

            struct ExitResult
            {
                std::optional<machining::TubeZone16> zone;
                double confidence = 0.0;
                double reliableLength = 0.0;
                EntityId finalEntityId = 0;
                double finalParameterBegin = 0.0;
                double finalParameterEnd = 0.0;
                bool usedFallback = false;
            };

            struct Candidate
            {
                GroupTraversal traversal;
                std::vector<ProcessPathFragment> fragments;
                machining::TubeZone16 zone =
                    machining::TubeZone16::TopFace;
                double runLength = 0.0;
                double maximumShellDeviation = 0.0;
                double confidence = 0.0;
                Vector3d midpoint;
                EntityId midpointEntityId = 0;
                std::size_t midpointSourceIndex = 0;
                double midpointSourceParameter = 0.0;
                Vector3d firstCutPoint;
                double exitConfidence = 0.0;
                double exitReliableLength = 0.0;
                EntityId finalEntityId = 0;
                double finalParameterBegin = 0.0;
                double finalParameterEnd = 0.0;
                bool exitUsedFallback = false;
                QString direction;
            };

            static bool finite(const Vector3d& value)
            {
                return std::isfinite(value.x) && std::isfinite(value.y)
                    && std::isfinite(value.z);
            }

            static Vector3d interpolate
            (
                const Vector3d& start,
                const Vector3d& end,
                double factor
            )
            {
                return
                {
                    start.x + (end.x - start.x) * factor,
                    start.y + (end.y - start.y) * factor,
                    start.z + (end.z - start.z) * factor
                };
            }

            static std::vector<geometry::PathVertex3D> orientedVertices
            (
                const DirectedEntity& directed
            )
            {
                if (directed.entity == nullptr) return {};
                std::vector<geometry::PathVertex3D> vertices =
                    directed.entity->path.vertices;
                if (directed.reverseRelativeToInput)
                    std::reverse(vertices.begin(), vertices.end());
                return vertices;
            }

            static bool strongZone(machining::TubeZone16 zone)
            {
                return machining::tubeZoneIndex(zone) % 2U == 0U;
            }

            static BreakSegment classifySegment
            (
                const DirectedEntity& directed,
                std::size_t entityOrder,
                std::size_t segmentOrder,
                const geometry::PathVertex3D& start,
                const geometry::PathVertex3D& end,
                const machining::TubeSectionModel& section,
                double projectionTolerance
            )
            {
                BreakSegment segment;
                segment.entityOrder = entityOrder;
                segment.segmentOrder = segmentOrder;
                segment.entityId = directed.entity->entityId;
                segment.sourceIndex = directed.entity->sourceIndex;
                segment.start = start.position;
                segment.end = end.position;
                segment.parameterBegin = start.sourceParameter;
                segment.parameterEnd = end.sourceParameter;
                segment.length = distance(segment.start, segment.end);
                segment.maximumShellDeviation = 0.0;
                segment.confidence = 1.0;
                std::array<int, machining::kTubeZone16Count> possibleCounts{};
                std::optional<machining::TubeZone16> reliableZone;
                bool allReliable = segment.length > kCalculationEpsilon;
                for (const double factor : { 0.25, 0.5, 0.75 })
                {
                    const Vector3d point =
                        interpolate(segment.start, segment.end, factor);
                    const machining::TubeSectionProjection projection =
                        machining::TubeSectionProjector::project
                        (
                            section, { point.y, point.z },
                            projectionTolerance
                        );
                    if (projection.valid)
                    {
                        ++possibleCounts[machining::tubeZoneIndex
                            (projection.zone)];
                    }
                    segment.maximumShellDeviation = std::max
                        (segment.maximumShellDeviation,
                            projection.absoluteDistanceToShell);
                    segment.confidence = std::min
                        (segment.confidence, projection.confidence);
                    segment.touchesBoundary = segment.touchesBoundary
                        || projection.onBoundary || projection.ambiguous
                        || !projection.valid
                        || !strongZone(projection.zone);
                    if (!projection.valid || projection.ambiguous
                        || projection.onBoundary
                        || !strongZone(projection.zone)
                        || projection.confidence < 0.5
                        || projection.absoluteDistanceToShell
                            > projectionTolerance * 0.8)
                    {
                        allReliable = false;
                        continue;
                    }
                    if (!reliableZone.has_value())
                        reliableZone = projection.zone;
                    else if (*reliableZone != projection.zone)
                        allReliable = false;
                }
                int bestCount = 0;
                bool tied = false;
                for (std::size_t index = 0U;
                    index < possibleCounts.size(); ++index)
                {
                    if (possibleCounts[index] > bestCount)
                    {
                        bestCount = possibleCounts[index];
                        segment.possibleZone =
                            static_cast<machining::TubeZone16>(index);
                        tied = false;
                    }
                    else if (possibleCounts[index] > 0
                        && possibleCounts[index] == bestCount)
                    {
                        tied = true;
                    }
                }
                if (tied) segment.possibleZone.reset();
                if (allReliable && reliableZone.has_value())
                    segment.strongZone = reliableZone;
                return segment;
            }

            static std::vector<BreakSegment> buildSegments
            (
                const std::vector<DirectedEntity>& cycle,
                const machining::TubeSectionModel& section,
                double projectionTolerance
            )
            {
                std::vector<BreakSegment> segments;
                for (std::size_t entityOrder = 0U;
                    entityOrder < cycle.size(); ++entityOrder)
                {
                    const DirectedEntity& directed = cycle[entityOrder];
                    const auto vertices = orientedVertices(directed);
                    for (std::size_t index = 1U;
                        index < vertices.size(); ++index)
                    {
                        if (!finite(vertices[index - 1U].position)
                            || !finite(vertices[index].position)
                            || distance(vertices[index - 1U].position,
                                vertices[index].position)
                                <= kCalculationEpsilon)
                        {
                            continue;
                        }
                        segments.push_back(classifySegment
                        (
                            directed, entityOrder, index - 1U,
                            vertices[index - 1U], vertices[index],
                            section, projectionTolerance
                        ));
                    }
                }
                return segments;
            }

            static void addSegmentToRun
            (
                BreakZoneRun& run,
                const BreakSegment& segment,
                std::size_t segmentIndex
            )
            {
                if (run.segmentIndices.empty())
                {
                    run.zone = *segment.strongZone;
                    run.confidence = segment.confidence;
                }
                run.segmentIndices.push_back(segmentIndex);
                run.length += segment.length;
                run.maximumShellDeviation = std::max
                    (run.maximumShellDeviation,
                        segment.maximumShellDeviation);
                run.confidence = std::min
                    (run.confidence, segment.confidence);
                run.strongZone = true;
                run.touchesBoundary = run.touchesBoundary
                    || segment.touchesBoundary;
            }

            static std::vector<BreakZoneRun> buildRuns
            (
                const std::vector<BreakSegment>& segments
            )
            {
                std::vector<BreakZoneRun> runs;
                for (std::size_t index = 0U; index < segments.size(); ++index)
                {
                    const BreakSegment& segment = segments[index];
                    if (!segment.strongZone.has_value())
                        continue;
                    if (runs.empty() || runs.back().zone != *segment.strongZone
                        || (!runs.back().segmentIndices.empty()
                            && runs.back().segmentIndices.back() + 1U != index))
                    {
                        runs.emplace_back();
                    }
                    addSegmentToRun(runs.back(), segment, index);
                }
                if (runs.size() > 1U
                    && !runs.front().segmentIndices.empty()
                    && !runs.back().segmentIndices.empty()
                    && runs.front().segmentIndices.front() == 0U
                    && runs.back().segmentIndices.back() + 1U
                        == segments.size()
                    && runs.front().zone == runs.back().zone)
                {
                    BreakZoneRun combined = runs.back();
                    for (const std::size_t index :
                        runs.front().segmentIndices)
                    {
                        addSegmentToRun(combined, segments[index], index);
                    }
                    runs.front() = std::move(combined);
                    runs.pop_back();
                }
                return runs;
            }

            static QString describeRun(const BreakZoneRun& run)
            {
                return QStringLiteral("%1|length=%2|deviation=%3|confidence=%4|strong=%5|boundary=%6")
                    .arg(machining::tubeZoneName(run.zone))
                    .arg(run.length, 0, 'g', 15)
                    .arg(run.maximumShellDeviation, 0, 'g', 15)
                    .arg(run.confidence, 0, 'g', 15)
                    .arg(run.strongZone ? 1 : 0)
                    .arg(run.touchesBoundary ? 1 : 0);
            }

            static std::optional<FragmentLocation> locateParameter
            (
                const std::vector<geometry::PathVertex3D>& vertices,
                double parameter
            )
            {
                if (vertices.size() < 2U || !std::isfinite(parameter))
                    return std::nullopt;
                const double epsilon = 1.0e-10;
                for (std::size_t index = 1U; index < vertices.size(); ++index)
                {
                    const double left = vertices[index - 1U].sourceParameter;
                    const double right = vertices[index].sourceParameter;
                    const double minimum = std::min(left, right) - epsilon;
                    const double maximum = std::max(left, right) + epsilon;
                    if (parameter < minimum || parameter > maximum)
                        continue;
                    const double denominator = right - left;
                    const double factor = std::abs(denominator) <= epsilon
                        ? 0.0 : std::clamp
                            ((parameter - left) / denominator, 0.0, 1.0);
                    FragmentLocation location;
                    location.segmentIndex = index - 1U;
                    location.vertex.position = interpolate
                        (vertices[index - 1U].position,
                            vertices[index].position, factor);
                    location.vertex.sourceParameter = parameter;
                    return location;
                }
                return std::nullopt;
            }

            static std::optional<std::vector<geometry::PathVertex3D>>
                fragmentVertices
            (
                const PlanningEntity& entity,
                double parameterBegin,
                double parameterEnd,
                bool reverse
            )
            {
                DirectedEntity directed;
                directed.entity = &entity;
                directed.reverseRelativeToInput = reverse;
                const auto vertices = orientedVertices(directed);
                const auto begin = locateParameter(vertices, parameterBegin);
                const auto end = locateParameter(vertices, parameterEnd);
                if (!begin.has_value() || !end.has_value()
                    || begin->segmentIndex > end->segmentIndex)
                {
                    return std::nullopt;
                }
                std::vector<geometry::PathVertex3D> result;
                result.push_back(begin->vertex);
                for (std::size_t index = begin->segmentIndex + 1U;
                    index <= end->segmentIndex && index < vertices.size();
                    ++index)
                {
                    if (distance(result.back().position,
                        vertices[index].position) > kCalculationEpsilon)
                    {
                        result.push_back(vertices[index]);
                    }
                }
                if (distance(result.back().position, end->vertex.position)
                    > kCalculationEpsilon)
                {
                    result.push_back(end->vertex);
                }
                else
                {
                    result.back() = end->vertex;
                }
                if (result.size() < 2U
                    || distance(result.front().position,
                        result.back().position) <= kCalculationEpsilon)
                {
                    return std::nullopt;
                }
                return result;
            }

            static double pathLength
            (
                const std::vector<geometry::PathVertex3D>& vertices
            )
            {
                double length = 0.0;
                for (std::size_t index = 1U;
                    index < vertices.size(); ++index)
                {
                    length += distance(vertices[index - 1U].position,
                        vertices[index].position);
                }
                return length;
            }

            static ExitResult resolveExit
            (
                const std::vector<ProcessPathFragment>& fragments,
                const std::unordered_map<EntityId,
                    const PlanningEntity*>& entities,
                const machining::TubeSectionModel& section,
                double projectionTolerance,
                machining::TubeZone16 startZone
            )
            {
                ExitResult result;
                std::optional<machining::TubeZone16> fallbackZone;
                double fallbackConfidence = 0.0;
                bool reliableZoneFound = false;
                for (auto iterator = fragments.rbegin();
                    iterator != fragments.rend(); ++iterator)
                {
                    const auto entity = entities.find(iterator->entityId);
                    if (entity == entities.end() || entity->second == nullptr)
                        continue;
                    const auto vertices = fragmentVertices
                    (
                        *entity->second, iterator->sourceParameterBegin,
                        iterator->sourceParameterEnd, iterator->reverse
                    );
                    if (!vertices.has_value()) continue;
                    for (std::size_t offset = 1U;
                        offset < vertices->size(); ++offset)
                    {
                        const auto& end =
                            (*vertices)[vertices->size() - offset];
                        const auto& begin =
                            (*vertices)[vertices->size() - offset - 1U];
                        const double segmentLength =
                            distance(begin.position, end.position);
                        if (segmentLength <= kCalculationEpsilon) continue;
                        DirectedEntity directed;
                        directed.entity = entity->second;
                        const BreakSegment classified = classifySegment
                        (
                            directed, 0U, 0U, begin, end,
                            section, projectionTolerance
                        );
                        if (classified.strongZone.has_value())
                        {
                            if (*classified.strongZone != startZone)
                                return reliableZoneFound
                                    ? result : ExitResult{};
                            if (!reliableZoneFound)
                            {
                                result.zone = *classified.strongZone;
                                result.confidence =
                                    classified.confidence;
                                result.finalEntityId =
                                    iterator->entityId;
                                result.finalParameterBegin =
                                    iterator->sourceParameterBegin;
                                result.finalParameterEnd =
                                    iterator->sourceParameterEnd;
                                reliableZoneFound = true;
                            }
                            else
                            {
                                result.confidence = std::min
                                    (result.confidence,
                                        classified.confidence);
                            }
                            result.reliableLength += segmentLength;
                            continue;
                        }
                        if (reliableZoneFound
                            && segmentLength > projectionTolerance)
                        {
                            return result;
                        }
                        if (!fallbackZone.has_value()
                            && classified.possibleZone.has_value()
                            && strongZone(*classified.possibleZone))
                        {
                            fallbackZone = classified.possibleZone;
                            fallbackConfidence = classified.confidence;
                        }
                    }
                }
                if (reliableZoneFound) return result;
                if (fallbackZone.has_value() && *fallbackZone == startZone)
                {
                    result.zone = fallbackZone;
                    result.confidence = fallbackConfidence;
                    result.usedFallback = true;
                    result.finalEntityId = fragments.empty()
                        ? 0U : fragments.back().entityId;
                    if (!fragments.empty())
                    {
                        result.finalParameterBegin =
                            fragments.back().sourceParameterBegin;
                        result.finalParameterEnd =
                            fragments.back().sourceParameterEnd;
                    }
                }
                return result;
            }

            static std::optional<Candidate> buildCandidate
            (
                int groupId,
                const std::vector<DirectedEntity>& cycle,
                const std::vector<BreakSegment>& segments,
                const BreakZoneRun& run,
                const std::unordered_map<EntityId,
                    const PlanningEntity*>& entities,
                const Vector3d& currentPosition,
                const ProcessPlanningPolicy& policy,
                const machining::TubeSectionModel& section,
                const std::optional<Vector2d>& tubeCenter,
                double projectionTolerance,
                double runFactor,
                std::optional<Vector3d> exactPoint,
                std::optional<double> exactParameter,
                bool requireMatchingExit,
                const QString& direction,
                bool& midpointLocated,
                bool& fragmentTraversalBuilt,
                bool& exitResolved
            )
            {
                if (run.segmentIndices.empty()
                    || run.length <= kCalculationEpsilon)
                {
                    return std::nullopt;
                }
                const double target = run.length * std::clamp
                    (runFactor, 0.0, 1.0);
                double accumulated = 0.0;
                const BreakSegment* midpointSegment = nullptr;
                double midpointFactor = 0.0;
                for (const std::size_t index : run.segmentIndices)
                {
                    const BreakSegment& segment = segments[index];
                    if (accumulated + segment.length
                        >= target - kCalculationEpsilon)
                    {
                        midpointSegment = &segment;
                        midpointFactor = std::clamp
                            ((target - accumulated) / segment.length,
                                0.0, 1.0);
                        break;
                    }
                    accumulated += segment.length;
                }
                if (midpointSegment == nullptr
                    || midpointFactor <= kCalculationEpsilon
                    || midpointFactor >= 1.0 - kCalculationEpsilon)
                {
                    return std::nullopt;
                }
                midpointLocated = true;

                const Vector3d midpoint = exactPoint.value_or(interpolate
                    (midpointSegment->start, midpointSegment->end,
                        midpointFactor));
                const double midpointParameter = exactParameter.value_or
                (
                    midpointSegment->parameterBegin
                    + (midpointSegment->parameterEnd
                        - midpointSegment->parameterBegin)
                        * midpointFactor
                );
                const std::size_t startEntityOrder =
                    midpointSegment->entityOrder;

                Candidate candidate;
                candidate.zone = run.zone;
                candidate.runLength = run.length;
                candidate.maximumShellDeviation =
                    run.maximumShellDeviation;
                candidate.confidence = run.confidence;
                candidate.midpoint = midpoint;
                candidate.midpointEntityId =
                    midpointSegment->entityId;
                candidate.midpointSourceIndex =
                    midpointSegment->sourceIndex;
                candidate.midpointSourceParameter =
                    midpointParameter;
                candidate.firstCutPoint = midpointSegment->end;
                candidate.direction = direction;
                candidate.traversal.groupId = groupId;

                std::vector<DirectedEntity> rotated;
                rotated.reserve(cycle.size());
                for (std::size_t offset = 0U;
                    offset < cycle.size(); ++offset)
                {
                    DirectedEntity directed =
                        cycle[(startEntityOrder + offset) % cycle.size()];
                    const auto vertices = orientedVertices(directed);
                    if (vertices.size() < 2U) return std::nullopt;
                    directed.start = vertices.front().position;
                    directed.end = vertices.back().position;
                    if (offset == 0U)
                    {
                        directed.start = midpoint;
                        directed.selectedStartParameter =
                            midpointParameter;
                    }
                    rotated.push_back(std::move(directed));
                }
                candidate.traversal.entities = rotated;
                candidate.traversal.start = midpoint;
                candidate.traversal.end = midpoint;

                const auto firstVertices =
                    orientedVertices(rotated.front());
                if (firstVertices.size() < 2U) return std::nullopt;
                ProcessPathFragment first;
                first.entityId = rotated.front().entity->entityId;
                first.fragmentOrder = 0;
                first.sourceParameterBegin = midpointParameter;
                first.sourceParameterEnd =
                    firstVertices.back().sourceParameter;
                first.reverse =
                    rotated.front().reverseRelativeToInput;
                candidate.fragments.push_back(first);

                for (std::size_t offset = 1U;
                    offset < rotated.size(); ++offset)
                {
                    const auto vertices = orientedVertices(rotated[offset]);
                    if (vertices.size() < 2U) return std::nullopt;
                    ProcessPathFragment fragment;
                    fragment.entityId =
                        rotated[offset].entity->entityId;
                    fragment.fragmentOrder =
                        static_cast<int>(candidate.fragments.size());
                    fragment.sourceParameterBegin =
                        vertices.front().sourceParameter;
                    fragment.sourceParameterEnd =
                        vertices.back().sourceParameter;
                    fragment.reverse =
                        rotated[offset].reverseRelativeToInput;
                    candidate.fragments.push_back(fragment);
                }

                ProcessPathFragment last;
                last.entityId = rotated.front().entity->entityId;
                last.fragmentOrder =
                    static_cast<int>(candidate.fragments.size());
                last.sourceParameterBegin =
                    firstVertices.front().sourceParameter;
                last.sourceParameterEnd = midpointParameter;
                last.reverse =
                    rotated.front().reverseRelativeToInput;
                candidate.fragments.push_back(last);

                std::map<EntityId, int> fragmentCounts;
                std::map<EntityId, double> fragmentLengths;
                std::vector<std::vector<geometry::PathVertex3D>>
                    fragmentPaths;
                for (const ProcessPathFragment& fragment :
                    candidate.fragments)
                {
                    auto path = fragmentVertices
                    (
                        *entities.at(fragment.entityId),
                        fragment.sourceParameterBegin,
                        fragment.sourceParameterEnd,
                        fragment.reverse
                    );
                    if (!path.has_value())
                    {
                        return std::nullopt;
                    }
                    ++fragmentCounts[fragment.entityId];
                    fragmentLengths[fragment.entityId] +=
                        pathLength(*path);
                    fragmentPaths.push_back(std::move(*path));
                }
                for (const DirectedEntity& directed : rotated)
                {
                    const int expected =
                        directed.entity->entityId
                            == midpointSegment->entityId ? 2 : 1;
                    if (fragmentCounts[directed.entity->entityId]
                        != expected)
                    {
                        return std::nullopt;
                    }
                    const double fullLength =
                        pathLength(directed.entity->path.vertices);
                    const double lengthTolerance =
                        std::max(1.0e-8, fullLength * 1.0e-9);
                    if (std::abs(fragmentLengths[directed.entity->entityId]
                        - fullLength) > lengthTolerance)
                    {
                        return std::nullopt;
                    }
                }
                for (std::size_t index = 1U;
                    index < fragmentPaths.size(); ++index)
                {
                    if (distance(fragmentPaths[index - 1U].back().position,
                        fragmentPaths[index].front().position)
                        > policy.connectionTolerance)
                    {
                        return std::nullopt;
                    }
                }
                if (fragmentPaths.empty()
                    || distance(fragmentPaths.back().back().position,
                        fragmentPaths.front().front().position)
                        > policy.connectionTolerance)
                {
                    return std::nullopt;
                }
                fragmentTraversalBuilt = true;

                if (requireMatchingExit)
                {
                    const ExitResult exit = resolveExit
                    (
                        candidate.fragments, entities, section,
                        projectionTolerance, candidate.zone
                    );
                    if (!exit.zone.has_value()
                        || *exit.zone != candidate.zone)
                    {
                        return std::nullopt;
                    }
                    exitResolved = true;
                    candidate.exitConfidence = exit.confidence;
                    candidate.exitReliableLength = exit.reliableLength;
                    candidate.finalEntityId = exit.finalEntityId;
                    candidate.finalParameterBegin =
                        exit.finalParameterBegin;
                    candidate.finalParameterEnd =
                        exit.finalParameterEnd;
                    candidate.exitUsedFallback = exit.usedFallback;
                }

                DirectedEntity& entry = candidate.traversal.entities.front();
                scoreEntrySmoothness
                (
                    entry, currentPosition, midpointSegment->end,
                    tubeCenter, policy.connectionTolerance
                );
                candidate.traversal.entryAxisReversalCount =
                    entry.entryAxisReversalCount;
                candidate.traversal.entryTangentCost =
                    entry.entryTangentCost;
                candidate.traversal.stableSourceIndex =
                    midpointSegment->sourceIndex;
                candidate.traversal.stableEntityId =
                    midpointSegment->entityId;
                scoreTraversal(candidate.traversal,
                    currentPosition, section);
                return candidate;
            }

            template<typename Candidate>
            static bool candidateLess
            (
                const Candidate& left,
                const Candidate& right,
                bool forcedTop,
                double projectionTolerance,
                ProcessOrderingStrategy selectionStrategy
            )
            {
                const double lengthTieTolerance =
                    std::max(1.0e-9, projectionTolerance * 1.0e-3);
                if (forcedTop
                    && std::abs(left.runLength - right.runLength)
                        > lengthTieTolerance)
                {
                    return left.runLength > right.runLength;
                }
                if (!forcedTop)
                {
                    if (std::abs(left.confidence - right.confidence)
                        > kCalculationEpsilon)
                    {
                        return left.confidence > right.confidence;
                    }
                    if (std::abs(left.runLength - right.runLength)
                        > kCalculationEpsilon)
                    {
                        return left.runLength > right.runLength;
                    }
                }
                if (std::abs(left.maximumShellDeviation
                    - right.maximumShellDeviation) > kCalculationEpsilon)
                {
                    return left.maximumShellDeviation
                        < right.maximumShellDeviation;
                }
                if (forcedTop
                    && std::abs(left.confidence - right.confidence)
                        > kCalculationEpsilon)
                {
                    return left.confidence > right.confidence;
                }
                if (forcedTop
                    && std::abs(left.runLength - right.runLength)
                        > kCalculationEpsilon)
                {
                    return left.runLength > right.runLength;
                }
                if (!forcedTop)
                {
                    if (traversalLess(left.traversal,
                        right.traversal, selectionStrategy))
                    {
                        return true;
                    }
                    if (traversalLess(right.traversal,
                        left.traversal, selectionStrategy))
                    {
                        return false;
                    }
                }
                if (left.midpointSourceIndex !=
                    right.midpointSourceIndex)
                {
                    return left.midpointSourceIndex
                        < right.midpointSourceIndex;
                }
                if (left.midpointEntityId != right.midpointEntityId)
                    return left.midpointEntityId < right.midpointEntityId;
                if (left.midpointSourceParameter
                    != right.midpointSourceParameter)
                {
                    return left.midpointSourceParameter
                        < right.midpointSourceParameter;
                }
                return left.direction < right.direction;
            }
        };

        QVariantMap breakDiagnosticValues
        (
            const BreakBoundaryTraversalReport& report
        )
        {
            QVariantMap values;
            values.insert(QStringLiteral("breakStartSummary"), true);
            values.insert(QStringLiteral("groupId"), report.groupId);
            values.insert(QStringLiteral("boundaryRank"), report.boundaryRank);
            values.insert(QStringLiteral("boundaryPairId"),
                report.boundaryPairId);
            values.insert(QStringLiteral("forcedTopMidpoint"),
                report.forcedTopMidpoint);
            values.insert(QStringLiteral("preferredStartZone"),
                machining::tubeZoneName(report.preferredStartZone));
            values.insert(QStringLiteral("candidateRunCount"),
                report.candidateRunCount);
            values.insert(QStringLiteral("candidateRuns"),
                report.candidateRuns);
            values.insert(QStringLiteral("strategy"),
                report.forcedTopMidpoint
                    ? QStringLiteral("ForcedTopMidpoint")
                    : QStringLiteral("StrongZoneMidpoint"));
            values.insert(QStringLiteral("selectedZone"),
                machining::tubeZoneName(report.startZone));
            values.insert(QStringLiteral("selectedRunLength"),
                report.selectedRunLength);
            values.insert(QStringLiteral("selectedMaximumShellDeviation"),
                report.selectedMaximumShellDeviation);
            values.insert(QStringLiteral("selectedConfidence"),
                report.selectedConfidence);
            values.insert(QStringLiteral("selectedMidpoint"),
                QStringLiteral("%1,%2,%3")
                    .arg(report.selectedMidpoint.x, 0, 'g', 15)
                    .arg(report.selectedMidpoint.y, 0, 'g', 15)
                    .arg(report.selectedMidpoint.z, 0, 'g', 15));
            values.insert(QStringLiteral("selectedEntityId"),
                QVariant::fromValue<qulonglong>(report.selectedEntityId));
            values.insert(QStringLiteral("selectedSourceParameter"),
                report.selectedSourceParameter);
            values.insert(QStringLiteral("exitZone"),
                report.exitZone.has_value()
                    ? machining::tubeZoneName(*report.exitZone)
                    : QStringLiteral("Unknown"));
            values.insert(QStringLiteral("exitConfidence"),
                report.exitConfidence);
            values.insert(QStringLiteral("exitReliableLength"),
                report.exitReliableLength);
            values.insert(QStringLiteral("finalEntityId"),
                QVariant::fromValue<qulonglong>(report.finalEntityId));
            values.insert(QStringLiteral("finalParameterBegin"),
                report.finalParameterBegin);
            values.insert(QStringLiteral("finalParameterEnd"),
                report.finalParameterEnd);
            values.insert(QStringLiteral("exitUsedFallback"),
                report.exitUsedFallback);
            values.insert(QStringLiteral("direction"), report.direction);
            values.insert(QStringLiteral("fragmentCount"),
                report.fragmentCount);
            values.insert(QStringLiteral("midpointFragmentUsed"),
                report.fragmentCount > 0);
            values.insert(QStringLiteral("nextPartitionId"),
                report.nextPartitionId);
            values.insert(QStringLiteral("partitionMappingFound"),
                report.partitionMappingFound);
            values.insert(QStringLiteral("partitionStartSucceeded"),
                report.partitionStartSucceeded);
            values.insert(QStringLiteral("status"), report.status);
            values.insert(QStringLiteral("failureReason"),
                report.failureReason);
            return values;
        }

        Diagnostic breakStartDiagnostic
        (
            const OperationContext& context,
            const BreakBoundaryTraversalReport& report
        )
        {
            return planningDiagnostic
            (
                context,
                DiagnosticCode::ProcessPlanningBreakStartSummary,
                QStringLiteral("加工断面已从可靠强区位边段中点建立闭环遍历。"),
                QStringLiteral("Break boundary traversal uses complementary source fragments around a strong-zone arc-length midpoint."),
                breakDiagnosticValues(report),
                report.exitUsedFallback
                    ? DiagnosticSeverity::Warning
                    : DiagnosticSeverity::Info
            );
        }

