        class ClosedLoopTraversalBuilder
        {
        public:
            struct Result
            {
                std::optional<GroupTraversal> traversal;
                ClosedLoopTraversalReport report;
            };

            static Result build
            (
                const ProcessGroup& group,
                const std::unordered_map<EntityId, const PlanningEntity*>& entities,
                const Vector3d& currentPosition,
                const ProcessPlanningPolicy& policy,
                const std::optional<machining::TubeSectionModel>& section,
                const std::optional<Vector2d>& tubeCenter,
                ProcessOrderingStrategy selectionStrategy,
                const TraversalSelectionContext* selection = nullptr
            )
            {
                Result result;
                result.report.groupId = group.groupId;
                result.report.memberEntityIds = group.entityIds;
                std::sort(result.report.memberEntityIds.begin(), result.report.memberEntityIds.end());
                result.report.memberCount = static_cast<int>(group.entityIds.size());

                struct Edge
                {
                    const PlanningEntity* entity = nullptr;
                    Vector3d sourceStart;
                    Vector3d sourceEnd;
                    int startNode = -1;
                    int endNode = -1;
                };
                std::vector<Edge> edges;
                edges.reserve(group.entityIds.size());
                std::set<EntityId> uniqueIds;
                for (const EntityId entityId : group.entityIds)
                {
                    const auto found = entities.find(entityId);
                    if (found == entities.end() || found->second == nullptr
                        || !uniqueIds.insert(entityId).second)
                    {
                        result.report.failureReason = QStringLiteral("Closed-loop member is missing or duplicated.");
                        return result;
                    }
                    const PlanningEntity& entity = *found->second;
                    if (entity.path.closed || entity.path.vertices.size() < 2U)
                    {
                        result.report.failureReason = entity.path.closed
                            ? QStringLiteral("Multi-entity closed loop contains a semantically closed member.")
                            : QStringLiteral("Closed-loop member has fewer than two path points.");
                        return result;
                    }
                    const Vector3d sourceStart = entity.path.vertices.front().position;
                    const Vector3d sourceEnd = entity.path.vertices.back().position;
                    if (!std::isfinite(sourceStart.x) || !std::isfinite(sourceStart.y)
                        || !std::isfinite(sourceStart.z) || !std::isfinite(sourceEnd.x)
                        || !std::isfinite(sourceEnd.y) || !std::isfinite(sourceEnd.z))
                    {
                        result.report.failureReason = QStringLiteral("Closed-loop member endpoint is not finite.");
                        return result;
                    }
                    edges.push_back({ &entity, sourceStart, sourceEnd });
                }
                std::sort(edges.begin(), edges.end(), [](const Edge& left, const Edge& right)
                {
                    if (left.entity->sourceIndex != right.entity->sourceIndex)
                        return left.entity->sourceIndex < right.entity->sourceIndex;
                    return left.entity->entityId < right.entity->entityId;
                });

                const std::size_t endpointCount = edges.size() * 2U;
                std::vector<std::size_t> parents(endpointCount);
                for (std::size_t index = 0; index < endpointCount; ++index) parents[index] = index;
                const auto findRoot = [&parents](std::size_t value)
                {
                    std::size_t root = value;
                    while (parents[root] != root) root = parents[root];
                    while (parents[value] != value)
                    {
                        const std::size_t next = parents[value];
                        parents[value] = root;
                        value = next;
                    }
                    return root;
                };
                const auto endpoint = [&edges](std::size_t index) -> const Vector3d&
                {
                    const Edge& edge = edges[index / 2U];
                    return index % 2U == 0U ? edge.sourceStart : edge.sourceEnd;
                };
                for (std::size_t left = 0; left < endpointCount; ++left)
                {
                    for (std::size_t right = left + 1U; right < endpointCount; ++right)
                    {
                        if (distance(endpoint(left), endpoint(right)) > policy.connectionTolerance) continue;
                        const std::size_t leftRoot = findRoot(left);
                        const std::size_t rightRoot = findRoot(right);
                        if (leftRoot != rightRoot) parents[rightRoot] = leftRoot;
                    }
                }

                std::map<std::size_t, int> nodeByRoot;
                for (std::size_t edgeIndex = 0; edgeIndex < edges.size(); ++edgeIndex)
                {
                    const std::size_t startRoot = findRoot(edgeIndex * 2U);
                    const std::size_t endRoot = findRoot(edgeIndex * 2U + 1U);
                    const auto nodeFor = [&nodeByRoot](std::size_t root)
                    {
                        const auto inserted = nodeByRoot.emplace
                            (root, static_cast<int>(nodeByRoot.size()));
                        return inserted.first->second;
                    };
                    edges[edgeIndex].startNode = nodeFor(startRoot);
                    edges[edgeIndex].endNode = nodeFor(endRoot);
                }
                result.report.nodeCount = static_cast<int>(nodeByRoot.size());

                std::vector<std::vector<std::size_t>> adjacency(nodeByRoot.size());
                for (std::size_t edgeIndex = 0; edgeIndex < edges.size(); ++edgeIndex)
                {
                    adjacency[static_cast<std::size_t>(edges[edgeIndex].startNode)].push_back(edgeIndex);
                    adjacency[static_cast<std::size_t>(edges[edgeIndex].endNode)].push_back(edgeIndex);
                }
                for (const auto& incidentEdges : adjacency)
                {
                    if (incidentEdges.size() > 2U) ++result.report.branchNodeCount;
                    if (incidentEdges.size() != 2U) ++result.report.invalidDegreeNodeCount;
                }

                std::vector<bool> visitedNodes(adjacency.size(), false);
                for (std::size_t node = 0; node < adjacency.size(); ++node)
                {
                    if (visitedNodes[node] || adjacency[node].empty()) continue;
                    ++result.report.connectedComponentCount;
                    std::vector<std::size_t> pending{ node };
                    visitedNodes[node] = true;
                    while (!pending.empty())
                    {
                        const std::size_t currentNode = pending.back();
                        pending.pop_back();
                        for (const std::size_t edgeIndex : adjacency[currentNode])
                        {
                            const Edge& edge = edges[edgeIndex];
                            const std::size_t nextNode = static_cast<std::size_t>
                                (edge.startNode == static_cast<int>(currentNode)
                                    ? edge.endNode : edge.startNode);
                            if (!visitedNodes[nextNode])
                            {
                                visitedNodes[nextNode] = true;
                                pending.push_back(nextNode);
                            }
                        }
                    }
                }

                result.report.simpleLoopValid = result.report.connectedComponentCount == 1
                    && result.report.branchNodeCount == 0
                    && result.report.invalidDegreeNodeCount == 0
                    && edges.size() == adjacency.size();
                if (!result.report.simpleLoopValid)
                {
                    result.report.failureReason = QStringLiteral("Closed-loop endpoint graph is not one simple cycle.");
                    return result;
                }

                std::optional<GroupTraversal> best;
                std::vector<GroupTraversal> connectionCandidates;
                int bestStartEndpoint = -1;
                int bestLoopDirection = -1;
                int wrongZoneRejectedCount = 0;
                for (std::size_t startEdgeIndex = 0; startEdgeIndex < edges.size(); ++startEdgeIndex)
                {
                    for (const bool startReverse : { false, true })
                    {
                        const Edge& startEdge = edges[startEdgeIndex];
                        if (!directionAllowed(*startEdge.entity, startReverse,
                            policy.allowReverse)
                            || !manualDirectionAllowed(*startEdge.entity,
                                startReverse, selection))
                        {
                            continue;
                        }

                        GroupTraversal candidate;
                        candidate.groupId = group.groupId;
                        std::vector<bool> used(edges.size(), false);
                        int currentNode = startReverse ? startEdge.endNode : startEdge.startNode;
                        const int initialNode = currentNode;
                        Vector3d previousEnd;
                        bool hasPreviousEnd = false;
                        Vector3d firstNextPoint;
                        bool candidateValid = true;

                        for (std::size_t step = 0; step < edges.size(); ++step)
                        {
                            std::vector<std::size_t> unusedIncident;
                            for (const std::size_t edgeIndex : adjacency[static_cast<std::size_t>(currentNode)])
                            {
                                if (!used[edgeIndex]
                                    && std::find(unusedIncident.begin(), unusedIncident.end(), edgeIndex)
                                        == unusedIncident.end())
                                    unusedIncident.push_back(edgeIndex);
                            }
                            const std::size_t edgeIndex = step == 0U
                                ? startEdgeIndex
                                : unusedIncident.size() == 1U
                                    ? unusedIncident.front() : edges.size();
                            if (edgeIndex >= edges.size() || used[edgeIndex])
                            {
                                candidateValid = false;
                                break;
                            }

                            const Edge& edge = edges[edgeIndex];
                            const bool reverse = edge.endNode == currentNode;
                            if ((edge.startNode != currentNode && edge.endNode != currentNode)
                                || !directionAllowed(*edge.entity, reverse,
                                    policy.allowReverse)
                                || !manualDirectionAllowed(*edge.entity,
                                    reverse, selection))
                            {
                                candidateValid = false;
                                break;
                            }
                            std::vector<Vector3d> points = directedPoints(*edge.entity, reverse);
                            if (points.size() < 2U
                                || (hasPreviousEnd
                                    && distance(previousEnd, points.front()) > policy.connectionTolerance))
                            {
                                candidateValid = false;
                                break;
                            }
                            const double threshold = entryThreshold(policy.connectionTolerance);
                            const auto nextPoint = std::find_if
                            (
                                points.cbegin() + 1,
                                points.cend(),
                                [&points, threshold](const Vector3d& point)
                                { return distance(points.front(), point) > threshold; }
                            );
                            if (nextPoint == points.cend())
                            {
                                candidateValid = false;
                                break;
                            }

                            DirectedEntity directed;
                            directed.entity = edge.entity;
                            directed.reverseRelativeToInput = reverse;
                            directed.selectedStartParameter = edge.entity->startParameter;
                            directed.start = points.front();
                            directed.end = points.back();
                            candidate.entities.push_back(directed);
                            if (step == 0U) firstNextPoint = *nextPoint;
                            previousEnd = directed.end;
                            hasPreviousEnd = true;
                            used[edgeIndex] = true;
                            currentNode = reverse ? edge.startNode : edge.endNode;
                        }

                        if (!candidateValid || currentNode != initialNode
                            || candidate.entities.size() != edges.size()) continue;
                        candidate.start = candidate.entities.front().start;
                        if (distance(candidate.entities.back().end, candidate.start)
                            > policy.connectionTolerance) continue;
                        candidate.end = candidate.start;
                        const DirectedEntity& firstDirected =
                            candidate.entities.front();
                        const auto firstVertices = firstDirected.reverseRelativeToInput
                            ? std::vector<geometry::PathVertex3D>
                                (firstDirected.entity->path.vertices.rbegin(),
                                    firstDirected.entity->path.vertices.rend())
                            : firstDirected.entity->path.vertices;
                        const std::optional<double> entryParameter =
                            firstVertices.empty() ? std::nullopt
                                : std::optional<double>
                                    (firstVertices.front().sourceParameter);
                        if (selection != nullptr
                            && selection->hardZoneConstraint
                            && selection->requiredEntryZone.has_value()
                            && section.has_value())
                        {
                            auto entry = classifyZoneEntry
                            (
                                ZoneEntryCandidateKind::ClosedLoopConnection,
                                firstDirected.entity->entityId,
                                entryParameter,
                                firstDirected.reverseRelativeToInput,
                                candidate.start, firstNextPoint,
                                firstDirected.entity->sourceKind, *section,
                                selection->projectionTolerance
                            );
                            const bool manualStartMatches =
                                !firstDirected.entity->manualStartParameter.has_value()
                                || (entryParameter.has_value()
                                    && std::abs(*entryParameter
                                        - *firstDirected.entity->manualStartParameter)
                                        <= 1.0e-10);
                            const bool otherManualStartExists = std::any_of
                            (
                                candidate.entities.cbegin() + 1,
                                candidate.entities.cend(),
                                [](const DirectedEntity& directed)
                                {
                                    return directed.entity != nullptr
                                        && directed.entity->manualStartParameter.has_value();
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
                            candidate.selectedEntry = std::move(entry);
                            scoreUnwrappedEntry(candidate.entities.front(),
                                selection->previousTransferAnchor, firstNextPoint,
                                *section, policy.connectionTolerance,
                                selection->projectionTolerance);
                            candidate.entryRefinementMode =
                                QStringLiteral("NearestConnection");
                            candidate.previousCutEnd =
                                selection->previousCutEnd;
                            candidate.previousTransferAnchor =
                                selection->previousTransferAnchor;
                            candidate.entryTravelDistance =
                                distance(selection->previousTransferAnchor,
                                    candidate.start);
                            candidate.nearestConnectionDistance =
                                candidate.entryTravelDistance;
                        }
                        else
                        {
                            scoreEntrySmoothness
                            (
                                candidate.entities.front(), currentPosition,
                                firstNextPoint, tubeCenter,
                                policy.connectionTolerance
                            );
                        }
                        candidate.entryAxisReversalCount =
                            candidate.entities.front().entryAxisReversalCount;
                        candidate.entryTangentCost = candidate.entities.front().entryTangentCost;
                        candidate.stableSourceIndex = candidate.entities.front().entity->sourceIndex;
                        candidate.stableEntityId = candidate.entities.front().entity->entityId;
                        scoreTraversal(candidate, currentPosition, section);
                        ++result.report.candidateCount;
                        if (selection != nullptr
                            && selection->hardZoneConstraint)
                        {
                            const double dotValue = std::clamp
                            (
                                1.0 - candidate.entryTangentCost,
                                -1.0,
                                1.0
                            );
                            candidate.approachCutDot = dotValue;
                            candidate.approachCutAngle =
                                std::acos(dotValue)
                                * 180.0 / 3.14159265358979323846;
                            connectionCandidates.push_back
                                (std::move(candidate));
                            continue;
                        }

                        const int startEndpoint = startReverse ? 1 : 0;
                        const int loopDirection = startReverse ? 1 : 0;
                        const auto stableLess = [&candidate, startEndpoint, loopDirection,
                            &best, bestStartEndpoint, bestLoopDirection,
                            selectionStrategy, selection]()
                        {
                            if (!best.has_value()) return true;
                            if (selection != nullptr
                                && zoneConstrainedTraversalLess(candidate,
                                    *best, *selection, selectionStrategy))
                            {
                                return true;
                            }
                            if (selection != nullptr
                                && zoneConstrainedTraversalLess(*best,
                                    candidate, *selection,
                                    selectionStrategy))
                            {
                                return false;
                            }
                            if (selection == nullptr
                                && traversalLess(candidate, *best,
                                    selectionStrategy))
                            {
                                return true;
                            }
                            if (selection == nullptr
                                && traversalLess(*best, candidate,
                                    selectionStrategy))
                            {
                                return false;
                            }
                            if (startEndpoint != bestStartEndpoint)
                                return startEndpoint < bestStartEndpoint;
                            if (loopDirection != bestLoopDirection)
                                return loopDirection < bestLoopDirection;
                            std::vector<EntityId> candidateOrder;
                            std::vector<EntityId> bestOrder;
                            for (const DirectedEntity& directed : candidate.entities)
                                candidateOrder.push_back(directed.entity->entityId);
                            for (const DirectedEntity& directed : best->entities)
                                bestOrder.push_back(directed.entity->entityId);
                            return candidateOrder < bestOrder;
                        };
                        if (stableLess())
                        {
                            best = std::move(candidate);
                            bestStartEndpoint = startEndpoint;
                            bestLoopDirection = loopDirection;
                        }
                    }
                }

                if (selection != nullptr
                    && selection->hardZoneConstraint
                    && !connectionCandidates.empty())
                {
                    const double tieTolerance = std::max
                    (
                        policy.connectionDistanceTieTolerance,
                        kCalculationEpsilon
                    );
                    const auto connectionLess =
                        [tieTolerance](const GroupTraversal& left,
                            const GroupTraversal& right)
                    {
                        if (std::abs(left.entryTravelDistance
                            - right.entryTravelDistance) > tieTolerance)
                        {
                            return left.entryTravelDistance
                                < right.entryTravelDistance;
                        }
                        if (std::abs(left.entryTangentCost
                            - right.entryTangentCost)
                            > kCalculationEpsilon)
                        {
                            return left.entryTangentCost
                                < right.entryTangentCost;
                        }
                        if (left.stableEntityId != right.stableEntityId)
                            return left.stableEntityId
                                < right.stableEntityId;
                        const bool leftReverse = !left.entities.empty()
                            && left.entities.front()
                                .reverseRelativeToInput;
                        const bool rightReverse = !right.entities.empty()
                            && right.entities.front()
                                .reverseRelativeToInput;
                        return leftReverse < rightReverse;
                    };
                    auto selected = std::min_element
                        (connectionCandidates.begin(),
                            connectionCandidates.end(), connectionLess);
                    best = *selected;
                    const double samePointTolerance = std::max
                        (policy.connectionDistanceTieTolerance, 1.0e-8);
                    best->forwardAngle =
                        std::numeric_limits<double>::quiet_NaN();
                    best->reverseAngle =
                        std::numeric_limits<double>::quiet_NaN();
                    for (const GroupTraversal& candidate :
                        connectionCandidates)
                    {
                        if (distance(candidate.start, best->start)
                            > samePointTolerance
                            || candidate.entities.empty())
                        {
                            continue;
                        }
                        if (candidate.entities.front()
                            .reverseRelativeToInput)
                        {
                            best->reverseAngle =
                                candidate.approachCutAngle;
                        }
                        else
                        {
                            best->forwardAngle =
                                candidate.approachCutAngle;
                        }
                    }
                }
                if (!best.has_value())
                {
                    result.report.wrongZoneRejectedCount =
                        wrongZoneRejectedCount;
                    result.report.failureReason = QStringLiteral("No complete loop traversal satisfies member direction constraints.");
                    return result;
                }
                result.report.status = QStringLiteral("Success");
                result.report.wrongZoneRejectedCount =
                    wrongZoneRejectedCount;
                best->wrongZoneRejectedCount = wrongZoneRejectedCount;
                for (const DirectedEntity& directed : best->entities)
                {
                    result.report.selectedOrder.push_back(directed.entity->entityId);
                    result.report.selectedReverse.push_back(directed.reverseRelativeToInput);
                }
                result.traversal = std::move(best);
                return result;
            }
        };

