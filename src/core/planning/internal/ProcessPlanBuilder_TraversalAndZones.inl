        QString sideName(BoundarySide side)
        {
            switch (side)
            {
            case BoundarySide::Left: return QStringLiteral("Left");
            case BoundarySide::OnBoundary: return QStringLiteral("OnBoundary");
            case BoundarySide::Right: return QStringLiteral("Right");
            case BoundarySide::Mixed: return QStringLiteral("Mixed");
            case BoundarySide::Indeterminate: return QStringLiteral("Indeterminate");
            }
            return QStringLiteral("Indeterminate");
        }

        XBounds groupXBounds
        (
            const ProcessGroup& group,
            const std::unordered_map<EntityId, const PlanningEntity*>& entities
        )
        {
            XBounds bounds;
            for (const EntityId entityId : group.entityIds)
            {
                const auto found = entities.find(entityId);
                if (found == entities.end()) return {};
                for (const geometry::PathVertex3D& vertex : found->second->path.vertices)
                {
                    if (!bounds.valid)
                    {
                        bounds.valid = true;
                        bounds.minimum = bounds.maximum = vertex.position.x;
                    }
                    else
                    {
                        bounds.minimum = std::min(bounds.minimum, vertex.position.x);
                        bounds.maximum = std::max(bounds.maximum, vertex.position.x);
                    }
                }
            }
            return bounds;
        }

        XBounds boundaryXBounds(const BoundaryData& boundary)
        {
            XBounds bounds;
            for (const machining::UnwrappedBoundaryPoint& point : boundary.analysis.unwrappedBoundary)
            {
                if (!bounds.valid)
                {
                    bounds.valid = true;
                    bounds.minimum = bounds.maximum = point.x;
                }
                else
                {
                    bounds.minimum = std::min(bounds.minimum, point.x);
                    bounds.maximum = std::max(bounds.maximum, point.x);
                }
            }
            return bounds;
        }

        QVariantMap boundaryDiagnosticValues
        (
            const ProcessPlanningInput& input,
            const ProcessPlanningPolicy& policy,
            const BoundaryData& boundary,
            const ProcessGroup& otherGroup,
            const std::unordered_map<EntityId, const PlanningEntity*>& entities,
            int boundarySpatialRank,
            BoundarySide relativeSide,
            BoundarySide reverseRelativeSide = BoundarySide::Indeterminate
        )
        {
            QVariantMap values = diagnosticValues
                (input, policy, 0U, 0U, boundary.pairId, otherGroup.groupId);
            const XBounds boundaryBounds = boundaryXBounds(boundary);
            const XBounds otherBounds = groupXBounds(otherGroup, entities);
            values.insert(QStringLiteral("boundaryGroupId"), boundary.groupId);
            values.insert(QStringLiteral("otherGroupId"), otherGroup.groupId);
            values.insert(QStringLiteral("boundarySpatialRank"), boundarySpatialRank);
            values.insert(QStringLiteral("relativeSide"), sideName(relativeSide));
            values.insert(QStringLiteral("reverseRelativeSide"), sideName(reverseRelativeSide));
            values.insert(QStringLiteral("boundaryMinimumX"),
                boundaryBounds.valid ? boundaryBounds.minimum : 0.0);
            values.insert(QStringLiteral("boundaryMaximumX"),
                boundaryBounds.valid ? boundaryBounds.maximum : 0.0);
            values.insert(QStringLiteral("otherMinimumX"),
                otherBounds.valid ? otherBounds.minimum : 0.0);
            values.insert(QStringLiteral("otherMaximumX"),
                otherBounds.valid ? otherBounds.maximum : 0.0);
            return values;
        }

        std::vector<Vector3d> directedPoints(const PlanningEntity& entity, bool reverse)
        {
            std::vector<Vector3d> points;
            points.reserve(entity.path.vertices.size());
            for (const geometry::PathVertex3D& vertex : entity.path.vertices) points.push_back(vertex.position);
            if (!reverse || points.size() < 2U) return points;
            if (entity.path.closed)
            {
                std::reverse(points.begin() + 1, points.end());
            }
            else
            {
                std::reverse(points.begin(), points.end());
            }
            return points;
        }

        std::vector<Vector3d> directedTraversalPoints(const DirectedEntity& directed)
        {
            if (directed.entity == nullptr) return {};
            const PlanningEntity& entity = *directed.entity;
            if (!entity.path.closed || entity.path.vertices.size() < 2U)
                return directedPoints(entity, directed.reverseRelativeToInput);

            std::size_t startIndex = 0U;
            if (directed.selectedStartParameter.has_value()
                && std::isfinite(*directed.selectedStartParameter))
            {
                double bestDifference = std::numeric_limits<double>::max();
                for (std::size_t index = 0U; index < entity.path.vertices.size(); ++index)
                {
                    const double difference = std::abs
                        (entity.path.vertices[index].sourceParameter
                            - *directed.selectedStartParameter);
                    if (difference < bestDifference)
                    {
                        bestDifference = difference;
                        startIndex = index;
                    }
                }
            }

            const std::size_t pointCount = entity.path.vertices.size();
            std::vector<Vector3d> points;
            points.reserve(pointCount + 1U);
            for (std::size_t offset = 0U; offset < pointCount; ++offset)
            {
                const std::size_t index = directed.reverseRelativeToInput
                    ? (startIndex + pointCount - offset) % pointCount
                    : (startIndex + offset) % pointCount;
                points.push_back(entity.path.vertices[index].position);
            }
            points.push_back(points.front());
            return points;
        }

        machining::TubeSurfaceRegion directedEndpointRegion
        (
            const DirectedEntity& directed,
            bool entry,
            const machining::TubeSectionModel& section,
            const SectionBounds& bounds,
            double tolerance
        )
        {
            const std::vector<Vector3d> points = directedPoints
                (*directed.entity, directed.reverseRelativeToInput);
            if (points.size() >= 2U)
            {
                const Vector3d& endpoint = entry ? points.front() : points.back();
                for (std::size_t offset = 1U; offset < points.size(); ++offset)
                {
                    const Vector3d& neighbor = entry
                        ? points[offset] : points[points.size() - 1U - offset];
                    if (distance(endpoint, neighbor) <= kCalculationEpsilon) continue;
                    const Vector3d middle
                    {
                        (endpoint.x + neighbor.x) * 0.5,
                        (endpoint.y + neighbor.y) * 0.5,
                        (endpoint.z + neighbor.z) * 0.5
                    };
                    const auto region = classifySurfacePoints
                        ({ endpoint, middle, neighbor }, section, bounds, tolerance);
                    if (region != machining::TubeSurfaceRegion::Unknown) return region;
                    break;
                }
            }
            return classifySurfacePoint
                (entry ? directed.start : directed.end, section, bounds, tolerance);
        }

        ProcessSurfaceFootprint buildSurfaceFootprint
        (
            const ProcessGroup& group,
            const std::unordered_map<EntityId, const PlanningEntity*>& entities,
            const machining::TubeSectionModel& section,
            const SectionBounds& bounds,
            const std::vector<double>& cumulative,
            double tolerance
        )
        {
            ProcessSurfaceFootprint footprint;
            std::vector<Vector3d> points;
            std::vector<double> xCoordinates;
            std::vector<double> perimeterPositions;
            const PlanningEntity* firstEntity = nullptr;
            for (const EntityId entityId : group.entityIds)
            {
                const auto found = entities.find(entityId);
                if (found == entities.end() || found->second == nullptr) continue;
                const PlanningEntity& entity = *found->second;
                if (firstEntity == nullptr
                    || entity.sourceIndex < firstEntity->sourceIndex
                    || (entity.sourceIndex == firstEntity->sourceIndex
                        && entity.entityId < firstEntity->entityId))
                    firstEntity = &entity;
                for (const auto& vertex : entity.path.vertices)
                {
                    const Vector3d& point = vertex.position;
                    if (!std::isfinite(point.x) || !std::isfinite(point.y)
                        || !std::isfinite(point.z)) continue;
                    points.push_back(point);
                    xCoordinates.push_back(point.x);
                    const SectionProjection projection = projectToSection
                        ({ point.y, point.z }, section.geometry, cumulative);
                    if (projection.valid) perimeterPositions.push_back
                        (projection.perimeterPosition);
                }
            }
            if (points.empty()) return footprint;

            footprint.dominantRegion = group.kind == ProcessGroupKind::BreakBoundary
                    || group.kind == ProcessGroupKind::WasteBoundary
                ? machining::TubeSurfaceRegion::Mixed
                : classifySurfacePoints(points, section, bounds, tolerance);
            std::sort(xCoordinates.begin(), xCoordinates.end());
            footprint.minimumX = xCoordinates.front();
            footprint.maximumX = xCoordinates.back();
            const std::size_t middle = xCoordinates.size() / 2U;
            footprint.anchorX = xCoordinates.size() % 2U == 0U
                ? (xCoordinates[middle - 1U] + xCoordinates[middle]) * 0.5
                : xCoordinates[middle];
            if (!perimeterPositions.empty())
            {
                const auto extrema = std::minmax_element
                    (perimeterPositions.begin(), perimeterPositions.end());
                footprint.minimumPerimeterPosition = *extrema.first;
                footprint.maximumPerimeterPosition = *extrema.second;
            }
            if (surfaceRegionIndex(footprint.dominantRegion) >= 0)
            {
                footprint.entryRegion = footprint.dominantRegion;
                footprint.exitRegion = footprint.dominantRegion;
            }
            else if (firstEntity != nullptr && !firstEntity->path.vertices.empty())
            {
                const Vector3d& first = firstEntity->path.vertices.front().position;
                const Vector3d& last = firstEntity->path.vertices.back().position;
                footprint.entryRegion = classifySurfacePoint
                    (first, section, bounds, tolerance);
                footprint.exitRegion = classifySurfacePoint
                    (last, section, bounds, tolerance);
            }
            return footprint;
        }

        ProcessSurfaceFootprint footprintForTraversal
        (
            ProcessSurfaceFootprint footprint,
            const GroupTraversal& traversal,
            const machining::TubeSectionModel& section,
            const SectionBounds& bounds,
            double tolerance
        )
        {
            if (surfaceRegionIndex(footprint.dominantRegion) >= 0)
            {
                footprint.entryRegion = footprint.dominantRegion;
                footprint.exitRegion = footprint.dominantRegion;
                return footprint;
            }
            if (!traversal.entities.empty())
            {
                footprint.entryRegion = directedEndpointRegion
                    (traversal.entities.front(), true, section, bounds, tolerance);
                footprint.exitRegion = directedEndpointRegion
                    (traversal.entities.back(), false, section, bounds, tolerance);
            }
            return footprint;
        }

        int traversalPerimeterDirection
        (
            const GroupTraversal& traversal,
            const machining::TubeSectionModel& section,
            const std::vector<double>& cumulative,
            double tolerance
        )
        {
            if (traversal.entities.empty() || section.geometry.perimeter <= 0.0) return 0;
            const DirectedEntity& last = traversal.entities.back();
            const std::vector<Vector3d> points = directedTraversalPoints(last);
            if (points.size() < 2U) return 0;
            const Vector3d& endpoint = points.back();
            for (std::size_t offset = 1U; offset < points.size(); ++offset)
            {
                const Vector3d& previous = points[points.size() - 1U - offset];
                if (distance(previous, endpoint) <= kCalculationEpsilon) continue;
                const SectionProjection from = projectToSection
                    ({ previous.y, previous.z }, section.geometry, cumulative);
                const SectionProjection to = projectToSection
                    ({ endpoint.y, endpoint.z }, section.geometry, cumulative);
                if (!from.valid || !to.valid) return 0;
                const double delta = wrappedPerimeterDelta
                    (from.perimeterPosition, to.perimeterPosition,
                        section.geometry.perimeter);
                if (delta > tolerance) return 1;
                if (delta < -tolerance) return -1;
                return 0;
            }
            return 0;
        }

        bool directionAllowed(const PlanningEntity& entity, bool reverse, bool allowReverse)
        {
            switch (entity.directionPreference)
            {
            case process::DirectionPreference::Forward: return !reverse;
            case process::DirectionPreference::Reverse: return reverse;
            case process::DirectionPreference::Auto: return !reverse || allowReverse;
            }
            return false;
        }

        bool manualDirectionAllowed
        (
            const PlanningEntity& entity,
            bool reverse,
            const TraversalSelectionContext* selection
        )
        {
            if (selection == nullptr || !selection->hardZoneConstraint)
                return true;
            switch (entity.manualDirectionPreference)
            {
            case process::DirectionPreference::Forward: return !reverse;
            case process::DirectionPreference::Reverse: return reverse;
            case process::DirectionPreference::Auto: return true;
            }
            return false;
        }

        bool hasManualEntryConstraint
        (
            const ProcessGroup& group,
            const std::unordered_map<EntityId, const PlanningEntity*>& entities
        )
        {
            return std::any_of
            (
                group.entityIds.cbegin(),
                group.entityIds.cend(),
                [&entities](EntityId entityId)
                {
                    const auto found = entities.find(entityId);
                    return found != entities.end()
                        && found->second != nullptr
                        && (found->second->manualStartParameter.has_value()
                            || found->second->manualDirectionPreference
                                != process::DirectionPreference::Auto);
                }
            );
        }

        std::optional<GroupTraversal> buildTraversal
        (
            const ProcessGroup& group,
            const std::unordered_map<EntityId, const PlanningEntity*>& entities,
            const Vector3d& currentPosition,
            bool allowReverse,
            double tolerance,
            std::optional<std::pair<EntityId, bool>> forcedStart = std::nullopt,
            bool requireAllEntities = true
        )
        {
            GroupTraversal traversal;
            traversal.groupId = group.groupId;
            std::unordered_set<EntityId> used;
            Vector3d cursor = currentPosition;

            while (used.size() < group.entityIds.size())
            {
                const PlanningEntity* selected = nullptr;
                bool selectedReverse = false;
                std::vector<Vector3d> selectedPoints;
                double bestDistance = std::numeric_limits<double>::max();

                for (const EntityId entityId : group.entityIds)
                {
                    if (used.find(entityId) != used.end()) continue;
                    const auto found = entities.find(entityId);
                    if (found == entities.end()) return std::nullopt;
                    const PlanningEntity& entity = *found->second;
                    for (const bool reverse : { false, true })
                    {
                        if (!directionAllowed(entity, reverse, allowReverse)) continue;
                        if (used.empty() && forcedStart.has_value()
                            && (forcedStart->first != entityId || forcedStart->second != reverse)) continue;
                        const std::vector<Vector3d> points = directedPoints(entity, reverse);
                        if (points.size() < 2U) continue;
                        const double candidateDistance = distance(cursor, points.front());
                        if (!used.empty() && candidateDistance > tolerance) continue;
                        const bool replace = selected == nullptr
                            || candidateDistance < bestDistance - kCalculationEpsilon
                            || (std::abs(candidateDistance - bestDistance) <= kCalculationEpsilon
                                && (entity.sourceIndex < selected->sourceIndex
                                    || (entity.sourceIndex == selected->sourceIndex && entity.entityId < selected->entityId)));
                        if (replace)
                        {
                            selected = &entity;
                            selectedReverse = reverse;
                            selectedPoints = points;
                            bestDistance = candidateDistance;
                        }
                    }
                }

                if (selected == nullptr)
                {
                    if (traversal.entities.empty() || requireAllEntities)
                        return std::nullopt;
                    break;
                }
                DirectedEntity directed;
                directed.entity = selected;
                directed.reverseRelativeToInput = selectedReverse;
                directed.selectedStartParameter = selected->startParameter;
                directed.start = selectedPoints.front();
                directed.end = selectedPoints.back();
                traversal.entities.push_back(directed);
                used.insert(selected->entityId);
                cursor = directed.end;
            }

            if (traversal.entities.empty()) return std::nullopt;
            traversal.start = traversal.entities.front().start;
            traversal.end = traversal.entities.back().end;
            if (group.closed && distance(traversal.end, traversal.start) > tolerance) return std::nullopt;
            if (group.closed) traversal.end = traversal.start;
            traversal.movementDistance = distance(currentPosition, traversal.start);
            traversal.entryAxisReversalCount =
                traversal.entities.front().entryAxisReversalCount;
            traversal.entryTangentCost = traversal.entities.front().entryTangentCost;
            traversal.stableSourceIndex = traversal.entities.front().entity->sourceIndex;
            traversal.stableEntityId = traversal.entities.front().entity->entityId;
            return traversal;
        }

        std::vector<std::vector<EntityId>> partitionOpenComponentIntoTraversableChains
        (
            const std::vector<EntityId>& componentEntityIds,
            const std::unordered_map<EntityId, const PlanningEntity*>& entities,
            bool allowReverse,
            double tolerance
        )
        {
            std::vector<std::vector<EntityId>> chains;
            std::vector<EntityId> remaining = componentEntityIds;
            while (!remaining.empty())
            {
                ProcessGroup pendingGroup;
                pendingGroup.kind = remaining.size() == 1U
                    ? ProcessGroupKind::SingleEntity
                    : ProcessGroupKind::ConnectedChain;
                pendingGroup.entityIds = remaining;

                std::optional<GroupTraversal> best;
                for (const EntityId entityId : remaining)
                {
                    const auto found = entities.find(entityId);
                    if (found == entities.end() || found->second == nullptr)
                        continue;
                    const PlanningEntity& entity = *found->second;
                    for (const bool reverse : { false, true })
                    {
                        if (!directionAllowed(entity, reverse, allowReverse))
                            continue;
                        const std::vector<Vector3d> points = directedPoints
                            (entity, reverse);
                        if (points.size() < 2U) continue;
                        auto candidate = buildTraversal
                        (
                            pendingGroup, entities, points.front(), allowReverse,
                            tolerance, std::make_pair(entityId, reverse), false
                        );
                        if (!candidate.has_value()) continue;
                        if (!best.has_value()
                            || candidate->entities.size() > best->entities.size())
                        {
                            best = std::move(candidate);
                        }
                    }
                }

                std::vector<EntityId> chain;
                if (best.has_value())
                {
                    chain.reserve(best->entities.size());
                    for (const DirectedEntity& directed : best->entities)
                    {
                        if (directed.entity != nullptr)
                            chain.push_back(directed.entity->entityId);
                    }
                }
                if (chain.empty()) chain.push_back(remaining.front());
                chains.push_back(chain);

                const std::unordered_set<EntityId> consumed
                    (chain.cbegin(), chain.cend());
                remaining.erase
                (
                    std::remove_if
                    (
                        remaining.begin(), remaining.end(),
                        [&consumed](EntityId entityId)
                        {
                            return consumed.find(entityId) != consumed.end();
                        }
                    ),
                    remaining.end()
                );
            }
            return chains;
        }

        double angleDegrees(const Vector3d& point, const machining::TubeSectionModel& section)
        {
            return std::atan2
            (
                point.z - section.geometry.centerZ,
                point.y - section.geometry.centerY
            ) * 180.0 / 3.14159265358979323846;
        }

        int surfaceIndex(double degrees)
        {
            double normalized = std::fmod(degrees + 360.0, 360.0);
            return static_cast<int>(std::floor((normalized + 45.0) / 90.0)) % 4;
        }

        void scoreTraversal
        (
            GroupTraversal& traversal,
            const Vector3d& currentPosition,
            const std::optional<machining::TubeSectionModel>& section
        )
        {
            traversal.movementDistance = distance(currentPosition, traversal.start);
            if (!section.has_value()) return;
            const double currentAngle = angleDegrees(currentPosition, *section);
            const double startAngle = angleDegrees(traversal.start, *section);
            traversal.rotationCost = std::abs(std::remainder(startAngle - currentAngle, 360.0));
            const int currentSurface = surfaceIndex(currentAngle);
            const int targetSurface = surfaceIndex(startAngle);
            const int rawDifference = std::abs(currentSurface - targetSurface);
            traversal.surfaceCost = std::min(rawDifference, 4 - rawDifference);
        }

        double entryThreshold(double connectionTolerance)
        {
            return std::max(kCalculationEpsilon, connectionTolerance * 1.0e-6);
        }

        std::optional<double> rotaryTravelLength
        (
            const Vector3d& start,
            const Vector3d& end,
            const Vector2d& center,
            double threshold
        )
        {
            const double startRadius = std::hypot(start.y - center.x, start.z - center.y);
            const double endRadius = std::hypot(end.y - center.x, end.z - center.y);
            const double localRadius = (startRadius + endRadius) * 0.5;
            if (!std::isfinite(localRadius) || localRadius <= threshold) return std::nullopt;

            const double startAngle = std::atan2(start.z - center.y, start.y - center.x);
            const double endAngle = std::atan2(end.z - center.y, end.y - center.x);
            const double angleDelta = std::remainder
                (endAngle - startAngle, 2.0 * 3.14159265358979323846);
            const double travel = localRadius * angleDelta;
            return std::isfinite(travel) ? std::optional<double>(travel) : std::nullopt;
        }

        void scoreEntrySmoothness
        (
            DirectedEntity& directed,
            const Vector3d& currentPosition,
            const Vector3d& nextPoint,
            const std::optional<Vector2d>& tubeCenter,
            double connectionTolerance
        )
        {
            const double threshold = entryThreshold(connectionTolerance);
            const double approachDx = directed.start.x - currentPosition.x;
            const double cutDx = nextPoint.x - directed.start.x;
            if (std::abs(approachDx) > threshold && std::abs(cutDx) > threshold
                && approachDx * cutDx < 0.0)
            {
                ++directed.entryAxisReversalCount;
            }

            if (tubeCenter.has_value()
                && std::isfinite(tubeCenter->x) && std::isfinite(tubeCenter->y))
            {
                const std::optional<double> approachRotary = rotaryTravelLength
                    (currentPosition, directed.start, *tubeCenter, threshold);
                const std::optional<double> cutRotary = rotaryTravelLength
                    (directed.start, nextPoint, *tubeCenter, threshold);
                if (approachRotary.has_value() && cutRotary.has_value()
                    && std::abs(*approachRotary) > threshold
                    && std::abs(*cutRotary) > threshold
                    && *approachRotary * *cutRotary < 0.0)
                {
                    ++directed.entryAxisReversalCount;
                }

                const double approachA = approachRotary.value_or(0.0);
                const double cutA = cutRotary.value_or(0.0);
                const double approachLength = std::hypot(approachDx, approachA);
                const double cutLength = std::hypot(cutDx, cutA);
                if (approachLength > threshold && cutLength > threshold)
                {
                    const double dotValue = (approachDx * cutDx + approachA * cutA)
                        / (approachLength * cutLength);
                    directed.entryTangentCost = std::clamp(1.0 - dotValue, 0.0, 2.0);
                }
                return;
            }

            const Vector3d approach
            {
                directed.start.x - currentPosition.x,
                directed.start.y - currentPosition.y,
                directed.start.z - currentPosition.z
            };
            const Vector3d cut
            {
                nextPoint.x - directed.start.x,
                nextPoint.y - directed.start.y,
                nextPoint.z - directed.start.z
            };
            const double approachLength = std::sqrt
                (approach.x * approach.x + approach.y * approach.y + approach.z * approach.z);
            const double cutLength = std::sqrt
                (cut.x * cut.x + cut.y * cut.y + cut.z * cut.z);
            if (approachLength > threshold && cutLength > threshold)
            {
                const double dotValue = (approach.x * cut.x + approach.y * cut.y
                    + approach.z * cut.z) / (approachLength * cutLength);
                directed.entryTangentCost = std::clamp(1.0 - dotValue, 0.0, 2.0);
            }
        }

        bool strongEntryZone(machining::TubeZone16 zone)
        {
            return machining::tubeZoneIndex(zone) % 2U == 0U;
        }

        bool reliableEntryProjection
        (
            const machining::TubeSectionProjection& projection,
            double projectionTolerance
        )
        {
            if (!projection.valid || projection.ambiguous
                || projection.confidence < 0.5
                || projection.absoluteDistanceToShell
                    > projectionTolerance * 0.8)
            {
                return false;
            }

            return projection.onBoundary
                ? !strongEntryZone(projection.zone)
                : strongEntryZone(projection.zone);
        }

        Vector3d interpolateEntryPoint
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

        std::optional<ZoneEntryCandidate> classifyZoneEntry
        (
            ZoneEntryCandidateKind kind,
            EntityId entityId,
            std::optional<double> sourceParameter,
            bool reverse,
            const Vector3d& entryPosition,
            const Vector3d& firstCutPoint,
            geometry::SourceGeometryKind sourceKind,
            const machining::TubeSectionModel& section,
            double projectionTolerance
        )
        {
            const double cutLength = distance(entryPosition, firstCutPoint);
            if (!std::isfinite(cutLength)
                || cutLength <= kCalculationEpsilon)
            {
                return std::nullopt;
            }

            std::optional<machining::TubeZone16> reliableZone;
            double confidence = 1.0;
            double maximumDeviation = 0.0;
            for (const double factor : { 0.25, 0.5, 0.75 })
            {
                const Vector3d point = interpolateEntryPoint
                    (entryPosition, firstCutPoint, factor);
                const machining::TubeSectionProjection projection =
                    machining::TubeSectionProjector::project
                    (
                        section, { point.y, point.z },
                        projectionTolerance
                    );
                if (!reliableEntryProjection
                    (projection, projectionTolerance))
                {
                    return std::nullopt;
                }
                if (!reliableZone.has_value())
                    reliableZone = projection.zone;
                else if (*reliableZone != projection.zone)
                    return std::nullopt;
                confidence = std::min(confidence, projection.confidence);
                maximumDeviation = std::max(maximumDeviation,
                    projection.absoluteDistanceToShell);
            }
            if (!reliableZone.has_value()) return std::nullopt;

            const machining::TubeSectionProjection entryProjection =
                machining::TubeSectionProjector::project
                (
                    section, { entryPosition.y, entryPosition.z },
                    projectionTolerance
                );
            if (entryProjection.valid && !entryProjection.onBoundary
                && !entryProjection.ambiguous
                && strongEntryZone(entryProjection.zone)
                && entryProjection.zone != *reliableZone)
            {
                return std::nullopt;
            }

            ZoneEntryCandidate candidate;
            candidate.zone = *reliableZone;
            candidate.kind = kind;
            candidate.entityId = entityId;
            candidate.sourceKind = sourceKind;
            candidate.sourceParameter = sourceParameter;
            candidate.reverse = reverse;
            candidate.entryPosition = entryPosition;
            candidate.firstCutPoint = firstCutPoint;
            candidate.firstCutTangent =
            {
                firstCutPoint.x - entryPosition.x,
                firstCutPoint.y - entryPosition.y,
                firstCutPoint.z - entryPosition.z
            };
            candidate.entryX = entryPosition.x;
            candidate.confidence = confidence;
            candidate.distanceToZoneBoundary =
                (entryProjection.onBoundary || entryProjection.ambiguous)
                ? cutLength * 0.25 : cutLength;
            candidate.distanceToZoneBoundary =
                std::max(0.0, candidate.distanceToZoneBoundary
                    - maximumDeviation);
            candidate.ambiguous = false;
            return candidate;
        }

        void scoreUnwrappedEntry
        (
            DirectedEntity& directed,
            const Vector3d& previousEnd,
            const Vector3d& firstCutPoint,
            const machining::TubeSectionModel& section,
            double connectionTolerance,
            double projectionTolerance
        )
        {
            scoreEntrySmoothness(directed, previousEnd, firstCutPoint,
                Vector2d{ section.geometry.centerY, section.geometry.centerZ },
                connectionTolerance);
            const double previousProjectionTolerance = std::max
            (
                projectionTolerance,
                std::hypot
                (
                    previousEnd.y - section.geometry.centerY,
                    previousEnd.z - section.geometry.centerZ
                ) + std::max(section.geometry.yLength,
                    section.geometry.zWidth)
            );
            const auto previous = machining::TubeSectionProjector::project
                (section, { previousEnd.y, previousEnd.z },
                    previousProjectionTolerance);
            const auto entry = machining::TubeSectionProjector::project
                (section, { directed.start.y, directed.start.z },
                    projectionTolerance);
            const auto cut = machining::TubeSectionProjector::project
                (section, { firstCutPoint.y, firstCutPoint.z },
                    projectionTolerance);
            if (!previous.valid || !entry.valid || !cut.valid
                || section.geometry.perimeter <= kCalculationEpsilon)
            {
                return;
            }

            const double approachX = directed.start.x - previousEnd.x;
            const double approachS = wrappedPerimeterDelta
                (previous.perimeterPosition, entry.perimeterPosition,
                    section.geometry.perimeter);
            const double cutX = firstCutPoint.x - directed.start.x;
            const double cutS = wrappedPerimeterDelta
                (entry.perimeterPosition, cut.perimeterPosition,
                    section.geometry.perimeter);
            const double threshold = entryThreshold(connectionTolerance);
            directed.entryAxisReversalCount = 0;
            if (std::abs(approachX) > threshold
                && std::abs(cutX) > threshold
                && approachX * cutX < 0.0)
            {
                ++directed.entryAxisReversalCount;
            }
            if (std::abs(approachS) > threshold
                && std::abs(cutS) > threshold
                && approachS * cutS < 0.0)
            {
                ++directed.entryAxisReversalCount;
            }
            const double approachLength = std::hypot(approachX, approachS);
            const double cutLength = std::hypot(cutX, cutS);
            if (approachLength > threshold && cutLength > threshold)
            {
                const double dotValue = (approachX * cutX
                    + approachS * cutS) / (approachLength * cutLength);
                directed.entryTangentCost =
                    std::clamp(1.0 - dotValue, 0.0, 2.0);
            }
        }

        bool isSingleClosedEntryOptimizedCurve
        (
            const ProcessGroup& group,
            const PlanningEntity& entity
        )
        {
            return group.entityIds.size() == 1U && entity.path.closed
                && (entity.sourceKind == geometry::SourceGeometryKind::Circle
                    || entity.sourceKind == geometry::SourceGeometryKind::Ellipse
                    || entity.sourceKind == geometry::SourceGeometryKind::Polyline
                    || entity.sourceKind == geometry::SourceGeometryKind::Spline);
        }

        std::optional<GroupTraversal> buildSingleClosedCurveTraversal
        (
            const ProcessGroup& group,
            const PlanningEntity& entity,
            const Vector3d& currentPosition,
            bool reverse,
            std::size_t startIndex,
            double connectionTolerance,
            const std::optional<machining::TubeSectionModel>& section,
            const std::optional<Vector2d>& tubeCenter
        )
        {
            const std::size_t pointCount = entity.path.vertices.size();
            if (pointCount < 2U || startIndex >= pointCount) return std::nullopt;

            std::vector<Vector3d> points;
            points.reserve(pointCount);
            for (std::size_t offset = 0U; offset < pointCount; ++offset)
            {
                const std::size_t index = reverse
                    ? (startIndex + pointCount - offset) % pointCount
                    : (startIndex + offset) % pointCount;
                points.push_back(entity.path.vertices[index].position);
            }

            const double threshold = entryThreshold(connectionTolerance);
            const auto next = std::find_if
            (
                points.cbegin() + 1,
                points.cend(),
                [&points, threshold](const Vector3d& point)
                {
                    return distance(points.front(), point) > threshold;
                }
            );
            if (next == points.cend()) return std::nullopt;

            const std::optional<double> selectedStartParameter = entity.startParameter.has_value()
                ? entity.startParameter
                : std::optional<double>(entity.path.vertices[startIndex].sourceParameter);
            if (!selectedStartParameter.has_value()
                || !std::isfinite(*selectedStartParameter)) return std::nullopt;

            DirectedEntity directed;
            directed.entity = &entity;
            directed.reverseRelativeToInput = reverse;
            directed.selectedStartParameter = selectedStartParameter;
            directed.start = points.front();
            directed.end = points.front();
            scoreEntrySmoothness
                (directed, currentPosition, *next, tubeCenter, connectionTolerance);

            GroupTraversal traversal;
            traversal.groupId = group.groupId;
            traversal.entities.push_back(directed);
            traversal.start = directed.start;
            traversal.end = directed.end;
            traversal.entryAxisReversalCount = directed.entryAxisReversalCount;
            traversal.entryTangentCost = directed.entryTangentCost;
            traversal.stableSourceIndex = entity.sourceIndex;
            traversal.stableEntityId = entity.entityId;
            scoreTraversal(traversal, currentPosition, section);
            return traversal;
        }

        bool selectedStartParameterLess
        (
            const std::optional<double>& left,
            const std::optional<double>& right
        )
        {
            if (left.has_value() != right.has_value()) return !left.has_value();
            if (!left.has_value()) return false;
            if (std::abs(*left - *right) > kCalculationEpsilon) return *left < *right;
            return false;
        }

        bool traversalLess
        (
            const GroupTraversal& left,
            const GroupTraversal& right,
            ProcessOrderingStrategy strategy
        )
        {
            if (strategy == ProcessOrderingStrategy::LazyRotation)
            {
                if (left.entryAxisReversalCount != right.entryAxisReversalCount)
                    return left.entryAxisReversalCount < right.entryAxisReversalCount;
                if (std::abs(left.entryTangentCost - right.entryTangentCost) > kCalculationEpsilon)
                    return left.entryTangentCost < right.entryTangentCost;
                if (std::abs(left.rotationCost - right.rotationCost) > kCalculationEpsilon)
                    return left.rotationCost < right.rotationCost;
                if (left.surfaceCost != right.surfaceCost) return left.surfaceCost < right.surfaceCost;
                if (std::abs(left.movementDistance - right.movementDistance) > kCalculationEpsilon)
                    return left.movementDistance < right.movementDistance;
            }
            else
            {
                if (std::abs(left.movementDistance - right.movementDistance) > kCalculationEpsilon)
                    return left.movementDistance < right.movementDistance;
                if (left.entryAxisReversalCount != right.entryAxisReversalCount)
                    return left.entryAxisReversalCount < right.entryAxisReversalCount;
                if (std::abs(left.entryTangentCost - right.entryTangentCost) > kCalculationEpsilon)
                    return left.entryTangentCost < right.entryTangentCost;
            }
            if (left.stableSourceIndex != right.stableSourceIndex)
                return left.stableSourceIndex < right.stableSourceIndex;
            if (left.stableEntityId != right.stableEntityId)
                return left.stableEntityId < right.stableEntityId;
            const std::optional<double> leftStart = left.entities.empty()
                ? std::nullopt : left.entities.front().selectedStartParameter;
            const std::optional<double> rightStart = right.entities.empty()
                ? std::nullopt : right.entities.front().selectedStartParameter;
            if (selectedStartParameterLess(leftStart, rightStart)) return true;
            if (selectedStartParameterLess(rightStart, leftStart)) return false;
            const bool leftReverse = !left.entities.empty()
                && left.entities.front().reverseRelativeToInput;
            const bool rightReverse = !right.entities.empty()
                && right.entities.front().reverseRelativeToInput;
            return leftReverse < rightReverse;
        }

        bool zoneConstrainedTraversalLess
        (
            const GroupTraversal& left,
            const GroupTraversal& right,
            const TraversalSelectionContext& selection,
            ProcessOrderingStrategy strategy
        )
        {
            if (!selection.hardZoneConstraint
                || !selection.requiredEntryZone.has_value()
                || !left.selectedEntry.has_value()
                || !right.selectedEntry.has_value())
            {
                return traversalLess(left, right, strategy);
            }
            const ZoneEntryCandidate& leftEntry = *left.selectedEntry;
            const ZoneEntryCandidate& rightEntry = *right.selectedEntry;
            if (leftEntry.ambiguous != rightEntry.ambiguous)
                return !leftEntry.ambiguous;
            if (std::abs(leftEntry.distanceToMemberEndpoint
                - rightEntry.distanceToMemberEndpoint)
                > kCalculationEpsilon)
            {
                return leftEntry.distanceToMemberEndpoint
                    > rightEntry.distanceToMemberEndpoint;
            }
            if (std::abs(leftEntry.distanceToZoneBoundary
                - rightEntry.distanceToZoneBoundary) > kCalculationEpsilon)
            {
                return leftEntry.distanceToZoneBoundary
                    > rightEntry.distanceToZoneBoundary;
            }
            if (left.entryAxisReversalCount != right.entryAxisReversalCount)
                return left.entryAxisReversalCount
                    < right.entryAxisReversalCount;
            if (std::abs(left.entryTangentCost
                - right.entryTangentCost) > kCalculationEpsilon)
            {
                return left.entryTangentCost < right.entryTangentCost;
            }
            const double leftHitDistance =
                std::abs(leftEntry.entryX - selection.zoneHitX);
            const double rightHitDistance =
                std::abs(rightEntry.entryX - selection.zoneHitX);
            if (std::abs(leftHitDistance - rightHitDistance)
                > kCalculationEpsilon)
            {
                return leftHitDistance < rightHitDistance;
            }
            if (std::abs(left.rotationCost - right.rotationCost)
                > kCalculationEpsilon)
            {
                return left.rotationCost < right.rotationCost;
            }
            if (std::abs(left.movementDistance - right.movementDistance)
                > kCalculationEpsilon)
            {
                return left.movementDistance < right.movementDistance;
            }
            return traversalLess(left, right, strategy);
        }

        bool surfaceSweepCandidateLess
        (
            const SchedulingCandidate& left,
            const SchedulingCandidate& right,
            const SurfaceSweepState& state,
            double tolerance
        )
        {
            const double leftDelta = left.footprint.anchorX - state.currentX;
            const double rightDelta = right.footprint.anchorX - state.currentX;
            const bool leftForward = state.longitudinalDirection >= 0
                ? leftDelta >= -tolerance : leftDelta <= tolerance;
            const bool rightForward = state.longitudinalDirection >= 0
                ? rightDelta >= -tolerance : rightDelta <= tolerance;
            if (leftForward != rightForward) return leftForward;

            const double leftBacktrack = leftForward ? 0.0 : std::abs(leftDelta);
            const double rightBacktrack = rightForward ? 0.0 : std::abs(rightDelta);
            if (std::abs(leftBacktrack - rightBacktrack) > kCalculationEpsilon)
                return leftBacktrack < rightBacktrack;
            if (std::abs(std::abs(leftDelta) - std::abs(rightDelta))
                > kCalculationEpsilon)
                return std::abs(leftDelta) < std::abs(rightDelta);
            if (std::abs(left.traversal.rotationCost - right.traversal.rotationCost)
                > kCalculationEpsilon)
                return left.traversal.rotationCost < right.traversal.rotationCost;
            if (left.traversal.entryAxisReversalCount
                != right.traversal.entryAxisReversalCount)
                return left.traversal.entryAxisReversalCount
                    < right.traversal.entryAxisReversalCount;
            if (std::abs(left.traversal.entryTangentCost
                    - right.traversal.entryTangentCost) > kCalculationEpsilon)
                return left.traversal.entryTangentCost
                    < right.traversal.entryTangentCost;
            if (std::abs(left.traversal.movementDistance
                    - right.traversal.movementDistance) > kCalculationEpsilon)
                return left.traversal.movementDistance
                    < right.traversal.movementDistance;
            if (left.traversal.stableSourceIndex != right.traversal.stableSourceIndex)
                return left.traversal.stableSourceIndex
                    < right.traversal.stableSourceIndex;
            if (left.traversal.stableEntityId != right.traversal.stableEntityId)
                return left.traversal.stableEntityId
                    < right.traversal.stableEntityId;
            return traversalLess(left.traversal, right.traversal,
                ProcessOrderingStrategy::NearestNext);
        }

        bool footprintUsesRegion
        (
            const ProcessSurfaceFootprint& footprint,
            machining::TubeSurfaceRegion region,
            bool allowMixed
        )
        {
            if (footprint.dominantRegion == region) return true;
            return allowMixed
                && footprint.dominantRegion == machining::TubeSurfaceRegion::Mixed
                && footprint.entryRegion == region;
        }

        machining::TubeZone16 zoneAtOffset
        (
            machining::TubeZone16 initialZone,
            int offset,
            int perimeterDirection
        )
        {
            const int initial = static_cast<int>(machining::tubeZoneIndex(initialZone));
            const int wrapped = (initial + offset * perimeterDirection
                + static_cast<int>(machining::kTubeZone16Count))
                % static_cast<int>(machining::kTubeZone16Count);
            return static_cast<machining::TubeZone16>(wrapped);
        }

        std::optional<machining::TubeZone16> firstZoneInSweep
        (
            machining::TubeZoneMask mask,
            machining::TubeZone16 initialZone,
            int perimeterDirection
        )
        {
            for (int offset = 0;
                offset < static_cast<int>(machining::kTubeZone16Count);
                ++offset)
            {
                const machining::TubeZone16 zone =
                    zoneAtOffset(initialZone, offset, perimeterDirection);
                if ((mask & machining::tubeZoneBit(zone)) != 0U)
                    return zone;
            }
            return std::nullopt;
        }
