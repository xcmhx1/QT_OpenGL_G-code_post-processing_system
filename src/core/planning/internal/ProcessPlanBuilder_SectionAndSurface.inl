        QString strategyName(ProcessOrderingStrategy strategy)
        {
            return strategy == ProcessOrderingStrategy::LazyRotation
                ? QStringLiteral("LazyRotation")
                : QStringLiteral("NearestNext");
        }

        QString groupKindName(ProcessGroupKind kind)
        {
            switch (kind)
            {
            case ProcessGroupKind::SingleEntity: return QStringLiteral("SingleEntity");
            case ProcessGroupKind::ConnectedChain: return QStringLiteral("ConnectedChain");
            case ProcessGroupKind::ClosedLoop: return QStringLiteral("ClosedLoop");
            case ProcessGroupKind::BreakBoundary: return QStringLiteral("BreakBoundary");
            case ProcessGroupKind::WasteBoundary: return QStringLiteral("WasteBoundary");
            }
            return QStringLiteral("Unknown");
        }

        QVariantMap diagnosticValues
        (
            const ProcessPlanningInput& input,
            const ProcessPlanningPolicy& policy,
            EntityId entityId = 0,
            std::size_t sourceIndex = 0,
            int boundaryPairId = -1,
            int groupId = -1,
            int predecessorGroupId = -1,
            int successorGroupId = -1,
            int candidateCount = 0,
            int eligibleCount = 0,
            int groupCount = 0,
            int assignmentCount = 0,
            int excludedCount = 0,
            int processOrder = -1,
            int continuousGroupId = -1,
            bool initialSelection = false,
            const GroupTraversal* selected = nullptr,
            ProcessGroupKind selectedGroupKind = ProcessGroupKind::SingleEntity,
            int blockedNearestBoundaryGroupId = -1
        )
        {
            QVariantMap values;
            values.insert(QStringLiteral("contentRevision"), QVariant::fromValue<qulonglong>(input.contentRevision));
            values.insert(QStringLiteral("entityId"), QVariant::fromValue<qulonglong>(entityId));
            values.insert(QStringLiteral("sourceIndex"), QVariant::fromValue<qulonglong>(sourceIndex));
            values.insert(QStringLiteral("boundaryPairId"), boundaryPairId);
            values.insert(QStringLiteral("groupId"), groupId);
            values.insert(QStringLiteral("predecessorGroupId"), predecessorGroupId);
            values.insert(QStringLiteral("successorGroupId"), successorGroupId);
            values.insert(QStringLiteral("orderingStrategy"), strategyName(policy.orderingStrategy));
            values.insert(QStringLiteral("candidateCount"), candidateCount);
            values.insert(QStringLiteral("eligibleCount"), eligibleCount);
            values.insert(QStringLiteral("groupCount"), groupCount);
            values.insert(QStringLiteral("assignmentCount"), assignmentCount);
            values.insert(QStringLiteral("excludedCount"), excludedCount);
            values.insert(QStringLiteral("processOrder"), processOrder);
            values.insert(QStringLiteral("continuousGroupId"), continuousGroupId);
            values.insert(QStringLiteral("initialSelection"), initialSelection);
            values.insert(QStringLiteral("initialPositionX"), policy.initialPosition.x);
            values.insert(QStringLiteral("initialPositionY"), policy.initialPosition.y);
            values.insert(QStringLiteral("initialPositionZ"), policy.initialPosition.z);
            values.insert(QStringLiteral("selectedGroupId"), selected != nullptr ? selected->groupId : -1);
            values.insert(QStringLiteral("selectedGroupKind"), selected != nullptr
                ? groupKindName(selectedGroupKind) : QString());
            values.insert(QStringLiteral("selectedMovementDistance"), selected != nullptr
                ? selected->movementDistance : -1.0);
            values.insert(QStringLiteral("selectedRotationCost"), selected != nullptr
                ? selected->rotationCost : -1.0);
            values.insert(QStringLiteral("selectedSurfaceCost"), selected != nullptr
                ? selected->surfaceCost : -1);
            values.insert(QStringLiteral("blockedNearestBoundaryGroupId"), blockedNearestBoundaryGroupId);
            return values;
        }

        Diagnostic planningDiagnostic
        (
            const OperationContext& context,
            DiagnosticCode code,
            const QString& message,
            const QString& detail,
            const QVariantMap& values,
            DiagnosticSeverity severity = DiagnosticSeverity::Error
        )
        {
            Diagnostic diagnostic;
            diagnostic.code = code;
            diagnostic.severity = severity;
            diagnostic.component = QStringLiteral("ProcessPlanBuilder");
            diagnostic.operation = QStringLiteral("BuildProcessPlan");
            diagnostic.stage = QStringLiteral("ProcessPlanning");
            diagnostic.userMessage = message;
            diagnostic.technicalDetail = detail;
            diagnostic.correlationId = context.correlationId;
            diagnostic.context = values;
            const qulonglong entityId = values.value(QStringLiteral("entityId")).toULongLong();
            if (entityId != 0U) diagnostic.entityId = entityId;
            const int groupId = values.value(QStringLiteral("groupId"), -1).toInt();
            if (groupId >= 0) diagnostic.groupId = groupId;
            return diagnostic;
        }

        template<typename T>
        OperationResult<T> failure
        (
            OperationStatus status,
            const OperationContext& context,
            DiagnosticCode code,
            const QString& message,
            const QString& detail,
            const QVariantMap& values
        )
        {
            OperationResult<T> result;
            result.status = status;
            result.addDiagnostic(planningDiagnostic(context, code, message, detail, values));
            return result;
        }

        std::vector<double> cumulativeSectionLengths(const machining::TubeSectionGeometry& section)
        {
            std::vector<double> cumulative(section.boundary.size() + 1U, 0.0);
            for (std::size_t index = 0; index < section.boundary.size(); ++index)
            {
                const Vector2d& first = section.boundary[index];
                const Vector2d& second = section.boundary[(index + 1U) % section.boundary.size()];
                cumulative[index + 1U] = cumulative[index]
                    + std::hypot(second.x - first.x, second.y - first.y);
            }
            return cumulative;
        }

        SectionProjection projectToSection
        (
            const Vector2d& point,
            const machining::TubeSectionGeometry& section,
            const std::vector<double>& cumulative
        )
        {
            SectionProjection best;
            const Vector2d localPoint
                { point.x - section.centerY, point.y - section.centerZ };
            for (std::size_t index = 0; index < section.boundary.size(); ++index)
            {
                const Vector2d start
                    { section.boundary[index].x - section.centerY,
                      section.boundary[index].y - section.centerZ };
                const Vector2d& worldEnd = section.boundary[(index + 1U) % section.boundary.size()];
                const Vector2d end
                    { worldEnd.x - section.centerY, worldEnd.y - section.centerZ };
                const double dy = end.x - start.x;
                const double dz = end.y - start.y;
                const double lengthSquared = dy * dy + dz * dz;
                if (lengthSquared <= kCalculationEpsilon) continue;
                const double factor = std::clamp
                (
                    ((localPoint.x - start.x) * dy + (localPoint.y - start.y) * dz)
                        / lengthSquared,
                    0.0,
                    1.0
                );
                const double projectedY = start.x + dy * factor;
                const double projectedZ = start.y + dz * factor;
                const double candidateDistance = std::hypot
                    (localPoint.x - projectedY, localPoint.y - projectedZ);
                if (candidateDistance < best.distance)
                {
                    best.valid = true;
                    best.distance = candidateDistance;
                    best.perimeterPosition = cumulative[index] + std::sqrt(lengthSquared) * factor;
                }
            }
            return best;
        }

        QString surfaceRegionName(machining::TubeSurfaceRegion region)
        {
            using Region = machining::TubeSurfaceRegion;
            switch (region)
            {
            case Region::Top: return QStringLiteral("Top");
            case Region::TopRightCorner: return QStringLiteral("TopRightCorner");
            case Region::Right: return QStringLiteral("Right");
            case Region::BottomRightCorner: return QStringLiteral("BottomRightCorner");
            case Region::Bottom: return QStringLiteral("Bottom");
            case Region::BottomLeftCorner: return QStringLiteral("BottomLeftCorner");
            case Region::Left: return QStringLiteral("Left");
            case Region::TopLeftCorner: return QStringLiteral("TopLeftCorner");
            case Region::Mixed: return QStringLiteral("Mixed");
            case Region::Unknown: return QStringLiteral("Unknown");
            }
            return QStringLiteral("Unknown");
        }

        int surfaceRegionIndex(machining::TubeSurfaceRegion region)
        {
            using Region = machining::TubeSurfaceRegion;
            switch (region)
            {
            case Region::Top: return 0;
            case Region::TopRightCorner: return 1;
            case Region::Right: return 2;
            case Region::BottomRightCorner: return 3;
            case Region::Bottom: return 4;
            case Region::BottomLeftCorner: return 5;
            case Region::Left: return 6;
            case Region::TopLeftCorner: return 7;
            case Region::Mixed:
            case Region::Unknown: return -1;
            }
            return -1;
        }

        machining::TubeSurfaceRegion surfaceRegionAt(int index)
        {
            using Region = machining::TubeSurfaceRegion;
            static constexpr std::array<Region, 8> regions
            {{
                Region::Top,
                Region::TopRightCorner,
                Region::Right,
                Region::BottomRightCorner,
                Region::Bottom,
                Region::BottomLeftCorner,
                Region::Left,
                Region::TopLeftCorner
            }};
            const int wrapped = ((index % 8) + 8) % 8;
            return regions[static_cast<std::size_t>(wrapped)];
        }

        SectionBounds sectionBounds(const machining::TubeSectionGeometry& section)
        {
            SectionBounds bounds;
            if (section.boundary.empty()) return bounds;
            bounds.valid = true;
            bounds.minimumY = bounds.maximumY = section.boundary.front().x;
            bounds.minimumZ = bounds.maximumZ = section.boundary.front().y;
            for (const Vector2d& point : section.boundary)
            {
                if (!std::isfinite(point.x) || !std::isfinite(point.y))
                    return SectionBounds{};
                bounds.minimumY = std::min(bounds.minimumY, point.x);
                bounds.maximumY = std::max(bounds.maximumY, point.x);
                bounds.minimumZ = std::min(bounds.minimumZ, point.y);
                bounds.maximumZ = std::max(bounds.maximumZ, point.y);
            }
            return bounds;
        }

        double surfaceClassificationTolerance
        (
            const machining::TubeSectionGeometry& section
        )
        {
            const double maximumDimension = std::max
                (std::abs(section.yLength), std::abs(section.zWidth));
            return std::max({ kCalculationEpsilon, 1.0e-6,
                maximumDimension * 1.0e-8 });
        }

        machining::TubeSurfaceRegion cornerRegion
        (
            const machining::TubeCornerGeometry& corner
        )
        {
            using Region = machining::TubeSurfaceRegion;
            if (corner.yDirection > 0 && corner.zDirection > 0)
                return Region::TopRightCorner;
            if (corner.yDirection > 0 && corner.zDirection < 0)
                return Region::BottomRightCorner;
            if (corner.yDirection < 0 && corner.zDirection < 0)
                return Region::BottomLeftCorner;
            if (corner.yDirection < 0 && corner.zDirection > 0)
                return Region::TopLeftCorner;
            return Region::Unknown;
        }

        const machining::TubeCornerGeometry* matchingTubeCorner
        (
            const Vector3d& point,
            const machining::TubeSectionModel& section
        )
        {
            for (const auto& corner : section.corners)
            {
                if (!std::isfinite(corner.radius) || corner.radius <= 0.0) continue;
                const double tolerance = std::max(0.01, corner.radius * 0.01);
                const double dy = point.y - corner.center.x;
                const double dz = point.z - corner.center.y;
                if (dy * corner.yDirection < -tolerance
                    || dz * corner.zDirection < -tolerance) continue;
                if (std::abs(std::hypot(dy, dz) - corner.radius) <= tolerance)
                    return &corner;
            }
            return nullptr;
        }

        std::optional<machining::TubeSurfaceRegion> flatRegionForPoints
        (
            const std::vector<Vector3d>& points,
            const SectionBounds& bounds,
            double tolerance
        )
        {
            using Region = machining::TubeSurfaceRegion;
            if (points.empty() || !bounds.valid) return std::nullopt;
            struct Candidate
            {
                Region region = Region::Unknown;
                double maximumDistance = 0.0;
                double averageDistance = 0.0;
            };
            const std::array<std::pair<Region, double>, 4> planes
            {{
                { Region::Top, bounds.maximumZ },
                { Region::Right, bounds.maximumY },
                { Region::Bottom, bounds.minimumZ },
                { Region::Left, bounds.minimumY }
            }};
            std::vector<Candidate> candidates;
            for (const auto& [region, coordinate] : planes)
            {
                double maximumDistance = 0.0;
                double totalDistance = 0.0;
                for (const Vector3d& point : points)
                {
                    const double value = region == Region::Top || region == Region::Bottom
                        ? point.z : point.y;
                    const double candidateDistance = std::abs(value - coordinate);
                    maximumDistance = std::max(maximumDistance, candidateDistance);
                    totalDistance += candidateDistance;
                }
                if (maximumDistance <= tolerance)
                {
                    candidates.push_back
                    ({ region, maximumDistance,
                        totalDistance / static_cast<double>(points.size()) });
                }
            }
            if (candidates.empty()) return std::nullopt;
            std::sort(candidates.begin(), candidates.end(), [](const Candidate& left,
                const Candidate& right)
            {
                if (std::abs(left.maximumDistance - right.maximumDistance)
                    > kCalculationEpsilon)
                    return left.maximumDistance < right.maximumDistance;
                if (std::abs(left.averageDistance - right.averageDistance)
                    > kCalculationEpsilon)
                    return left.averageDistance < right.averageDistance;
                return surfaceRegionIndex(left.region) < surfaceRegionIndex(right.region);
            });
            if (candidates.size() == 1U) return candidates.front().region;
            if (candidates[0].maximumDistance
                    < candidates[1].maximumDistance - kCalculationEpsilon
                || (std::abs(candidates[0].maximumDistance
                        - candidates[1].maximumDistance) <= kCalculationEpsilon
                    && candidates[0].averageDistance
                        < candidates[1].averageDistance - kCalculationEpsilon))
                return candidates.front().region;
            return std::nullopt;
        }

        std::optional<machining::TubeSurfaceRegion> cornerRegionForPoints
        (
            const std::vector<Vector3d>& points,
            const machining::TubeSectionModel& section
        )
        {
            if (points.empty()) return std::nullopt;
            const machining::TubeCornerGeometry* selected = nullptr;
            for (const Vector3d& point : points)
            {
                const auto* corner = matchingTubeCorner(point, section);
                if (corner == nullptr || (selected != nullptr && selected != corner))
                    return std::nullopt;
                selected = corner;
            }
            if (selected == nullptr) return std::nullopt;
            const auto region = cornerRegion(*selected);
            return surfaceRegionIndex(region) >= 0
                ? std::optional<machining::TubeSurfaceRegion>(region) : std::nullopt;
        }

        machining::TubeSurfaceRegion classifySurfacePoints
        (
            const std::vector<Vector3d>& points,
            const machining::TubeSectionModel& section,
            const SectionBounds& bounds,
            double tolerance
        )
        {
            if (const auto flat = flatRegionForPoints(points, bounds, tolerance))
                return *flat;
            if (const auto corner = cornerRegionForPoints(points, section))
                return *corner;
            return points.empty() ? machining::TubeSurfaceRegion::Unknown
                : machining::TubeSurfaceRegion::Mixed;
        }

        machining::TubeSurfaceRegion classifySurfacePoint
        (
            const Vector3d& point,
            const machining::TubeSectionModel& section,
            const SectionBounds& bounds,
            double tolerance
        )
        {
            if (const auto* corner = matchingTubeCorner(point, section))
            {
                const auto region = cornerRegion(*corner);
                if (surfaceRegionIndex(region) >= 0) return region;
            }
            const std::vector<Vector3d> singlePoint{ point };
            if (const auto flat = flatRegionForPoints(singlePoint, bounds, tolerance))
                return *flat;
            return machining::TubeSurfaceRegion::Unknown;
        }

        double wrappedPerimeterDelta(double from, double to, double perimeter)
        {
            if (!std::isfinite(from) || !std::isfinite(to)
                || !std::isfinite(perimeter) || perimeter <= 0.0) return 0.0;
            return std::remainder(to - from, perimeter);
        }

        BoundarySide classifyPoint
        (
            const Vector3d& point,
            const TubeCutAnalysis& boundary,
            const machining::TubeSectionGeometry& section,
            double tolerance
        )
        {
            if (boundary.unwrappedBoundary.size() < 2U || section.boundary.size() < 3U || section.perimeter <= 0.0)
            {
                return BoundarySide::Indeterminate;
            }

            const std::vector<double> cumulative = cumulativeSectionLengths(section);
            const SectionProjection projection = projectToSection({ point.y, point.z }, section, cumulative);
            const double safeTolerance = std::max(1.0e-6, std::abs(tolerance));
            if (!projection.valid || projection.distance > safeTolerance)
            {
                return BoundarySide::Indeterminate;
            }

            const double perimeter = section.perimeter;
            double minimumBoundaryX = boundary.unwrappedBoundary.front().x;
            double maximumBoundaryX = minimumBoundaryX;
            for (const machining::UnwrappedBoundaryPoint& sample : boundary.unwrappedBoundary)
            {
                minimumBoundaryX = std::min(minimumBoundaryX, sample.x);
                maximumBoundaryX = std::max(maximumBoundaryX, sample.x);
            }
            const double referenceX = minimumBoundaryX
                + (maximumBoundaryX - minimumBoundaryX) * 0.5;
            const double localPointX = point.x - referenceX;
            double queryPosition = projection.perimeterPosition;
            double minimumPosition = boundary.unwrappedBoundary.front().perimeterPosition;
            double maximumPosition = minimumPosition;
            for (const machining::UnwrappedBoundaryPoint& sample : boundary.unwrappedBoundary)
            {
                minimumPosition = std::min(minimumPosition, sample.perimeterPosition);
                maximumPosition = std::max(maximumPosition, sample.perimeterPosition);
            }
            const double intervalCenter = (minimumPosition + maximumPosition) * 0.5;
            while (queryPosition - intervalCenter > perimeter * 0.5) queryPosition -= perimeter;
            while (queryPosition - intervalCenter < -perimeter * 0.5) queryPosition += perimeter;

            std::vector<double> intersections;
            bool onBoundary = false;
            for (const double shift : { -perimeter, 0.0, perimeter })
            {
                for (std::size_t index = 0; index + 1U < boundary.unwrappedBoundary.size(); ++index)
                {
                    const auto& first = boundary.unwrappedBoundary[index];
                    const auto& second = boundary.unwrappedBoundary[index + 1U];
                    const double firstX = first.x - referenceX;
                    const double secondX = second.x - referenceX;
                    const double firstS = first.perimeterPosition + shift;
                    const double secondS = second.perimeterPosition + shift;
                    const double edgeX = secondX - firstX;
                    const double edgeS = secondS - firstS;
                    const double lengthSquared = edgeX * edgeX + edgeS * edgeS;
                    if (lengthSquared > kCalculationEpsilon)
                    {
                        const double factor = std::clamp
                        (
                            ((localPointX - firstX) * edgeX
                                + (queryPosition - firstS) * edgeS) / lengthSquared,
                            0.0,
                            1.0
                        );
                        const double nearestX = firstX + edgeX * factor;
                        const double nearestS = firstS + edgeS * factor;
                        onBoundary = onBoundary
                            || std::hypot(localPointX - nearestX,
                                queryPosition - nearestS) <= safeTolerance;
                    }
                    const bool crosses = (firstS <= queryPosition && queryPosition < secondS)
                        || (secondS <= queryPosition && queryPosition < firstS);
                    if (!crosses || std::abs(edgeS) <= kCalculationEpsilon) continue;
                    const double factor = (queryPosition - firstS) / edgeS;
                    const double intersectionX = firstX + edgeX * factor;
                    intersections.push_back(intersectionX);
                    onBoundary = onBoundary
                        || std::abs(intersectionX - localPointX) <= safeTolerance;
                }
            }
            if (onBoundary) return BoundarySide::OnBoundary;

            std::sort(intersections.begin(), intersections.end());
            std::vector<double> unique;
            for (const double x : intersections)
            {
                if (!unique.empty() && std::abs(x - unique.back()) <= safeTolerance)
                {
                    unique.back() = (unique.back() + x) * 0.5;
                }
                else
                {
                    unique.push_back(x);
                }
            }
            const int crossings = static_cast<int>(std::count_if
            (
                unique.cbegin(), unique.cend(),
                [localPointX, safeTolerance](double x)
                { return x < localPointX - safeTolerance; }
            ));
            return crossings % 2 == 0 ? BoundarySide::Left : BoundarySide::Right;
        }

        BoundarySide classifyGroup
        (
            const ProcessGroup& group,
            const std::unordered_map<EntityId, const PlanningEntity*>& entities,
            const BoundaryData& boundary,
            const machining::TubeSectionGeometry& section,
            double tolerance
        )
        {
            double groupMinimumX = std::numeric_limits<double>::max();
            double groupMaximumX = std::numeric_limits<double>::lowest();
            for (const EntityId entityId : group.entityIds)
            {
                const auto found = entities.find(entityId);
                if (found == entities.end() || found->second->path.vertices.empty())
                {
                    return BoundarySide::Indeterminate;
                }
                for (const geometry::PathVertex3D& vertex : found->second->path.vertices)
                {
                    groupMinimumX = std::min(groupMinimumX, vertex.position.x);
                    groupMaximumX = std::max(groupMaximumX, vertex.position.x);
                }
            }

            double boundaryMinimumX = std::numeric_limits<double>::max();
            double boundaryMaximumX = std::numeric_limits<double>::lowest();
            for (const machining::UnwrappedBoundaryPoint& point : boundary.analysis.unwrappedBoundary)
            {
                boundaryMinimumX = std::min(boundaryMinimumX, point.x);
                boundaryMaximumX = std::max(boundaryMaximumX, point.x);
            }
            const double safeTolerance = std::max(1.0e-6, std::abs(tolerance));
            if (groupMaximumX < boundaryMinimumX - safeTolerance)
            {
                return BoundarySide::Left;
            }
            if (groupMinimumX > boundaryMaximumX + safeTolerance)
            {
                return BoundarySide::Right;
            }

            bool left = false;
            bool right = false;
            bool onBoundary = false;
            for (const EntityId entityId : group.entityIds)
            {
                const auto found = entities.find(entityId);
                if (found == entities.end()) return BoundarySide::Indeterminate;
                const auto& vertices = found->second->path.vertices;
                if (vertices.empty()) return BoundarySide::Indeterminate;
                for (std::size_t index = 0; index < vertices.size(); ++index)
                {
                    const BoundarySide side = classifyPoint(vertices[index].position, boundary.analysis, section, tolerance);
                    if (side == BoundarySide::Indeterminate) return side;
                    left = left || side == BoundarySide::Left;
                    right = right || side == BoundarySide::Right;
                    onBoundary = onBoundary || side == BoundarySide::OnBoundary;
                    if (index + 1U < vertices.size())
                    {
                        const Vector3d& first = vertices[index].position;
                        const Vector3d& second = vertices[index + 1U].position;
                        const Vector3d midpoint
                        {
                            first.x + (second.x - first.x) * 0.5,
                            first.y + (second.y - first.y) * 0.5,
                            first.z + (second.z - first.z) * 0.5
                        };
                        const BoundarySide middleSide = classifyPoint(midpoint, boundary.analysis, section, tolerance);
                        if (middleSide == BoundarySide::Indeterminate) return middleSide;
                        left = left || middleSide == BoundarySide::Left;
                        right = right || middleSide == BoundarySide::Right;
                        onBoundary = onBoundary || middleSide == BoundarySide::OnBoundary;
                    }
                }
            }
            if (left && right) return BoundarySide::Mixed;
            if (left) return BoundarySide::Left;
            if (right) return BoundarySide::Right;
            return onBoundary ? BoundarySide::OnBoundary : BoundarySide::Indeterminate;
        }

