        using geometry::EntityId;
        using geometry::Vector2d;
        using geometry::Vector3d;
        using machining::TubeCutAnalysis;
        using machining::TubeCutBoundaryClassifier;
        using machining::TubeCutResult;

        constexpr double kCalculationEpsilon = 1.0e-12;

        struct BoundaryData
        {
            int groupId = -1;
            int pairId = -1;
            BoundaryRole role = BoundaryRole::None;
            TubeCutAnalysis analysis;
        };

        struct BoundaryIdentity
        {
            int pairId = -1;
            BoundaryRole role = BoundaryRole::None;
            std::vector<EntityId> entityIds;
            std::size_t stableSourceIndex = 0;
            EntityId stableEntityId = 0;
        };

        struct XBounds
        {
            bool valid = false;
            double minimum = 0.0;
            double maximum = 0.0;
        };

        struct DirectedEntity
        {
            const PlanningEntity* entity = nullptr;
            bool reverseRelativeToInput = false;
            std::optional<double> selectedStartParameter;
            Vector3d start;
            Vector3d end;
            int entryAxisReversalCount = 0;
            double entryTangentCost = 0.0;
        };

        enum class ZoneEntryCandidateKind
        {
            OpenEndpoint,
            ClosedCurveParameter,
            ClosedLoopConnection,
            ClosedLoopArcInterior,
            ClosedLoopEllipseInterior,
            ClosedLoopZoneRunMidpoint,
            BreakZoneMidpoint
        };

        struct ZoneEntryCandidate
        {
            machining::TubeZone16 zone =
                machining::TubeZone16::TopFace;
            ZoneEntryCandidateKind kind =
                ZoneEntryCandidateKind::OpenEndpoint;
            EntityId entityId = 0;
            geometry::SourceGeometryKind sourceKind =
                geometry::SourceGeometryKind::Unknown;
            std::optional<double> sourceParameter;
            bool reverse = false;
            Vector3d entryPosition;
            Vector3d firstCutPoint;
            Vector3d firstCutTangent;
            double entryX = 0.0;
            double confidence = 0.0;
            double distanceToMemberEndpoint = 0.0;
            double distanceToZoneBoundary = 0.0;
            bool ambiguous = false;
        };

        struct TraversalSelectionContext
        {
            std::optional<machining::TubeZone16> requiredEntryZone;
            int longitudinalDirection = 1;
            double zoneHitX = 0.0;
            double frontierX = 0.0;
            double projectionTolerance = 0.0;
            Vector3d previousEnd;
            Vector3d previousCutEnd;
            Vector3d previousTransferAnchor;
            bool hardZoneConstraint = false;
            bool allowZoneRunMidpointFallback = false;
        };

        struct GroupTraversal
        {
            int groupId = -1;
            std::vector<DirectedEntity> entities;
            Vector3d start;
            Vector3d end;
            double movementDistance = 0.0;
            double rotationCost = 0.0;
            int surfaceCost = 0;
            int entryAxisReversalCount = 0;
            double entryTangentCost = 0.0;
            int entryCandidateCount = 0;
            int connectionCandidateCount = 0;
            int arcInteriorCandidateCount = 0;
            int ellipseInteriorCandidateCount = 0;
            int zoneRunMidpointCandidateCount = 0;
            int curveCandidateRejectedCount = 0;
            int wrongZoneRejectedCount = 0;
            std::size_t stableSourceIndex = 0;
            EntityId stableEntityId = 0;
            std::vector<EntityId> arcInteriorCandidateEntityIds;
            std::vector<EntityId> ellipseInteriorCandidateEntityIds;
            std::optional<ZoneEntryCandidate> selectedEntry;
            std::vector<ProcessPathFragment> fragments;
            QString entryRefinementMode;
            Vector3d previousCutEnd;
            Vector3d previousTransferAnchor;
            int curveMemberCount = 0;
            int arcTangentRootCount = 0;
            int ellipseTangentRootCount = 0;
            int validTangentCount = 0;
            double entryTravelDistance = 0.0;
            double approachCutAngle = 0.0;
            double nearestConnectionDistance = 0.0;
            double forwardAngle = 0.0;
            double reverseAngle = 0.0;
            double tangentResidual = 0.0;
            double approachCutDot = 0.0;
        };

        struct ClosedLoopTraversalReport
        {
            int groupId = -1;
            std::vector<EntityId> memberEntityIds;
            int memberCount = 0;
            int nodeCount = 0;
            int connectedComponentCount = 0;
            int branchNodeCount = 0;
            int invalidDegreeNodeCount = 0;
            int candidateCount = 0;
            int wrongZoneRejectedCount = 0;
            std::vector<EntityId> selectedOrder;
            std::vector<bool> selectedReverse;
            bool simpleLoopValid = false;
            QString status = QStringLiteral("Failed");
            QString failureReason;
        };

        struct SectionProjection
        {
            bool valid = false;
            double perimeterPosition = 0.0;
            double distance = std::numeric_limits<double>::max();
        };

        struct SectionBounds
        {
            bool valid = false;
            double minimumY = 0.0;
            double maximumY = 0.0;
            double minimumZ = 0.0;
            double maximumZ = 0.0;
        };

        struct ProcessSurfaceFootprint
        {
            machining::TubeSurfaceRegion dominantRegion =
                machining::TubeSurfaceRegion::Unknown;
            machining::TubeSurfaceRegion entryRegion =
                machining::TubeSurfaceRegion::Unknown;
            machining::TubeSurfaceRegion exitRegion =
                machining::TubeSurfaceRegion::Unknown;
            double minimumX = 0.0;
            double maximumX = 0.0;
            double anchorX = 0.0;
            double minimumPerimeterPosition = 0.0;
            double maximumPerimeterPosition = 0.0;
        };

        struct SurfaceSweepState
        {
            machining::TubeSurfaceRegion currentRegion =
                machining::TubeSurfaceRegion::Unknown;
            int perimeterDirection = 0;
            int longitudinalDirection = 0;
            double currentX = 0.0;
            double currentPerimeterPosition = 0.0;
            bool initialized = false;
        };

        struct SurfaceSweepReport
        {
            int partitionId = -1;
            machining::TubeSurfaceRegion initialRegion =
                machining::TubeSurfaceRegion::Unknown;
            int perimeterDirection = 0;
            int longitudinalDirection = 0;
            int selectedUnitCount = 0;
            int regionTransitionCount = 0;
            int backtrackCount = 0;
            double longitudinalBacktrackDistance = 0.0;
            QStringList selectedUnits;
            bool active = false;
        };

        struct SchedulingCandidate
        {
            GroupTraversal traversal;
            ProcessSurfaceFootprint footprint;
            std::optional<ClosedLoopTraversalReport> closedLoopReport;
            struct BreakBoundaryTraversalReport
            {
                int groupId = -1;
                int boundaryRank = -1;
                int boundaryPairId = -1;
                bool forcedTopMidpoint = false;
                machining::TubeZone16 preferredStartZone =
                    machining::TubeZone16::TopFace;
                int candidateRunCount = 0;
                QString candidateRuns;
                machining::TubeZone16 startZone =
                    machining::TubeZone16::TopFace;
                double selectedRunLength = 0.0;
                double selectedMaximumShellDeviation = 0.0;
                double selectedConfidence = 0.0;
                Vector3d selectedMidpoint;
                EntityId selectedEntityId = 0;
                double selectedSourceParameter = 0.0;
                std::optional<machining::TubeZone16> exitZone;
                double exitConfidence = 0.0;
                double exitReliableLength = 0.0;
                EntityId finalEntityId = 0;
                double finalParameterBegin = 0.0;
                double finalParameterEnd = 0.0;
                bool exitUsedFallback = false;
                int fragmentCount = 0;
                int nextPartitionId = -1;
                bool partitionMappingFound = false;
                bool partitionStartSucceeded = false;
                QString direction;
                QString status = QStringLiteral("Failed");
                QString failureReason;
                DiagnosticCode failureCode =
                    DiagnosticCode::ProcessPlanningBreakFragmentTraversalInvalid;
                std::vector<ProcessPathFragment> fragments;
            };
            std::optional<BreakBoundaryTraversalReport> breakReport;
        };
        using BreakBoundaryTraversalReport =
            SchedulingCandidate::BreakBoundaryTraversalReport;

        struct ProcessGroupZoneProfile
        {
            machining::TubeZoneMask certainMask = 0U;
            machining::TubeZoneMask possibleMask = 0U;
            machining::TubeZoneMask connectionEntryMask = 0U;
            machining::TubeZoneMask curveInteriorEntryMask = 0U;
            machining::TubeZoneMask zoneRunMidpointEntryMask = 0U;
            machining::TubeZoneMask legalEntryMask = 0U;
            machining::TubeZoneMask schedulableMask = 0U;
            std::array<machining::TubeZoneSpan,
                machining::kTubeZone16Count> zoneSpans;
            std::array<std::vector<ZoneEntryCandidate>,
                machining::kTubeZone16Count> entryCandidates;
            std::array<int, machining::kTubeZone16Count>
                entryCandidateCounts{};
            std::array<std::vector<EntityId>,
                machining::kTubeZone16Count> arcMemberIdsByZone;
            std::array<std::vector<EntityId>,
                machining::kTubeZone16Count> ellipseMemberIdsByZone;
            bool closed = false;
            bool uncertain = false;
        };

        struct ZoneSweepSelection
        {
            int groupId = -1;
            machining::TubeZone16 zone = machining::TubeZone16::TopFace;
            machining::TubeZoneSpan span;
            double hitX = 0.0;
            double frontierBefore = 0.0;
            bool fallbackOwner = false;
        };

        struct ZoneSweepOwnership
        {
            int groupId = -1;
            machining::TubeZone16 ownerZone =
                machining::TubeZone16::TopFace;
            bool usedPossibleFallback = false;
            bool usedBoundaryFallback = false;
            machining::TubeZoneMask ownerCandidateMask = 0U;
            machining::TubeZoneMask legalEntryMaskBefore = 0U;
        };

        struct TubeZoneSweepPartition
        {
            int partitionId = -1;
            double minimumX = 0.0;
            double maximumX = 0.0;
            int longitudinalDirection = 1;
            machining::TubeZone16 initialZone =
                machining::TubeZone16::TopFace;
            int perimeterDirection = 1;
            std::array<std::vector<int>,
                machining::kTubeZone16Count> zoneBuckets;
            std::unordered_set<int> groupIds;
            std::unordered_map<int, ZoneSweepOwnership> ownerships;
        };

        struct Zone16SweepState
        {
            int partitionId = -1;
            machining::TubeZone16 initialZone =
                machining::TubeZone16::TopFace;
            int currentZoneOffset = 0;
            int longitudinalDirection = 1;
            double frontierX = 0.0;
            bool zoneEntered = false;
            bool active = false;
            std::array<bool, machining::kTubeZone16Count> enteredZones{};
            std::array<bool, machining::kTubeZone16Count> completedZones{};
        };

        struct Zone16SweepReport
        {
            int partitionId = -1;
            machining::TubeZone16 initialZone =
                machining::TubeZone16::TopFace;
            int perimeterDirection = 1;
            int longitudinalDirection = 1;
            double partitionMinimumX = 0.0;
            double partitionMaximumX = 0.0;
            int processedUnitCount = 0;
            int zoneTransitionCount = 0;
            int backtrackCount = 0;
            QStringList selectedUnits;
            QString status = QStringLiteral("Success");
            bool active = false;
        };

        double distance(const Vector3d& left, const Vector3d& right)
        {
            return std::sqrt
            (
                (left.x - right.x) * (left.x - right.x)
                + (left.y - right.y) * (left.y - right.y)
                + (left.z - right.z) * (left.z - right.z)
            );
        }

        Vector3d entrySubtract(const Vector3d& left, const Vector3d& right)
        {
            return { left.x - right.x, left.y - right.y, left.z - right.z };
        }

        double entryDot(const Vector3d& left, const Vector3d& right)
        {
            return left.x * right.x + left.y * right.y + left.z * right.z;
        }

        Vector3d entryCross(const Vector3d& left, const Vector3d& right)
        {
            return
            {
                left.y * right.z - left.z * right.y,
                left.z * right.x - left.x * right.z,
                left.x * right.y - left.y * right.x
            };
        }

        double entryLength(const Vector3d& value)
        {
            return std::sqrt(entryDot(value, value));
        }

        std::optional<Vector3d> entryNormalized(const Vector3d& value)
        {
            const double length = entryLength(value);
            if (!std::isfinite(length) || length <= kCalculationEpsilon)
                return std::nullopt;
            return Vector3d
            {
                value.x / length,
                value.y / length,
                value.z / length
            };
        }

        constexpr double kEntryTwoPi =
            6.283185307179586476925286766559;

        std::optional<double> parameterInPositiveSweep
        (
            double parameter,
            double start,
            double end,
            double tolerance
        )
        {
            if (!std::isfinite(parameter) || !std::isfinite(start)
                || !std::isfinite(end))
            {
                return std::nullopt;
            }
            while (end <= start) end += kEntryTwoPi;
            while (parameter < start - tolerance) parameter += kEntryTwoPi;
            while (parameter > end + tolerance
                && parameter - kEntryTwoPi >= start - tolerance)
            {
                parameter -= kEntryTwoPi;
            }
            if (parameter < start - tolerance || parameter > end + tolerance)
                return std::nullopt;
            return std::clamp(parameter, start, end);
        }

        struct ExactCurveTangentRoot
        {
            double parameter = 0.0;
            Vector3d point;
            double residual = 0.0;
        };

        std::vector<ExactCurveTangentRoot> arcTangentRoots
        (
            const geometry::ArcGeometry& arc,
            const Vector3d& externalPoint,
            double tolerance
        )
        {
            std::vector<ExactCurveTangentRoot> roots;
            const auto axisU = entryNormalized(arc.axisU);
            const auto axisV = entryNormalized(arc.axisV);
            if (!axisU.has_value() || !axisV.has_value()
                || !std::isfinite(arc.radius) || arc.radius <= tolerance)
            {
                return roots;
            }
            const Vector3d offset = entrySubtract(externalPoint, arc.center);
            const double localX = entryDot(offset, *axisU);
            const double localY = entryDot(offset, *axisV);
            const double radialDistance = std::hypot(localX, localY);
            if (!std::isfinite(radialDistance)
                || radialDistance <= arc.radius + tolerance)
            {
                return roots;
            }
            const double base = std::atan2(localY, localX);
            const double delta = std::acos(std::clamp
                (arc.radius / radialDistance, -1.0, 1.0));
            for (const double rawParameter : { base - delta, base + delta })
            {
                const auto parameter = parameterInPositiveSweep
                    (rawParameter, arc.startParameter, arc.endParameter,
                        tolerance);
                if (!parameter.has_value()) continue;
                const Vector3d radial
                {
                    axisU->x * arc.radius * std::cos(*parameter)
                        + axisV->x * arc.radius * std::sin(*parameter),
                    axisU->y * arc.radius * std::cos(*parameter)
                        + axisV->y * arc.radius * std::sin(*parameter),
                    axisU->z * arc.radius * std::cos(*parameter)
                        + axisV->z * arc.radius * std::sin(*parameter)
                };
                const Vector3d point
                {
                    arc.center.x + radial.x,
                    arc.center.y + radial.y,
                    arc.center.z + radial.z
                };
                const double scale = std::max
                    (1.0, arc.radius * distance(externalPoint, point));
                const double residual =
                    std::abs(entryDot(radial,
                        entrySubtract(externalPoint, point))) / scale;
                roots.push_back({ *parameter, point, residual });
            }
            return roots;
        }

        std::vector<ExactCurveTangentRoot> ellipseTangentRoots
        (
            const geometry::EllipseGeometry& ellipse,
            const Vector3d& externalPoint,
            double tolerance
        )
        {
            std::vector<ExactCurveTangentRoot> roots;
            const Vector3d normal = entryCross
                (ellipse.majorAxis, ellipse.minorAxis);
            const auto unitNormal = entryNormalized(normal);
            if (!unitNormal.has_value()) return roots;
            double start = ellipse.startParameter;
            double end = ellipse.endParameter;
            while (end <= start) end += kEntryTwoPi;
            const auto pointAt = [&ellipse](double parameter)
            {
                return Vector3d
                {
                    ellipse.center.x
                        + ellipse.majorAxis.x * std::cos(parameter)
                        + ellipse.minorAxis.x * std::sin(parameter),
                    ellipse.center.y
                        + ellipse.majorAxis.y * std::cos(parameter)
                        + ellipse.minorAxis.y * std::sin(parameter),
                    ellipse.center.z
                        + ellipse.majorAxis.z * std::cos(parameter)
                        + ellipse.minorAxis.z * std::sin(parameter)
                };
            };
            const auto derivativeAt = [&ellipse](double parameter)
            {
                return Vector3d
                {
                    -ellipse.majorAxis.x * std::sin(parameter)
                        + ellipse.minorAxis.x * std::cos(parameter),
                    -ellipse.majorAxis.y * std::sin(parameter)
                        + ellipse.minorAxis.y * std::cos(parameter),
                    -ellipse.majorAxis.z * std::sin(parameter)
                        + ellipse.minorAxis.z * std::cos(parameter)
                };
            };
            const auto equation = [&](double parameter)
            {
                const Vector3d point = pointAt(parameter);
                return entryDot(entryCross
                    (entrySubtract(externalPoint, point),
                        derivativeAt(parameter)), *unitNormal);
            };
            constexpr int kScanIntervals = 64;
            constexpr int kMaximumIterations = 64;
            const double parameterTolerance = std::max(1.0e-12, tolerance);
            const double residualTolerance = std::max
                (1.0e-10, tolerance * std::max
                    (entryLength(ellipse.majorAxis),
                        entryLength(ellipse.minorAxis)));
            double left = start;
            double leftValue = equation(left);
            for (int interval = 1; interval <= kScanIntervals; ++interval)
            {
                const double right = start + (end - start)
                    * static_cast<double>(interval)
                    / static_cast<double>(kScanIntervals);
                const double rightValue = equation(right);
                std::optional<double> root;
                if (std::abs(leftValue) <= residualTolerance)
                    root = left;
                else if (std::isfinite(leftValue)
                    && std::isfinite(rightValue)
                    && leftValue * rightValue < 0.0)
                {
                    double low = left;
                    double high = right;
                    double lowValue = leftValue;
                    for (int iteration = 0;
                        iteration < kMaximumIterations; ++iteration)
                    {
                        const double middle = (low + high) * 0.5;
                        const double middleValue = equation(middle);
                        if (std::abs(middleValue) <= residualTolerance
                            || high - low <= parameterTolerance)
                        {
                            low = high = middle;
                            break;
                        }
                        if (lowValue * middleValue <= 0.0)
                            high = middle;
                        else
                        {
                            low = middle;
                            lowValue = middleValue;
                        }
                    }
                    root = (low + high) * 0.5;
                }
                if (root.has_value())
                {
                    bool duplicate = false;
                    for (const auto& existing : roots)
                    {
                        duplicate = duplicate
                            || std::abs(existing.parameter - *root)
                                <= parameterTolerance * 8.0;
                    }
                    if (!duplicate)
                    {
                        roots.push_back
                        ({
                            *root,
                            pointAt(*root),
                            std::abs(equation(*root))
                        });
                    }
                }
                left = right;
                leftValue = rightValue;
            }
            if (std::abs(leftValue) <= residualTolerance)
            {
                bool duplicate = false;
                for (const auto& existing : roots)
                    duplicate = duplicate
                        || std::abs(existing.parameter - end)
                            <= parameterTolerance * 8.0;
                if (!duplicate)
                    roots.push_back({ end, pointAt(end), std::abs(leftValue) });
            }
            return roots;
        }

