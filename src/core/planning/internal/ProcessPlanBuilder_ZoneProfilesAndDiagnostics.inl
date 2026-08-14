        machining::TubeZoneMask strongTubeZoneMask()
        {
            machining::TubeZoneMask mask = 0U;
            for (std::size_t index = 0U;
                index < machining::kTubeZone16Count; index += 2U)
            {
                mask |= machining::tubeZoneBit
                    (static_cast<machining::TubeZone16>(index));
            }
            return mask;
        }

        bool zoneCompleted
        (
            const TubeZoneSweepPartition& partition,
            machining::TubeZone16 zone,
            const std::unordered_set<int>& scheduled
        )
        {
            const auto& bucket =
                partition.zoneBuckets[machining::tubeZoneIndex(zone)];
            return std::all_of(bucket.cbegin(), bucket.cend(),
                [&scheduled](int groupId)
                {
                    return scheduled.find(groupId) != scheduled.end();
                });
        }

        QString zoneMaskText(machining::TubeZoneMask mask)
        {
            return QStringLiteral("0x%1").arg(static_cast<unsigned int>(mask),
                4, 16, QLatin1Char('0')).toUpper();
        }

        QString zoneSpansText(const ProcessGroupZoneProfile& profile)
        {
            QStringList spans;
            for (std::size_t index = 0U;
                index < machining::kTubeZone16Count; ++index)
            {
                const machining::TubeZone16 zone =
                    static_cast<machining::TubeZone16>(index);
                if ((profile.possibleMask & machining::tubeZoneBit(zone)) == 0U)
                    continue;
                const machining::TubeZoneSpan& span = profile.zoneSpans[index];
                spans.push_back(QStringLiteral("%1:[%2,%3]")
                    .arg(machining::tubeZoneName(zone))
                    .arg(span.minimumX, 0, 'f', 6)
                    .arg(span.maximumX, 0, 'f', 6));
            }
            return spans.join(QLatin1Char(';'));
        }

        QString processGroupKeyText(const ProcessGroup& group)
        {
            std::vector<EntityId> ids = group.entityIds;
            std::sort(ids.begin(), ids.end());
            QStringList values;
            values.reserve(static_cast<qsizetype>(ids.size()));
            for (const EntityId entityId : ids)
                values.push_back(QString::number(entityId));
            return values.join(QLatin1Char('+'));
        }

        bool processGroupStableLess
        (
            const ProcessGroup& left,
            const ProcessGroup& right
        )
        {
            std::vector<EntityId> leftIds = left.entityIds;
            std::vector<EntityId> rightIds = right.entityIds;
            std::sort(leftIds.begin(), leftIds.end());
            std::sort(rightIds.begin(), rightIds.end());
            return leftIds < rightIds;
        }

        std::optional<machining::TubeZone16> traversalExitZone
        (
            const GroupTraversal& traversal,
            const machining::TubeSectionModel& section,
            double projectionTolerance
        )
        {
            if (traversal.entities.empty()) return std::nullopt;
            const DirectedEntity& directed = traversal.entities.back();
            if (directed.entity == nullptr) return std::nullopt;
            const std::vector<Vector3d> points = directedTraversalPoints(directed);
            for (std::size_t offset = 1U; offset < points.size(); ++offset)
            {
                const Vector3d& end = points[points.size() - offset];
                const Vector3d& start = points[points.size() - offset - 1U];
                if (distance(start, end) <= kCalculationEpsilon) continue;

                std::array<machining::TubeSectionProjection, 3> samples;
                for (std::size_t sample = 0U; sample < samples.size(); ++sample)
                {
                    const double parameter = 0.25
                        + static_cast<double>(sample) * 0.25;
                    const Vector2d yz
                    {
                        start.y + (end.y - start.y) * parameter,
                        start.z + (end.z - start.z) * parameter
                    };
                    samples[sample] = machining::TubeSectionProjector::project
                        (section, yz, projectionTolerance);
                }

                std::array<int, machining::kTubeZone16Count> counts{};
                for (const auto& sample : samples)
                {
                    if (sample.valid && !sample.ambiguous)
                        ++counts[machining::tubeZoneIndex(sample.zone)];
                }
                int bestCount = 0;
                std::optional<machining::TubeZone16> bestZone;
                bool tied = false;
                for (std::size_t index = 0U; index < counts.size(); ++index)
                {
                    if (counts[index] > bestCount)
                    {
                        bestCount = counts[index];
                        bestZone = static_cast<machining::TubeZone16>(index);
                        tied = false;
                    }
                    else if (counts[index] > 0 && counts[index] == bestCount)
                    {
                        tied = true;
                    }
                }
                if (bestZone.has_value() && !tied) return bestZone;
                if (samples[1].valid && !samples[1].ambiguous)
                    return samples[1].zone;
                return std::nullopt;
            }
            return std::nullopt;
        }

        Diagnostic zone16SweepDiagnostic
        (
            const OperationContext& context,
            const Zone16SweepReport& report
        )
        {
            QVariantMap values;
            values.insert(QStringLiteral("zone16SweepSummary"), true);
            values.insert(QStringLiteral("partitionId"), report.partitionId);
            values.insert(QStringLiteral("initialZone"),
                machining::tubeZoneName(report.initialZone));
            values.insert(QStringLiteral("perimeterDirection"),
                report.perimeterDirection >= 0
                    ? QStringLiteral("Clockwise")
                    : QStringLiteral("CounterClockwise"));
            values.insert(QStringLiteral("longitudinalDirection"),
                report.longitudinalDirection);
            values.insert(QStringLiteral("partitionMinimumX"),
                report.partitionMinimumX);
            values.insert(QStringLiteral("partitionMaximumX"),
                report.partitionMaximumX);
            values.insert(QStringLiteral("processedUnitCount"),
                report.processedUnitCount);
            values.insert(QStringLiteral("zoneTransitions"),
                report.zoneTransitionCount);
            values.insert(QStringLiteral("backtrackCount"),
                report.backtrackCount);
            values.insert(QStringLiteral("selectedUnits"),
                report.selectedUnits);
            values.insert(QStringLiteral("status"), report.status);
            return planningDiagnostic
            (
                context,
                DiagnosticCode::ProcessPlanningZone16SweepSummary,
                QStringLiteral("四轴 16 区位扫描加工段已完成。"),
                QStringLiteral("Zone16 sweep partition completed."),
                values,
                report.status == QStringLiteral("Success")
                    ? DiagnosticSeverity::Info : DiagnosticSeverity::Warning
            );
        }

        Diagnostic zoneOwnershipDiagnostic
        (
            const OperationContext& context,
            const TubeZoneSweepPartition& partition,
            const ProcessGroup& group,
            const ProcessGroupZoneProfile& profile,
            const ZoneSweepOwnership& ownership
        )
        {
            QVariantMap values;
            values.insert(QStringLiteral("zoneOwnership"), true);
            values.insert(QStringLiteral("partitionId"), partition.partitionId);
            values.insert(QStringLiteral("unitKey"), processGroupKeyText(group));
            values.insert(QStringLiteral("certainMask"),
                zoneMaskText(profile.certainMask));
            values.insert(QStringLiteral("possibleMask"),
                zoneMaskText(profile.possibleMask));
            values.insert(QStringLiteral("ownerCandidateMask"),
                zoneMaskText(ownership.ownerCandidateMask));
            values.insert(QStringLiteral("ownerZone"),
                machining::tubeZoneName(ownership.ownerZone));
            values.insert(QStringLiteral("ownerBasis"),
                ownership.usedPossibleFallback
                ? QStringLiteral("PossibleOccupancyFallback")
                : QStringLiteral("CertainOccupancy"));
            values.insert(QStringLiteral("legalEntryMaskBefore"),
                zoneMaskText(ownership.legalEntryMaskBefore));
            values.insert(QStringLiteral("usedPossibleFallback"),
                ownership.usedPossibleFallback);
            values.insert(QStringLiteral("usedBoundaryFallback"),
                ownership.usedBoundaryFallback);
            return planningDiagnostic
            (
                context,
                ownership.usedPossibleFallback
                    || ownership.usedBoundaryFallback
                    ? DiagnosticCode::ProcessPlanningZoneSweepFallbackOwner
                    : DiagnosticCode::ProcessPlanningZone16SweepSummary,
                ownership.usedPossibleFallback
                    || ownership.usedBoundaryFallback
                    ? QStringLiteral("加工单元使用保守区位作为唯一生产归属。")
                    : QStringLiteral("加工单元已确定唯一生产区位。"),
                QStringLiteral("A single immutable owner zone was selected for this partition."),
                values,
                ownership.usedPossibleFallback
                    || ownership.usedBoundaryFallback
                    ? DiagnosticSeverity::Warning : DiagnosticSeverity::Info
            );
        }

        Diagnostic zonePhaseDiagnostic
        (
            const OperationContext& context,
            const TubeZoneSweepPartition& partition,
            machining::TubeZone16 zone,
            const std::unordered_set<int>& scheduled,
            const QString& event
        )
        {
            const auto& bucket =
                partition.zoneBuckets[machining::tubeZoneIndex(zone)];
            const int processedUnitCount = static_cast<int>(std::count_if
            (
                bucket.cbegin(), bucket.cend(),
                [&scheduled](int groupId)
                {
                    return scheduled.find(groupId) != scheduled.end();
                }
            ));
            QVariantMap values;
            values.insert(QStringLiteral("zonePhase"), true);
            values.insert(QStringLiteral("partitionId"), partition.partitionId);
            values.insert(QStringLiteral("zone"), machining::tubeZoneName(zone));
            values.insert(QStringLiteral("event"), event);
            values.insert(QStringLiteral("ownedUnitCount"),
                static_cast<int>(bucket.size()));
            values.insert(QStringLiteral("processedUnitCount"),
                processedUnitCount);
            values.insert(QStringLiteral("remainingUnitCount"),
                static_cast<int>(bucket.size()) - processedUnitCount);
            return planningDiagnostic
            (
                context,
                DiagnosticCode::ProcessPlanningZone16SweepSummary,
                event == QStringLiteral("Enter")
                    ? QStringLiteral("16 区位加工阶段已进入。")
                    : QStringLiteral("16 区位加工阶段已完成。"),
                QStringLiteral("Zone phase lifecycle event."),
                values,
                DiagnosticSeverity::Info
            );
        }

        QVariantMap closedLoopDiagnosticValues(const ClosedLoopTraversalReport& report)
        {
            auto entityIdsText = [](const std::vector<EntityId>& entityIds)
            {
                QStringList values;
                values.reserve(static_cast<qsizetype>(entityIds.size()));
                for (const EntityId entityId : entityIds)
                    values.push_back(QString::number(entityId));
                return values.join(QLatin1Char(','));
            };
            QStringList reverseValues;
            reverseValues.reserve(static_cast<qsizetype>(report.selectedReverse.size()));
            for (const bool reverse : report.selectedReverse)
                reverseValues.push_back(reverse ? QStringLiteral("1") : QStringLiteral("0"));

            QVariantMap values;
            values.insert(QStringLiteral("closedLoopSummary"), true);
            values.insert(QStringLiteral("groupId"), report.groupId);
            values.insert(QStringLiteral("memberCount"), report.memberCount);
            values.insert(QStringLiteral("memberEntityIds"), entityIdsText(report.memberEntityIds));
            values.insert(QStringLiteral("nodeCount"), report.nodeCount);
            values.insert(QStringLiteral("connectedComponentCount"), report.connectedComponentCount);
            values.insert(QStringLiteral("branchNodeCount"), report.branchNodeCount);
            values.insert(QStringLiteral("invalidDegreeNodeCount"), report.invalidDegreeNodeCount);
            values.insert(QStringLiteral("candidateCount"), report.candidateCount);
            values.insert(QStringLiteral("selectedOrder"), entityIdsText(report.selectedOrder));
            values.insert(QStringLiteral("selectedReverse"), reverseValues.join(QLatin1Char(',')));
            values.insert(QStringLiteral("status"), report.status);
            values.insert(QStringLiteral("failureReason"), report.failureReason);
            return values;
        }

        Diagnostic surfaceSweepDiagnostic
        (
            const OperationContext& context,
            const SurfaceSweepReport& report
        )
        {
            QVariantMap values;
            values.insert(QStringLiteral("surfaceSweepSummary"), true);
            values.insert(QStringLiteral("partitionId"), report.partitionId);
            values.insert(QStringLiteral("initialRegion"),
                surfaceRegionName(report.initialRegion));
            values.insert(QStringLiteral("perimeterDirection"),
                report.perimeterDirection);
            values.insert(QStringLiteral("longitudinalDirection"),
                report.longitudinalDirection);
            values.insert(QStringLiteral("selectedUnits"),
                report.selectedUnits.join(QLatin1Char(',')));
            values.insert(QStringLiteral("selectedUnitCount"),
                report.selectedUnitCount);
            values.insert(QStringLiteral("regionTransitions"),
                report.regionTransitionCount);
            values.insert(QStringLiteral("backtrackCount"),
                report.backtrackCount);
            values.insert(QStringLiteral("longitudinalBacktrackDistance"),
                report.longitudinalBacktrackDistance);
            values.insert(QStringLiteral("status"), QStringLiteral("Success"));
            return planningDiagnostic
            (
                context,
                DiagnosticCode::ProcessPlanningSurfaceSweepSummary,
                QStringLiteral("四轴按面扫描分区已完成。"),
                QStringLiteral("LazyRotation surface sweep partition completed."),
                values,
                DiagnosticSeverity::Info
            );
        }

