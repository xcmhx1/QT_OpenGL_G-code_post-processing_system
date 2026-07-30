#include "core/planning/SingleClosedEntryRefiner.h"

#include "core/geometry/GeometryCompiler.h"
#include "core/machine/ProcessUnitExecutionResolver.h"
#include "core/machine/RotaryTransferPlanner.h"
#include "core/machining/TubeSectionProjector.h"

#include <QVariantMap>

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>

namespace cadcam::planning
{
    namespace
    {
        constexpr double kTwoPi = 6.28318530717958647692;
        constexpr int kSearchSamples = 256;
        constexpr int kMaximumRootIterations = 48;
        constexpr std::size_t kMaximumRootCandidates = 32U;
        constexpr double kRootResidualTolerance = 1.0e-6;

        enum class SingleClosedEntryMode
        {
            ExactDynamicTangent,
            ClosestOwnerZoneParameterFallback
        };

        struct EntryCandidate
        {
            double parameter = 0.0;
            bool reverse = false;
            geometry::Path3D path;
            std::vector<machine::MachinePose4D> poses;
            machine::RotaryTransferPreview transfer;
            machine::ProcessUnitExecutionResult execution;
            machine::MachinePose4D previousCutEnd;
            geometry::Vector3d previousSourceEnd;
            double signedResidual = 0.0;
            double tangentResidual = 0.0;
            double approachCutDot = -1.0;
            double approachCutAngle = 3.14159265358979323846;
            bool tangentComparable = true;
        };

        struct SearchResult
        {
            std::optional<EntryCandidate> selected;
            SingleClosedEntryMode mode =
                SingleClosedEntryMode::ExactDynamicTangent;
            int intervalCount = 0;
            int rootCandidateCount = 0;
            int validTangentCount = 0;
            int evaluationCount = 0;
            int validEvaluationCount = 0;
            int missingInputRejectedCount = 0;
            int pathCompileRejectedCount = 0;
            int invalidProjectionRejectedCount = 0;
            int ambiguousProjectionRejectedCount = 0;
            int wrongOwnerZoneRejectedCount = 0;
            int executionRejectedCount = 0;
            int transferPreviewRejectedCount = 0;
            int curveEvaluationRejectedCount = 0;
            int invalidTangentRejectedCount = 0;
            int nonPlanarApproachCount = 0;
            int fallbackCandidateCount = 0;
            int initialApproachCount = 0;
            int sameZoneSurfaceTransferCount = 0;
            int sameZoneClearanceTransferCount = 0;
            int crossZoneRotaryTransferCount = 0;
        };

        double distance3D
        (
            const geometry::Vector3d& left,
            const geometry::Vector3d& right
        )
        {
            return std::sqrt
            (
                (left.x - right.x) * (left.x - right.x)
                + (left.y - right.y) * (left.y - right.y)
                + (left.z - right.z) * (left.z - right.z)
            );
        }

        geometry::Vector3d interpolatePoint
        (
            const geometry::Vector3d& start,
            const geometry::Vector3d& end,
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

        machine::RotaryMachinePolicy machinePolicy
        (
            const ProcessPlanningPolicy& policy,
            const machining::TubeSectionModel& section
        )
        {
            machine::RotaryMachinePolicy value;
            value.rotaryAxisY = policy.rotaryAxisY;
            value.rotaryAxisZ = policy.rotaryAxisZ;
            value.tubeCenterY = section.geometry.centerY;
            value.tubeCenterZ = section.geometry.centerZ;
            value.invertAAxisDirection = policy.invertAAxisDirection;
            value.aAxisOffsetDegrees = policy.aAxisOffsetDegrees;
            value.keepContinuousAngle = policy.keepContinuousAngle;
            value.useInitialMachinePoint = policy.useInitialMachinePoint;
            value.initialMachinePoint =
            {
                policy.initialMachinePoint.x,
                policy.initialMachinePoint.y,
                policy.initialMachinePoint.z,
                policy.initialMachinePoint.aDegrees
            };
            value.transfer.rotationSafetyClearance =
                policy.rotationSafetyClearance;
            value.transfer.sameZoneTransferClearance =
                policy.sameZoneTransferClearance;
            value.transfer.coordinatedTransferEnabled =
                policy.coordinatedTransferEnabled;
            value.machiningPlaneZOffset =
                policy.machiningPlaneZOffset;
            value.overcutDistance = policy.overcutDistance;
            value.continuousConnectionTolerance =
                policy.connectionTolerance;
            value.numericalEpsilon = policy.numericalEpsilon;
            value.surfaceClassificationTolerance =
                std::max(policy.numericalEpsilon,
                    std::max(section.geometry.yLength,
                        section.geometry.zWidth) * 1.0e-8);
            return value;
        }

        double normalizeParameter(double parameter, double start)
        {
            while (parameter < start) parameter += kTwoPi;
            while (parameter >= start + kTwoPi) parameter -= kTwoPi;
            return parameter;
        }

        bool curvePointAndDerivative
        (
            const PlanningEntity& entity,
            double parameter,
            geometry::Vector3d& point,
            geometry::Vector3d& derivative
        )
        {
            if (!entity.sourceEntity.has_value()) return false;
            if (entity.sourceKind == geometry::SourceGeometryKind::Circle)
            {
                const auto* circle = std::get_if<geometry::CircleGeometry>
                    (&entity.sourceEntity->geometry);
                if (circle == nullptr || !std::isfinite(circle->radius)
                    || circle->radius <= 0.0)
                {
                    return false;
                }
                point =
                {
                    circle->center.x
                        + circle->axisU.x * circle->radius
                            * std::cos(parameter)
                        + circle->axisV.x * circle->radius
                            * std::sin(parameter),
                    circle->center.y
                        + circle->axisU.y * circle->radius
                            * std::cos(parameter)
                        + circle->axisV.y * circle->radius
                            * std::sin(parameter),
                    circle->center.z
                        + circle->axisU.z * circle->radius
                            * std::cos(parameter)
                        + circle->axisV.z * circle->radius
                            * std::sin(parameter)
                };
                derivative =
                {
                    -circle->axisU.x * circle->radius
                        * std::sin(parameter)
                        + circle->axisV.x * circle->radius
                            * std::cos(parameter),
                    -circle->axisU.y * circle->radius
                        * std::sin(parameter)
                        + circle->axisV.y * circle->radius
                            * std::cos(parameter),
                    -circle->axisU.z * circle->radius
                        * std::sin(parameter)
                        + circle->axisV.z * circle->radius
                            * std::cos(parameter)
                };
                return true;
            }
            const auto* ellipse = std::get_if<geometry::EllipseGeometry>
                (&entity.sourceEntity->geometry);
            if (ellipse == nullptr || !ellipse->fullEllipse) return false;
            point =
            {
                ellipse->center.x
                    + ellipse->majorAxis.x * std::cos(parameter)
                    + ellipse->minorAxis.x * std::sin(parameter),
                ellipse->center.y
                    + ellipse->majorAxis.y * std::cos(parameter)
                    + ellipse->minorAxis.y * std::sin(parameter),
                ellipse->center.z
                    + ellipse->majorAxis.z * std::cos(parameter)
                    + ellipse->minorAxis.z * std::sin(parameter)
            };
            derivative =
            {
                -ellipse->majorAxis.x * std::sin(parameter)
                    + ellipse->minorAxis.x * std::cos(parameter),
                -ellipse->majorAxis.y * std::sin(parameter)
                    + ellipse->minorAxis.y * std::cos(parameter),
                -ellipse->majorAxis.z * std::sin(parameter)
                    + ellipse->minorAxis.z * std::cos(parameter)
            };
            return true;
        }

        double parameterStart(const PlanningEntity& entity)
        {
            if (entity.sourceKind == geometry::SourceGeometryKind::Ellipse
                && entity.sourceEntity.has_value())
            {
                const auto* ellipse =
                    std::get_if<geometry::EllipseGeometry>
                        (&entity.sourceEntity->geometry);
                if (ellipse != nullptr) return ellipse->startParameter;
            }
            return 0.0;
        }

        bool sameUnitKey
        (
            const ProcessUnitKey& left,
            const ProcessUnitKey& right
        )
        {
            return left.memberEntityIds == right.memberEntityIds;
        }

        QString unitKeyText(const ProcessUnitKey& key)
        {
            QStringList values;
            for (const geometry::EntityId entityId : key.memberEntityIds)
                values.push_back(QString::number(entityId));
            return values.join(QLatin1Char('+'));
        }

        QString poseText(const machine::MachinePose4D& pose)
        {
            return QStringLiteral("(%1,%2,%3,%4)")
                .arg(pose.x, 0, 'g', 15)
                .arg(pose.y, 0, 'g', 15)
                .arg(pose.z, 0, 'g', 15)
                .arg(pose.aDegrees, 0, 'g', 15);
        }

        Diagnostic failureDiagnostic
        (
            const OperationContext& context,
            const QString& message,
            const QString& detail,
            geometry::EntityId entityId = 0
        )
        {
            Diagnostic diagnostic;
            diagnostic.code =
                DiagnosticCode::ProcessPlanningInvariantViolation;
            diagnostic.severity = DiagnosticSeverity::Error;
            diagnostic.component =
                QStringLiteral("SingleClosedEntryRefiner");
            diagnostic.operation = context.operationName;
            diagnostic.stage =
                QStringLiteral("refine-single-closed-entry");
            diagnostic.userMessage = message;
            diagnostic.technicalDetail = detail;
            diagnostic.correlationId = context.correlationId;
            if (entityId != 0U) diagnostic.entityId = entityId;
            return diagnostic;
        }

        void appendSearchContext
        (
            QVariantMap& values,
            const SearchResult& search,
            const ProcessUnit& unit,
            int processUnitIndex,
            int previousProcessUnitIndex,
            const std::optional<machining::TubeZone16>& previousOwnerZone,
            const machine::RotaryMachinePolicy& policy,
            double projectionTolerance,
            bool selected
        )
        {
            values.insert(QStringLiteral("singleClosedEntrySearch"), true);
            values.insert(QStringLiteral("searchOutcome"),
                selected ? QStringLiteral("Selected")
                    : QStringLiteral("Failed"));
            values.insert(QStringLiteral("processUnitIndex"),
                processUnitIndex);
            values.insert(QStringLiteral("unitKey"), unitKeyText(unit.key));
            values.insert(QStringLiteral("ownerZone"),
                unit.ownerZone.has_value()
                    ? machining::tubeZoneName(*unit.ownerZone)
                    : QStringLiteral("None"));
            values.insert(QStringLiteral("previousProcessUnitIndex"),
                previousProcessUnitIndex);
            values.insert(QStringLiteral("previousOwnerZone"),
                previousOwnerZone.has_value()
                    ? machining::tubeZoneName(*previousOwnerZone)
                    : QStringLiteral("None"));
            values.insert(QStringLiteral("sameZoneTransferClearance"),
                policy.transfer.sameZoneTransferClearance);
            values.insert(QStringLiteral("rotationSafetyClearance"),
                policy.transfer.rotationSafetyClearance);
            values.insert(QStringLiteral("coordinatedTransferEnabled"),
                policy.transfer.coordinatedTransferEnabled);
            values.insert(QStringLiteral("projectionTolerance"),
                projectionTolerance);
            values.insert(QStringLiteral("evaluationCount"),
                search.evaluationCount);
            values.insert(QStringLiteral("validEvaluationCount"),
                search.validEvaluationCount);
            values.insert(QStringLiteral("searchIntervalCount"),
                search.intervalCount);
            values.insert(QStringLiteral("rootCandidateCount"),
                search.rootCandidateCount);
            values.insert(QStringLiteral("validTangentCount"),
                search.validTangentCount);
            values.insert(QStringLiteral("fallbackCandidateCount"),
                search.fallbackCandidateCount);
            values.insert(QStringLiteral("missingInputRejectedCount"),
                search.missingInputRejectedCount);
            values.insert(QStringLiteral("pathCompileRejectedCount"),
                search.pathCompileRejectedCount);
            values.insert(QStringLiteral("invalidProjectionRejectedCount"),
                search.invalidProjectionRejectedCount);
            values.insert(QStringLiteral("ambiguousProjectionRejectedCount"),
                search.ambiguousProjectionRejectedCount);
            values.insert(QStringLiteral("wrongOwnerZoneRejectedCount"),
                search.wrongOwnerZoneRejectedCount);
            values.insert(QStringLiteral("executionRejectedCount"),
                search.executionRejectedCount);
            values.insert(QStringLiteral("transferPreviewRejectedCount"),
                search.transferPreviewRejectedCount);
            values.insert(QStringLiteral("curveEvaluationRejectedCount"),
                search.curveEvaluationRejectedCount);
            values.insert(QStringLiteral("invalidTangentRejectedCount"),
                search.invalidTangentRejectedCount);
            values.insert(QStringLiteral("nonPlanarApproachCount"),
                search.nonPlanarApproachCount);
            values.insert(QStringLiteral("initialApproachCount"),
                search.initialApproachCount);
            values.insert(QStringLiteral("sameZoneSurfaceTransferCount"),
                search.sameZoneSurfaceTransferCount);
            values.insert(QStringLiteral("sameZoneClearanceTransferCount"),
                search.sameZoneClearanceTransferCount);
            values.insert(QStringLiteral("crossZoneRotaryTransferCount"),
                search.crossZoneRotaryTransferCount);
        }

        Diagnostic failedSearchDiagnostic
        (
            const OperationContext& context,
            const ProcessUnit& unit,
            int processUnitIndex,
            int previousProcessUnitIndex,
            const std::optional<machining::TubeZone16>& previousOwnerZone,
            const PlanningEntity& entity,
            const machine::RotaryMachinePolicy& policy,
            double projectionTolerance,
            const SearchResult& search
        )
        {
            Diagnostic diagnostic = failureDiagnostic
            (
                context,
                QStringLiteral("单图元闭合曲线在所属区位内没有可用动态起刀点。"),
                QStringLiteral("No exact tangent or stable owner-zone fallback parameter was found."),
                entity.entityId
            );
            appendSearchContext(diagnostic.context, search, unit,
                processUnitIndex, previousProcessUnitIndex,
                previousOwnerZone, policy, projectionTolerance, false);
            diagnostic.context.insert(QStringLiteral("entityId"),
                QVariant::fromValue<qulonglong>(entity.entityId));
            diagnostic.context.insert(QStringLiteral("sourceKind"),
                entity.sourceKind == geometry::SourceGeometryKind::Circle
                    ? QStringLiteral("Circle")
                    : QStringLiteral("Ellipse"));
            return diagnostic;
        }

        PlannedMachinePose4D plannedPose
        (
            const machine::MachinePose4D& pose
        )
        {
            return { pose.x, pose.y, pose.z, pose.aDegrees };
        }

        PlannedTransferMotionKind plannedKind
        (
            machine::TransferMotionKind kind
        )
        {
            switch (kind)
            {
            case machine::TransferMotionKind::InitialApproach:
                return PlannedTransferMotionKind::InitialApproach;
            case machine::TransferMotionKind::SameZoneSurfaceTransfer:
                return PlannedTransferMotionKind::SameZoneSurfaceTransfer;
            case machine::TransferMotionKind::SameZoneClearanceTransfer:
                return PlannedTransferMotionKind::SameZoneClearanceTransfer;
            case machine::TransferMotionKind::CrossZoneRotaryTransfer:
                return PlannedTransferMotionKind::CrossZoneRotaryTransfer;
            }
            return PlannedTransferMotionKind::InitialApproach;
        }

        PlannedTransferMotionPhase plannedPhase
        (
            machine::TransferMotionPhase phase
        )
        {
            switch (phase)
            {
            case machine::TransferMotionPhase::SurfaceTransfer:
                return PlannedTransferMotionPhase::SurfaceTransfer;
            case machine::TransferMotionPhase::CoordinatedDeparture:
                return PlannedTransferMotionPhase::CoordinatedDeparture;
            case machine::TransferMotionPhase::SafeRotaryTransfer:
                return PlannedTransferMotionPhase::SafeRotaryTransfer;
            case machine::TransferMotionPhase::CoordinatedApproach:
                return PlannedTransferMotionPhase::CoordinatedApproach;
            case machine::TransferMotionPhase::None:
                break;
            }
            return PlannedTransferMotionPhase::SurfaceTransfer;
        }

        PlannedTransferSignature plannedSignature
        (
            const machine::RotaryTransferPreview& preview,
            const machine::MachinePose4D& previousCutEnd,
            const geometry::Vector3d& previousSourceEnd
        )
        {
            PlannedTransferSignature signature;
            signature.kind = plannedKind(preview.kind);
            signature.previousCutEnd = plannedPose(previousCutEnd);
            signature.previousSourceEnd = previousSourceEnd;
            signature.finalApproachOrigin =
                plannedPose(preview.finalApproachOrigin);
            signature.cutStart = plannedPose(preview.cutStart);
            signature.targets.reserve(preview.targets.size());
            signature.phases.reserve(preview.phases.size());
            for (const machine::MachinePose4D& target : preview.targets)
                signature.targets.push_back(plannedPose(target));
            for (const machine::TransferMotionPhase phase : preview.phases)
                signature.phases.push_back(plannedPhase(phase));
            return signature;
        }

        bool candidateLess
        (
            const EntryCandidate& left,
            const EntryCandidate& right,
            bool exactMode
        )
        {
            const bool leftForward = left.approachCutDot >= 0.0;
            const bool rightForward = right.approachCutDot >= 0.0;
            if (leftForward != rightForward) return leftForward;
            if (exactMode
                && std::abs(left.tangentResidual
                    - right.tangentResidual) > 1.0e-12)
            {
                return left.tangentResidual < right.tangentResidual;
            }
            if (std::abs(left.approachCutDot
                - right.approachCutDot) > 1.0e-12)
            {
                return left.approachCutDot > right.approachCutDot;
            }
            if (std::abs(left.transfer.cost.linearDistance
                - right.transfer.cost.linearDistance) > 1.0e-9)
            {
                return left.transfer.cost.linearDistance
                    < right.transfer.cost.linearDistance;
            }
            if (left.transfer.cost.rotaryDirectionChanges
                != right.transfer.cost.rotaryDirectionChanges)
            {
                return left.transfer.cost.rotaryDirectionChanges
                    < right.transfer.cost.rotaryDirectionChanges;
            }
            if (std::abs(left.transfer.cost.rotaryDegrees
                - right.transfer.cost.rotaryDegrees) > 1.0e-9)
            {
                return left.transfer.cost.rotaryDegrees
                    < right.transfer.cost.rotaryDegrees;
            }
            if (left.parameter != right.parameter)
                return left.parameter < right.parameter;
            return !left.reverse && right.reverse;
        }

        std::optional<EntryCandidate> evaluateCandidate
        (
            const PlanningEntity& entity,
            double parameter,
            bool reverse,
            const ProcessUnit& unit,
            int processUnitIndex,
            int previousProcessUnitIndex,
            const std::optional<machining::TubeZone16>& previousOwnerZone,
            const std::optional<machine::ProcessUnitExecutionResult>&
                previousExecution,
            const machine::RotaryMachinePolicy& policy,
            const std::optional<machining::TubeSectionModel>& section,
            double projectionTolerance,
            double rotationSafeMachineZ,
            SearchResult& search,
            const OperationContext& context
        )
        {
            ++search.evaluationCount;
            if (!unit.ownerZone.has_value()
                || !entity.sourceEntity.has_value())
            {
                ++search.missingInputRejectedCount;
                return std::nullopt;
            }
            geometry::GeometryCompiler compiler;
            geometry::PathCompileOptions options;
            options.reverse = reverse;
            options.startParameter = parameter;
            auto compiled = compiler.compile
            (
                *entity.sourceEntity,
                entity.executionSamplingPolicy,
                options,
                context
            );
            if (!compiled.succeeded() || !compiled.value.has_value()
                || compiled.value->vertices.size() < 2U)
            {
                ++search.pathCompileRejectedCount;
                return std::nullopt;
            }
            const geometry::Vector3d& sourceStart =
                compiled.value->vertices.front().position;
            std::optional<geometry::Vector3d> firstCutPoint;
            for (std::size_t index = 1U;
                index < compiled.value->vertices.size(); ++index)
            {
                if (distance3D(sourceStart,
                    compiled.value->vertices[index].position)
                    > policy.numericalEpsilon)
                {
                    firstCutPoint =
                        compiled.value->vertices[index].position;
                    break;
                }
            }
            if (!firstCutPoint.has_value())
            {
                ++search.pathCompileRejectedCount;
                return std::nullopt;
            }
            for (const double factor : { 0.0, 0.25, 0.5, 0.75 })
            {
                const geometry::Vector3d point = interpolatePoint
                    (sourceStart, *firstCutPoint, factor);
                const machining::TubeSectionProjection projection =
                    machining::TubeSectionProjector::project
                    (*section, { point.y, point.z },
                        projectionTolerance);
                if (!projection.valid)
                {
                    ++search.invalidProjectionRejectedCount;
                    return std::nullopt;
                }
                if (projection.ambiguous)
                {
                    ++search.ambiguousProjectionRejectedCount;
                    return std::nullopt;
                }
                if (projection.zone != *unit.ownerZone)
                {
                    ++search.wrongOwnerZoneRejectedCount;
                    return std::nullopt;
                }
            }

            machine::ProcessUnitExecutionPath executionPath;
            executionPath.entityId = entity.entityId;
            executionPath.sourceIndex = entity.sourceIndex;
            executionPath.sourceKind = entity.sourceKind;
            executionPath.sourceProcessOrder = processUnitIndex;
            executionPath.processUnitIndex = processUnitIndex;
            executionPath.path = *compiled.value;
            auto execution = machine::ProcessUnitExecutionResolver::resolve
            (
                { executionPath },
                true,
                policy,
                section,
                previousExecution.has_value()
                    ? std::optional<machine::MachinePose4D>
                        (previousExecution->finalCutPose)
                    : std::nullopt,
                context
            );
            if (!execution.succeeded()
                || !execution.value.has_value()
                || execution.value->posesByPath.empty()
                || execution.value->posesByPath.front().empty())
            {
                ++search.executionRejectedCount;
                return std::nullopt;
            }
            std::vector<machine::MachinePose4D> poses =
                execution.value->posesByPath.front();

            machine::RotaryTransferRequest request;
            request.previousCutEnd = previousExecution.has_value()
                ? previousExecution->finalCutPose
                : policy.useInitialMachinePoint
                    ? policy.initialMachinePoint
                    : machine::MachinePose4D
                        { poses.front().x, poses.front().y, 0.0, 0.0 };
            request.nextCutStart = poses.front();
            request.previousSourceEnd = previousExecution.has_value()
                ? previousExecution->finalSourcePosition : sourceStart;
            request.nextSourceStart = sourceStart;
            request.previousProcessUnitIndex =
                previousProcessUnitIndex;
            request.nextProcessUnitIndex = processUnitIndex;
            request.previousOwnerZone = previousOwnerZone;
            request.nextOwnerZone = unit.ownerZone;
            request.policy = policy.transfer;
            request.tubeSection = &section;
            request.tubeCenterY = policy.tubeCenterY;
            request.tubeCenterZ = policy.tubeCenterZ;
            request.rotationSafeMachineZ = rotationSafeMachineZ;
            request.numericalEpsilon = policy.numericalEpsilon;
            auto preview = machine::RotaryTransferPlanner::preview
                (request, context);
            if (!preview.succeeded() || !preview.value.has_value())
            {
                ++search.transferPreviewRejectedCount;
                return std::nullopt;
            }
            switch (preview.value->kind)
            {
            case machine::TransferMotionKind::InitialApproach:
                ++search.initialApproachCount;
                break;
            case machine::TransferMotionKind::SameZoneSurfaceTransfer:
                ++search.sameZoneSurfaceTransferCount;
                break;
            case machine::TransferMotionKind::SameZoneClearanceTransfer:
                ++search.sameZoneClearanceTransferCount;
                break;
            case machine::TransferMotionKind::CrossZoneRotaryTransfer:
                ++search.crossZoneRotaryTransferCount;
                break;
            }

            geometry::Vector3d sourcePoint;
            geometry::Vector3d sourceDerivative;
            if (!curvePointAndDerivative(entity, parameter,
                sourcePoint, sourceDerivative))
            {
                ++search.curveEvaluationRejectedCount;
                return std::nullopt;
            }
            if (reverse)
            {
                sourceDerivative.x = -sourceDerivative.x;
                sourceDerivative.y = -sourceDerivative.y;
                sourceDerivative.z = -sourceDerivative.z;
            }
            const double radians =
                poses.front().aDegrees
                * 3.14159265358979323846 / 180.0;
            const double tangentX = sourceDerivative.x;
            const double tangentY = sourceDerivative.y
                    * std::cos(radians)
                - sourceDerivative.z * std::sin(radians);
            const double approachX = poses.front().x
                - preview.value->finalApproachOrigin.x;
            const double approachY = poses.front().y
                - preview.value->finalApproachOrigin.y;
            const double tangentLength =
                std::hypot(tangentX, tangentY);
            const double approachLength =
                std::hypot(approachX, approachY);
            if (!std::isfinite(tangentLength)
                || !std::isfinite(approachLength)
                || tangentLength <= policy.numericalEpsilon)
            {
                ++search.invalidTangentRejectedCount;
                return std::nullopt;
            }

            EntryCandidate candidate;
            candidate.parameter = parameter;
            candidate.reverse = reverse;
            candidate.path = std::move(*compiled.value);
            candidate.poses = std::move(poses);
            candidate.transfer = std::move(*preview.value);
            candidate.previousCutEnd = request.previousCutEnd;
            candidate.previousSourceEnd = request.previousSourceEnd;
            if (approachLength <= policy.numericalEpsilon)
            {
                // A non-coordinated clearance transfer approaches only in Z,
                // so no machine-XY tangent exists to compare.
                candidate.signedResidual = 0.0;
                candidate.tangentResidual =
                    std::numeric_limits<double>::infinity();
                candidate.approachCutDot = 0.0;
                candidate.approachCutAngle =
                    3.14159265358979323846 * 0.5;
                candidate.tangentComparable = false;
                ++search.nonPlanarApproachCount;
            }
            else
            {
                const double signedResidual =
                    ((candidate.transfer.finalApproachOrigin.x
                            - candidate.poses.front().x) * tangentY
                        - (candidate.transfer.finalApproachOrigin.y
                            - candidate.poses.front().y) * tangentX)
                    / (approachLength * tangentLength);
                const double dot =
                    (approachX * tangentX + approachY * tangentY)
                    / (approachLength * tangentLength);
                if (!std::isfinite(signedResidual)
                    || !std::isfinite(dot))
                {
                    ++search.invalidTangentRejectedCount;
                    return std::nullopt;
                }
                candidate.signedResidual = signedResidual;
                candidate.tangentResidual = std::abs(signedResidual);
                candidate.approachCutDot =
                    std::clamp(dot, -1.0, 1.0);
                candidate.approachCutAngle =
                    std::acos(candidate.approachCutDot);
            }
            candidate.execution = std::move(*execution.value);
            ++search.validEvaluationCount;
            return candidate;
        }

        SearchResult findDynamicEntry
        (
            const PlanningEntity& entity,
            const ProcessUnit& unit,
            int processUnitIndex,
            int previousProcessUnitIndex,
            const std::optional<machining::TubeZone16>& previousOwnerZone,
            const std::optional<machine::ProcessUnitExecutionResult>&
                previousExecution,
            const machine::RotaryMachinePolicy& policy,
            const std::optional<machining::TubeSectionModel>& section,
            double projectionTolerance,
            double rotationSafeMachineZ,
            const OperationContext& context
        )
        {
            SearchResult result;
            const double start = parameterStart(entity);
            const double step = kTwoPi
                / static_cast<double>(kSearchSamples);
            std::vector<EntryCandidate> exactCandidates;
            std::vector<EntryCandidate> fallbackCandidates;
            for (const bool reverse : { false, true })
            {
                if (entity.manualDirectionPreference
                        == process::DirectionPreference::Forward
                    && reverse)
                {
                    continue;
                }
                if (entity.manualDirectionPreference
                        == process::DirectionPreference::Reverse
                    && !reverse)
                {
                    continue;
                }

                std::optional<EntryCandidate> previous;
                double previousParameter = start;
                for (int sample = 0; sample <= kSearchSamples; ++sample)
                {
                    const double parameter =
                        sample == kSearchSamples
                        ? start + kTwoPi
                        : start + step * static_cast<double>(sample);
                    auto current = evaluateCandidate
                    (
                        entity, normalizeParameter(parameter, start),
                        reverse, unit, processUnitIndex,
                        previousProcessUnitIndex, previousOwnerZone,
                        previousExecution, policy, section,
                        projectionTolerance, rotationSafeMachineZ,
                        result, context
                    );
                    if (current.has_value()
                        && current->tangentComparable
                        && current->tangentResidual
                            <= kRootResidualTolerance)
                    {
                        exactCandidates.push_back(*current);
                    }
                    if (previous.has_value() && current.has_value())
                    {
                        ++result.intervalCount;
                        const double midpoint =
                            (previousParameter + parameter) * 0.5;
                        auto fallback = evaluateCandidate
                        (
                            entity, normalizeParameter(midpoint, start),
                            reverse, unit, processUnitIndex,
                            previousProcessUnitIndex, previousOwnerZone,
                            previousExecution, policy, section,
                            projectionTolerance, rotationSafeMachineZ,
                            result, context
                        );
                        if (fallback.has_value())
                            fallbackCandidates.push_back
                                (std::move(*fallback));

                        if (previous->tangentComparable
                            && current->tangentComparable
                            && previous->signedResidual
                            * current->signedResidual < 0.0)
                        {
                            double left = previousParameter;
                            double right = parameter;
                            EntryCandidate leftCandidate = *previous;
                            std::optional<EntryCandidate> root;
                            for (int iteration = 0;
                                iteration < kMaximumRootIterations;
                                ++iteration)
                            {
                                const double middle =
                                    (left + right) * 0.5;
                                auto middleCandidate = evaluateCandidate
                                (
                                    entity,
                                    normalizeParameter(middle, start),
                                    reverse, unit, processUnitIndex,
                                    previousProcessUnitIndex,
                                    previousOwnerZone, previousExecution,
                                    policy, section,
                                    projectionTolerance,
                                    rotationSafeMachineZ, result, context
                                );
                                if (!middleCandidate.has_value())
                                    break;
                                root = *middleCandidate;
                                if (root->tangentResidual
                                        <= kRootResidualTolerance
                                    || right - left <= 1.0e-10)
                                {
                                    break;
                                }
                                if (leftCandidate.signedResidual
                                    * root->signedResidual <= 0.0)
                                {
                                    right = middle;
                                }
                                else
                                {
                                    left = middle;
                                    leftCandidate = *root;
                                }
                            }
                            if (root.has_value()
                                && root->tangentResidual
                                    <= kRootResidualTolerance)
                            {
                                exactCandidates.push_back
                                    (std::move(*root));
                            }
                        }
                    }
                    previous = std::move(current);
                    previousParameter = parameter;
                }
            }

            std::sort(exactCandidates.begin(), exactCandidates.end(),
                [](const EntryCandidate& left,
                    const EntryCandidate& right)
                {
                    if (left.parameter != right.parameter)
                        return left.parameter < right.parameter;
                    return left.reverse < right.reverse;
                });
            exactCandidates.erase(std::unique(exactCandidates.begin(),
                exactCandidates.end(),
                [](const EntryCandidate& left,
                    const EntryCandidate& right)
                {
                    return left.reverse == right.reverse
                        && std::abs(left.parameter - right.parameter)
                            <= 1.0e-7;
                }), exactCandidates.end());
            if (exactCandidates.size() > kMaximumRootCandidates)
                exactCandidates.resize(kMaximumRootCandidates);
            result.rootCandidateCount =
                static_cast<int>(exactCandidates.size());
            result.validTangentCount =
                static_cast<int>(exactCandidates.size());
            result.fallbackCandidateCount =
                static_cast<int>(fallbackCandidates.size());
            if (!exactCandidates.empty())
            {
                result.mode =
                    SingleClosedEntryMode::ExactDynamicTangent;
                result.selected = *std::min_element
                (
                    exactCandidates.begin(), exactCandidates.end(),
                    [](const EntryCandidate& left,
                        const EntryCandidate& right)
                    {
                        return candidateLess(left, right, true);
                    }
                );
                return result;
            }

            if (!fallbackCandidates.empty())
            {
                result.mode = SingleClosedEntryMode::
                    ClosestOwnerZoneParameterFallback;
                result.selected = *std::min_element
                (
                    fallbackCandidates.begin(),
                    fallbackCandidates.end(),
                    [](const EntryCandidate& left,
                        const EntryCandidate& right)
                    {
                        return candidateLess(left, right, false);
                    }
                );
            }
            return result;
        }

        Diagnostic refinementDiagnostic
        (
            const OperationContext& context,
            const ProcessUnit& unit,
            int processUnitIndex,
            int previousProcessUnitIndex,
            const std::optional<machining::TubeZone16>& previousOwnerZone,
            const PlanningEntity& entity,
            const SearchResult& search,
            const machine::RotaryMachinePolicy& policy,
            double projectionTolerance
        )
        {
            const EntryCandidate& selected = *search.selected;
            Diagnostic diagnostic;
            diagnostic.code =
                DiagnosticCode::ProcessPlanningEntrySelectionSummary;
            diagnostic.severity = DiagnosticSeverity::Info;
            diagnostic.component =
                QStringLiteral("SingleClosedEntryRefiner");
            diagnostic.operation = context.operationName;
            diagnostic.stage =
                QStringLiteral("refine-single-closed-entry");
            diagnostic.userMessage =
                QStringLiteral("单图元闭合曲线已按真实动态转移精化起刀点。");
            diagnostic.correlationId = context.correlationId;
            diagnostic.entityId = entity.entityId;
            diagnostic.context =
            {
                { QStringLiteral("singleClosedEntryRefinement"), true },
                { QStringLiteral("processUnitIndex"), processUnitIndex },
                { QStringLiteral("unitKey"), unitKeyText(unit.key) },
                { QStringLiteral("entityId"),
                    QVariant::fromValue<qulonglong>(entity.entityId) },
                { QStringLiteral("sourceKind"),
                    entity.sourceKind
                        == geometry::SourceGeometryKind::Circle
                    ? QStringLiteral("Circle")
                    : QStringLiteral("Ellipse") },
                { QStringLiteral("ownerZone"),
                    unit.ownerZone.has_value()
                    ? machining::tubeZoneName(*unit.ownerZone)
                    : QStringLiteral("None") },
                { QStringLiteral("previousProcessUnitIndex"),
                    previousProcessUnitIndex },
                { QStringLiteral("fromProcessUnit"),
                    previousProcessUnitIndex },
                { QStringLiteral("toProcessUnit"),
                    processUnitIndex },
                { QStringLiteral("previousCutEnd"),
                    poseText(selected.previousCutEnd) },
                { QStringLiteral("transferKind"),
                    static_cast<int>(selected.transfer.kind) },
                { QStringLiteral("searchIntervalCount"),
                    search.intervalCount },
                { QStringLiteral("rootCandidateCount"),
                    search.rootCandidateCount },
                { QStringLiteral("validTangentCount"),
                    search.validTangentCount },
                { QStringLiteral("mode"),
                    search.mode
                        == SingleClosedEntryMode::ExactDynamicTangent
                    ? QStringLiteral("ExactDynamicTangent")
                    : QStringLiteral(
                        "ClosestOwnerZoneParameterFallback") },
                { QStringLiteral("selectedSourceParameter"),
                    selected.parameter },
                { QStringLiteral("selectedReverse"),
                    selected.reverse },
                { QStringLiteral("finalApproachOrigin"),
                    poseText(selected.transfer.finalApproachOrigin) },
                { QStringLiteral("selectedCutStart"),
                    poseText(selected.transfer.cutStart) },
                { QStringLiteral("approachCutDot"),
                    selected.approachCutDot },
                { QStringLiteral("approachCutAngle"),
                    selected.approachCutAngle },
                { QStringLiteral("tangentResidual"),
                    selected.tangentResidual },
                { QStringLiteral("previewSegmentCount"),
                    static_cast<int>
                        (selected.transfer.targets.size()) }
            };
            appendSearchContext(diagnostic.context, search, unit,
                processUnitIndex, previousProcessUnitIndex,
                previousOwnerZone, policy, projectionTolerance, true);
            return diagnostic;
        }

        const Diagnostic* continuityFailure
        (
            const QVector<Diagnostic>& diagnostics
        )
        {
            const auto found = std::find_if
            (
                diagnostics.cbegin(),
                diagnostics.cend(),
                [](const Diagnostic& diagnostic)
                {
                    return diagnostic.code
                        == DiagnosticCode::MachineTrajectoryContinuityFailure;
                }
            );
            return found == diagnostics.cend() ? nullptr : &*found;
        }

        ProcessUnit splitUnitPart
        (
            const ProcessUnit& source,
            std::vector<geometry::EntityId> members
        )
        {
            ProcessUnit result;
            result.orderedMemberEntityIds = std::move(members);
            result.key.memberEntityIds = result.orderedMemberEntityIds;
            std::sort(result.key.memberEntityIds.begin(),
                result.key.memberEntityIds.end());
            result.ownerZone = source.ownerZone;
            result.closed = false;
            return result;
        }

        bool splitProcessUnit
        (
            ProcessPlan& plan,
            std::size_t unitIndex,
            geometry::EntityId nextEntityId
        )
        {
            if (unitIndex >= plan.processUnits.size()) return false;
            const ProcessUnit& source = plan.processUnits[unitIndex];
            const auto split = std::find
            (
                source.orderedMemberEntityIds.cbegin() + 1,
                source.orderedMemberEntityIds.cend(),
                nextEntityId
            );
            if (source.closed || split == source.orderedMemberEntityIds.cend())
                return false;
            for (const ProcessPathFragment& fragment : plan.plannedFragments)
            {
                if (fragment.processUnitIndex
                    == static_cast<int>(unitIndex))
                {
                    return false;
                }
            }

            ProcessPlan updated = plan;
            const std::size_t splitIndex = static_cast<std::size_t>
                (std::distance(source.orderedMemberEntityIds.cbegin(), split));
            ProcessUnit left = splitUnitPart
            (
                source,
                std::vector<geometry::EntityId>
                (
                    source.orderedMemberEntityIds.cbegin(),
                    source.orderedMemberEntityIds.cbegin() + splitIndex
                )
            );
            ProcessUnit right = splitUnitPart
            (
                source,
                std::vector<geometry::EntityId>
                (
                    source.orderedMemberEntityIds.cbegin() + splitIndex,
                    source.orderedMemberEntityIds.cend()
                )
            );
            updated.processUnits[unitIndex] = left;
            updated.processUnits.insert
                (updated.processUnits.begin() + unitIndex + 1U, right);
            updated.processUnitSequence.units[unitIndex] = left.key;
            updated.processUnitSequence.units.insert
            (
                updated.processUnitSequence.units.begin() + unitIndex + 1U,
                right.key
            );

            std::unordered_map<geometry::EntityId, int> unitByEntity;
            for (std::size_t index = 0U;
                index < updated.processUnits.size(); ++index)
            {
                for (const geometry::EntityId entityId :
                    updated.processUnits[index].orderedMemberEntityIds)
                {
                    if (!unitByEntity.emplace
                        (entityId, static_cast<int>(index)).second)
                    {
                        return false;
                    }
                }
            }
            for (std::size_t index = 0U;
                index < updated.assignments.size(); ++index)
            {
                ProcessAssignment& assignment = updated.assignments[index];
                const auto unit = unitByEntity.find(assignment.entityId);
                if (unit == unitByEntity.end()) return false;
                assignment.processOrder = static_cast<int>(index);
                assignment.processUnitIndex = unit->second;
            }
            for (ProcessPathFragment& fragment : updated.plannedFragments)
            {
                const auto unit = unitByEntity.find(fragment.entityId);
                if (unit == unitByEntity.end()) return false;
                fragment.processUnitIndex = unit->second;
            }
            if (!validateProcessUnitStructure(updated)) return false;
            plan = std::move(updated);
            return true;
        }

        OperationReport normalizeProcessUnitContinuity
        (
            ProcessPlan& plan,
            const std::vector<machine::ProcessUnitExecutionSource>& sources,
            const machine::RotaryMachinePolicy& policy,
            const std::optional<machining::TubeSectionModel>& section,
            const OperationContext& context
        )
        {
            OperationReport report;
            for (std::size_t unitIndex = 0U;
                unitIndex < plan.processUnits.size();)
            {
                const ProcessUnit unit = plan.processUnits[unitIndex];
                auto paths = machine::ProcessUnitExecutionResolver::compilePaths
                (
                    unit,
                    static_cast<int>(unitIndex),
                    plan.assignments,
                    plan.plannedFragments,
                    sources,
                    context
                );
                report.mergeDiagnostics(paths);
                if (!paths.succeeded() || !paths.value.has_value())
                {
                    report.status = paths.status;
                    return report;
                }

                auto execution = machine::ProcessUnitExecutionResolver::resolve
                (
                    *paths.value,
                    unit.closed,
                    policy,
                    section,
                    std::nullopt,
                    context
                );
                if (execution.succeeded() && execution.value.has_value())
                {
                    report.mergeDiagnostics(execution);
                    ++unitIndex;
                    continue;
                }

                const Diagnostic* failure =
                    continuityFailure(execution.diagnostics);
                if (failure == nullptr
                    || unit.closed
                    || unit.orderedMemberEntityIds.size() < 2U)
                {
                    report.mergeDiagnostics(execution);
                    report.status = execution.status;
                    return report;
                }

                const geometry::EntityId nextEntityId =
                    failure->context.value(QStringLiteral("nextEntityId"))
                    .toULongLong();
                if (nextEntityId == 0U
                    || !splitProcessUnit(plan, unitIndex, nextEntityId))
                {
                    report.mergeDiagnostics(execution);
                    report.status = OperationStatus::Failed;
                    return report;
                }

                Diagnostic splitDiagnostic = *failure;
                splitDiagnostic.severity = DiagnosticSeverity::Notice;
                splitDiagnostic.component =
                    QStringLiteral("SingleClosedEntryRefiner");
                splitDiagnostic.stage =
                    QStringLiteral("split-discontinuous-process-unit");
                splitDiagnostic.userMessage =
                    QStringLiteral("连续加工单元存在实际执行间隙，已拆分并改用安全转移。");
                splitDiagnostic.technicalDetail =
                    QStringLiteral("Machine-space continuity failure split the open ProcessUnit before publication.");
                splitDiagnostic.context.insert
                    (QStringLiteral("splitApplied"), true);
                splitDiagnostic.context.insert
                    (QStringLiteral("leftProcessUnitIndex"),
                        static_cast<int>(unitIndex));
                splitDiagnostic.context.insert
                    (QStringLiteral("rightProcessUnitIndex"),
                        static_cast<int>(unitIndex + 1U));
                report.addDiagnostic(splitDiagnostic);
            }

            report.status = OperationStatus::Success;
            report.value = std::monostate{};
            return report;
        }
    }

    OperationReport SingleClosedEntryRefiner::refine
    (
        ProcessPlan& plan,
        const ProcessPlanningInput& input,
        const ProcessPlanningPolicy& policy,
        const OperationContext& context
    )
    {
        OperationReport report;
        if (plan.mode != ProcessPlanMode::Rotary4Axis
            || !input.tubeSection.has_value())
        {
            report.status = OperationStatus::Success;
            report.value = std::monostate{};
            return report;
        }

        std::unordered_map<geometry::EntityId, const PlanningEntity*>
            entities;
        std::vector<machine::ProcessUnitExecutionSource> executionSources;
        executionSources.reserve(input.entities.size());
        for (const PlanningEntity& entity : input.entities)
        {
            entities.emplace(entity.entityId, &entity);
            if (!entity.sourceEntity.has_value()) continue;
            executionSources.push_back
            ({
                entity.entityId,
                entity.sourceIndex,
                entity.sourceKind,
                &*entity.sourceEntity,
                entity.executionSamplingPolicy
            });
        }

        const machine::RotaryMachinePolicy rotaryPolicy =
            machinePolicy(policy, *input.tubeSection);
        OperationReport continuity = normalizeProcessUnitContinuity
            (plan, executionSources, rotaryPolicy, input.tubeSection, context);
        report.mergeDiagnostics(continuity.diagnostics);
        if (!continuity.succeeded())
        {
            report.status = continuity.status;
            return report;
        }

        if (policy.sortIntent != ProcessSortIntent::RebuildSequence
            || policy.orderingStrategy
                != ProcessOrderingStrategy::LazyRotation)
        {
            report.status = OperationStatus::Success;
            report.value = std::monostate{};
            return report;
        }

        const std::vector<ProcessUnitKey> originalSequence =
            plan.processUnitSequence.units;
        std::vector<std::optional<machining::TubeZone16>>
            originalOwnerZones;
        originalOwnerZones.reserve(plan.processUnits.size());
        for (const ProcessUnit& unit : plan.processUnits)
            originalOwnerZones.push_back(unit.ownerZone);

        const double maximumRadius =
            machine::RotaryKinematics::sectionMaximumCollisionRadius
            (
                *input.tubeSection,
                rotaryPolicy.tubeCenterY,
                rotaryPolicy.tubeCenterZ
            );
        const double rotationSafeMachineZ =
            machine::RotaryKinematics::rotationSafeMachineZ
            (
                rotaryPolicy.tubeCenterZ,
                maximumRadius,
                policy.rotationSafetyClearance
            );
        const double projectionTolerance = std::max
        (
            policy.numericalEpsilon,
            std::max(input.tubeSection->geometry.yLength,
                input.tubeSection->geometry.zWidth) * 1.0e-6
        );

        std::optional<machine::ProcessUnitExecutionResult>
            previousExecution;
        std::optional<machining::TubeZone16> previousOwnerZone;
        int previousProcessUnitIndex = -1;
        for (std::size_t unitIndex = 0U;
            unitIndex < plan.processUnits.size(); ++unitIndex)
        {
            ProcessUnit& unit = plan.processUnits[unitIndex];
            const bool singleClosed = unit.closed
                && unit.key.memberEntityIds.size() == 1U;
            const auto entityFound = singleClosed
                ? entities.find(unit.key.memberEntityIds.front())
                : entities.end();
            const PlanningEntity* entity =
                entityFound != entities.end()
                ? entityFound->second : nullptr;
            const bool fullEllipse = entity != nullptr
                && entity->sourceEntity.has_value()
                && entity->sourceKind
                    == geometry::SourceGeometryKind::Ellipse
                && std::get_if<geometry::EllipseGeometry>
                    (&entity->sourceEntity->geometry) != nullptr
                && std::get<geometry::EllipseGeometry>
                    (entity->sourceEntity->geometry).fullEllipse;
            const bool eligible = entity != nullptr
                && entity->boundaryRole == BoundaryRole::None
                && !entity->manualStartParameter.has_value()
                && (entity->sourceKind
                        == geometry::SourceGeometryKind::Circle
                    || fullEllipse);

            if (eligible)
            {
                SearchResult search = findDynamicEntry
                (
                    *entity, unit, static_cast<int>(unitIndex),
                    previousProcessUnitIndex, previousOwnerZone,
                    previousExecution, rotaryPolicy, input.tubeSection,
                    projectionTolerance, rotationSafeMachineZ, context
                );
                if (!search.selected.has_value())
                {
                    report.status = OperationStatus::Failed;
                    report.addDiagnostic(failedSearchDiagnostic
                    (
                        context, unit, static_cast<int>(unitIndex),
                        previousProcessUnitIndex, previousOwnerZone,
                        *entity, rotaryPolicy, projectionTolerance, search
                    ));
                    return report;
                }
                auto assignment = std::find_if(plan.assignments.begin(),
                    plan.assignments.end(),
                    [unitIndex](const ProcessAssignment& value)
                    {
                        return value.processUnitIndex
                            == static_cast<int>(unitIndex);
                    });
                if (assignment == plan.assignments.end())
                {
                    report.status = OperationStatus::InternalError;
                    report.addDiagnostic(failureDiagnostic
                    (
                        context,
                        QStringLiteral("单图元闭合曲线缺少加工分配。"),
                        QStringLiteral("Eligible ProcessUnit has no ProcessAssignment."),
                        entity->entityId
                    ));
                    return report;
                }
                assignment->startParameter =
                    search.selected->parameter;
                assignment->reverse = search.selected->reverse;
                assignment->plannedIncomingTransfer =
                    plannedSignature
                    (
                        search.selected->transfer,
                        search.selected->previousCutEnd,
                        search.selected->previousSourceEnd
                    );
                previousExecution =
                    search.selected->execution;
                report.addDiagnostic(refinementDiagnostic
                    (context, unit, static_cast<int>(unitIndex),
                        previousProcessUnitIndex, previousOwnerZone,
                        *entity, search, rotaryPolicy,
                        projectionTolerance));
            }
            else
            {
                auto paths =
                    machine::ProcessUnitExecutionResolver::compilePaths
                (
                    unit,
                    static_cast<int>(unitIndex),
                    plan.assignments,
                    plan.plannedFragments,
                    executionSources,
                    context
                );
                report.mergeDiagnostics(paths);
                if (!paths.succeeded() || !paths.value.has_value())
                {
                    report.status = paths.status;
                    report.addDiagnostic(failureDiagnostic
                    (
                        context,
                        QStringLiteral("无法展开既定加工单元的实际执行路径。"),
                        QStringLiteral("Frozen unit path expansion failed before single-closed entry refinement.")
                    ));
                    return report;
                }
                auto execution =
                    machine::ProcessUnitExecutionResolver::resolve
                (
                    *paths.value,
                    unit.closed,
                    rotaryPolicy,
                    input.tubeSection,
                    previousExecution.has_value()
                    ? std::optional<machine::MachinePose4D>
                        (previousExecution->finalCutPose)
                    : std::nullopt,
                    context
                );
                report.mergeDiagnostics(execution);
                if (!execution.succeeded()
                    || !execution.value.has_value())
                {
                    report.status = execution.status;
                    report.addDiagnostic(failureDiagnostic
                    (
                        context,
                        QStringLiteral("无法推演既定加工单元的真实结束位置。"),
                        QStringLiteral("Frozen unit execution failed before single-closed entry refinement.")
                    ));
                    return report;
                }
                previousExecution = std::move(*execution.value);
            }
            previousOwnerZone = unit.ownerZone;
            previousProcessUnitIndex = static_cast<int>(unitIndex);
        }

        if (plan.processUnitSequence.units.size()
                != originalSequence.size()
            || plan.processUnits.size()
                != originalOwnerZones.size())
        {
            report.status = OperationStatus::InternalError;
            report.addDiagnostic(failureDiagnostic
            (
                context,
                QStringLiteral("入口精化改变了加工单元数量。"),
                QStringLiteral("ProcessUnit sequence size changed during entry refinement.")
            ));
            return report;
        }
        for (std::size_t index = 0U;
            index < originalSequence.size(); ++index)
        {
            if (!sameUnitKey(plan.processUnitSequence.units[index],
                    originalSequence[index])
                || plan.processUnits[index].ownerZone
                    != originalOwnerZones[index])
            {
                report.status = OperationStatus::InternalError;
                report.addDiagnostic(failureDiagnostic
                (
                    context,
                    QStringLiteral("入口精化改变了加工顺序或区位归属。"),
                    QStringLiteral("ProcessUnitKey sequence or ownerZone changed during entry refinement.")
                ));
                return report;
            }
        }

        report.status = report.diagnostics.isEmpty()
            ? OperationStatus::Success
            : OperationStatus::PartialSuccess;
        report.value = std::monostate{};
        return report;
    }
}
