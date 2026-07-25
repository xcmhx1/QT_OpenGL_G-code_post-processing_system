#include "core/machine/PlanarTrajectoryBuilder.h"

#include <cmath>

namespace cadcam::machine
{
    namespace
    {
        bool finite(const geometry::Vector3d& point)
        {
            return std::isfinite(point.x)
                && std::isfinite(point.y)
                && std::isfinite(point.z);
        }

        double distance
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

        MachinePose4D pose(const geometry::Vector3d& point)
        {
            return { point.x, point.y, point.z, 0.0 };
        }

        Diagnostic invalidInputDiagnostic(const OperationContext& context)
        {
            Diagnostic diagnostic;
            diagnostic.code = DiagnosticCode::MachineTrajectoryInputInvalid;
            diagnostic.severity = DiagnosticSeverity::Error;
            diagnostic.component = QStringLiteral("PlanarTrajectoryBuilder");
            diagnostic.operation = context.operationName;
            diagnostic.stage = QStringLiteral("build-planar-clearance");
            diagnostic.userMessage = QStringLiteral("三轴安全移动轨迹输入无效。");
            diagnostic.correlationId = context.correlationId;
            return diagnostic;
        }
    }

    OperationResult<PlanarTrajectory> PlanarTrajectoryBuilder::build
    (
        const std::vector<PlanarTrajectoryEntityInput>& entities,
        const ToolClearancePolicy& clearance,
        const OperationContext& context
    )
    {
        OperationResult<PlanarTrajectory> result;
        if (entities.empty()
            || !std::isfinite(clearance.retractClearance)
            || clearance.retractClearance <= 0.0
            || !std::isfinite(clearance.approachClearance)
            || clearance.approachClearance < 0.0
            || clearance.retractClearance < clearance.approachClearance)
        {
            result.status = OperationStatus::InvalidInput;
            result.addDiagnostic(invalidInputDiagnostic(context));
            return result;
        }

        PlanarTrajectory trajectory;
        trajectory.entities.reserve(entities.size());
        std::optional<geometry::Vector3d> previousCutEnd;
        std::optional<int> previousProcessUnitIndex;
        for (const PlanarTrajectoryEntityInput& entity : entities)
        {
            if (entity.entityId == 0 || entity.processUnitIndex < 0
                || !finite(entity.cutStart) || !finite(entity.cutEnd))
            {
                result.status = OperationStatus::InvalidInput;
                Diagnostic diagnostic = invalidInputDiagnostic(context);
                diagnostic.entityId = entity.entityId;
                result.addDiagnostic(diagnostic);
                return result;
            }

            PlanarEntityTrajectory planned;
            planned.entityId = entity.entityId;
            const bool unitChanged = !previousProcessUnitIndex.has_value()
                || *previousProcessUnitIndex != entity.processUnitIndex;
            const auto append = [&planned](const geometry::Vector3d& point)
            {
                if (!planned.approachPoses.empty())
                {
                    const MachinePose4D& previous =
                        planned.approachPoses.back();
                    if (distance
                        (
                            { previous.x, previous.y, previous.z },
                            point
                        ) <= 1.0e-12)
                    {
                        return;
                    }
                }
                planned.approachPoses.push_back(pose(point));
            };

            if (unitChanged)
            {
                if (previousCutEnd.has_value())
                {
                    geometry::Vector3d departure = *previousCutEnd;
                    departure.z += clearance.retractClearance;
                    append(departure);
                }
                geometry::Vector3d transfer = entity.cutStart;
                transfer.z += clearance.retractClearance;
                append(transfer);
                geometry::Vector3d approach = entity.cutStart;
                approach.z += clearance.approachClearance;
                append(approach);
            }
            append(entity.cutStart);

            trajectory.entities.push_back(std::move(planned));
            previousCutEnd = entity.cutEnd;
            previousProcessUnitIndex = entity.processUnitIndex;
        }

        result.status = OperationStatus::Success;
        result.value = std::move(trajectory);
        return result;
    }
}
