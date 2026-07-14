#include "core/nc/NcProgramBuilder.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>

namespace
{
    Diagnostic builderDiagnostic
    (
        DiagnosticCode code,
        const QString& message,
        const OperationContext& context,
        cadcam::geometry::EntityId entityId = 0,
        const QVariantMap& values = {}
    )
    {
        Diagnostic diagnostic;
        diagnostic.code = code;
        diagnostic.severity = DiagnosticSeverity::Error;
        diagnostic.component = QStringLiteral("NcProgramBuilder");
        diagnostic.operation = context.operationName;
        diagnostic.stage = QStringLiteral("build-rotary-program");
        diagnostic.userMessage = message;
        diagnostic.correlationId = context.correlationId;
        diagnostic.context = values;
        if (entityId != 0) diagnostic.entityId = entityId;
        return diagnostic;
    }

    std::string debugValue(double value)
    {
        return QString::number(value, 'f', 6).toStdString();
    }

    void appendRotaryComments
    (
        cadcam::nc::NcProgram& program,
        const cadcam::machine::RotaryTrajectoryContext& rotary,
        const std::optional<cadcam::machining::TubeSectionModel>& tubeSection
    )
    {
        const auto append = [&program](const std::string& text)
        {
            program.leadingComments.push_back({ text });
        };
        append("TUBE CENTER Y: " + debugValue(rotary.tubeCenterY));
        append("TUBE CENTER Z: " + debugValue(rotary.tubeCenterZ));
        append("ROTARY AXIS Y: " + debugValue(rotary.rotaryAxisY));
        append("ROTARY AXIS Z: " + debugValue(rotary.rotaryAxisZ));
        append("MAX COLLISION RADIUS: " + debugValue(rotary.maximumCollisionRadius));
        append("FINAL SAFE MACHINE Z: " + debugValue(rotary.safeMachineZ));
        if (!rotary.hasSectionBounds) return;

        append("SQUARE TUBE SECTION Y: " + debugValue(rotary.sectionMinimumY)
            + " -> " + debugValue(rotary.sectionMaximumY));
        append("SQUARE TUBE SECTION Z: " + debugValue(rotary.sectionMinimumZ)
            + " -> " + debugValue(rotary.sectionMaximumZ));
        if (!tubeSection.has_value()) return;

        for (const auto& corner : tubeSection->corners)
        {
            const char* name = corner.zDirection > 0
                ? (corner.yDirection > 0 ? "TOP RIGHT" : "TOP LEFT")
                : (corner.yDirection > 0 ? "BOTTOM RIGHT" : "BOTTOM LEFT");
            append(std::string("SQUARE TUBE ") + name + " CORNER CENTER Y/Z: "
                + debugValue(corner.center.x) + ", " + debugValue(corner.center.y));
        }
    }

    bool mapMove
    (
        const cadcam::machine::MachineMove& source,
        cadcam::nc::NcMotion& target
    )
    {
        using cadcam::machine::MachineMoveKind;
        using cadcam::nc::NcMotionKind;
        using cadcam::nc::NcSourceMoveKind;
        switch (source.kind)
        {
        case MachineMoveKind::Rapid:
            target.kind = NcMotionKind::Rapid;
            target.sourceKind = NcSourceMoveKind::Rapid;
            break;
        case MachineMoveKind::Cutting:
            target.kind = NcMotionKind::Linear;
            target.sourceKind = NcSourceMoveKind::Cutting;
            break;
        case MachineMoveKind::CuttingConnection:
            target.kind = NcMotionKind::Linear;
            target.sourceKind = NcSourceMoveKind::CuttingConnection;
            break;
        case MachineMoveKind::Overcut:
            target.kind = NcMotionKind::Linear;
            target.sourceKind = NcSourceMoveKind::Overcut;
            break;
        default:
            return false;
        }
        target.axes.x = source.target.x;
        target.axes.y = source.target.y;
        target.axes.z = source.target.z;
        target.axes.a = source.target.aDegrees;
        target.entityId = source.entityId;
        target.processGroupId = source.processGroupId;
        return true;
    }

    bool finiteMove(const cadcam::nc::NcMotion& motion)
    {
        return motion.axes.x.has_value() && std::isfinite(*motion.axes.x)
            && motion.axes.y.has_value() && std::isfinite(*motion.axes.y)
            && motion.axes.z.has_value() && std::isfinite(*motion.axes.z)
            && motion.axes.a.has_value() && std::isfinite(*motion.axes.a);
    }
}

OperationResult<cadcam::nc::NcProgram> cadcam::nc::NcProgramBuilder::buildRotary
(
    const machine::MachineTrajectory& trajectory,
    const std::vector<NcEntityMetadata>& metadata,
    const OperationContext& context,
    const std::optional<machining::TubeSectionModel>& tubeSection
)
{
    OperationResult<NcProgram> result;
    if (trajectory.contentRevision == 0 || trajectory.entities.empty())
    {
        result.status = OperationStatus::InvalidInput;
        result.addDiagnostic(builderDiagnostic(DiagnosticCode::NcProgramInputInvalid,
            QStringLiteral("四轴轨迹为空或文档版本无效，无法构造 NC 程序。"), context, 0,
            { { QStringLiteral("contentRevision"), QVariant::fromValue<qulonglong>(trajectory.contentRevision) } }));
        return result;
    }

    std::map<geometry::EntityId, const NcEntityMetadata*> metadataById;
    for (const auto& entry : metadata)
    {
        if (entry.entityId == 0 || !metadataById.emplace(entry.entityId, &entry).second)
        {
            result.status = OperationStatus::InvalidInput;
            result.addDiagnostic(builderDiagnostic(DiagnosticCode::NcProgramDuplicateEntity,
                QStringLiteral("NC 图元元数据包含零编号或重复编号。"), context, entry.entityId));
            return result;
        }
    }

    std::vector<const machine::EntityTrajectory*> ordered;
    ordered.reserve(trajectory.entities.size());
    std::set<geometry::EntityId> trajectoryIds;
    for (const auto& entity : trajectory.entities)
    {
        if (entity.entityId == 0 || !trajectoryIds.insert(entity.entityId).second)
        {
            result.status = OperationStatus::InvalidInput;
            result.addDiagnostic(builderDiagnostic(DiagnosticCode::NcProgramDuplicateEntity,
                QStringLiteral("四轴轨迹包含零编号或重复图元。"), context, entity.entityId));
            return result;
        }
        ordered.push_back(&entity);
    }
    std::sort(ordered.begin(), ordered.end(), [](const auto* left, const auto* right)
    {
        return left->processOrder < right->processOrder;
    });

    NcProgram program;
    program.contentRevision = trajectory.contentRevision;
    program.processStateRevision = trajectory.processStateRevision;
    program.mode = NcProgramMode::Rotary4Axis;
    appendRotaryComments(program, trajectory.rotaryContext, tubeSection);
    program.entities.reserve(ordered.size());

    for (std::size_t index = 0; index < ordered.size(); ++index)
    {
        const machine::EntityTrajectory& source = *ordered[index];
        const auto metadataIt = metadataById.find(source.entityId);
        if (metadataIt == metadataById.end())
        {
            result.status = OperationStatus::Failed;
            result.addDiagnostic(builderDiagnostic(DiagnosticCode::NcProgramMetadataMissing,
                QStringLiteral("四轴轨迹图元缺少 NC 元数据。"), context, source.entityId,
                { { QStringLiteral("processOrder"), source.processOrder } }));
            return result;
        }
        const NcEntityMetadata& entityMetadata = *metadataIt->second;
        if (source.processOrder != static_cast<int>(index)
            || entityMetadata.processOrder != source.processOrder
            || entityMetadata.processGroupId != source.processGroupId
            || entityMetadata.sourceIndex != source.sourceIndex
            || entityMetadata.sourceKind != source.sourceKind)
        {
            result.status = OperationStatus::Conflict;
            result.addDiagnostic(builderDiagnostic(DiagnosticCode::NcProgramInvariantViolation,
                QStringLiteral("四轴轨迹与图元元数据不一致。"), context, source.entityId,
                {
                    { QStringLiteral("sourceIndex"), static_cast<qulonglong>(source.sourceIndex) },
                    { QStringLiteral("processOrder"), source.processOrder },
                    { QStringLiteral("processGroupId"), source.processGroupId }
                }));
            return result;
        }

        NcEntityBlock block;
        block.metadata = entityMetadata;
        const auto appendMoves = [&](const std::vector<machine::MachineMove>& moves) -> bool
        {
            for (std::size_t motionIndex = 0; motionIndex < moves.size(); ++motionIndex)
            {
                NcMotion motion;
                if (!mapMove(moves[motionIndex], motion)) return false;
                if (!finiteMove(motion) || motion.entityId != source.entityId
                    || motion.processGroupId != source.processGroupId) return false;
                block.motions.push_back(std::move(motion));
            }
            return true;
        };
        if (!appendMoves(source.approachMoves)
            || !appendMoves(source.cuttingMoves)
            || !appendMoves(source.overcutMoves))
        {
            result.status = OperationStatus::Failed;
            result.addDiagnostic(builderDiagnostic(DiagnosticCode::NcProgramUnsupportedMotion,
                QStringLiteral("四轴轨迹包含不支持或无效的运动。"), context, source.entityId,
                { { QStringLiteral("processOrder"), source.processOrder } }));
            return result;
        }
        if (block.motions.empty())
        {
            result.status = OperationStatus::Failed;
            result.addDiagnostic(builderDiagnostic(DiagnosticCode::NcProgramEntityMissing,
                QStringLiteral("NC 图元块没有任何运动。"), context, source.entityId));
            return result;
        }
        program.entities.push_back(std::move(block));
    }

    if (metadataById.size() != program.entities.size())
    {
        result.status = OperationStatus::Conflict;
        result.addDiagnostic(builderDiagnostic(DiagnosticCode::NcProgramInvariantViolation,
            QStringLiteral("存在未被四轴轨迹使用的 NC 图元元数据。"), context, 0,
            {
                { QStringLiteral("expectedCount"), static_cast<qulonglong>(program.entities.size()) },
                { QStringLiteral("actualCount"), static_cast<qulonglong>(metadataById.size()) }
            }));
        return result;
    }

    result.status = OperationStatus::Success;
    result.value = std::move(program);
    return result;
}
