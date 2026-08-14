#include "core/nc/NcProgramBuilder.h"
#include "core/diagnostics/SummaryLog.h"

#include <QDebug>

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

    bool cuttingMotion(const cadcam::nc::NcMotion& motion)
    {
        using cadcam::nc::NcSourceMoveKind;
        return motion.sourceKind == NcSourceMoveKind::Cutting
            || motion.sourceKind == NcSourceMoveKind::CuttingConnection
            || motion.sourceKind == NcSourceMoveKind::Overcut;
    }

    struct CuttingControlSummary
    {
        int processUnitIndex = -1;
        int firstProcessOrder = -1;
        int lastProcessOrder = -1;
        int memberBlockCount = 0;
        int fragmentBlockCount = 0;
        int cuttingMotionCount = 0;
        int connectionMotionCount = 0;
        int overcutMotionCount = 0;
        int enableCount = 0;
        int disableCount = 0;
    };
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
    std::set<geometry::EntityId> wholeTrajectoryIds;
    std::set<geometry::EntityId> fragmentedTrajectoryIds;
    std::set<std::pair<geometry::EntityId, int>> trajectoryFragments;
    for (const auto& entity : trajectory.entities)
    {
        bool identityValid = entity.entityId != 0;
        if (identityValid && entity.fragmentOrder >= 0)
        {
            identityValid = wholeTrajectoryIds.count(entity.entityId) == 0U
                && trajectoryFragments.emplace
                    (entity.entityId, entity.fragmentOrder).second;
            fragmentedTrajectoryIds.insert(entity.entityId);
        }
        else if (identityValid)
        {
            identityValid =
                fragmentedTrajectoryIds.count(entity.entityId) == 0U
                && wholeTrajectoryIds.insert(entity.entityId).second;
        }
        if (!identityValid)
        {
            result.status = OperationStatus::InvalidInput;
            result.addDiagnostic(builderDiagnostic(DiagnosticCode::NcProgramDuplicateEntity,
                QStringLiteral("四轴轨迹包含零编号、重复完整图元或重复片段。"),
                context, entity.entityId));
            return result;
        }
        ordered.push_back(&entity);
    }
    std::sort(ordered.begin(), ordered.end(), [](const auto* left, const auto* right)
    {
        return left->processOrder < right->processOrder;
    });
    int expectedProcessUnitIndex = 0;
    int currentProcessUnitIndex = -1;
    std::set<int> completedProcessUnits;
    for (std::size_t index = 0; index < ordered.size(); ++index)
    {
        const machine::EntityTrajectory& source = *ordered[index];
        if (source.processUnitIndex < 0)
        {
            result.status = OperationStatus::InvalidInput;
            result.addDiagnostic(builderDiagnostic
            (
                DiagnosticCode::NcProgramCuttingControlInvalid,
                QStringLiteral("四轴轨迹缺少有效的加工单元编号。"),
                context,
                source.entityId,
                {
                    { QStringLiteral("processOrder"), source.processOrder },
                    { QStringLiteral("processUnitIndex"),
                        source.processUnitIndex }
                }
            ));
            return result;
        }
        if (source.processUnitIndex == currentProcessUnitIndex) continue;
        if (currentProcessUnitIndex >= 0)
            completedProcessUnits.insert(currentProcessUnitIndex);
        if (completedProcessUnits.count(source.processUnitIndex) != 0U
            || source.processUnitIndex != expectedProcessUnitIndex)
        {
            result.status = OperationStatus::Conflict;
            result.addDiagnostic(builderDiagnostic
            (
                DiagnosticCode::NcProgramCuttingControlInvalid,
                QStringLiteral("四轴轨迹中的加工单元不连续或顺序无效。"),
                context,
                source.entityId,
                {
                    { QStringLiteral("processOrder"), source.processOrder },
                    { QStringLiteral("processUnitIndex"),
                        source.processUnitIndex },
                    { QStringLiteral("expectedProcessUnitIndex"),
                        expectedProcessUnitIndex }
                }
            ));
            return result;
        }
        currentProcessUnitIndex = source.processUnitIndex;
        ++expectedProcessUnitIndex;
    }

    NcProgram program;
    program.contentRevision = trajectory.contentRevision;
    program.processStateRevision = trajectory.processStateRevision;
    program.mode = NcProgramMode::Rotary4Axis;
    appendRotaryComments(program, trajectory.rotaryContext, tubeSection);
    program.entities.reserve(ordered.size());
    std::vector<CuttingControlSummary> cuttingControlSummaries
        (static_cast<std::size_t>(expectedProcessUnitIndex));

    for (std::size_t index = 0; index < ordered.size(); ++index)
    {
        const machine::EntityTrajectory& source = *ordered[index];
        const bool firstInUnit = index == 0U
            || ordered[index - 1U]->processUnitIndex
                != source.processUnitIndex;
        const bool lastInUnit = index + 1U == ordered.size()
            || ordered[index + 1U]->processUnitIndex
                != source.processUnitIndex;
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
            || source.sourceProcessOrder < 0
            || entityMetadata.processOrder != source.sourceProcessOrder
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
        block.processUnitIndex = source.processUnitIndex;
        block.beforeCutting = firstInUnit
            ? NcCuttingControl::Enable : NcCuttingControl::None;
        block.afterCutting = lastInUnit
            ? NcCuttingControl::Disable : NcCuttingControl::None;
        block.metadata = entityMetadata;
        block.metadata.processOrder = source.processOrder;
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

        CuttingControlSummary& summary = cuttingControlSummaries
            [static_cast<std::size_t>(source.processUnitIndex)];
        if (firstInUnit)
        {
            summary.processUnitIndex = source.processUnitIndex;
            summary.firstProcessOrder = source.processOrder;
        }
        summary.lastProcessOrder = source.processOrder;
        ++summary.memberBlockCount;
        if (source.fragmentOrder >= 0) ++summary.fragmentBlockCount;
        if (block.beforeCutting == NcCuttingControl::Enable)
            ++summary.enableCount;
        if (block.afterCutting == NcCuttingControl::Disable)
            ++summary.disableCount;

        bool cuttingStarted = false;
        int rapidCount = 0;
        for (const NcMotion& motion : block.motions)
        {
            if (motion.sourceKind == NcSourceMoveKind::Rapid)
            {
                ++rapidCount;
                if (!firstInUnit || cuttingStarted)
                {
                    result.status = OperationStatus::Conflict;
                    result.addDiagnostic(builderDiagnostic
                    (
                        DiagnosticCode::NcProgramCuttingControlInvalid,
                        QStringLiteral("加工单元内部或切削开始后出现了快速运动。"),
                        context,
                        source.entityId,
                        {
                            { QStringLiteral("processOrder"),
                                source.processOrder },
                            { QStringLiteral("processUnitIndex"),
                                source.processUnitIndex },
                            { QStringLiteral("firstInUnit"), firstInUnit }
                        }
                    ));
                    return result;
                }
                continue;
            }
            if (!cuttingMotion(motion))
            {
                result.status = OperationStatus::NotSupported;
                result.addDiagnostic(builderDiagnostic
                (
                    DiagnosticCode::NcProgramCuttingControlInvalid,
                    QStringLiteral("加工单元包含未知的切削运动类型。"),
                    context,
                    source.entityId
                ));
                return result;
            }
            cuttingStarted = true;
            if (motion.sourceKind == NcSourceMoveKind::Cutting)
                ++summary.cuttingMotionCount;
            else if (motion.sourceKind
                == NcSourceMoveKind::CuttingConnection)
            {
                ++summary.connectionMotionCount;
            }
            else if (motion.sourceKind == NcSourceMoveKind::Overcut)
                ++summary.overcutMotionCount;
        }
        if (!cuttingStarted
            || (firstInUnit && source.processUnitIndex > 0
                && rapidCount == 0))
        {
            result.status = OperationStatus::Conflict;
            result.addDiagnostic(builderDiagnostic
            (
                DiagnosticCode::NcProgramCuttingControlInvalid,
                !cuttingStarted
                    ? QStringLiteral("加工单元图元块不包含切削运动。")
                    : QStringLiteral("相邻加工单元之间缺少关闭状态下的快速转移。"),
                context,
                source.entityId,
                {
                    { QStringLiteral("processOrder"), source.processOrder },
                    { QStringLiteral("processUnitIndex"),
                        source.processUnitIndex },
                    { QStringLiteral("rapidCount"), rapidCount }
                }
            ));
            return result;
        }
        program.entities.push_back(std::move(block));
    }

    for (const CuttingControlSummary& summary : cuttingControlSummaries)
    {
        const int cuttingLikeCount = summary.cuttingMotionCount
            + summary.connectionMotionCount + summary.overcutMotionCount;
        if (summary.processUnitIndex < 0
            || summary.enableCount != 1 || summary.disableCount != 1
            || cuttingLikeCount <= 0)
        {
            result.status = OperationStatus::Conflict;
            result.addDiagnostic(builderDiagnostic
            (
                DiagnosticCode::NcProgramCuttingControlInvalid,
                QStringLiteral("加工单元启停边界不完整。"),
                context,
                0,
                {
                    { QStringLiteral("processUnitIndex"),
                        summary.processUnitIndex },
                    { QStringLiteral("enableCount"), summary.enableCount },
                    { QStringLiteral("disableCount"),
                        summary.disableCount },
                    { QStringLiteral("cuttingMotionCount"),
                        cuttingLikeCount }
                }
            ));
            return result;
        }
        cadcam::core::emitSummaryLog
        (
            QStringLiteral("NcProgram"),
            QStringLiteral("CuttingControl"),
            QStringLiteral("processUnitIndex=%1 "
                "firstProcessOrder=%2 lastProcessOrder=%3 "
                "memberBlockCount=%4 fragmentBlockCount=%5 "
                "cuttingMotionCount=%6 connectionMotionCount=%7 "
                "overcutMotionCount=%8 enableCount=%9 disableCount=%10")
            .arg(summary.processUnitIndex)
            .arg(summary.firstProcessOrder)
            .arg(summary.lastProcessOrder)
            .arg(summary.memberBlockCount)
            .arg(summary.fragmentBlockCount)
            .arg(summary.cuttingMotionCount)
            .arg(summary.connectionMotionCount)
            .arg(summary.overcutMotionCount)
            .arg(summary.enableCount)
            .arg(summary.disableCount));
    }

    std::set<geometry::EntityId> usedEntityIds;
    for (const auto& entity : trajectory.entities)
        usedEntityIds.insert(entity.entityId);
    if (metadataById.size() != usedEntityIds.size())
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
