#include "infrastructure/nc/GCodePostProcessor.h"

#include "infrastructure/config/GProfile.h"

#include <QDebug>
#include <QRegularExpression>
#include <QStringList>

#include <cmath>
#include <set>

namespace cadcam::infrastructure::nc
{
    namespace
    {
        Diagnostic postDiagnostic
        (
            DiagnosticCode code,
            const QString& message,
            const OperationContext& context,
            geometry::EntityId entityId = 0,
            const QVariantMap& values = {}
        )
        {
            Diagnostic diagnostic;
            diagnostic.code = code;
            diagnostic.severity = DiagnosticSeverity::Error;
            diagnostic.component = QStringLiteral("GCodePostProcessor");
            diagnostic.operation = context.operationName;
            diagnostic.stage = QStringLiteral("render-gcode");
            diagnostic.userMessage = message;
            diagnostic.correlationId = context.correlationId;
            diagnostic.context = values;
            if (entityId != 0) diagnostic.entityId = entityId;
            return diagnostic;
        }

        void appendTextBlock(QStringList& output, const QString& text)
        {
            QString normalized = text;
            normalized.replace(QStringLiteral("\r\n"), QStringLiteral("\n"));
            normalized.replace(QLatin1Char('\r'), QLatin1Char('\n'));
            const QStringList lines = normalized.split(QLatin1Char('\n'), Qt::KeepEmptyParts);
            for (const QString& line : lines)
            {
                if (!line.trimmed().isEmpty()) output.push_back(line);
            }
        }

        bool containsCuttingControlCode(const QString& text)
        {
            static const QRegularExpression code
            (
                QStringLiteral("(^|\\s)M0?(3|5)(?=\\s|$)"),
                QRegularExpression::CaseInsensitiveOption
            );
            QString normalized = text;
            normalized.replace(QStringLiteral("\r\n"), QStringLiteral("\n"));
            normalized.replace(QLatin1Char('\r'), QLatin1Char('\n'));
            for (const QString& line : normalized.split(QLatin1Char('\n')))
            {
                if (code.match(line).hasMatch()) return true;
            }
            return false;
        }

        QString formatMotion
        (
            const cadcam::nc::NcMotion& motion,
            const GCodePostProcessorProfile& profile,
            bool preserveLegacyOutputQuantization
        )
        {
            QString code;
            switch (motion.kind)
            {
            case cadcam::nc::NcMotionKind::Rapid:
                code = profile.rapidCode;
                break;
            case cadcam::nc::NcMotionKind::Linear:
                code = profile.linearCode;
                break;
            case cadcam::nc::NcMotionKind::CircularClockwise:
                code = QStringLiteral("G02");
                break;
            case cadcam::nc::NcMotionKind::CircularCounterclockwise:
                code = QStringLiteral("G03");
                break;
            }
            QStringList words { code };
            const auto append = [&](const QChar name, const std::optional<double>& value, int precision)
            {
                if (value.has_value())
                {
                    const double outputValue = preserveLegacyOutputQuantization
                        ? static_cast<double>(static_cast<float>(*value)) : *value;
                    words.push_back(name + QString::number(outputValue, 'f', precision));
                }
            };
            append(QLatin1Char('X'), motion.axes.x, profile.coordinatePrecision);
            append(QLatin1Char('Y'), motion.axes.y, profile.coordinatePrecision);
            append(QLatin1Char('Z'), motion.axes.z, profile.coordinatePrecision);
            append(QLatin1Char('A'), motion.axes.a, profile.anglePrecision);
            append(QLatin1Char('I'), motion.axes.i, profile.coordinatePrecision);
            append(QLatin1Char('J'), motion.axes.j, profile.coordinatePrecision);
            append(QLatin1Char('K'), motion.axes.k, profile.coordinatePrecision);
            append(QLatin1Char('R'), motion.axes.r, profile.coordinatePrecision);
            return words.join(QLatin1Char(' '));
        }

        bool finiteAxes(const cadcam::nc::NcAxisWords& axes)
        {
            const auto valid = [](const std::optional<double>& value)
            {
                return !value.has_value() || std::isfinite(*value);
            };
            return valid(axes.x) && valid(axes.y) && valid(axes.z) && valid(axes.a)
                && valid(axes.i) && valid(axes.j) && valid(axes.k) && valid(axes.r);
        }

        bool validMotion
        (
            const cadcam::nc::NcMotion& motion,
            cadcam::nc::NcProgramMode mode,
            bool rapid
        )
        {
            if (!finiteAxes(motion.axes)) return false;
            if (mode == cadcam::nc::NcProgramMode::Rotary4Axis)
            {
                return (rapid ? motion.kind == cadcam::nc::NcMotionKind::Rapid
                              : motion.kind == cadcam::nc::NcMotionKind::Linear)
                    && motion.axes.x.has_value() && motion.axes.y.has_value()
                    && motion.axes.z.has_value() && motion.axes.a.has_value()
                    && !motion.axes.i.has_value() && !motion.axes.j.has_value()
                    && !motion.axes.k.has_value() && !motion.axes.r.has_value();
            }
            if (rapid)
            {
                return motion.kind == cadcam::nc::NcMotionKind::Rapid
                    && motion.axes.x.has_value() && motion.axes.y.has_value()
                    && !motion.axes.a.has_value() && !motion.axes.i.has_value()
                    && !motion.axes.j.has_value() && !motion.axes.k.has_value()
                    && !motion.axes.r.has_value();
            }
            if (motion.kind == cadcam::nc::NcMotionKind::Linear)
                return (motion.axes.x.has_value() || motion.axes.y.has_value()
                        || motion.axes.z.has_value())
                    && !motion.axes.a.has_value() && !motion.axes.i.has_value()
                    && !motion.axes.j.has_value() && !motion.axes.k.has_value()
                    && !motion.axes.r.has_value();
            if (motion.kind == cadcam::nc::NcMotionKind::CircularClockwise
                || motion.kind == cadcam::nc::NcMotionKind::CircularCounterclockwise)
            {
                if (motion.axes.a.has_value() || motion.axes.r.has_value()) return false;
                if (motion.plane == cadcam::nc::NcPlane::XY)
                    return motion.axes.x.has_value() && motion.axes.y.has_value()
                        && motion.axes.i.has_value() && motion.axes.j.has_value();
                if (motion.plane == cadcam::nc::NcPlane::ZX)
                    return motion.axes.x.has_value() && motion.axes.z.has_value()
                        && motion.axes.i.has_value() && motion.axes.k.has_value();
                return motion.axes.y.has_value() && motion.axes.z.has_value()
                    && motion.axes.j.has_value() && motion.axes.k.has_value();
            }
            return false;
        }

        GCodeBlock toBlock(const GProfileCodeBlock& source)
        {
            return { source.header, source.footer };
        }
    }

    GCodePostProcessorProfile makeGCodePostProcessorProfile(const GProfile& profile)
    {
        GCodePostProcessorProfile result;
        result.programHeader = profile.fileCode().header;
        result.programFooter = profile.fileCode().footer;
        result.processUnitBlock = toBlock(profile.processUnitCode());
        for (auto it = profile.entityTypeCodes().cbegin(); it != profile.entityTypeCodes().cend(); ++it)
            result.entityTypeBlocks.insert(it.key(), toBlock(it.value()));
        if (!result.entityTypeBlocks.contains(QStringLiteral("SPLINE")))
            result.entityTypeBlocks.insert(QStringLiteral("SPLINE"),
                toBlock(profile.entityTypeCode(QStringLiteral("SPLINE"))));
        for (auto it = profile.layerCodes().cbegin(); it != profile.layerCodes().cend(); ++it)
            result.layerBlocks.insert(it.key(), toBlock(it.value()));
        for (auto it = profile.entityColorCodes().cbegin(); it != profile.entityColorCodes().cend(); ++it)
            result.colorBlocks.insert(it.key(), toBlock(it.value()));
        return result;
    }

    OperationResult<QString> GCodePostProcessor::render
    (
        const cadcam::nc::NcProgram& program,
        const GCodePostProcessorProfile& profile,
        const OperationContext& context
    )
    {
        OperationResult<QString> result;
        if (program.contentRevision == 0 || program.entities.empty())
        {
            result.status = OperationStatus::InvalidInput;
            result.addDiagnostic(postDiagnostic(DiagnosticCode::GCodeRenderingFailed,
                QStringLiteral("后处理器只接受有效的 NC 程序。"), context));
            return result;
        }
        if (profile.coordinatePrecision < 0 || profile.coordinatePrecision > 15
            || profile.anglePrecision < 0 || profile.anglePrecision > 15
            || profile.rapidCode.trimmed().isEmpty() || profile.linearCode.trimmed().isEmpty())
        {
            result.status = OperationStatus::InvalidInput;
            result.addDiagnostic(postDiagnostic(DiagnosticCode::GCodeProfileInvalid,
                QStringLiteral("G 代码后处理配置无效。"), context, 0,
                {
                    { QStringLiteral("coordinatePrecision"), profile.coordinatePrecision },
                    { QStringLiteral("anglePrecision"), profile.anglePrecision }
                }));
            return result;
        }
        const bool rotary =
            program.mode == cadcam::nc::NcProgramMode::Rotary4Axis;
        if (rotary
            && (profile.processUnitBlock.header.trimmed().isEmpty()
                || profile.processUnitBlock.footer.trimmed().isEmpty()))
        {
            result.status = OperationStatus::InvalidInput;
            result.addDiagnostic(postDiagnostic
            (
                DiagnosticCode::GCodeProfileInvalid,
                QStringLiteral("四轴加工单元开始或结束代码为空。"),
                context
            ));
            return result;
        }
        if (rotary)
        {
            const auto invalidRule = [](const auto& blocks)
            {
                for (auto it = blocks.cbegin(); it != blocks.cend(); ++it)
                {
                    if (containsCuttingControlCode(it.value().header)
                        || containsCuttingControlCode(it.value().footer))
                    {
                        return it.key();
                    }
                }
                return QString();
            };
            QString invalidRuleKey = invalidRule(profile.layerBlocks);
            if (invalidRuleKey.isEmpty())
                invalidRuleKey = invalidRule(profile.colorBlocks);
            if (invalidRuleKey.isEmpty())
                invalidRuleKey = invalidRule(profile.entityTypeBlocks);
            if (!invalidRuleKey.isEmpty())
            {
                result.status = OperationStatus::InvalidInput;
                result.addDiagnostic(postDiagnostic
                (
                    DiagnosticCode::GCodeProfileInvalid,
                    QStringLiteral("图层、颜色或图元类型规则仍包含 M03/M05。"),
                    context,
                    0,
                    { { QStringLiteral("ruleKey"), invalidRuleKey } }
                ));
                return result;
            }
        }

        QStringList lines;
        appendTextBlock(lines, profile.programHeader);
        for (const auto& comment : program.leadingComments)
            lines.push_back(QLatin1Char('(') + QString::fromStdString(comment.text) + QLatin1Char(')'));

        enum class CuttingState { Off, On };
        CuttingState cuttingState = CuttingState::Off;
        int enableCount = 0;
        int disableCount = 0;
        int rapidWhileEnabledCount = 0;
        int cuttingWhileDisabledCount = 0;
        std::set<int> processUnitIndices;
        for (std::size_t entityIndex = 0; entityIndex < program.entities.size(); ++entityIndex)
        {
            const cadcam::nc::NcEntityBlock& entity = program.entities[entityIndex];
            const bool preserveLegacyOutputQuantization =
                program.mode == cadcam::nc::NcProgramMode::Planar3Axis
                || entity.metadata.sourceKind == geometry::SourceGeometryKind::Circle
                || entity.metadata.sourceKind == geometry::SourceGeometryKind::Ellipse
                || entity.metadata.sourceKind == geometry::SourceGeometryKind::Arc;
            if (entity.metadata.entityId == 0
                || entity.metadata.processOrder != static_cast<int>(entityIndex)
                || entity.motions.empty()
                || (rotary && entity.processUnitIndex < 0))
            {
                result.status = OperationStatus::Failed;
                result.addDiagnostic(postDiagnostic(DiagnosticCode::NcProgramInvariantViolation,
                    QStringLiteral("NC 图元块的加工顺序或运动数据无效。"), context,
                    entity.metadata.entityId,
                    {
                        { QStringLiteral("sourceIndex"), static_cast<qulonglong>(entity.metadata.sourceIndex) },
                        { QStringLiteral("processOrder"), entity.metadata.processOrder },
                        { QStringLiteral("processGroupId"), entity.metadata.processGroupId }
                    }));
                return result;
            }

            std::size_t motionIndex = 0;
            for (; motionIndex < entity.motions.size(); ++motionIndex)
            {
                const cadcam::nc::NcMotion& motion = entity.motions[motionIndex];
                if (motion.sourceKind != cadcam::nc::NcSourceMoveKind::Rapid) break;
                if (rotary && cuttingState != CuttingState::Off)
                {
                    ++rapidWhileEnabledCount;
                    result.status = OperationStatus::Conflict;
                    result.addDiagnostic(postDiagnostic
                    (
                        DiagnosticCode::GCodeCuttingStateViolation,
                        QStringLiteral("切削开启状态下出现了快速运动。"),
                        context,
                        entity.metadata.entityId,
                        {
                            { QStringLiteral("processUnitIndex"),
                                entity.processUnitIndex },
                            { QStringLiteral("motionIndex"),
                                static_cast<qulonglong>(motionIndex) }
                        }
                    ));
                    return result;
                }
                if (!validMotion(motion, program.mode, true))
                {
                    result.status = OperationStatus::Failed;
                    result.addDiagnostic(postDiagnostic(DiagnosticCode::NcProgramUnsupportedMotion,
                        QStringLiteral("NC 程序包含无效的快速运动。"), context, entity.metadata.entityId,
                        { { QStringLiteral("motionIndex"), static_cast<qulonglong>(motionIndex) } }));
                    return result;
                }
                lines.push_back(formatMotion
                    (motion, profile, preserveLegacyOutputQuantization));
            }
            if (motionIndex == entity.motions.size()) continue;

            const QString typeKey = QString::fromStdString(entity.metadata.entityTypeKey);
            const QString layerKey = QString::fromStdString(entity.metadata.layerKey);
            const QString colorKey = QString::fromStdString(entity.metadata.colorKey);
            const GCodeBlock typeBlock = profile.entityTypeBlocks.value(typeKey);
            const GCodeBlock layerBlock = profile.layerBlocks.value(layerKey);
            const GCodeBlock colorBlock = profile.colorBlocks.value(colorKey);
            appendTextBlock(lines, layerBlock.header);
            appendTextBlock(lines, colorBlock.header);
            appendTextBlock(lines, typeBlock.header);
            if (rotary)
            {
                processUnitIndices.insert(entity.processUnitIndex);
                if (entity.beforeCutting
                    == cadcam::nc::NcCuttingControl::Enable)
                {
                    if (cuttingState != CuttingState::Off)
                    {
                        result.status = OperationStatus::Conflict;
                        result.addDiagnostic(postDiagnostic
                        (
                            DiagnosticCode::GCodeCuttingStateViolation,
                            QStringLiteral("加工单元重复开启切削状态。"),
                            context,
                            entity.metadata.entityId,
                            {
                                { QStringLiteral("processUnitIndex"),
                                    entity.processUnitIndex }
                            }
                        ));
                        return result;
                    }
                    appendTextBlock(lines, profile.processUnitBlock.header);
                    cuttingState = CuttingState::On;
                    ++enableCount;
                }
                else if (entity.beforeCutting
                    == cadcam::nc::NcCuttingControl::Disable)
                {
                    result.status = OperationStatus::Conflict;
                    result.addDiagnostic(postDiagnostic
                    (
                        DiagnosticCode::GCodeCuttingStateViolation,
                        QStringLiteral("加工单元切削前控制语义无效。"),
                        context,
                        entity.metadata.entityId
                    ));
                    return result;
                }
            }

            for (; motionIndex < entity.motions.size(); ++motionIndex)
            {
                const cadcam::nc::NcMotion& motion = entity.motions[motionIndex];
                if (motion.sourceKind == cadcam::nc::NcSourceMoveKind::Rapid
                    || !validMotion(motion, program.mode, false)
                    || motion.entityId != entity.metadata.entityId
                    || motion.processGroupId != entity.metadata.processGroupId
                    )
                {
                    result.status = OperationStatus::NotSupported;
                    result.addDiagnostic(postDiagnostic(DiagnosticCode::NcProgramUnsupportedMotion,
                        QStringLiteral("NC 程序包含不支持或无效的运动。"), context,
                        entity.metadata.entityId,
                        {
                            { QStringLiteral("motionIndex"), static_cast<qulonglong>(motionIndex) },
                            { QStringLiteral("entityTypeKey"), typeKey },
                            { QStringLiteral("layerKey"), layerKey },
                            { QStringLiteral("colorKey"), colorKey }
                        }));
                    return result;
                }
                if (rotary && cuttingState != CuttingState::On)
                {
                    ++cuttingWhileDisabledCount;
                    result.status = OperationStatus::Conflict;
                    result.addDiagnostic(postDiagnostic
                    (
                        DiagnosticCode::GCodeCuttingStateViolation,
                        QStringLiteral("切削关闭状态下出现了切削运动。"),
                        context,
                        entity.metadata.entityId,
                        {
                            { QStringLiteral("processUnitIndex"),
                                entity.processUnitIndex },
                            { QStringLiteral("motionIndex"),
                                static_cast<qulonglong>(motionIndex) }
                        }
                    ));
                    return result;
                }
                const bool circular = motion.kind == cadcam::nc::NcMotionKind::CircularClockwise
                    || motion.kind == cadcam::nc::NcMotionKind::CircularCounterclockwise;
                if (circular && motion.plane == cadcam::nc::NcPlane::ZX)
                    lines.push_back(QStringLiteral("G18"));
                else if (circular && motion.plane == cadcam::nc::NcPlane::YZ)
                    lines.push_back(QStringLiteral("G19"));
                lines.push_back(formatMotion
                    (motion, profile, preserveLegacyOutputQuantization));
                if (circular && motion.plane != cadcam::nc::NcPlane::XY)
                    lines.push_back(QStringLiteral("G17"));
            }

            if (rotary)
            {
                if (entity.afterCutting
                    == cadcam::nc::NcCuttingControl::Disable)
                {
                    if (cuttingState != CuttingState::On)
                    {
                        result.status = OperationStatus::Conflict;
                        result.addDiagnostic(postDiagnostic
                        (
                            DiagnosticCode::GCodeCuttingStateViolation,
                            QStringLiteral("加工单元结束时切削状态未开启。"),
                            context,
                            entity.metadata.entityId,
                            {
                                { QStringLiteral("processUnitIndex"),
                                    entity.processUnitIndex }
                            }
                        ));
                        return result;
                    }
                    appendTextBlock(lines, profile.processUnitBlock.footer);
                    cuttingState = CuttingState::Off;
                    ++disableCount;
                }
                else if (entity.afterCutting
                    == cadcam::nc::NcCuttingControl::Enable)
                {
                    result.status = OperationStatus::Conflict;
                    result.addDiagnostic(postDiagnostic
                    (
                        DiagnosticCode::GCodeCuttingStateViolation,
                        QStringLiteral("加工单元切削后控制语义无效。"),
                        context,
                        entity.metadata.entityId
                    ));
                    return result;
                }
            }
            appendTextBlock(lines, typeBlock.footer);
            appendTextBlock(lines, colorBlock.footer);
            appendTextBlock(lines, layerBlock.footer);
        }

        if (rotary
            && (cuttingState != CuttingState::Off
                || enableCount != static_cast<int>(processUnitIndices.size())
                || disableCount != static_cast<int>(processUnitIndices.size())))
        {
            result.status = OperationStatus::Conflict;
            result.addDiagnostic(postDiagnostic
            (
                DiagnosticCode::GCodeCuttingStateViolation,
                QStringLiteral("四轴程序结束时加工单元启停状态不完整。"),
                context,
                0,
                {
                    { QStringLiteral("processUnitCount"),
                        static_cast<int>(processUnitIndices.size()) },
                    { QStringLiteral("enableCount"), enableCount },
                    { QStringLiteral("disableCount"), disableCount }
                }
            ));
            return result;
        }
        appendTextBlock(lines, profile.programFooter);
        if (lines.empty())
        {
            result.status = OperationStatus::Failed;
            result.addDiagnostic(postDiagnostic(DiagnosticCode::GCodeRenderingFailed,
                QStringLiteral("生成的 G 代码为空。"), context));
            return result;
        }
        if (rotary)
        {
            qInfo().noquote() << QStringLiteral(
                "[GCode][CuttingState] processUnitCount=%1 enableCount=%2 "
                "disableCount=%3 rapidWhileEnabledCount=%4 "
                "cuttingWhileDisabledCount=%5 "
                "legacyRestartOptimization=false status=Success")
                .arg(processUnitIndices.size())
                .arg(enableCount)
                .arg(disableCount)
                .arg(rapidWhileEnabledCount)
                .arg(cuttingWhileDisabledCount);
        }

        result.status = OperationStatus::Success;
        result.value = lines.join(QStringLiteral("\r\n")) + QStringLiteral("\r\n");
        return result;
    }
}
