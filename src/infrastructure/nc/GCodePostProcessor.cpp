#include "infrastructure/nc/GCodePostProcessor.h"

#include "infrastructure/config/GProfile.h"
#include "core/diagnostics/SummaryLog.h"

#include <QDebug>
#include <QRegularExpression>
#include <QStringList>

#include <cmath>
#include <memory>
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

        class GCodeRenderStrategy
        {
        public:
            explicit GCodeRenderStrategy(cadcam::nc::NcProgramMode mode)
                : m_mode(mode)
            {
            }

            virtual ~GCodeRenderStrategy() = default;

            virtual bool validateProgram
            (
                const GCodePostProcessorProfile& profile,
                OperationResult<QString>& result,
                const OperationContext& context
            ) = 0;

            virtual bool validateEntity
            (
                const cadcam::nc::NcEntityBlock& entity,
                OperationResult<QString>& result,
                const OperationContext& context
            ) = 0;

            bool preserveLegacyOutputQuantization
                (const cadcam::nc::NcEntityBlock& entity) const
            {
                return m_mode == cadcam::nc::NcProgramMode::Planar3Axis
                    || entity.metadata.sourceKind == geometry::SourceGeometryKind::Circle
                    || entity.metadata.sourceKind == geometry::SourceGeometryKind::Ellipse
                    || entity.metadata.sourceKind == geometry::SourceGeometryKind::Arc;
            }

            bool isValidMotion
                (const cadcam::nc::NcMotion& motion, bool rapid) const
            {
                return validMotion(motion, m_mode, rapid);
            }

        protected:
            cadcam::nc::NcProgramMode m_mode;
        };

        class PlanarRenderStrategy final : public GCodeRenderStrategy
        {
        public:
            PlanarRenderStrategy()
                : GCodeRenderStrategy(cadcam::nc::NcProgramMode::Planar3Axis)
            {
            }

            bool validateProgram
            (
                const GCodePostProcessorProfile& profile,
                OperationResult<QString>& result,
                const OperationContext& context
            ) override
            {
                Q_UNUSED(profile);
                Q_UNUSED(result);
                Q_UNUSED(context);
                return true;
            }

            bool validateEntity
            (
                const cadcam::nc::NcEntityBlock& entity,
                OperationResult<QString>& result,
                const OperationContext& context
            ) override
            {
                Q_UNUSED(entity);
                Q_UNUSED(result);
                Q_UNUSED(context);
                return true;
            }
        };

        class RotaryRenderStrategy final : public GCodeRenderStrategy
        {
        public:
            RotaryRenderStrategy()
                : GCodeRenderStrategy(cadcam::nc::NcProgramMode::Rotary4Axis)
            {
            }

            bool validateProgram
            (
                const GCodePostProcessorProfile& profile,
                OperationResult<QString>& result,
                const OperationContext& context
            ) override
            {
                if (profile.processUnitBlock.header.trimmed().isEmpty()
                    || profile.processUnitBlock.footer.trimmed().isEmpty())
                {
                    result.status = OperationStatus::InvalidInput;
                    result.addDiagnostic(postDiagnostic
                    (
                        DiagnosticCode::GCodeProfileInvalid,
                        QStringLiteral("四轴加工单元开始或结束代码为空。"),
                        context
                    ));
                    return false;
                }

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
                    return false;
                }
                return true;
            }

            bool validateEntity
            (
                const cadcam::nc::NcEntityBlock& entity,
                OperationResult<QString>& result,
                const OperationContext& context
            ) override
            {
                if (entity.processUnitIndex >= 0) return true;
                result.status = OperationStatus::Failed;
                result.addDiagnostic(postDiagnostic(DiagnosticCode::NcProgramInvariantViolation,
                    QStringLiteral("NC 图元块的加工顺序或运动数据无效。"), context,
                    entity.metadata.entityId,
                    {
                        { QStringLiteral("sourceIndex"), static_cast<qulonglong>(entity.metadata.sourceIndex) },
                        { QStringLiteral("processOrder"), entity.metadata.processOrder },
                        { QStringLiteral("processGroupId"), entity.metadata.processGroupId }
                    }));
                return false;
            }
        };

        // 单元级切割控制排序器：与加工模式无关，三轴与四轴共用同一状态机。
        // 程序不含 Enable/Disable 块时保持未激活，不产生校验与输出，因此
        // 三轴默认输出与历史行为完全一致；程序启用单元级控制后自动生效。
        class CuttingControlSequencer
        {
        public:
            explicit CuttingControlSequencer(const GCodePostProcessorProfile& profile)
                : m_profile(profile)
            {
            }

            void ensureActive(const cadcam::nc::NcEntityBlock& entity)
            {
                if (m_active) return;
                if (entity.beforeCutting != cadcam::nc::NcCuttingControl::None
                    || entity.afterCutting != cadcam::nc::NcCuttingControl::None)
                {
                    m_active = true;
                }
            }

            bool onLeadingRapid
            (
                const cadcam::nc::NcEntityBlock& entity,
                std::size_t motionIndex,
                OperationResult<QString>& result,
                const OperationContext& context
            )
            {
                if (!m_active || m_cuttingState == CuttingState::Off) return true;
                ++m_rapidWhileEnabledCount;
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
                return false;
            }

            bool beginBlock
            (
                const cadcam::nc::NcEntityBlock& entity,
                QStringList& lines,
                OperationResult<QString>& result,
                const OperationContext& context
            )
            {
                if (!m_active) return true;
                m_processUnitIndices.insert(entity.processUnitIndex);
                if (entity.beforeCutting
                    == cadcam::nc::NcCuttingControl::Enable)
                {
                    if (m_cuttingState != CuttingState::Off)
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
                        return false;
                    }
                    appendTextBlock(lines, m_profile.processUnitBlock.header);
                    m_cuttingState = CuttingState::On;
                    ++m_enableCount;
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
                    return false;
                }
                return true;
            }

            bool checkCuttingMotion
            (
                const cadcam::nc::NcEntityBlock& entity,
                std::size_t motionIndex,
                OperationResult<QString>& result,
                const OperationContext& context
            )
            {
                if (!m_active || m_cuttingState == CuttingState::On) return true;
                ++m_cuttingWhileDisabledCount;
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
                return false;
            }

            bool endBlock
            (
                const cadcam::nc::NcEntityBlock& entity,
                QStringList& lines,
                OperationResult<QString>& result,
                const OperationContext& context
            )
            {
                if (!m_active) return true;
                if (entity.afterCutting
                    == cadcam::nc::NcCuttingControl::Disable)
                {
                    if (m_cuttingState != CuttingState::On)
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
                        return false;
                    }
                    appendTextBlock(lines, m_profile.processUnitBlock.footer);
                    m_cuttingState = CuttingState::Off;
                    ++m_disableCount;
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
                    return false;
                }
                return true;
            }

            bool finalize
            (
                OperationResult<QString>& result,
                const OperationContext& context
            )
            {
                if (!m_active) return true;
                if (m_cuttingState != CuttingState::Off
                    || m_enableCount != static_cast<int>(m_processUnitIndices.size())
                    || m_disableCount != static_cast<int>(m_processUnitIndices.size()))
                {
                    result.status = OperationStatus::Conflict;
                    result.addDiagnostic(postDiagnostic
                    (
                        DiagnosticCode::GCodeCuttingStateViolation,
                        QStringLiteral("程序结束时加工单元启停状态不完整。"),
                        context,
                        0,
                        {
                            { QStringLiteral("processUnitCount"),
                                static_cast<int>(m_processUnitIndices.size()) },
                            { QStringLiteral("enableCount"), m_enableCount },
                            { QStringLiteral("disableCount"), m_disableCount }
                        }
                    ));
                    return false;
                }
                cadcam::core::emitSummaryLog
                (
                    QStringLiteral("GCode"),
                    QStringLiteral("CuttingState"),
                    QStringLiteral("processUnitCount=%1 enableCount=%2 "
                        "disableCount=%3 rapidWhileEnabledCount=%4 "
                        "cuttingWhileDisabledCount=%5 "
                        "legacyRestartOptimization=false status=Success")
                        .arg(m_processUnitIndices.size())
                    .arg(m_enableCount)
                    .arg(m_disableCount)
                    .arg(m_rapidWhileEnabledCount)
                    .arg(m_cuttingWhileDisabledCount));
                return true;
            }

        private:
            enum class CuttingState { Off, On };

            const GCodePostProcessorProfile& m_profile;
            bool m_active = false;
            CuttingState m_cuttingState = CuttingState::Off;
            int m_enableCount = 0;
            int m_disableCount = 0;
            int m_rapidWhileEnabledCount = 0;
            int m_cuttingWhileDisabledCount = 0;
            std::set<int> m_processUnitIndices;
        };

        std::unique_ptr<GCodeRenderStrategy> strategy = rotary
            ? std::unique_ptr<GCodeRenderStrategy>(new RotaryRenderStrategy())
            : std::unique_ptr<GCodeRenderStrategy>(new PlanarRenderStrategy());
        if (!strategy->validateProgram(profile, result, context)) return result;

        QStringList lines;
        appendTextBlock(lines, profile.programHeader);
        for (const auto& comment : program.leadingComments)
            lines.push_back(QLatin1Char('(') + QString::fromStdString(comment.text) + QLatin1Char(')'));

        CuttingControlSequencer cuttingControl(profile);
        for (std::size_t entityIndex = 0; entityIndex < program.entities.size(); ++entityIndex)
        {
            const cadcam::nc::NcEntityBlock& entity = program.entities[entityIndex];
            const bool preserveLegacyOutputQuantization =
                strategy->preserveLegacyOutputQuantization(entity);
            if (entity.metadata.entityId == 0
                || entity.metadata.processOrder != static_cast<int>(entityIndex)
                || entity.motions.empty())
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
            if (!strategy->validateEntity(entity, result, context)) return result;
            cuttingControl.ensureActive(entity);

            std::size_t motionIndex = 0;
            for (; motionIndex < entity.motions.size(); ++motionIndex)
            {
                const cadcam::nc::NcMotion& motion = entity.motions[motionIndex];
                if (motion.sourceKind != cadcam::nc::NcSourceMoveKind::Rapid) break;
                if (!cuttingControl.onLeadingRapid(entity, motionIndex, result, context))
                {
                    return result;
                }
                if (!strategy->isValidMotion(motion, true))
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
            if (!cuttingControl.beginBlock(entity, lines, result, context))
            {
                return result;
            }

            for (; motionIndex < entity.motions.size(); ++motionIndex)
            {
                const cadcam::nc::NcMotion& motion = entity.motions[motionIndex];
                if (motion.sourceKind == cadcam::nc::NcSourceMoveKind::Rapid
                    || !strategy->isValidMotion(motion, false)
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
                if (!cuttingControl.checkCuttingMotion(entity, motionIndex, result, context))
                {
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

            if (!cuttingControl.endBlock(entity, lines, result, context))
            {
                return result;
            }
            appendTextBlock(lines, typeBlock.footer);
            appendTextBlock(lines, colorBlock.footer);
            appendTextBlock(lines, layerBlock.footer);
        }

        if (!cuttingControl.finalize(result, context)) return result;
        appendTextBlock(lines, profile.programFooter);
        if (lines.empty())
        {
            result.status = OperationStatus::Failed;
            result.addDiagnostic(postDiagnostic(DiagnosticCode::GCodeRenderingFailed,
                QStringLiteral("生成的 G 代码为空。"), context));
            return result;
        }

        result.status = OperationStatus::Success;
        result.value = lines.join(QStringLiteral("\r\n")) + QStringLiteral("\r\n");
        return result;
    }

}


