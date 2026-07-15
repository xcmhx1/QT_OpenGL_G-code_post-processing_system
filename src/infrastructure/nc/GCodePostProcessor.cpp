#include "infrastructure/nc/GCodePostProcessor.h"

#include "infrastructure/config/GProfile.h"

#include <QStringList>

#include <cmath>

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

        bool standaloneMCode(const QString& line, const QString& padded, const QString& shortCode)
        {
            const QString normalized = line.trimmed().toUpper();
            return normalized == padded || normalized == shortCode;
        }

        bool optimizeLaserRestarts(QStringList& lines)
        {
            QStringList optimized;
            optimized.reserve(lines.size());
            for (const QString& line : lines)
            {
                const bool startsLaser = standaloneMCode
                    (line, QStringLiteral("M03"), QStringLiteral("M3"));
                const bool previousStopsLaser = !optimized.isEmpty()
                    && standaloneMCode(optimized.constLast(), QStringLiteral("M05"), QStringLiteral("M5"));
                if (startsLaser && previousStopsLaser)
                {
                    optimized.removeLast();
                    continue;
                }
                optimized.push_back(line);
            }
            lines = std::move(optimized);
            return true;
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

        QStringList lines;
        appendTextBlock(lines, profile.programHeader);
        for (const auto& comment : program.leadingComments)
            lines.push_back(QLatin1Char('(') + QString::fromStdString(comment.text) + QLatin1Char(')'));

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

            std::size_t motionIndex = 0;
            for (; motionIndex < entity.motions.size(); ++motionIndex)
            {
                const cadcam::nc::NcMotion& motion = entity.motions[motionIndex];
                if (motion.sourceKind != cadcam::nc::NcSourceMoveKind::Rapid) break;
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

            appendTextBlock(lines, typeBlock.footer);
            appendTextBlock(lines, colorBlock.footer);
            appendTextBlock(lines, layerBlock.footer);
        }

        appendTextBlock(lines, profile.programFooter);
        if (!optimizeLaserRestarts(lines))
        {
            result.status = OperationStatus::Failed;
            result.addDiagnostic(postDiagnostic(DiagnosticCode::GCodeTextOptimizationFailed,
                QStringLiteral("G 代码文本优化失败。"), context));
            return result;
        }
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
