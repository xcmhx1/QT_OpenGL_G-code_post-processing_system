#include "infrastructure/nc/GCodePostProcessor.h"

#include "GProfile.h"

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
            const GCodePostProcessorProfile& profile
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
            case cadcam::nc::NcMotionKind::CircularCounterclockwise:
            default:
                return QString();
            }
            return QStringLiteral("%1 X%2 Y%3 Z%4 A%5")
                .arg(code)
                .arg(QString::number(*motion.axes.x, 'f', profile.coordinatePrecision))
                .arg(QString::number(*motion.axes.y, 'f', profile.coordinatePrecision))
                .arg(QString::number(*motion.axes.z, 'f', profile.coordinatePrecision))
                .arg(QString::number(*motion.axes.a, 'f', profile.anglePrecision));
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
        if (program.mode != cadcam::nc::NcProgramMode::Rotary4Axis
            || program.contentRevision == 0 || program.entities.empty())
        {
            result.status = OperationStatus::InvalidInput;
            result.addDiagnostic(postDiagnostic(DiagnosticCode::GCodeRenderingFailed,
                QStringLiteral("后处理器只接受有效的四轴 NC 程序。"), context));
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
                if (motion.kind != cadcam::nc::NcMotionKind::Rapid
                    || !motion.axes.x.has_value() || !motion.axes.y.has_value()
                    || !motion.axes.z.has_value() || !motion.axes.a.has_value()
                    || !std::isfinite(*motion.axes.x) || !std::isfinite(*motion.axes.y)
                    || !std::isfinite(*motion.axes.z) || !std::isfinite(*motion.axes.a))
                {
                    result.status = OperationStatus::Failed;
                    result.addDiagnostic(postDiagnostic(DiagnosticCode::NcProgramUnsupportedMotion,
                        QStringLiteral("NC 程序包含无效的快速运动。"), context, entity.metadata.entityId,
                        { { QStringLiteral("motionIndex"), static_cast<qulonglong>(motionIndex) } }));
                    return result;
                }
                lines.push_back(formatMotion(motion, profile));
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
                const bool validLinear = motion.kind == cadcam::nc::NcMotionKind::Linear
                    && motion.sourceKind != cadcam::nc::NcSourceMoveKind::Rapid;
                if (!validLinear || motion.entityId != entity.metadata.entityId
                    || motion.processGroupId != entity.metadata.processGroupId
                    || !motion.axes.x.has_value() || !motion.axes.y.has_value()
                    || !motion.axes.z.has_value() || !motion.axes.a.has_value()
                    || !std::isfinite(*motion.axes.x) || !std::isfinite(*motion.axes.y)
                    || !std::isfinite(*motion.axes.z) || !std::isfinite(*motion.axes.a))
                {
                    result.status = OperationStatus::NotSupported;
                    result.addDiagnostic(postDiagnostic(DiagnosticCode::NcProgramUnsupportedMotion,
                        QStringLiteral("NC 程序包含不支持或无效的四轴运动。"), context,
                        entity.metadata.entityId,
                        {
                            { QStringLiteral("motionIndex"), static_cast<qulonglong>(motionIndex) },
                            { QStringLiteral("entityTypeKey"), typeKey },
                            { QStringLiteral("layerKey"), layerKey },
                            { QStringLiteral("colorKey"), colorKey }
                        }));
                    return result;
                }
                lines.push_back(formatMotion(motion, profile));
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
                QStringLiteral("生成的四轴 G 代码为空。"), context));
            return result;
        }

        result.status = OperationStatus::Success;
        result.value = lines.join(QStringLiteral("\r\n")) + QStringLiteral("\r\n");
        return result;
    }
}
