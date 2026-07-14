#include "pch.h"

#include "Gcode_postprocessing_system.h"

#include "CadItem.h"
#include "CadProcessVisualUtils.h"
#include "application/messaging/DebugMessageSink.h"
#include "application/messaging/UserInterfaceMessageSink.h"

#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QMessageBox>
#include <QSettings>
#include <QStandardPaths>
#include <QSet>

#include <cmath>
#include <limits>

namespace
{
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kTwoPi = 6.28318530717958647692;
    constexpr double kDedupTolerance = 1.0e-6;

    Diagnostic makeExportDiagnostic
    (
        const OperationContext& context,
        DiagnosticCode code,
        DiagnosticSeverity severity,
        const QString& operation,
        const QString& stage,
        const QString& userMessage,
        const QString& technicalDetail = QString(),
        const QVariantMap& diagnosticContext = QVariantMap()
    )
    {
        Diagnostic diagnostic;
        diagnostic.code = code;
        diagnostic.severity = severity;
        diagnostic.component = QStringLiteral("GCodeExport");
        diagnostic.operation = operation;
        diagnostic.stage = stage;
        diagnostic.userMessage = userMessage;
        diagnostic.technicalDetail = technicalDetail;
        diagnostic.correlationId = context.correlationId;
        diagnostic.context = diagnosticContext;
        return diagnostic;
    }

    qint64 quantizeDedupValue(double value)
    {
        const qint64 quantized = static_cast<qint64>(std::llround(value / kDedupTolerance));
        return quantized == 0 ? 0 : quantized;
    }

    QString dedupNumberToken(double value)
    {
        return QString::number(quantizeDedupValue(value));
    }

    QString dedupCoordToken(const DRW_Coord& coord)
    {
        return QStringLiteral("%1,%2,%3")
            .arg(dedupNumberToken(coord.x))
            .arg(dedupNumberToken(coord.y))
            .arg(dedupNumberToken(coord.z));
    }

    QString dedupCoordToken(double x, double y, double z = 0.0)
    {
        return QStringLiteral("%1,%2,%3")
            .arg(dedupNumberToken(x))
            .arg(dedupNumberToken(y))
            .arg(dedupNumberToken(z));
    }

    double normalizeAngleZeroToTwoPi(double angle)
    {
        double normalized = std::fmod(angle, kTwoPi);

        if (normalized < 0.0)
        {
            normalized += kTwoPi;
        }

        return normalized;
    }

    QString dedupAngleToken(double angle)
    {
        return dedupNumberToken(normalizeAngleZeroToTwoPi(angle));
    }

    QString buildLineDuplicateKey(const DRW_Line* line)
    {
        if (line == nullptr)
        {
            return QString();
        }

        const QString start = dedupCoordToken(line->basePoint);
        const QString end = dedupCoordToken(line->secPoint);
        const auto ordered = start <= end
            ? std::pair<QString, QString>(start, end)
            : std::pair<QString, QString>(end, start);

        return QStringLiteral("LINE|%1|%2").arg(ordered.first, ordered.second);
    }

    QString buildCircleDuplicateKey(const DRW_Circle* circle)
    {
        if (circle == nullptr)
        {
            return QString();
        }

        return QStringLiteral("CIRCLE|%1|%2")
            .arg(dedupCoordToken(circle->basePoint))
            .arg(dedupNumberToken(circle->radious));
    }

    QString buildArcDuplicateKey(const DRW_Arc* arc)
    {
        if (arc == nullptr)
        {
            return QString();
        }

        return QStringLiteral("ARC|%1|%2|%3|%4")
            .arg(dedupCoordToken(arc->basePoint))
            .arg(dedupNumberToken(arc->radious))
            .arg(dedupAngleToken(arc->staangle))
            .arg(dedupAngleToken(arc->endangle));
    }

    QString buildEllipseDuplicateKey(const DRW_Ellipse* ellipse)
    {
        if (ellipse == nullptr)
        {
            return QString();
        }

        DRW_Coord majorAxis = ellipse->secPoint;
        double startParam = ellipse->staparam;
        double endParam = ellipse->endparam;
        const double span = endParam - startParam;
        const bool isFullEllipse = std::abs(span) < kDedupTolerance
            || std::abs(std::abs(span) - kTwoPi) < kDedupTolerance;

        const bool flipMajorAxis =
            quantizeDedupValue(majorAxis.x) < 0
            || (quantizeDedupValue(majorAxis.x) == 0 && quantizeDedupValue(majorAxis.y) < 0)
            || (quantizeDedupValue(majorAxis.x) == 0 && quantizeDedupValue(majorAxis.y) == 0 && quantizeDedupValue(majorAxis.z) < 0);

        if (flipMajorAxis)
        {
            majorAxis.x = -majorAxis.x;
            majorAxis.y = -majorAxis.y;
            majorAxis.z = -majorAxis.z;
            startParam += kPi;
            endParam += kPi;
        }

        if (isFullEllipse)
        {
            startParam = 0.0;
            endParam = kTwoPi;
        }
        else
        {
            startParam = normalizeAngleZeroToTwoPi(startParam);
            double normalizedEnd = normalizeAngleZeroToTwoPi(endParam);

            while (normalizedEnd <= startParam)
            {
                normalizedEnd += kTwoPi;
            }

            endParam = normalizedEnd;
        }

        DRW_Coord normal = ellipse->extPoint;
        const double normalLength = std::sqrt(normal.x * normal.x + normal.y * normal.y + normal.z * normal.z);

        if (normalLength > kDedupTolerance)
        {
            normal.x /= normalLength;
            normal.y /= normalLength;
            normal.z /= normalLength;
        }

        return QStringLiteral("ELLIPSE|%1|%2|%3|%4|%5|%6")
            .arg(dedupCoordToken(ellipse->basePoint))
            .arg(dedupCoordToken(majorAxis))
            .arg(dedupCoordToken(normal))
            .arg(dedupNumberToken(ellipse->ratio))
            .arg(dedupNumberToken(startParam))
            .arg(dedupNumberToken(endParam));
    }

    QString buildPolylineVertexSequenceToken(const std::vector<std::shared_ptr<DRW_Vertex>>& vertlist)
    {
        QStringList vertexTokens;
        vertexTokens.reserve(static_cast<int>(vertlist.size()));

        for (const std::shared_ptr<DRW_Vertex>& vertex : vertlist)
        {
            if (!vertex)
            {
                vertexTokens.push_back(QStringLiteral("null"));
                continue;
            }

            vertexTokens.push_back
            (
                QStringLiteral("%1|%2")
                .arg(dedupCoordToken(vertex->basePoint))
                .arg(dedupNumberToken(vertex->bulge))
            );
        }

        return vertexTokens.join(QStringLiteral(";"));
    }

    QString buildLWPolylineVertexSequenceToken(const std::vector<std::shared_ptr<DRW_Vertex2D>>& vertlist)
    {
        QStringList vertexTokens;
        vertexTokens.reserve(static_cast<int>(vertlist.size()));

        for (const std::shared_ptr<DRW_Vertex2D>& vertex : vertlist)
        {
            if (!vertex)
            {
                vertexTokens.push_back(QStringLiteral("null"));
                continue;
            }

            vertexTokens.push_back
            (
                QStringLiteral("%1|%2")
                .arg(dedupCoordToken(vertex->x, vertex->y, 0.0))
                .arg(dedupNumberToken(vertex->bulge))
            );
        }

        return vertexTokens.join(QStringLiteral(";"));
    }

    QString buildPolylineDuplicateKey(const DRW_Polyline* polyline)
    {
        if (polyline == nullptr)
        {
            return QString();
        }

        return QStringLiteral("POLYLINE|%1|%2|%3")
            .arg((polyline->flags & 0x01) != 0 ? QStringLiteral("C") : QStringLiteral("O"))
            .arg(dedupCoordToken(polyline->extPoint))
            .arg(buildPolylineVertexSequenceToken(polyline->vertlist));
    }

    QString buildLWPolylineDuplicateKey(const DRW_LWPolyline* polyline)
    {
        if (polyline == nullptr)
        {
            return QString();
        }

        return QStringLiteral("LWPOLYLINE|%1|%2|%3|%4")
            .arg((polyline->flags & 0x01) != 0 ? QStringLiteral("C") : QStringLiteral("O"))
            .arg(dedupNumberToken(polyline->elevation))
            .arg(dedupCoordToken(polyline->extPoint))
            .arg(buildLWPolylineVertexSequenceToken(polyline->vertlist));
    }

    QString buildSplineDuplicateKey(const DRW_Spline* spline)
    {
        if (spline == nullptr)
        {
            return QString();
        }
        QStringList values
        {
            QStringLiteral("SPLINE"),
            QString::number(spline->flags),
            QString::number(spline->degree)
        };
        for (double knot : spline->knotslist)
        {
            values.push_back(dedupNumberToken(knot));
        }
        for (double weight : spline->weightlist)
        {
            values.push_back(dedupNumberToken(weight));
        }
        for (const std::shared_ptr<DRW_Coord>& point : spline->controllist)
        {
            values.push_back(point != nullptr
                ? dedupCoordToken(*point) : QStringLiteral("NULL"));
        }
        for (const std::shared_ptr<DRW_Coord>& point : spline->fitlist)
        {
            values.push_back(point != nullptr
                ? dedupCoordToken(*point) : QStringLiteral("NULL"));
        }
        return values.join('|');
    }

    QString duplicateGeometryKey(const CadItem* item)
    {
        if (item == nullptr || item->m_nativeEntity == nullptr)
        {
            return QString();
        }

        switch (item->m_type)
        {
        case DRW::ETYPE::LINE:
            return buildLineDuplicateKey(static_cast<const DRW_Line*>(item->m_nativeEntity));
        case DRW::ETYPE::CIRCLE:
            return buildCircleDuplicateKey(static_cast<const DRW_Circle*>(item->m_nativeEntity));
        case DRW::ETYPE::ARC:
            return buildArcDuplicateKey(static_cast<const DRW_Arc*>(item->m_nativeEntity));
        case DRW::ETYPE::ELLIPSE:
            return buildEllipseDuplicateKey(static_cast<const DRW_Ellipse*>(item->m_nativeEntity));
        case DRW::ETYPE::POLYLINE:
            return buildPolylineDuplicateKey(static_cast<const DRW_Polyline*>(item->m_nativeEntity));
        case DRW::ETYPE::LWPOLYLINE:
            return buildLWPolylineDuplicateKey(static_cast<const DRW_LWPolyline*>(item->m_nativeEntity));
        case DRW::ETYPE::SPLINE:
            return buildSplineDuplicateKey(static_cast<const DRW_Spline*>(item->m_nativeEntity));
        default:
            return QString();
        }
    }

    bool isExportSortableEntityType(DRW::ETYPE type)
    {
        switch (type)
        {
        case DRW::ETYPE::LINE:
        case DRW::ETYPE::ARC:
        case DRW::ETYPE::CIRCLE:
        case DRW::ETYPE::ELLIPSE:
        case DRW::ETYPE::POLYLINE:
        case DRW::ETYPE::LWPOLYLINE:
        case DRW::ETYPE::SPLINE:
            return true;
        default:
            return false;
        }
    }

}

bool Gcode_postprocessing_system::exportGCode()
{
    const GGenerator::GenerationMode generationMode = resolveGenerationMode();
    const QString modeDisplayName = QStringLiteral("当前模式 %1").arg(generationModeDisplayName(generationMode));

    return exportGCode(generationMode, modeDisplayName);
}

QString Gcode_postprocessing_system::defaultGCodeExportPathForCurrentDocument() const
{
    if (!loadUseDefaultExportPath())
    {
        return QString();
    }

    QString exportDirectory;
    const QString lastExportPath = loadLastGCodeExportPath();

    if (!lastExportPath.isEmpty())
    {
        const QString lastExportDirectory = QFileInfo(lastExportPath).absolutePath();

        if (QDir(lastExportDirectory).exists())
        {
            exportDirectory = lastExportDirectory;
        }
    }

    if (exportDirectory.trimmed().isEmpty())
    {
        exportDirectory = QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation);
    }

    if (exportDirectory.trimmed().isEmpty())
    {
        exportDirectory = QDir::homePath();
    }

    QString outputFileName = QStringLiteral("output.nc");

    if (!m_currentDocumentPath.trimmed().isEmpty())
    {
        const QString baseName = QFileInfo(m_currentDocumentPath).completeBaseName().trimmed();

        if (!baseName.isEmpty())
        {
            outputFileName = baseName + QStringLiteral(".nc");
        }
    }

    return QDir(exportDirectory).filePath(outputFileName);
}

bool Gcode_postprocessing_system::prepareDocumentForGCodeExport(GGenerator::GenerationMode generationMode)
{
    return prepareDocumentForGCodeExport
    (
        generationMode,
        createOperationContext(QStringLiteral("export-gcode"))
    ).succeeded();
}

OperationReport Gcode_postprocessing_system::prepareDocumentForGCodeExport
(
    GGenerator::GenerationMode generationMode,
    const OperationContext& context
)
{
    OperationReport report;
    const int excludedCount = refreshWasteProcessingExclusions();

    if (excludedCount > 0)
    {
        report.addDiagnostic(makeExportDiagnostic
        (
            context,
            DiagnosticCode::None,
            DiagnosticSeverity::Notice,
            QStringLiteral("PrepareDocumentForGCodeExport"),
            QStringLiteral("ApplyExclusions"),
            QStringLiteral("废面规则已排除 %1 个图元，G代码不会包含这些图元。").arg(excludedCount),
            QString(),
            { { QStringLiteral("excludedCount"), excludedCount } }
        ));
    }

    const bool hasProcessableEntity = std::any_of
    (
        m_document.m_entities.begin(),
        m_document.m_entities.end(),
        [](const std::unique_ptr<CadItem>& entity)
        {
            return entity != nullptr
                && !entity->m_excludedFromProcessing
                && entity->m_nativeEntity != nullptr
                && isExportSortableEntityType(entity->m_type);
        }
    );

    if (!hasProcessableEntity)
    {
        report.status = OperationStatus::InvalidInput;
        report.addDiagnostic(makeExportDiagnostic
        (
            context,
            DiagnosticCode::InvalidGeometry,
            DiagnosticSeverity::Error,
            QStringLiteral("PrepareDocumentForGCodeExport"),
            QStringLiteral("ValidateDocument"),
            QStringLiteral("废面区间过滤后没有可加工图元。"),
            QStringLiteral("No export-sortable entity remains after exclusions")
        ));
        return report;
    }

    if (!hasCompleteProcessOrderForExport(generationMode))
    {
        report.addDiagnostic(makeExportDiagnostic
        (
            context,
            DiagnosticCode::MissingProcessOrder,
            DiagnosticSeverity::Notice,
            QStringLiteral("PrepareDocumentForGCodeExport"),
            QStringLiteral("EnsureProcessOrder"),
            QStringLiteral("检测到当前图元尚未完成排序，导出前自动执行智能排序。")
        ));

        const bool sorted = generationMode == GGenerator::GenerationMode::Mode3D
            ? smartSortEntities3D()
            : smartSortEntities();

        if (!sorted)
        {
            report.status = OperationStatus::Failed;
            report.addDiagnostic(makeExportDiagnostic
            (
                context,
                DiagnosticCode::MissingProcessOrder,
                DiagnosticSeverity::Error,
                QStringLiteral("PrepareDocumentForGCodeExport"),
                QStringLiteral("EnsureProcessOrder"),
                QStringLiteral("自动智能排序失败，G代码导出已取消。"),
                QStringLiteral("The existing smart sort operation returned false")
            ));
            return report;
        }
    }

    report.status = OperationStatus::Success;
    report.value = std::monostate{};
    return report;
}

bool Gcode_postprocessing_system::exportGCode
(
    GGenerator::GenerationMode generationMode,
    const QString& modeDisplayName
)
{
    const OperationContext context = createOperationContext(QStringLiteral("export-gcode"));
    UserInterfaceMessageSink userInterfaceSink
    (
        [this](const QString& message)
        {
            ui->openGLWidget->appendCommandMessage(message);
            ui->openGLWidget->refreshCommandPrompt();
        },
        [this](const QString& message, int durationMs)
        {
            statusBar()->showMessage(message, durationMs);
        },
        [this](const QString&, const QString& message)
        {
            QMessageBox::warning(this, QStringLiteral("导出G代码失败"), message);
        },
        true
    );
    DebugMessageSink debugSink;
    MessageCenter messageCenter;
    messageCenter.addSink(&userInterfaceSink);
    messageCenter.addSink(&debugSink);

    if (generationMode == GGenerator::GenerationMode::Mode3D && !ensureFeatureAvailable(AppFeature::FourAxisExport, QStringLiteral("4轴(绕A) G代码导出")))
    {
        return false;
    }

    const OperationReport prepareResult = prepareDocumentForGCodeExport(generationMode, context);
    messageCenter.publish(prepareResult);
    if (!prepareResult.succeeded())
    {
        return false;
    }

    GGenerator generator;
    generator.setDocument(&m_document);
    generator.setProfile(&m_activeProfile);
    generator.setGenerationMode(generationMode);
    generator.setRotaryTubeCenter
    (
        m_rotaryTubeSectionModel.centerY,
        m_rotaryTubeSectionModel.centerZ,
        m_rotaryTubeSectionModel.valid && m_rotaryTubeSectionModel.centerValid
    );

    QString resolvedExportPath;
    const QString exportFilters = QStringLiteral("NC 文件 (*.nc);;GCode 文件 (*.gcode);;文本文件 (*.txt)");

    if (loadUseDxfFileNameOnExport())
    {
        if (m_currentDocumentPath.trimmed().isEmpty())
        {
            messageCenter.publish(makeExportDiagnostic
            (
                context,
                DiagnosticCode::InvalidArgument,
                DiagnosticSeverity::Error,
                QStringLiteral("ExportGCode"),
                QStringLiteral("ResolveOutputPath"),
                QStringLiteral("当前没有可用的DXF文件名，无法自动使用dxf文件名导出。"),
                QStringLiteral("m_currentDocumentPath is empty")
            ));
            return false;
        }

        const QString suggestedFileName = QFileInfo(m_currentDocumentPath).completeBaseName().trimmed() + QStringLiteral(".nc");

        if (loadUseDefaultExportPath())
        {
            resolvedExportPath = defaultGCodeExportPathForCurrentDocument();
        }
        else
        {
            resolvedExportPath = QFileDialog::getSaveFileName
            (
                this,
                QStringLiteral("导出 G 代码"),
                suggestedFileName,
                exportFilters
            );

            if (resolvedExportPath.isEmpty())
            {
                return false;
            }
        }

        if (QFileInfo(resolvedExportPath).suffix().isEmpty())
        {
            resolvedExportPath.append(QStringLiteral(".nc"));
        }

        if (QFileInfo::exists(resolvedExportPath))
        {
            const QMessageBox::StandardButton overwrite = QMessageBox::question
            (
                this,
                QStringLiteral("目标文件已存在"),
                QStringLiteral("目标文件已存在：\n%1\n\n是否覆盖导出？").arg(QDir::toNativeSeparators(resolvedExportPath)),
                QMessageBox::Yes | QMessageBox::No,
                QMessageBox::No
            );

            if (overwrite != QMessageBox::Yes)
            {
                return false;
            }
        }

    }
    else
    {
        resolvedExportPath = QFileDialog::getSaveFileName
        (
            this,
            QStringLiteral("导出 G 代码"),
            loadUseDefaultExportPath() ? defaultGCodeExportPathForCurrentDocument() : QString(),
            exportFilters
        );

        if (resolvedExportPath.isEmpty())
        {
            return false;
        }

        if (QFileInfo(resolvedExportPath).suffix().isEmpty())
        {
            resolvedExportPath.append(QStringLiteral(".nc"));
        }

    }

    const OperationResult<QString> buildResult = generator.buildProgramText(context);
    messageCenter.publish(buildResult);
    if (!buildResult.succeeded() || !buildResult.value.has_value())
    {
        return false;
    }

    const OperationReport writeResult = generator.writeProgramText
    (
        resolvedExportPath,
        *buildResult.value,
        context
    );
    messageCenter.publish(writeResult);
    if (!writeResult.succeeded())
    {
        return false;
    }

    saveLastGCodeExportPath(resolvedExportPath);

    const QString resolvedModeDisplayName = modeDisplayName.trimmed().isEmpty()
        ? generationModeDisplayName(generationMode)
        : modeDisplayName;

    messageCenter.publish(makeExportDiagnostic
    (
        context,
        DiagnosticCode::None,
        DiagnosticSeverity::Notice,
        QStringLiteral("ExportGCode"),
        QStringLiteral("Complete"),
        QStringLiteral("G代码已导出（%1）。").arg(resolvedModeDisplayName),
        QString(),
        {
            { QStringLiteral("filePath"), resolvedExportPath },
            { QStringLiteral("statusMessage"), QStringLiteral("G代码导出完成（%1）").arg(resolvedModeDisplayName) },
            { QStringLiteral("statusDurationMs"), 5000 }
        }
    ));
    return true;
}

bool Gcode_postprocessing_system::removeDuplicateEntities(bool interactive)
{
    if (m_document.m_entities.empty())
    {
        const QString message = QStringLiteral("当前文档没有可去重的图元");
        ui->openGLWidget->appendCommandMessage(message);
        statusBar()->showMessage(message, 3000);
        return false;
    }

    QSet<QString> seenKeys;
    QVector<CadItem*> duplicates;

    for (auto it = m_document.m_entities.rbegin(); it != m_document.m_entities.rend(); ++it)
    {
        CadItem* item = it->get();

        if (item == nullptr || item->m_excludedFromProcessing || item->m_nativeEntity == nullptr)
        {
            continue;
        }

        const QString geometryKey = duplicateGeometryKey(item);

        if (geometryKey.isEmpty())
        {
            continue;
        }

        if (seenKeys.contains(geometryKey))
        {
            duplicates.push_back(item);
            continue;
        }

        seenKeys.insert(geometryKey);
    }

    if (duplicates.isEmpty())
    {
        const QString message = QStringLiteral("未发现完全重叠的同类型图元");
        ui->openGLWidget->appendCommandMessage(message);
        statusBar()->showMessage(message, 3000);
        return true;
    }

    std::reverse(duplicates.begin(), duplicates.end());

    if (!m_editer.deleteEntities(duplicates))
    {
        const QString message = QStringLiteral("删除重复图元时发生错误。");

        if (interactive)
        {
            QMessageBox::warning(this, QStringLiteral("去重失败"), message);
        }

        ui->openGLWidget->appendCommandMessage(QStringLiteral("去重失败：%1").arg(message));
        statusBar()->showMessage(QStringLiteral("去重失败"), 4000);
        return false;
    }

    ui->openGLWidget->clearSelection();
    ui->openGLWidget->appendCommandMessage(QStringLiteral("去重完成，删除重复图元 %1 个").arg(duplicates.size()));
    statusBar()->showMessage(QStringLiteral("去重完成，删除重复图元 %1 个").arg(duplicates.size()), 4000);
    return true;
}

bool Gcode_postprocessing_system::hasCompleteProcessOrderForExport(GGenerator::GenerationMode generationMode) const
{
    Q_UNUSED(generationMode);

    int sortableCount = 0;
    bool hasMissingOrder = false;
    QSet<int> assignedOrders;

    for (const std::unique_ptr<CadItem>& entity : m_document.m_entities)
    {
        const CadItem* item = entity.get();

        if (item == nullptr
            || item->m_excludedFromProcessing
            || item->m_nativeEntity == nullptr
            || !isExportSortableEntityType(item->m_type))
        {
            continue;
        }

        ++sortableCount;

        if (item->m_processOrder < 0)
        {
            hasMissingOrder = true;
            continue;
        }

        if (assignedOrders.contains(item->m_processOrder))
        {
            return false;
        }

        assignedOrders.insert(item->m_processOrder);
    }

    if (sortableCount <= 1)
    {
        return true;
    }

    return !hasMissingOrder && assignedOrders.size() == sortableCount;
}

QString Gcode_postprocessing_system::generationModeDisplayName(GGenerator::GenerationMode generationMode) const
{
    return generationMode == GGenerator::GenerationMode::Mode3D
        ? QStringLiteral("4轴(绕A)")
        : QStringLiteral("3轴");
}

QString Gcode_postprocessing_system::loadLastGCodeExportPath() const
{
    QSettings settings(QStringLiteral("GCodePostProcessingSystem"), QStringLiteral("GCodePostProcessingSystem"));
    return settings.value(QStringLiteral("gcode/lastExportPath"), QString()).toString().trimmed();
}

void Gcode_postprocessing_system::saveLastGCodeExportPath(const QString& filePath) const
{
    if (filePath.trimmed().isEmpty())
    {
        return;
    }

    QSettings settings(QStringLiteral("GCodePostProcessingSystem"), QStringLiteral("GCodePostProcessingSystem"));
    settings.setValue(QStringLiteral("gcode/lastExportPath"), filePath.trimmed());
}
