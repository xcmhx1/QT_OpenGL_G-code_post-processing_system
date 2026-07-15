#include "platform/pch.h"

#include "desktop/Gcode_postprocessing_system.h"

#include "ui/dialogs/CadBitmapImportDialog.h"
#include "infrastructure/image/CadBitmapVectorizer.h"

#include <QApplication>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QMessageBox>
#include <QProgressDialog>
#include <QSettings>
#include <QStandardPaths>

namespace
{
    bool hasSuffix(const QString& filePath, std::initializer_list<const char*> suffixes)
    {
        const QString normalizedPath = QDir::fromNativeSeparators(filePath).trimmed().toLower();

        for (const char* suffix : suffixes)
        {
            if (normalizedPath.endsWith(QLatin1String(suffix)))
            {
                return true;
            }
        }

        return false;
    }

    bool isCadVectorFile(const QString& filePath)
    {
        return hasSuffix(filePath, { ".dxf", ".dwg" });
    }

    bool isBitmapFile(const QString& filePath)
    {
        return hasSuffix(filePath, { ".bmp", ".png", ".jpg", ".jpeg" });
    }
}

bool Gcode_postprocessing_system::importCadFile(const QString& filePath)
{
    if (filePath.isEmpty())
    {
        return false;
    }

    saveLastImportDirectory(filePath);

    if (isCadVectorFile(filePath))
    {
        return importDxfFile(filePath);
    }

    if (isBitmapFile(filePath))
    {
        return importBitmapFile(filePath);
    }

    QMessageBox::warning(this, QStringLiteral("导入失败"), QStringLiteral("当前不支持该文件类型: %1").arg(QFileInfo(filePath).suffix()));
    return false;
}

bool Gcode_postprocessing_system::importDxfFile(const QString& filePath)
{
    m_editer.clearHistory();
    m_processState.clear();
    invalidateCurrentProcessPlan();
    m_document.readDxfDocument(filePath);
    m_rotaryTubeSectionModel = RotaryTubeSectionModel();
    m_currentDocumentPath = ensureDxfSuffix(filePath);
    ui->openGLWidget->setDocument(&m_document);
    ui->openGLWidget->clearSelection();
    runDxfImportPostProcessing();
    ui->openGLWidget->appendCommandMessage(QStringLiteral("已导入文件: %1").arg(QFileInfo(filePath).fileName()));
    ui->openGLWidget->refreshCommandPrompt();
    statusBar()->showMessage(QStringLiteral("已导入: %1").arg(QFileInfo(filePath).fileName()), 5000);

    if (m_document.m_entities.empty())
    {
        QMessageBox::warning(this, QStringLiteral("导入结果"), QStringLiteral("文件已读取，但未生成可显示的 CAD 图元。"));
    }

    syncToolPanelState();
    return true;
}

void Gcode_postprocessing_system::runDxfImportPostProcessing()
{
    auto reportFailure = [this](const QString& message)
    {
        ui->openGLWidget->appendCommandMessage(message);
        statusBar()->showMessage(message, 5000);
    };

    if (loadAutoDeduplicateOnImport() && !removeDuplicateEntities(false))
    {
        reportFailure(QStringLiteral("导入后自动去重未完成，文件仍已正常导入。"));
    }

    if (loadAutoRecognizeRotaryTubeSectionOnImport() && !recognizeRotaryTubeSection(false))
    {
        reportFailure(QStringLiteral("导入后未能自动识别方管垂直截面，文件仍已正常导入。"));
    }

    if (loadAutoRecognizeRotaryEndCutsOnImport())
    {
        if (!m_rotaryTubeSectionModel.valid)
        {
            reportFailure(QStringLiteral("方管垂直截面未识别，已跳过自动加工断面识别。"));
        }
        else if (!recognizeAllRotaryEndCuts(false))
        {
            reportFailure(QStringLiteral("导入后未识别到有效加工断面，文件仍已正常导入。"));
        }
    }

    if (loadAutoRemoveInternalPathsOnImport())
    {
        if (!m_rotaryTubeSectionModel.valid)
        {
            reportFailure(QStringLiteral("方管垂直截面未识别，已跳过导入后内部线条清理。"));
        }
        else if (!removeInternalMachiningPaths(false))
        {
            reportFailure(QStringLiteral("导入后内部线条清理未完成，文件仍已正常导入。"));
        }
    }

    refreshWasteProcessingExclusions();
    syncMachiningSettingsState();
}

bool Gcode_postprocessing_system::importBitmapFile(const QString& filePath)
{
    if (!ensureFeatureAvailable(AppFeature::BitmapImport, QStringLiteral("位图导入")))
    {
        return false;
    }

    CadBitmapImportDialog dialog(filePath, this);

    if (!dialog.isReady())
    {
        QMessageBox::warning(this, QStringLiteral("位图导入失败"), dialog.errorMessage());
        return false;
    }

    if (dialog.exec() != QDialog::Accepted)
    {
        return false;
    }

    const CadBitmapImportOptions importOptions = dialog.options();
    CadBitmapImportResult importResult;
    QString errorMessage;
    QProgressDialog progressDialog(QStringLiteral("正在矢量化位图..."), QStringLiteral("取消"), 0, 100, this);
    progressDialog.setWindowTitle(QStringLiteral("位图导入"));
    progressDialog.setWindowModality(Qt::ApplicationModal);
    progressDialog.setMinimumDuration(300);

    QApplication::setOverrideCursor(Qt::WaitCursor);
    const bool vectorizeSuccess = CadBitmapVectorizer::vectorize
    (
        dialog.sourceImage(),
        importOptions,
        importResult,
        &errorMessage,
        [&progressDialog](int current, int total)
        {
            if (total > 0)
            {
                progressDialog.setRange(0, total);
                progressDialog.setValue(current);
            }
            else
            {
                progressDialog.setRange(0, 0);
            }

            QApplication::processEvents();
            return !progressDialog.wasCanceled();
        }
    );
    QApplication::restoreOverrideCursor();
    progressDialog.setValue(progressDialog.maximum());

    if (!vectorizeSuccess)
    {
        QMessageBox::warning(this, QStringLiteral("位图导入失败"), errorMessage);
        return false;
    }

    const bool replaceExisting = importOptions.importMode == CadBitmapImportMode::ReplaceDocument;
    m_editer.clearHistory();

    const int appendedCount = m_document.appendEntities(std::move(importResult.entities), replaceExisting);

    if (replaceExisting)
    {
        m_processState.clear();
        invalidateCurrentProcessPlan();
        m_rotaryTubeSectionModel = RotaryTubeSectionModel();
        m_currentDocumentPath.clear();
        ui->openGLWidget->clearSelection();
    }

    if (importOptions.autoFitScene)
    {
        ui->openGLWidget->fitScene();
    }

    ui->openGLWidget->appendCommandMessage
    (
        QStringLiteral("位图导入完成: %1，图层 %2，%3")
            .arg(QFileInfo(filePath).fileName())
            .arg(importOptions.layerName)
            .arg(importResult.summaryText)
    );
    ui->openGLWidget->refreshCommandPrompt();

    if (appendedCount <= 0)
    {
        QMessageBox::warning(this, QStringLiteral("位图导入结果"), QStringLiteral("位图处理完成，但没有生成可显示的 CAD 图元。"));
        return false;
    }

    statusBar()->showMessage
    (
        QStringLiteral("位图已导入: %1，新增实体 %2").arg(QFileInfo(filePath).fileName()).arg(appendedCount),
        5000
    );

    syncToolPanelState();
    return true;
}

bool Gcode_postprocessing_system::saveCurrentDocument()
{
    QString filePath = m_currentDocumentPath.trimmed();

    if (filePath.isEmpty())
    {
        filePath = QFileDialog::getSaveFileName
        (
            this,
            QStringLiteral("保存DXF文件"),
            defaultDxfPathForCurrentDocument(),
            QStringLiteral("DXF 文件 (*.dxf)")
        );

        if (filePath.isEmpty())
        {
            return false;
        }
    }

    filePath = ensureDxfSuffix(filePath);

    if (!writeDocumentToDxf(filePath, true, false))
    {
        return false;
    }

    ui->openGLWidget->appendCommandMessage(QStringLiteral("文件已保存: %1").arg(QFileInfo(filePath).fileName()));
    ui->openGLWidget->refreshCommandPrompt();
    statusBar()->showMessage(QStringLiteral("保存完成: %1").arg(QFileInfo(filePath).fileName()), 5000);
    return true;
}

bool Gcode_postprocessing_system::exportDxfDocument(bool safeMode)
{
    QString filePath = QFileDialog::getSaveFileName
    (
        this,
        safeMode ? QStringLiteral("导出为DXF（安全模式）") : QStringLiteral("导出为DXF"),
        defaultDxfPathForCurrentDocument(),
        QStringLiteral("DXF 文件 (*.dxf)")
    );

    if (filePath.isEmpty())
    {
        return false;
    }

    filePath = ensureDxfSuffix(filePath);

    if (!writeDocumentToDxf(filePath, false, safeMode))
    {
        return false;
    }

    ui->openGLWidget->appendCommandMessage
    (
        safeMode
            ? QStringLiteral("文件已安全导出: %1").arg(QFileInfo(filePath).fileName())
            : QStringLiteral("文件已导出: %1").arg(QFileInfo(filePath).fileName())
    );
    ui->openGLWidget->refreshCommandPrompt();
    statusBar()->showMessage
    (
        safeMode
            ? QStringLiteral("安全导出完成: %1").arg(QFileInfo(filePath).fileName())
            : QStringLiteral("导出完成: %1").arg(QFileInfo(filePath).fileName()),
        5000
    );
    return true;
}

QString Gcode_postprocessing_system::defaultImportPath() const
{
    if (!loadUseDefaultImportPath())
    {
        return QString();
    }

    const QString lastImportDirectory = loadLastImportDirectory();

    if (!lastImportDirectory.isEmpty() && QDir(lastImportDirectory).exists())
    {
        return lastImportDirectory;
    }

    QString baseDirectory = QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation);

    if (baseDirectory.trimmed().isEmpty())
    {
        baseDirectory = QDir::homePath();
    }

    return baseDirectory;
}

bool Gcode_postprocessing_system::writeDocumentToDxf(const QString& filePath, bool updateCurrentPath, bool safeMode)
{
    if (filePath.trimmed().isEmpty())
    {
        return false;
    }

    const QString normalizedPath = ensureDxfSuffix(filePath);
    const bool writeSuccess = updateCurrentPath
        ? m_document.saveDxfDocument(normalizedPath, safeMode)
        : m_document.eportDxfDocument(normalizedPath, safeMode);

    if (!writeSuccess)
    {
        QMessageBox::warning(this, QStringLiteral("文件操作失败"), QStringLiteral("写入 DXF 文件失败: %1").arg(normalizedPath));
        return false;
    }

    if (updateCurrentPath)
    {
        m_currentDocumentPath = normalizedPath;
    }

    return true;
}

QString Gcode_postprocessing_system::ensureDxfSuffix(const QString& filePath) const
{
    const QString trimmedPath = filePath.trimmed();

    if (trimmedPath.isEmpty())
    {
        return QString();
    }

    if (trimmedPath.endsWith(QStringLiteral(".dxf"), Qt::CaseInsensitive))
    {
        return trimmedPath;
    }

    const QFileInfo fileInfo(trimmedPath);

    if (!fileInfo.suffix().isEmpty())
    {
        return fileInfo.absolutePath()
            + QLatin1Char('/')
            + fileInfo.completeBaseName()
            + QStringLiteral(".dxf");
    }

    return trimmedPath + QStringLiteral(".dxf");
}

QString Gcode_postprocessing_system::defaultDxfPathForCurrentDocument() const
{
    if (!m_currentDocumentPath.trimmed().isEmpty())
    {
        return ensureDxfSuffix(m_currentDocumentPath);
    }

    return QStringLiteral("untitled.dxf");
}

QString Gcode_postprocessing_system::loadLastImportDirectory() const
{
    QSettings settings(QStringLiteral("GCodePostProcessingSystem"), QStringLiteral("GCodePostProcessingSystem"));
    return settings.value(QStringLiteral("ui/lastImportDirectory"), QString()).toString().trimmed();
}

void Gcode_postprocessing_system::saveLastImportDirectory(const QString& filePath) const
{
    const QFileInfo fileInfo(filePath);
    const QString directoryPath = fileInfo.exists() ? fileInfo.absolutePath() : QFileInfo(QDir::fromNativeSeparators(filePath)).absolutePath();

    if (directoryPath.trimmed().isEmpty())
    {
        return;
    }

    QSettings settings(QStringLiteral("GCodePostProcessingSystem"), QStringLiteral("GCodePostProcessingSystem"));
    settings.setValue(QStringLiteral("ui/lastImportDirectory"), directoryPath);
}
