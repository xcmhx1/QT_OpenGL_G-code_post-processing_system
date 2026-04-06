#include "pch.h"

#include "Gcode_postprocessing_system.h"
#include "CadBitmapImportDialog.h"
#include "CadBitmapVectorizer.h"

#include <QFileDialog>
#include <QFileInfo>
#include <QMessageBox>
#include <QStatusBar>
#include <QVBoxLayout>

namespace
{
    bool hasSuffix(const QString& filePath, std::initializer_list<const char*> suffixes)
    {
        for (const char* suffix : suffixes)
        {
            if (filePath.endsWith(QString::fromLatin1(suffix), Qt::CaseInsensitive))
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

Gcode_postprocessing_system::Gcode_postprocessing_system(QWidget* parent)
    : QMainWindow(parent)
    , ui(new Ui::Gcode_postprocessing_systemClass())
{
    ui->setupUi(this);

    m_commandLineWidget = new CadCommandLineWidget(this);
    m_statusPaneWidget = new CadStatusPaneWidget(this);

    if (QVBoxLayout* centralLayout = qobject_cast<QVBoxLayout*>(ui->centralWidget->layout()))
    {
        centralLayout->addWidget(m_commandLineWidget);
        centralLayout->addWidget(m_statusPaneWidget);
    }

    m_editer.setDocument(&m_document);
    ui->openGLWidget->setEditer(&m_editer);
    ui->openGLWidget->setDocument(&m_document);
    ui->openGLWidget->refreshCommandPrompt();

    connect(ui->openGLWidget, &CadViewer::hoveredWorldPositionChanged, m_statusPaneWidget, &CadStatusPaneWidget::setWorldPosition);
    connect(ui->openGLWidget, &CadViewer::commandPromptChanged, m_commandLineWidget, &CadCommandLineWidget::setPrompt);
    connect(ui->openGLWidget, &CadViewer::commandMessageAppended, m_commandLineWidget, &CadCommandLineWidget::appendMessage);
    connect
    (
        ui->openGLWidget,
        &CadViewer::fileDropRequested,
        this,
        [this](const QString& filePath)
        {
            importCadFile(filePath);
        }
    );

    connect
    (
        ui->action_File_Import_Dxf,
        &QAction::triggered,
        this,
        [this]()
        {
            const QString filePath = QFileDialog::getOpenFileName
            (
                this,
                QStringLiteral("导入文件"),
                QString(),
                QStringLiteral("支持文件 (*.dxf *.dwg *.bmp *.png *.jpg *.jpeg);;CAD 文件 (*.dxf *.dwg);;位图文件 (*.bmp *.png *.jpg *.jpeg)")
            );

            if (filePath.isEmpty())
            {
                return;
            }

            importCadFile(filePath);
        }
    );
}

Gcode_postprocessing_system::~Gcode_postprocessing_system()
{
    delete ui;
}

bool Gcode_postprocessing_system::importCadFile(const QString& filePath)
{
    if (filePath.isEmpty())
    {
        return false;
    }

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
    m_document.readDxfDocument(filePath);
    ui->openGLWidget->setDocument(&m_document);
    ui->openGLWidget->appendCommandMessage(QStringLiteral("已导入文件: %1").arg(QFileInfo(filePath).fileName()));
    ui->openGLWidget->refreshCommandPrompt();
    statusBar()->showMessage(QStringLiteral("已导入: %1").arg(QFileInfo(filePath).fileName()), 5000);

    if (m_document.m_entities.empty())
    {
        QMessageBox::warning(this, QStringLiteral("导入结果"), QStringLiteral("文件已读取，但未生成可显示的 CAD 图元。"));
    }

    return true;
}

bool Gcode_postprocessing_system::importBitmapFile(const QString& filePath)
{
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

    CadBitmapImportResult importResult;
    QString errorMessage;

    if (!CadBitmapVectorizer::vectorize(dialog.sourceImage(), dialog.options(), importResult, &errorMessage))
    {
        QMessageBox::warning(this, QStringLiteral("位图导入失败"), errorMessage);
        return false;
    }

    const bool replaceExisting = dialog.options().importMode == CadBitmapImportMode::ReplaceDocument;
    m_editer.clearHistory();

    const int appendedCount = m_document.appendEntities(std::move(importResult.entities), replaceExisting);
    ui->openGLWidget->setDocument(&m_document);
    ui->openGLWidget->appendCommandMessage
    (
        QStringLiteral("位图导入完成: %1，%2")
            .arg(QFileInfo(filePath).fileName())
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

    return true;
}
