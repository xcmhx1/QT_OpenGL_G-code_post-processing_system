#include "pch.h"

#include "Gcode_postprocessing_system.h"

#include <QFileDialog>
#include <QFileInfo>
#include <QMessageBox>
#include <QStatusBar>
#include <QVBoxLayout>

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
                QStringLiteral("导入 CAD 文件"),
                QString(),
                QStringLiteral("CAD 文件 (*.dxf *.dwg)")
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
