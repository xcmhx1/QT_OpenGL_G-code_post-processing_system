#include "pch.h"

#include "Gcode_postprocessing_system.h"

#include <QFileDialog>
#include <QFileInfo>
#include <QMessageBox>
#include <QStatusBar>

Gcode_postprocessing_system::Gcode_postprocessing_system(QWidget* parent)
    : QMainWindow(parent)
    , ui(new Ui::Gcode_postprocessing_systemClass())
{
    ui->setupUi(this);
    ui->openGLWidget->setDocument(&m_document);

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

            m_document.readDxfDocument(filePath);
            ui->openGLWidget->setDocument(&m_document);
            statusBar()->showMessage(QStringLiteral("已导入: %1").arg(QFileInfo(filePath).fileName()), 5000);

            if (m_document.m_entities.isEmpty())
            {
                QMessageBox::warning(this, QStringLiteral("导入结果"), QStringLiteral("文件已读取，但未生成可显示的 CAD 图元。"));
            }
        }
    );
}

Gcode_postprocessing_system::~Gcode_postprocessing_system()
{
    delete ui;
}
