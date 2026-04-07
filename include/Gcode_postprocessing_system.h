#pragma once

#include "CadCommandLineWidget.h"
#include "CadEditer.h"
#include "CadDocument.h"
#include "CadStatusPaneWidget.h"

#include <QtWidgets/QMainWindow>

#include "ui_Gcode_postprocessing_system.h"

QT_BEGIN_NAMESPACE
namespace Ui { class Gcode_postprocessing_systemClass; }
QT_END_NAMESPACE

class Gcode_postprocessing_system : public QMainWindow
{
    Q_OBJECT

public:
    Gcode_postprocessing_system(QWidget* parent = nullptr);
    ~Gcode_postprocessing_system();

private:
    bool importCadFile(const QString& filePath);
    bool importDxfFile(const QString& filePath);
    bool importBitmapFile(const QString& filePath);

private:
    Ui::Gcode_postprocessing_systemClass* ui = nullptr;
    CadCommandLineWidget* m_commandLineWidget = nullptr;
    CadStatusPaneWidget* m_statusPaneWidget = nullptr;
    CadEditer m_editer;
    CadDocument m_document;
};
