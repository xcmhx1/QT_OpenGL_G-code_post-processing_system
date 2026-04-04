#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_Gcode_postprocessing_system.h"

QT_BEGIN_NAMESPACE
namespace Ui { class Gcode_postprocessing_systemClass; };
QT_END_NAMESPACE

class Gcode_postprocessing_system : public QMainWindow
{
    Q_OBJECT

public:
    Gcode_postprocessing_system(QWidget *parent = nullptr);
    ~Gcode_postprocessing_system();

private:
    Ui::Gcode_postprocessing_systemClass *ui;
};

