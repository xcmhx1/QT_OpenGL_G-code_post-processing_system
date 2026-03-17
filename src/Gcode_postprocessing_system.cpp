#include "Gcode_postprocessing_system.h"

Gcode_postprocessing_system::Gcode_postprocessing_system(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::Gcode_postprocessing_systemClass())
{
    ui->setupUi(this);
}

Gcode_postprocessing_system::~Gcode_postprocessing_system()
{
    delete ui;
}

