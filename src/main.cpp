#include "Gcode_postprocessing_system.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    Gcode_postprocessing_system window;
    window.show();
    return app.exec();
}
