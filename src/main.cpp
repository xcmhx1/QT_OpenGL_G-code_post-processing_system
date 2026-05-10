#include "pch.h"

#include "AppLicense.h"
#include "Gcode_postprocessing_system.h"

#include <QtWidgets/QApplication>
#include <QCoreApplication>
#include <QDir>
#include <QFile>

namespace
{
    int saveMachineCodeFile()
    {
        QFile file(QDir(QCoreApplication::applicationDirPath()).absoluteFilePath(QStringLiteral("机器码.txt")));

        if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
        {
            return 1;
        }

        file.write(AppLicense::currentMachineId().toUtf8());
        file.write("\n");
        return 0;
    }
}

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);

    if (QApplication::arguments().contains(QStringLiteral("--license-request")))
    {
        return saveMachineCodeFile();
    }

    Gcode_postprocessing_system window;
    window.show();
    return app.exec();
}
