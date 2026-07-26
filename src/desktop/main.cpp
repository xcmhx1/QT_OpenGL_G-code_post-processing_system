#include "platform/pch.h"

#include "desktop/AppLicense.h"
#include "desktop/Gcode_postprocessing_system.h"

#include <QtWidgets/QApplication>
#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QGuiApplication>
#include <QIcon>

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
    const AppBranding branding = AppBranding::load();
    QCoreApplication::setApplicationName(branding.applicationName());
    QCoreApplication::setOrganizationName(QStringLiteral("G-code Post Processing"));
    QGuiApplication::setApplicationDisplayName(branding.applicationName());

    const QIcon applicationIcon(QStringLiteral(":/branding/G.svg"));
    if (applicationIcon.isNull())
    {
        qWarning("Failed to load application icon from :/branding/G.svg");
    }
    else
    {
        QApplication::setWindowIcon(applicationIcon);
    }

    if (QApplication::arguments().contains(QStringLiteral("--license-request")))
    {
        return saveMachineCodeFile();
    }

    Gcode_postprocessing_system window;
    window.show();
    return app.exec();
}
