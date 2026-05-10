#include "pch.h"

#include "AppBranding.h"
#include "AppLicense.h"
#include "Gcode_postprocessing_system.h"

#include <QtWidgets/QApplication>
#include <QClipboard>
#include <QDateTime>
#include <QDialog>
#include <QDialogButtonBox>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QPushButton>
#include <QTextEdit>
#include <QVBoxLayout>

namespace
{
    QString buildLicenseRequestText()
    {
        const AppBranding branding = AppBranding::load();
        QJsonObject request;
        request.insert(QStringLiteral("type"), QStringLiteral("license-request"));
        request.insert(QStringLiteral("applicationName"), branding.applicationName());
        request.insert(QStringLiteral("requestedEdition"), QStringLiteral("pro"));
        request.insert(QStringLiteral("customer"), QStringLiteral("请填写客户名称"));
        request.insert(QStringLiteral("machineId"), AppLicense::currentMachineId());
        request.insert(QStringLiteral("generatedAt"), QDateTime::currentDateTimeUtc().toString(Qt::ISODate));
        return QString::fromUtf8(QJsonDocument(request).toJson(QJsonDocument::Indented));
    }

    int showLicenseRequestDialog()
    {
        QDialog dialog;
        dialog.setWindowTitle(QStringLiteral("授权申请"));
        dialog.resize(620, 420);

        QVBoxLayout* layout = new QVBoxLayout(&dialog);
        QLabel* label = new QLabel(QStringLiteral("请将下面内容完整复制并发送给软件提供方，用于生成 license.dat。"), &dialog);
        label->setWordWrap(true);
        layout->addWidget(label);

        QTextEdit* editor = new QTextEdit(&dialog);
        editor->setReadOnly(true);
        editor->setPlainText(buildLicenseRequestText());
        editor->selectAll();
        layout->addWidget(editor);

        QDialogButtonBox* buttons = new QDialogButtonBox(&dialog);
        QPushButton* copyButton = buttons->addButton(QStringLiteral("复制"), QDialogButtonBox::ActionRole);
        QPushButton* closeButton = buttons->addButton(QStringLiteral("关闭"), QDialogButtonBox::AcceptRole);
        layout->addWidget(buttons);

        QObject::connect(copyButton, &QPushButton::clicked, [&dialog, editor, copyButton]()
        {
            QApplication::clipboard()->setText(editor->toPlainText());
            copyButton->setText(QStringLiteral("已复制"));
            dialog.window()->setWindowTitle(QStringLiteral("授权申请 - 已复制"));
        });

        QObject::connect(closeButton, &QPushButton::clicked, &dialog, &QDialog::accept);

        dialog.exec();
        return 0;
    }
}

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);

    if (QApplication::arguments().contains(QStringLiteral("--license-request")))
    {
        return showLicenseRequestDialog();
    }

    Gcode_postprocessing_system window;
    window.show();
    return app.exec();
}
