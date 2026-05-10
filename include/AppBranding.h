#pragma once

#include <QIcon>
#include <QString>

class AppBranding
{
public:
    static AppBranding load();

    QString applicationName() const;
    QString companyName() const;
    QString website() const;
    QString supportText() const;
    QString aboutText() const;
    QString windowTitleSuffix() const;
    QString iconPath() const;
    QIcon icon() const;

private:
    QString m_applicationName = QStringLiteral("G-code Post Processing System");
    QString m_companyName = QStringLiteral("G-code Post Processing");
    QString m_website;
    QString m_supportText;
    QString m_aboutText;
    QString m_windowTitleSuffix;
    QString m_iconPath;
};
