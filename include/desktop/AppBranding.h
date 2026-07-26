#pragma once

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

private:
    QString m_companyName = QStringLiteral("G-code Post Processing");
    QString m_website;
    QString m_supportText;
    QString m_aboutText;
};
