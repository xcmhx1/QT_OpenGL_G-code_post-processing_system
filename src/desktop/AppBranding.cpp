#include "platform/pch.h"

#include "desktop/AppBranding.h"

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>

namespace
{
    QString jsonStringValue(const QJsonObject& object, const QString& key, const QString& fallback = QString())
    {
        const QJsonValue value = object.value(key);
        return value.isString() ? value.toString().trimmed() : fallback;
    }

}

AppBranding AppBranding::load()
{
    AppBranding branding;
    const QString filePath = QDir(QCoreApplication::applicationDirPath()).absoluteFilePath(QStringLiteral("branding.json"));
    QFile file(filePath);

    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        return branding;
    }

    const QJsonDocument document = QJsonDocument::fromJson(file.readAll());

    if (!document.isObject())
    {
        return branding;
    }

    const QJsonObject object = document.object();
    branding.m_companyName = jsonStringValue(object, QStringLiteral("companyName"), branding.m_companyName);
    branding.m_website = jsonStringValue(object, QStringLiteral("website"), branding.m_website);
    branding.m_supportText = jsonStringValue(object, QStringLiteral("support"), branding.m_supportText);
    branding.m_aboutText = jsonStringValue(object, QStringLiteral("about"), branding.m_aboutText);
    return branding;
}

QString AppBranding::applicationName() const
{
    return QStringLiteral("G-code Post Processing System");
}

QString AppBranding::companyName() const
{
    return m_companyName;
}

QString AppBranding::website() const
{
    return m_website;
}

QString AppBranding::supportText() const
{
    return m_supportText;
}

QString AppBranding::aboutText() const
{
    return m_aboutText;
}
