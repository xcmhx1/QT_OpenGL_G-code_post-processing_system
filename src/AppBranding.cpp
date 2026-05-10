#include "pch.h"

#include "AppBranding.h"

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>

namespace
{
    QString jsonStringValue(const QJsonObject& object, const QString& key, const QString& fallback = QString())
    {
        const QJsonValue value = object.value(key);
        return value.isString() ? value.toString().trimmed() : fallback;
    }

    QString resolveRuntimePath(const QString& path)
    {
        if (path.trimmed().isEmpty())
        {
            return QString();
        }

        const QFileInfo fileInfo(path);

        if (fileInfo.isAbsolute())
        {
            return QDir::cleanPath(path);
        }

        return QDir(QCoreApplication::applicationDirPath()).absoluteFilePath(path);
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
    branding.m_applicationName = jsonStringValue(object, QStringLiteral("applicationName"), branding.m_applicationName);
    branding.m_companyName = jsonStringValue(object, QStringLiteral("companyName"), branding.m_companyName);
    branding.m_website = jsonStringValue(object, QStringLiteral("website"), branding.m_website);
    branding.m_supportText = jsonStringValue(object, QStringLiteral("support"), branding.m_supportText);
    branding.m_aboutText = jsonStringValue(object, QStringLiteral("about"), branding.m_aboutText);
    branding.m_windowTitleSuffix = jsonStringValue(object, QStringLiteral("windowTitleSuffix"), branding.m_windowTitleSuffix);
    branding.m_iconPath = resolveRuntimePath(jsonStringValue(object, QStringLiteral("iconPath"), branding.m_iconPath));
    return branding;
}

QString AppBranding::applicationName() const
{
    return m_applicationName;
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

QString AppBranding::windowTitleSuffix() const
{
    return m_windowTitleSuffix;
}

QString AppBranding::iconPath() const
{
    return m_iconPath;
}

QIcon AppBranding::icon() const
{
    if (m_iconPath.isEmpty() || !QFileInfo::exists(m_iconPath))
    {
        return QIcon();
    }

    return QIcon(m_iconPath);
}
