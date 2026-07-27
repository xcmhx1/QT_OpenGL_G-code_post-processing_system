#include "platform/pch.h"

#include "infrastructure/config/RecentDocumentStore.h"

#include <QDir>
#include <QFileInfo>
#include <QSet>
#include <QSettings>

namespace
{
    constexpr const char* kSettingsOrganization = "GCodePostProcessingSystem";
    constexpr const char* kSettingsApplication = "GCodePostProcessingSystem";
    constexpr const char* kSettingsGroup = "RecentDocuments";
    constexpr const char* kSettingsKey = "Paths";

    bool isSupportedCadPath(const QString& filePath)
    {
        const QString suffix = QFileInfo(filePath).suffix();
        return suffix.compare(QStringLiteral("dxf"), Qt::CaseInsensitive) == 0
            || suffix.compare(QStringLiteral("dwg"), Qt::CaseInsensitive) == 0;
    }

    QStringList readStoredPaths()
    {
        QSettings settings
        (
            QString::fromLatin1(kSettingsOrganization),
            QString::fromLatin1(kSettingsApplication)
        );
        settings.beginGroup(QString::fromLatin1(kSettingsGroup));
        const QStringList paths =
            settings.value(QString::fromLatin1(kSettingsKey)).toStringList();
        settings.endGroup();
        return paths;
    }
}

QStringList RecentDocumentStore::load() const
{
    QStringList paths;
    QSet<QString> seenPaths;

    for (const QString& storedPath : readStoredPaths())
    {
        const QString normalizedPath = normalizePath(storedPath);
        if (normalizedPath.isEmpty() || !isSupportedCadPath(normalizedPath))
        {
            continue;
        }

        const QString key = comparisonKey(normalizedPath);
        if (key.isEmpty() || seenPaths.contains(key))
        {
            continue;
        }

        seenPaths.insert(key);
        paths.append(normalizedPath);
        if (paths.size() >= MaximumCount)
        {
            break;
        }
    }

    return paths;
}

QStringList RecentDocumentStore::loadAndPrune() const
{
    const QStringList storedPaths = readStoredPaths();
    const QStringList loadedPaths = load();
    QStringList existingPaths;

    for (const QString& path : loadedPaths)
    {
        if (QFileInfo::exists(path))
        {
            existingPaths.append(path);
        }
    }

    if (existingPaths != storedPaths)
    {
        save(existingPaths);
    }
    return existingPaths;
}

void RecentDocumentStore::add(const QString& filePath) const
{
    const QString normalizedPath = normalizePath(filePath);
    if (normalizedPath.isEmpty() || !isSupportedCadPath(normalizedPath))
    {
        return;
    }

    const QString key = comparisonKey(normalizedPath);
    QStringList paths = load();
    for (int index = paths.size() - 1; index >= 0; --index)
    {
        if (comparisonKey(paths.at(index)) == key)
        {
            paths.removeAt(index);
        }
    }

    paths.prepend(normalizedPath);
    while (paths.size() > MaximumCount)
    {
        paths.removeLast();
    }
    save(paths);
}

void RecentDocumentStore::remove(const QString& filePath) const
{
    const QString key = comparisonKey(filePath);
    if (key.isEmpty())
    {
        return;
    }

    QStringList paths = load();
    bool changed = false;
    for (int index = paths.size() - 1; index >= 0; --index)
    {
        if (comparisonKey(paths.at(index)) == key)
        {
            paths.removeAt(index);
            changed = true;
        }
    }

    if (changed)
    {
        save(paths);
    }
}

void RecentDocumentStore::clear() const
{
    QSettings settings
    (
        QString::fromLatin1(kSettingsOrganization),
        QString::fromLatin1(kSettingsApplication)
    );
    settings.beginGroup(QString::fromLatin1(kSettingsGroup));
    settings.remove(QString::fromLatin1(kSettingsKey));
    settings.endGroup();
    settings.sync();
}

QString RecentDocumentStore::normalizePath(const QString& filePath) const
{
    const QString trimmedPath = filePath.trimmed();
    if (trimmedPath.isEmpty())
    {
        return QString();
    }

    const QFileInfo info(trimmedPath);
    const QString absolutePath = QDir::cleanPath(info.absoluteFilePath());
    if (absolutePath.isEmpty() || !QDir::isAbsolutePath(absolutePath))
    {
        return QString();
    }

    if (info.exists())
    {
        const QString canonicalPath = info.canonicalFilePath();
        if (!canonicalPath.isEmpty())
        {
            return QDir::cleanPath(canonicalPath);
        }
    }
    return absolutePath;
}

QString RecentDocumentStore::comparisonKey(const QString& filePath) const
{
    return normalizePath(filePath).toCaseFolded();
}

void RecentDocumentStore::save(const QStringList& paths) const
{
    QSettings settings
    (
        QString::fromLatin1(kSettingsOrganization),
        QString::fromLatin1(kSettingsApplication)
    );
    settings.beginGroup(QString::fromLatin1(kSettingsGroup));
    if (paths.isEmpty())
    {
        settings.remove(QString::fromLatin1(kSettingsKey));
    }
    else
    {
        settings.setValue(QString::fromLatin1(kSettingsKey), paths);
    }
    settings.endGroup();
    settings.sync();
}
