#include "pch.h"

#include "GProfilePathStore.h"

#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QSettings>

namespace
{
    const QString kOrganization = QStringLiteral("GCodePostProcessingSystem");
    const QString kApplication = QStringLiteral("GCodePostProcessingSystem");
    const QString kDirectoriesKey = QStringLiteral("gcode/profileSearchDirectories");
    const QString kLastDirectoryKey = QStringLiteral("gcode/lastProfileDirectory");
}

QStringList GProfilePathStore::directories()
{
    QSettings settings(kOrganization, kApplication);

    if (!settings.contains(kDirectoriesKey))
    {
        return QStringList() << QDir::cleanPath(QCoreApplication::applicationDirPath());
    }

    QStringList result;

    for (const QString& directory : settings.value(kDirectoriesKey).toStringList())
    {
        const QString normalized = normalizedDirectory(directory);

        if (normalized.isEmpty())
        {
            continue;
        }

        const bool duplicate = std::any_of(result.cbegin(), result.cend(), [&normalized](const QString& existing)
        {
            return existing.compare(normalized, Qt::CaseInsensitive) == 0;
        });

        if (!duplicate)
        {
            result.append(normalized);
        }
    }

    return result;
}

void GProfilePathStore::saveDirectories(const QStringList& directories)
{
    QStringList normalizedDirectories;

    for (const QString& directory : directories)
    {
        const QString normalized = normalizedDirectory(directory);

        if (normalized.isEmpty())
        {
            continue;
        }

        const bool duplicate = std::any_of(normalizedDirectories.cbegin(), normalizedDirectories.cend(), [&normalized](const QString& existing)
        {
            return existing.compare(normalized, Qt::CaseInsensitive) == 0;
        });

        if (!duplicate)
        {
            normalizedDirectories.append(normalized);
        }
    }

    QSettings settings(kOrganization, kApplication);
    settings.setValue(kDirectoriesKey, normalizedDirectories);

    const QString lastStoredDirectory = normalizedDirectory(settings.value(kLastDirectoryKey).toString());
    const bool lastDirectoryStillManaged = std::any_of
    (
        normalizedDirectories.cbegin(),
        normalizedDirectories.cend(),
        [&lastStoredDirectory](const QString& existing)
        {
            return existing.compare(lastStoredDirectory, Qt::CaseInsensitive) == 0;
        }
    );

    if (!lastDirectoryStillManaged)
    {
        settings.setValue
        (
            kLastDirectoryKey,
            normalizedDirectories.isEmpty() ? QString() : normalizedDirectories.first()
        );
    }
}

void GProfilePathStore::recordDirectory(const QString& directoryPath)
{
    const QString normalized = normalizedDirectory(directoryPath);

    if (normalized.isEmpty())
    {
        return;
    }

    QStringList storedDirectories = directories();
    const bool alreadyStored = std::any_of(storedDirectories.cbegin(), storedDirectories.cend(), [&normalized](const QString& existing)
    {
        return existing.compare(normalized, Qt::CaseInsensitive) == 0;
    });

    if (!alreadyStored)
    {
        storedDirectories.append(normalized);
        saveDirectories(storedDirectories);
    }

    QSettings settings(kOrganization, kApplication);
    settings.setValue(kLastDirectoryKey, normalized);
}

QString GProfilePathStore::lastDirectory()
{
    QSettings settings(kOrganization, kApplication);
    const QString stored = normalizedDirectory(settings.value(kLastDirectoryKey).toString());

    if (!stored.isEmpty())
    {
        return stored;
    }

    const QStringList storedDirectories = directories();
    return storedDirectories.isEmpty()
        ? QDir::cleanPath(QCoreApplication::applicationDirPath())
        : storedDirectories.first();
}

QString GProfilePathStore::profileIdForFile(const QString& filePath)
{
    const QString absolutePath = QDir::cleanPath(QFileInfo(filePath).absoluteFilePath());
    return QStringLiteral("file:%1").arg(QDir::toNativeSeparators(absolutePath));
}

QString GProfilePathStore::normalizedDirectory(const QString& directoryPath)
{
    const QString trimmed = directoryPath.trimmed();
    return trimmed.isEmpty() ? QString() : QDir::cleanPath(QFileInfo(trimmed).absoluteFilePath());
}
