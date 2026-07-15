#pragma once

#include <QString>
#include <QStringList>

class GProfilePathStore
{
public:
    static QStringList directories();
    static void saveDirectories(const QStringList& directories);
    static void recordDirectory(const QString& directoryPath);
    static QString lastDirectory();
    static QString profileIdForFile(const QString& filePath);

private:
    static QString normalizedDirectory(const QString& directoryPath);
};
