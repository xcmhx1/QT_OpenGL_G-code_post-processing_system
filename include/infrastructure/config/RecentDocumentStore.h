#pragma once

#include <QString>
#include <QStringList>

class RecentDocumentStore
{
public:
    static constexpr int MaximumCount = 15;

    QStringList load() const;
    QStringList loadAndPrune() const;

    void add(const QString& filePath) const;
    void remove(const QString& filePath) const;
    void clear() const;

private:
    QString normalizePath(const QString& filePath) const;
    QString comparisonKey(const QString& filePath) const;
    void save(const QStringList& paths) const;
};
