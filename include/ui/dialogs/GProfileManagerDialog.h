#pragma once

#include "desktop/AppTheme.h"

#include <QDialog>
#include <QString>
#include <QStringList>

class QLabel;
class QListWidget;
class QTreeWidget;
class QTreeWidgetItem;

class GProfileManagerDialog : public QDialog
{
public:
    explicit GProfileManagerDialog
    (
        const QString& activeProfileId,
        const AppThemeColors& theme,
        QWidget* parent = nullptr
    );

    QString selectedProfilePath() const;

private:
    void buildUi();
    void applyTheme();
    void refreshDirectoryList(const QString& preferredDirectory = QString());
    void refreshProfileTree();
    void addDirectory();
    void removeSelectedDirectory();
    void deleteProfileFile(const QString& filePath);
    void useSelectedProfile();
    QStringList directoryPathsFromList() const;
    QString profilePathForItem(const QTreeWidgetItem* item) const;

private:
    QString m_activeProfileId;
    QString m_selectedProfilePath;
    AppThemeColors m_theme = buildAppThemeColors(AppThemeMode::Light);
    QTreeWidget* m_profileTree = nullptr;
    QListWidget* m_directoryList = nullptr;
    QLabel* m_statusLabel = nullptr;
};
