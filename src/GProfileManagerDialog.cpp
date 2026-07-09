#include "pch.h"

#include "GProfileManagerDialog.h"

#include "GProfile.h"
#include "GProfilePathStore.h"

#include <QAbstractItemView>
#include <QDir>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QListWidget>
#include <QMessageBox>
#include <QPushButton>
#include <QSplitter>
#include <QStyle>
#include <QToolButton>
#include <QTreeWidget>
#include <QVBoxLayout>

namespace
{
    constexpr int kProfilePathRole = Qt::UserRole;
}

GProfileManagerDialog::GProfileManagerDialog
(
    const QString& activeProfileId,
    const AppThemeColors& theme,
    QWidget* parent
)
    : QDialog(parent),
      m_activeProfileId(activeProfileId),
      m_theme(theme)
{
    setWindowTitle(QStringLiteral("G代码配置文件管理"));
    resize(920, 560);
    setMinimumSize(760, 460);
    buildUi();
    applyTheme();
    refreshDirectoryList();
    refreshProfileTree();
}

QString GProfileManagerDialog::selectedProfilePath() const
{
    return m_selectedProfilePath;
}

void GProfileManagerDialog::buildUi()
{
    QVBoxLayout* rootLayout = new QVBoxLayout(this);
    rootLayout->setContentsMargins(16, 16, 16, 14);
    rootLayout->setSpacing(12);

    QLabel* descriptionLabel = new QLabel
    (
        QStringLiteral("统一管理 G 代码配置文件及其检索目录。双击配置文件可直接使用。"),
        this
    );
    descriptionLabel->setProperty("secondaryText", true);
    rootLayout->addWidget(descriptionLabel);

    QSplitter* splitter = new QSplitter(Qt::Horizontal, this);
    splitter->setChildrenCollapsible(false);

    QWidget* profilePanel = new QWidget(splitter);
    profilePanel->setProperty("managerPanel", true);
    QVBoxLayout* profileLayout = new QVBoxLayout(profilePanel);
    profileLayout->setContentsMargins(12, 12, 12, 12);
    profileLayout->setSpacing(8);
    QLabel* profileTitle = new QLabel(QStringLiteral("配置文件"), profilePanel);
    profileTitle->setProperty("panelTitle", true);
    profileLayout->addWidget(profileTitle);

    m_profileTree = new QTreeWidget(profilePanel);
    m_profileTree->setColumnCount(2);
    m_profileTree->setHeaderLabels(QStringList() << QStringLiteral("名称 / 所在目录") << QStringLiteral("删除"));
    m_profileTree->setRootIsDecorated(true);
    m_profileTree->setAlternatingRowColors(true);
    m_profileTree->setSelectionMode(QAbstractItemView::SingleSelection);
    m_profileTree->header()->setSectionResizeMode(0, QHeaderView::Stretch);
    m_profileTree->header()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
    profileLayout->addWidget(m_profileTree, 1);

    QWidget* directoryPanel = new QWidget(splitter);
    directoryPanel->setProperty("managerPanel", true);
    QVBoxLayout* directoryLayout = new QVBoxLayout(directoryPanel);
    directoryLayout->setContentsMargins(12, 12, 12, 12);
    directoryLayout->setSpacing(8);
    QLabel* directoryTitle = new QLabel(QStringLiteral("检索路径"), directoryPanel);
    directoryTitle->setProperty("panelTitle", true);
    directoryLayout->addWidget(directoryTitle);

    m_directoryList = new QListWidget(directoryPanel);
    m_directoryList->setSelectionMode(QAbstractItemView::SingleSelection);
    directoryLayout->addWidget(m_directoryList, 1);

    QHBoxLayout* directoryButtonLayout = new QHBoxLayout();
    QPushButton* addDirectoryButton = new QPushButton(QStringLiteral("添加文件夹"), directoryPanel);
    QPushButton* removeDirectoryButton = new QPushButton(QStringLiteral("移除路径"), directoryPanel);
    directoryButtonLayout->addWidget(addDirectoryButton);
    directoryButtonLayout->addWidget(removeDirectoryButton);
    directoryLayout->addLayout(directoryButtonLayout);

    splitter->addWidget(profilePanel);
    splitter->addWidget(directoryPanel);
    splitter->setStretchFactor(0, 3);
    splitter->setStretchFactor(1, 2);
    rootLayout->addWidget(splitter, 1);

    QHBoxLayout* bottomLayout = new QHBoxLayout();
    m_statusLabel = new QLabel(this);
    m_statusLabel->setProperty("secondaryText", true);
    QPushButton* refreshButton = new QPushButton(QStringLiteral("刷新"), this);
    QPushButton* useButton = new QPushButton(QStringLiteral("使用"), this);
    QPushButton* closeButton = new QPushButton(QStringLiteral("关闭"), this);
    useButton->setDefault(true);
    bottomLayout->addWidget(m_statusLabel, 1);
    bottomLayout->addWidget(refreshButton);
    bottomLayout->addWidget(useButton);
    bottomLayout->addWidget(closeButton);
    rootLayout->addLayout(bottomLayout);

    connect(addDirectoryButton, &QPushButton::clicked, this, [this]() { addDirectory(); });
    connect(removeDirectoryButton, &QPushButton::clicked, this, [this]() { removeSelectedDirectory(); });
    connect(refreshButton, &QPushButton::clicked, this, [this]() { refreshProfileTree(); });
    connect(useButton, &QPushButton::clicked, this, [this]() { useSelectedProfile(); });
    connect(closeButton, &QPushButton::clicked, this, &QDialog::reject);
    connect(m_profileTree, &QTreeWidget::itemDoubleClicked, this, [this](QTreeWidgetItem* item, int)
    {
        if (!profilePathForItem(item).isEmpty())
        {
            m_profileTree->setCurrentItem(item);
            useSelectedProfile();
        }
    });
}

void GProfileManagerDialog::applyTheme()
{
    setPalette(m_theme.palette);
    setStyleSheet
    (
        QStringLiteral
        (
            "QDialog { background: %1; color: %2; }"
            "QWidget[managerPanel=\"true\"] { background: %3; border: 1px solid %4; border-radius: 5px; }"
            "QLabel[panelTitle=\"true\"] { color: %2; font-size: 13px; font-weight: 600; border: none; }"
            "QLabel[secondaryText=\"true\"] { color: %5; }"
            "QTreeWidget, QListWidget { background: %6; color: %2; border: 1px solid %4; border-radius: 3px; }"
            "QTreeWidget::item, QListWidget::item { min-height: 25px; }"
            "QPushButton, QToolButton { background: %3; color: %2; border: 1px solid %4; border-radius: 3px; padding: 5px 12px; }"
            "QPushButton:hover, QToolButton:hover { background: %7; }"
            "QPushButton:pressed, QToolButton:pressed { background: %8; }"
        )
        .arg(m_theme.windowBackground.name())
        .arg(m_theme.textPrimaryColor.name())
        .arg(m_theme.panelBackground.name())
        .arg(m_theme.borderColor.name())
        .arg(m_theme.textSecondaryColor.name())
        .arg(m_theme.surfaceBackground.name())
        .arg(m_theme.hoverBackgroundColor.name())
        .arg(m_theme.pressedBackgroundColor.name())
    );
}

void GProfileManagerDialog::refreshDirectoryList(const QString& preferredDirectory)
{
    const QString selectedDirectory = preferredDirectory.isEmpty() && m_directoryList->currentItem() != nullptr
        ? m_directoryList->currentItem()->text()
        : preferredDirectory;

    m_directoryList->clear();

    for (const QString& directoryPath : GProfilePathStore::directories())
    {
        QListWidgetItem* item = new QListWidgetItem
        (
            style()->standardIcon(QStyle::SP_DirIcon),
            QDir::toNativeSeparators(directoryPath),
            m_directoryList
        );
        item->setToolTip(QDir(directoryPath).exists()
            ? directoryPath
            : QStringLiteral("目录当前不可用: %1").arg(directoryPath));

        if (!QDir(directoryPath).exists())
        {
            item->setForeground(m_theme.textSecondaryColor);
        }

        if (directoryPath.compare(selectedDirectory, Qt::CaseInsensitive) == 0
            || QDir::toNativeSeparators(directoryPath).compare(selectedDirectory, Qt::CaseInsensitive) == 0)
        {
            m_directoryList->setCurrentItem(item);
        }
    }

    if (m_directoryList->currentItem() == nullptr && m_directoryList->count() > 0)
    {
        m_directoryList->setCurrentRow(0);
    }
}

void GProfileManagerDialog::refreshProfileTree()
{
    m_profileTree->clear();
    int profileCount = 0;

    for (const QString& directoryPath : GProfilePathStore::directories())
    {
        const QDir directory(directoryPath);
        const bool directoryAvailable = directory.exists();
        const QString directoryText = directoryAvailable
            ? QDir::toNativeSeparators(directoryPath)
            : QStringLiteral("%1  (目录不可用)").arg(QDir::toNativeSeparators(directoryPath));
        QTreeWidgetItem* directoryItem = new QTreeWidgetItem(m_profileTree, QStringList() << directoryText);
        directoryItem->setIcon(0, style()->standardIcon(QStyle::SP_DirIcon));
        directoryItem->setToolTip(0, directoryPath);

        if (!directoryAvailable)
        {
            directoryItem->setForeground(0, m_theme.textSecondaryColor);
            continue;
        }

        const QFileInfoList files = directory.entryInfoList
        (
            QStringList() << QStringLiteral("*.json"),
            QDir::Files | QDir::Readable,
            QDir::Name
        );
        int directoryProfileCount = 0;

        for (const QFileInfo& fileInfo : files)
        {
            QString errorMessage;
            const GProfile profile = GProfile::loadFromFile(fileInfo.absoluteFilePath(), &errorMessage);

            if (!errorMessage.trimmed().isEmpty())
            {
                continue;
            }

            const QString displayName = profile.profileName().trimmed().isEmpty()
                ? fileInfo.completeBaseName()
                : profile.profileName().trimmed();
            const bool active = GProfilePathStore::profileIdForFile(fileInfo.absoluteFilePath()).compare
            (
                m_activeProfileId,
                Qt::CaseInsensitive
            ) == 0;
            QTreeWidgetItem* profileItem = new QTreeWidgetItem
            (
                directoryItem,
                QStringList() << QStringLiteral("%1%2  (%3)")
                    .arg(active ? QStringLiteral("[当前] ") : QString())
                    .arg(displayName)
                    .arg(fileInfo.fileName())
            );
            profileItem->setData(0, kProfilePathRole, fileInfo.absoluteFilePath());
            profileItem->setIcon(0, style()->standardIcon(QStyle::SP_FileIcon));
            profileItem->setToolTip(0, fileInfo.absoluteFilePath());

            QToolButton* deleteButton = new QToolButton(m_profileTree);
            deleteButton->setText(QStringLiteral("X"));
            deleteButton->setToolTip(QStringLiteral("删除此配置文件"));
            deleteButton->setFixedSize(28, 24);
            m_profileTree->setItemWidget(profileItem, 1, deleteButton);
            connect(deleteButton, &QToolButton::clicked, this, [this, filePath = fileInfo.absoluteFilePath()]()
            {
                deleteProfileFile(filePath);
            });

            ++directoryProfileCount;
            ++profileCount;
        }

        if (directoryProfileCount == 0)
        {
            QTreeWidgetItem* emptyItem = new QTreeWidgetItem(directoryItem, QStringList() << QStringLiteral("未找到有效的 G代码配置"));
            emptyItem->setForeground(0, m_theme.textSecondaryColor);
            emptyItem->setDisabled(true);
        }

        directoryItem->setExpanded(true);
    }

    m_statusLabel->setText(QStringLiteral("共检索到 %1 个配置文件").arg(profileCount));
}

void GProfileManagerDialog::addDirectory()
{
    const QString directoryPath = QFileDialog::getExistingDirectory
    (
        this,
        QStringLiteral("添加 G 代码配置检索目录"),
        GProfilePathStore::lastDirectory()
    );

    if (directoryPath.isEmpty())
    {
        return;
    }

    GProfilePathStore::recordDirectory(directoryPath);
    refreshDirectoryList(directoryPath);
    refreshProfileTree();
}

void GProfileManagerDialog::removeSelectedDirectory()
{
    QListWidgetItem* selectedItem = m_directoryList->currentItem();

    if (selectedItem == nullptr)
    {
        return;
    }

    delete m_directoryList->takeItem(m_directoryList->row(selectedItem));
    GProfilePathStore::saveDirectories(directoryPathsFromList());
    refreshProfileTree();
}

void GProfileManagerDialog::deleteProfileFile(const QString& filePath)
{
    const QMessageBox::StandardButton answer = QMessageBox::question
    (
        this,
        QStringLiteral("删除配置文件"),
        QStringLiteral("确定永久删除配置文件吗？\n%1").arg(QDir::toNativeSeparators(filePath)),
        QMessageBox::Yes | QMessageBox::No,
        QMessageBox::No
    );

    if (answer != QMessageBox::Yes)
    {
        return;
    }

    if (!QFile::remove(filePath))
    {
        QMessageBox::warning(this, QStringLiteral("删除失败"), QStringLiteral("无法删除配置文件，请检查文件权限。"));
        return;
    }

    refreshProfileTree();
}

void GProfileManagerDialog::useSelectedProfile()
{
    const QString profilePath = profilePathForItem(m_profileTree->currentItem());

    if (profilePath.isEmpty())
    {
        m_statusLabel->setText(QStringLiteral("请先选择一个配置文件"));
        return;
    }

    m_selectedProfilePath = profilePath;
    accept();
}

QStringList GProfileManagerDialog::directoryPathsFromList() const
{
    QStringList paths;

    for (int index = 0; index < m_directoryList->count(); ++index)
    {
        paths.append(QDir::fromNativeSeparators(m_directoryList->item(index)->text()));
    }

    return paths;
}

QString GProfileManagerDialog::profilePathForItem(const QTreeWidgetItem* item) const
{
    return item == nullptr ? QString() : item->data(0, kProfilePathRole).toString();
}
