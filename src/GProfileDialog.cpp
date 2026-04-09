#include "pch.h"

#include "GProfileDialog.h"

#include <QColorDialog>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMessageBox>
#include <QPainter>
#include <QPlainTextEdit>
#include <QPixmap>
#include <QPushButton>
#include <QTabWidget>
#include <QVBoxLayout>

#include <algorithm>

namespace
{
    QStringList builtInColorKeys()
    {
        return
        {
            QStringLiteral("BYLAYER"),
            QStringLiteral("BYBLOCK"),
            QStringLiteral("ACI:1"),
            QStringLiteral("ACI:2"),
            QStringLiteral("ACI:3"),
            QStringLiteral("ACI:4"),
            QStringLiteral("ACI:5"),
            QStringLiteral("ACI:6"),
            QStringLiteral("ACI:7"),
            QStringLiteral("ACI:8"),
            QStringLiteral("ACI:9")
        };
    }

    QString blockText(const QPlainTextEdit* editor)
    {
        return editor != nullptr ? editor->toPlainText().trimmed() : QString();
    }

    void setBlockText(QPlainTextEdit* editor, const QString& text)
    {
        if (editor != nullptr)
        {
            editor->setPlainText(text);
        }
    }

    QColor colorFromAci(int colorIndex)
    {
        static const QRgb aciStandardColors[] =
        {
            qRgb(0, 0, 0),
            qRgb(255, 0, 0),
            qRgb(255, 255, 0),
            qRgb(0, 255, 0),
            qRgb(0, 255, 255),
            qRgb(0, 0, 255),
            qRgb(255, 0, 255),
            qRgb(255, 255, 255),
            qRgb(128, 128, 128),
            qRgb(192, 192, 192)
        };

        if (colorIndex >= 1 && colorIndex <= 9)
        {
            return QColor(aciStandardColors[colorIndex]);
        }

        return QColor();
    }

    QColor colorForColorKey(const QString& colorKey)
    {
        const QString normalizedKey = GProfile::normalizeColorKey(colorKey);

        if (normalizedKey == QStringLiteral("BYLAYER"))
        {
            return QColor(216, 220, 226);
        }

        if (normalizedKey == QStringLiteral("BYBLOCK"))
        {
            return QColor(92, 100, 112);
        }

        if (normalizedKey.startsWith(QStringLiteral("ACI:")))
        {
            bool ok = false;
            const int colorIndex = normalizedKey.mid(4).toInt(&ok);

            if (ok)
            {
                const QColor aciColor = colorFromAci(colorIndex);

                if (aciColor.isValid())
                {
                    return aciColor;
                }
            }
        }

        const QColor trueColor(normalizedKey);
        return trueColor.isValid() ? trueColor : QColor(Qt::white);
    }

    QIcon buildColorChipIcon(const QColor& color, const QColor& borderColor)
    {
        QPixmap pixmap(12, 12);
        pixmap.fill(Qt::transparent);

        QPainter painter(&pixmap);
        painter.setRenderHint(QPainter::Antialiasing, true);
        painter.setPen(QPen(borderColor, 1.0));
        painter.setBrush(color);
        painter.drawRect(QRectF(1.0, 1.0, 10.0, 10.0));
        return QIcon(pixmap);
    }

    QStringList mergedLayerKeys(const QStringList& availableLayerNames, const QMap<QString, GProfileCodeBlock>& layerBlocks)
    {
        QStringList orderedKeys;

        auto appendUniqueKey =
            [&orderedKeys](const QString& rawKey)
            {
                const QString normalizedKey = GProfile::normalizeLayerKey(rawKey);

                if (!normalizedKey.isEmpty() && !orderedKeys.contains(normalizedKey))
                {
                    orderedKeys.append(normalizedKey);
                }
            };

        for (const QString& layerName : availableLayerNames)
        {
            appendUniqueKey(layerName);
        }

        QStringList customLayerKeys = layerBlocks.keys();
        std::sort
        (
            customLayerKeys.begin(),
            customLayerKeys.end(),
            [](const QString& left, const QString& right)
            {
                return QString::compare(left, right, Qt::CaseInsensitive) < 0;
            }
        );

        for (const QString& layerName : customLayerKeys)
        {
            appendUniqueKey(layerName);
        }

        if (!orderedKeys.contains(QStringLiteral("0")))
        {
            orderedKeys.prepend(QStringLiteral("0"));
        }

        return orderedKeys;
    }

    bool isBuiltInColorKey(const QString& colorKey)
    {
        return builtInColorKeys().contains(GProfile::normalizeColorKey(colorKey));
    }
}

GProfileDialog::GProfileDialog
(
    const GProfile& profile,
    const QStringList& availableLayerNames,
    const QMap<QString, QColor>& availableLayerColors,
    const AppThemeColors& theme,
    QWidget* parent
)
    : QDialog(parent)
    , m_availableLayerNames(availableLayerNames)
    , m_availableLayerColors(availableLayerColors)
    , m_theme(theme)
{
    setObjectName(QStringLiteral("GProfileDialog"));
    setWindowTitle(QStringLiteral("G代码配置"));
    resize(980, 760);
    buildUi();
    applyTheme();
    applyProfile(profile);
}

GProfile GProfileDialog::profile() const
{
    return m_profile;
}

void GProfileDialog::accept()
{
    updateCurrentEntityTypeBlock();
    updateCurrentLayerRule();
    updateCurrentColorRule();
    m_profile = collectProfile();
    QDialog::accept();
}

void GProfileDialog::buildUi()
{
    QVBoxLayout* rootLayout = new QVBoxLayout(this);
    rootLayout->setContentsMargins(12, 12, 12, 12);
    rootLayout->setSpacing(8);

    QHBoxLayout* actionLayout = new QHBoxLayout();
    QPushButton* importButton = new QPushButton(QStringLiteral("导入JSON..."), this);
    QPushButton* exportButton = new QPushButton(QStringLiteral("导出JSON..."), this);
    QPushButton* resetButton = new QPushButton(QStringLiteral("恢复默认"), this);

    actionLayout->addWidget(importButton);
    actionLayout->addWidget(exportButton);
    actionLayout->addWidget(resetButton);
    actionLayout->addStretch(1);
    rootLayout->addLayout(actionLayout);

    QTabWidget* tabWidget = new QTabWidget(this);
    rootLayout->addWidget(tabWidget, 1);

    QWidget* fileTab = new QWidget(tabWidget);
    QVBoxLayout* fileLayout = new QVBoxLayout(fileTab);
    fileLayout->setContentsMargins(8, 8, 8, 8);
    fileLayout->setSpacing(8);

    QFormLayout* fileFormLayout = new QFormLayout();
    fileFormLayout->setContentsMargins(0, 0, 0, 0);
    fileFormLayout->setSpacing(8);

    m_profileNameEdit = new QLineEdit(fileTab);
    fileFormLayout->addRow(QStringLiteral("配置名称"), m_profileNameEdit);

    m_fileHeaderEdit = new QPlainTextEdit(fileTab);
    m_fileHeaderEdit->setMinimumHeight(120);
    fileFormLayout->addRow(QStringLiteral("文件头"), m_fileHeaderEdit);

    m_fileFooterEdit = new QPlainTextEdit(fileTab);
    m_fileFooterEdit->setMinimumHeight(120);
    fileFormLayout->addRow(QStringLiteral("文件尾"), m_fileFooterEdit);

    m_fileCommentEdit = new QPlainTextEdit(fileTab);
    m_fileCommentEdit->setMinimumHeight(80);
    fileFormLayout->addRow(QStringLiteral("说明"), m_fileCommentEdit);

    fileLayout->addLayout(fileFormLayout);
    tabWidget->addTab(fileTab, QStringLiteral("文件级"));

    QWidget* typeTab = new QWidget(tabWidget);
    QVBoxLayout* typeLayout = new QVBoxLayout(typeTab);
    typeLayout->setContentsMargins(8, 8, 8, 8);
    typeLayout->setSpacing(8);

    QFormLayout* typeFormLayout = new QFormLayout();
    typeFormLayout->setContentsMargins(0, 0, 0, 0);
    typeFormLayout->setSpacing(8);

    m_entityTypeComboBox = new QComboBox(typeTab);
    populateEntityTypeCombo();
    typeFormLayout->addRow(QStringLiteral("实体类型"), m_entityTypeComboBox);

    m_entityTypeHeaderEdit = new QPlainTextEdit(typeTab);
    m_entityTypeHeaderEdit->setMinimumHeight(120);
    typeFormLayout->addRow(QStringLiteral("类型头"), m_entityTypeHeaderEdit);

    m_entityTypeFooterEdit = new QPlainTextEdit(typeTab);
    m_entityTypeFooterEdit->setMinimumHeight(120);
    typeFormLayout->addRow(QStringLiteral("类型尾"), m_entityTypeFooterEdit);

    m_entityTypeCommentEdit = new QPlainTextEdit(typeTab);
    m_entityTypeCommentEdit->setMinimumHeight(80);
    typeFormLayout->addRow(QStringLiteral("说明"), m_entityTypeCommentEdit);

    typeLayout->addLayout(typeFormLayout);
    tabWidget->addTab(typeTab, QStringLiteral("实体类型"));

    QWidget* layerTab = new QWidget(tabWidget);
    QHBoxLayout* layerLayout = new QHBoxLayout(layerTab);
    layerLayout->setContentsMargins(8, 8, 8, 8);
    layerLayout->setSpacing(10);

    QVBoxLayout* layerListLayout = new QVBoxLayout();
    layerListLayout->setSpacing(6);
    layerListLayout->addWidget(new QLabel(QStringLiteral("图层规则"), layerTab));

    m_layerRuleListWidget = new QListWidget(layerTab);
    m_layerRuleListWidget->setMinimumWidth(220);
    layerListLayout->addWidget(m_layerRuleListWidget, 1);

    QHBoxLayout* layerButtonsLayout = new QHBoxLayout();
    QPushButton* addLayerButton = new QPushButton(QStringLiteral("新增"), layerTab);
    QPushButton* removeLayerButton = new QPushButton(QStringLiteral("删除"), layerTab);
    layerButtonsLayout->addWidget(addLayerButton);
    layerButtonsLayout->addWidget(removeLayerButton);
    layerListLayout->addLayout(layerButtonsLayout);

    layerLayout->addLayout(layerListLayout, 0);

    QWidget* layerEditorPanel = new QWidget(layerTab);
    QFormLayout* layerFormLayout = new QFormLayout(layerEditorPanel);
    layerFormLayout->setContentsMargins(0, 0, 0, 0);
    layerFormLayout->setSpacing(8);

    m_layerKeyEdit = new QLineEdit(layerEditorPanel);
    m_layerKeyEdit->setReadOnly(true);
    layerFormLayout->addRow(QStringLiteral("图层名"), m_layerKeyEdit);

    m_layerHeaderEdit = new QPlainTextEdit(layerEditorPanel);
    m_layerHeaderEdit->setMinimumHeight(120);
    layerFormLayout->addRow(QStringLiteral("图层头"), m_layerHeaderEdit);

    m_layerFooterEdit = new QPlainTextEdit(layerEditorPanel);
    m_layerFooterEdit->setMinimumHeight(120);
    layerFormLayout->addRow(QStringLiteral("图层尾"), m_layerFooterEdit);

    m_layerCommentEdit = new QPlainTextEdit(layerEditorPanel);
    m_layerCommentEdit->setMinimumHeight(80);
    layerFormLayout->addRow(QStringLiteral("说明"), m_layerCommentEdit);

    layerLayout->addWidget(layerEditorPanel, 1);
    tabWidget->addTab(layerTab, QStringLiteral("图层规则"));

    QWidget* colorTab = new QWidget(tabWidget);
    QHBoxLayout* colorLayout = new QHBoxLayout(colorTab);
    colorLayout->setContentsMargins(8, 8, 8, 8);
    colorLayout->setSpacing(10);

    QVBoxLayout* colorListLayout = new QVBoxLayout();
    colorListLayout->setSpacing(6);
    colorListLayout->addWidget(new QLabel(QStringLiteral("颜色规则"), colorTab));

    m_colorRuleListWidget = new QListWidget(colorTab);
    m_colorRuleListWidget->setMinimumWidth(220);
    colorListLayout->addWidget(m_colorRuleListWidget, 1);

    QHBoxLayout* colorButtonsLayout = new QHBoxLayout();
    QPushButton* addColorButton = new QPushButton(QStringLiteral("新增真彩色"), colorTab);
    QPushButton* removeColorButton = new QPushButton(QStringLiteral("清除/删除"), colorTab);
    colorButtonsLayout->addWidget(addColorButton);
    colorButtonsLayout->addWidget(removeColorButton);
    colorListLayout->addLayout(colorButtonsLayout);

    colorLayout->addLayout(colorListLayout, 0);

    QWidget* colorEditorPanel = new QWidget(colorTab);
    QFormLayout* colorFormLayout = new QFormLayout(colorEditorPanel);
    colorFormLayout->setContentsMargins(0, 0, 0, 0);
    colorFormLayout->setSpacing(8);

    m_colorKeyEdit = new QLineEdit(colorEditorPanel);
    m_colorKeyEdit->setReadOnly(true);
    colorFormLayout->addRow(QStringLiteral("颜色键"), m_colorKeyEdit);

    m_colorHeaderEdit = new QPlainTextEdit(colorEditorPanel);
    m_colorHeaderEdit->setMinimumHeight(120);
    colorFormLayout->addRow(QStringLiteral("颜色头"), m_colorHeaderEdit);

    m_colorFooterEdit = new QPlainTextEdit(colorEditorPanel);
    m_colorFooterEdit->setMinimumHeight(120);
    colorFormLayout->addRow(QStringLiteral("颜色尾"), m_colorFooterEdit);

    m_colorCommentEdit = new QPlainTextEdit(colorEditorPanel);
    m_colorCommentEdit->setMinimumHeight(80);
    colorFormLayout->addRow(QStringLiteral("说明"), m_colorCommentEdit);

    colorLayout->addWidget(colorEditorPanel, 1);
    tabWidget->addTab(colorTab, QStringLiteral("颜色规则"));

    m_buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    rootLayout->addWidget(m_buttonBox);

    connect(importButton, &QPushButton::clicked, this, [this]() { importProfileFromFile(); });
    connect(exportButton, &QPushButton::clicked, this, [this]() { exportProfileToFile(); });
    connect(resetButton, &QPushButton::clicked, this, [this]() { resetToDefaultProfile(); });
    connect(m_buttonBox, &QDialogButtonBox::accepted, this, &GProfileDialog::accept);
    connect(m_buttonBox, &QDialogButtonBox::rejected, this, &GProfileDialog::reject);

    connect
    (
        m_entityTypeComboBox,
        QOverload<int>::of(&QComboBox::currentIndexChanged),
        this,
        [this](int)
        {
            loadSelectedEntityTypeBlock();
        }
    );

    connect(m_entityTypeHeaderEdit, &QPlainTextEdit::textChanged, this, [this]() { updateCurrentEntityTypeBlock(); });
    connect(m_entityTypeFooterEdit, &QPlainTextEdit::textChanged, this, [this]() { updateCurrentEntityTypeBlock(); });
    connect(m_entityTypeCommentEdit, &QPlainTextEdit::textChanged, this, [this]() { updateCurrentEntityTypeBlock(); });

    connect
    (
        m_layerRuleListWidget,
        &QListWidget::currentItemChanged,
        this,
        [this](QListWidgetItem*, QListWidgetItem*)
        {
            loadSelectedLayerRule();
        }
    );

    connect(m_layerHeaderEdit, &QPlainTextEdit::textChanged, this, [this]() { updateCurrentLayerRule(); });
    connect(m_layerFooterEdit, &QPlainTextEdit::textChanged, this, [this]() { updateCurrentLayerRule(); });
    connect(m_layerCommentEdit, &QPlainTextEdit::textChanged, this, [this]() { updateCurrentLayerRule(); });

    connect(addLayerButton, &QPushButton::clicked, this, [this]() { addLayerRule(); });
    connect(removeLayerButton, &QPushButton::clicked, this, [this]() { removeSelectedLayerRule(); });

    connect
    (
        m_colorRuleListWidget,
        &QListWidget::currentItemChanged,
        this,
        [this](QListWidgetItem*, QListWidgetItem*)
        {
            loadSelectedColorRule();
        }
    );

    connect(m_colorHeaderEdit, &QPlainTextEdit::textChanged, this, [this]() { updateCurrentColorRule(); });
    connect(m_colorFooterEdit, &QPlainTextEdit::textChanged, this, [this]() { updateCurrentColorRule(); });
    connect(m_colorCommentEdit, &QPlainTextEdit::textChanged, this, [this]() { updateCurrentColorRule(); });

    connect(addColorButton, &QPushButton::clicked, this, [this]() { addCustomColorRule(); });
    connect(removeColorButton, &QPushButton::clicked, this, [this]() { removeSelectedColorRule(); });
}

void GProfileDialog::applyTheme()
{
    setPalette(m_theme.palette);
    setStyleSheet
    (
        QStringLiteral
        (
            "#GProfileDialog {"
            "background-color: %1;"
            "color: %2;"
            "}"
            "#GProfileDialog QLabel {"
            "color: %2;"
            "}"
            "#GProfileDialog QTabWidget::pane {"
            "border: 1px solid %3;"
            "background-color: %4;"
            "top: -1px;"
            "}"
            "#GProfileDialog QTabBar::tab {"
            "background-color: %5;"
            "color: %6;"
            "border: 1px solid %3;"
            "padding: 6px 14px;"
            "margin-right: 2px;"
            "}"
            "#GProfileDialog QTabBar::tab:selected {"
            "background-color: %4;"
            "color: %2;"
            "border-bottom-color: %4;"
            "}"
            "#GProfileDialog QLineEdit,"
            "#GProfileDialog QComboBox,"
            "#GProfileDialog QListWidget,"
            "#GProfileDialog QPlainTextEdit {"
            "background-color: %4;"
            "color: %2;"
            "border: 1px solid %7;"
            "selection-background-color: %8;"
            "selection-color: %9;"
            "padding: 4px;"
            "}"
            "#GProfileDialog QComboBox::drop-down {"
            "border-left: 1px solid %3;"
            "width: 20px;"
            "}"
            "#GProfileDialog QListWidget::item {"
            "padding: 3px 4px;"
            "}"
            "#GProfileDialog QListWidget::item:selected {"
            "background-color: %8;"
            "color: %9;"
            "}"
            "#GProfileDialog QPushButton,"
            "#GProfileDialog QDialogButtonBox QPushButton {"
            "background-color: %5;"
            "color: %2;"
            "border: 1px solid %3;"
            "padding: 5px 12px;"
            "min-height: 26px;"
            "}"
            "#GProfileDialog QPushButton:hover,"
            "#GProfileDialog QDialogButtonBox QPushButton:hover {"
            "background-color: %10;"
            "}"
            "#GProfileDialog QPushButton:pressed,"
            "#GProfileDialog QDialogButtonBox QPushButton:pressed {"
            "background-color: %11;"
            "}"
        )
        .arg(m_theme.windowBackground.name())
        .arg(m_theme.textPrimaryColor.name())
        .arg(m_theme.borderColor.name())
        .arg(m_theme.surfaceBackground.name())
        .arg(m_theme.panelBackground.name())
        .arg(m_theme.textSecondaryColor.name())
        .arg(m_theme.borderStrongColor.name())
        .arg(m_theme.accentColor.name())
        .arg(m_theme.accentTextColor.name())
        .arg(m_theme.hoverBackgroundColor.name())
        .arg(m_theme.pressedBackgroundColor.name())
    );
}

void GProfileDialog::applyProfile(const GProfile& profile)
{
    m_updatingUi = true;
    m_profile = profile;
    m_entityTypeBlocks = profile.entityTypeCodes();
    m_layerBlocks = profile.layerCodes();
    m_colorBlocks = profile.entityColorCodes();

    m_profileNameEdit->setText(profile.profileName());
    setBlockText(m_fileHeaderEdit, profile.fileCode().header);
    setBlockText(m_fileFooterEdit, profile.fileCode().footer);
    setBlockText(m_fileCommentEdit, profile.fileCode().comment);

    if (m_entityTypeComboBox->count() > 0)
    {
        m_entityTypeComboBox->setCurrentIndex(0);
    }

    loadSelectedEntityTypeBlock();
    refreshLayerRuleList();
    refreshColorRuleList();
    m_updatingUi = false;
}

GProfile GProfileDialog::collectProfile() const
{
    GProfile profile;
    profile.setRotaryAxisConfig(m_profile.rotaryAxisConfig());
    profile.setProfileName(m_profileNameEdit->text().trimmed());
    profile.setFileCode
    (
        {
            blockText(m_fileHeaderEdit),
            blockText(m_fileFooterEdit),
            blockText(m_fileCommentEdit)
        }
    );

    for (auto it = m_entityTypeBlocks.cbegin(); it != m_entityTypeBlocks.cend(); ++it)
    {
        if (hasMeaningfulContent(it.value()))
        {
            profile.setEntityTypeCode(it.key(), it.value());
        }
    }

    for (auto it = m_layerBlocks.cbegin(); it != m_layerBlocks.cend(); ++it)
    {
        if (hasMeaningfulContent(it.value()))
        {
            profile.setLayerCode(it.key(), it.value());
        }
    }

    for (auto it = m_colorBlocks.cbegin(); it != m_colorBlocks.cend(); ++it)
    {
        if (hasMeaningfulContent(it.value()))
        {
            profile.setEntityColorCode(it.key(), it.value());
        }
    }

    return profile;
}

void GProfileDialog::populateEntityTypeCombo()
{
    for (const QString& entityTypeKey : supportedEntityTypes())
    {
        m_entityTypeComboBox->addItem(displayNameForEntityType(entityTypeKey), entityTypeKey);
    }
}

void GProfileDialog::loadSelectedEntityTypeBlock()
{
    const QString entityTypeKey = currentEntityTypeKey();
    const GProfileCodeBlock codeBlock = m_entityTypeBlocks.value(entityTypeKey);

    m_updatingUi = true;
    setBlockText(m_entityTypeHeaderEdit, codeBlock.header);
    setBlockText(m_entityTypeFooterEdit, codeBlock.footer);
    setBlockText(m_entityTypeCommentEdit, codeBlock.comment);
    m_updatingUi = false;
}

void GProfileDialog::updateCurrentEntityTypeBlock()
{
    if (m_updatingUi)
    {
        return;
    }

    const QString entityTypeKey = currentEntityTypeKey();

    if (entityTypeKey.isEmpty())
    {
        return;
    }

    m_entityTypeBlocks.insert
    (
        entityTypeKey,
        {
            blockText(m_entityTypeHeaderEdit),
            blockText(m_entityTypeFooterEdit),
            blockText(m_entityTypeCommentEdit)
        }
    );
}

void GProfileDialog::refreshLayerRuleList(const QString& preferredLayerKey)
{
    const QString targetLayerKey = preferredLayerKey.isEmpty() ? currentLayerKey() : GProfile::normalizeLayerKey(preferredLayerKey);
    const QStringList layerKeys = mergedLayerKeys(m_availableLayerNames, m_layerBlocks);

    m_updatingUi = true;
    m_layerRuleListWidget->clear();

    for (const QString& layerKey : layerKeys)
    {
        const QColor chipColor = m_availableLayerColors.value(layerKey, QColor(214, 220, 228));
        QListWidgetItem* item = new QListWidgetItem(buildColorChipIcon(chipColor, m_theme.borderStrongColor), layerKey, m_layerRuleListWidget);
        item->setData(Qt::UserRole, layerKey);
    }

    int targetIndex = -1;

    for (int index = 0; index < m_layerRuleListWidget->count(); ++index)
    {
        if (m_layerRuleListWidget->item(index)->data(Qt::UserRole).toString() == targetLayerKey)
        {
            targetIndex = index;
            break;
        }
    }

    if (targetIndex >= 0)
    {
        m_layerRuleListWidget->setCurrentRow(targetIndex);
    }
    else if (m_layerRuleListWidget->count() > 0)
    {
        m_layerRuleListWidget->setCurrentRow(0);
    }

    m_updatingUi = false;
    loadSelectedLayerRule();
}

void GProfileDialog::loadSelectedLayerRule()
{
    const QString layerKey = currentLayerKey();
    const GProfileCodeBlock codeBlock = m_layerBlocks.value(layerKey);

    m_updatingUi = true;
    m_layerKeyEdit->setText(layerKey);
    setBlockText(m_layerHeaderEdit, codeBlock.header);
    setBlockText(m_layerFooterEdit, codeBlock.footer);
    setBlockText(m_layerCommentEdit, codeBlock.comment);
    m_updatingUi = false;
}

void GProfileDialog::updateCurrentLayerRule()
{
    if (m_updatingUi)
    {
        return;
    }

    const QString layerKey = currentLayerKey();

    if (layerKey.isEmpty())
    {
        return;
    }

    m_layerBlocks.insert
    (
        layerKey,
        {
            blockText(m_layerHeaderEdit),
            blockText(m_layerFooterEdit),
            blockText(m_layerCommentEdit)
        }
    );
}

void GProfileDialog::addLayerRule()
{
    bool accepted = false;
    const QString layerName = QInputDialog::getText
    (
        this,
        QStringLiteral("新增图层规则"),
        QStringLiteral("图层名"),
        QLineEdit::Normal,
        QString(),
        &accepted
    );

    if (!accepted)
    {
        return;
    }

    const QString normalizedLayerName = GProfile::normalizeLayerKey(layerName);

    if (normalizedLayerName.isEmpty())
    {
        return;
    }

    if (!m_availableLayerNames.contains(normalizedLayerName))
    {
        m_availableLayerNames.append(normalizedLayerName);
    }

    if (!m_layerBlocks.contains(normalizedLayerName))
    {
        m_layerBlocks.insert(normalizedLayerName, GProfileCodeBlock());
    }

    refreshLayerRuleList(normalizedLayerName);
}

void GProfileDialog::removeSelectedLayerRule()
{
    const QString layerKey = currentLayerKey();

    if (layerKey.isEmpty())
    {
        return;
    }

    m_layerBlocks.remove(layerKey);

    if (!m_availableLayerNames.contains(layerKey))
    {
        m_availableLayerColors.remove(layerKey);
    }

    refreshLayerRuleList(layerKey);
}

void GProfileDialog::refreshColorRuleList(const QString& preferredColorKey)
{
    const QString targetColorKey = preferredColorKey.isEmpty() ? currentColorKey() : GProfile::normalizeColorKey(preferredColorKey);
    QStringList colorKeys = supportedDefaultColorKeys();

    QStringList customColorKeys = m_colorBlocks.keys();
    std::sort
    (
        customColorKeys.begin(),
        customColorKeys.end(),
        [](const QString& left, const QString& right)
        {
            return QString::compare(left, right, Qt::CaseInsensitive) < 0;
        }
    );

    for (const QString& colorKey : customColorKeys)
    {
        const QString normalizedKey = GProfile::normalizeColorKey(colorKey);

        if (!colorKeys.contains(normalizedKey))
        {
            colorKeys.append(normalizedKey);
        }
    }

    m_updatingUi = true;
    m_colorRuleListWidget->clear();

    for (const QString& colorKey : colorKeys)
    {
        QListWidgetItem* item = new QListWidgetItem
        (
            buildColorChipIcon(colorForColorKey(colorKey), m_theme.borderStrongColor),
            displayNameForColorKey(colorKey),
            m_colorRuleListWidget
        );
        item->setData(Qt::UserRole, colorKey);
    }

    int targetIndex = -1;

    for (int index = 0; index < m_colorRuleListWidget->count(); ++index)
    {
        if (m_colorRuleListWidget->item(index)->data(Qt::UserRole).toString() == targetColorKey)
        {
            targetIndex = index;
            break;
        }
    }

    if (targetIndex >= 0)
    {
        m_colorRuleListWidget->setCurrentRow(targetIndex);
    }
    else if (m_colorRuleListWidget->count() > 0)
    {
        m_colorRuleListWidget->setCurrentRow(0);
    }

    m_updatingUi = false;
    loadSelectedColorRule();
}

void GProfileDialog::loadSelectedColorRule()
{
    const QString colorKey = currentColorKey();
    const GProfileCodeBlock codeBlock = m_colorBlocks.value(colorKey);

    m_updatingUi = true;
    m_colorKeyEdit->setText(colorKey);
    setBlockText(m_colorHeaderEdit, codeBlock.header);
    setBlockText(m_colorFooterEdit, codeBlock.footer);
    setBlockText(m_colorCommentEdit, codeBlock.comment);
    m_updatingUi = false;
}

void GProfileDialog::updateCurrentColorRule()
{
    if (m_updatingUi)
    {
        return;
    }

    const QString colorKey = currentColorKey();

    if (colorKey.isEmpty())
    {
        return;
    }

    m_colorBlocks.insert
    (
        colorKey,
        {
            blockText(m_colorHeaderEdit),
            blockText(m_colorFooterEdit),
            blockText(m_colorCommentEdit)
        }
    );
}

void GProfileDialog::addCustomColorRule()
{
    const QColor color = QColorDialog::getColor(QColor(Qt::red), this, QStringLiteral("新增真彩色规则"));

    if (!color.isValid())
    {
        return;
    }

    const QString colorKey = GProfile::colorKeyFromColor(color);

    if (!m_colorBlocks.contains(colorKey))
    {
        m_colorBlocks.insert(colorKey, GProfileCodeBlock());
    }

    refreshColorRuleList(colorKey);
}

void GProfileDialog::removeSelectedColorRule()
{
    const QString colorKey = currentColorKey();

    if (colorKey.isEmpty())
    {
        return;
    }

    m_colorBlocks.remove(colorKey);
    refreshColorRuleList(isBuiltInColorKey(colorKey) ? colorKey : QString());
}

void GProfileDialog::importProfileFromFile()
{
    const QString filePath = QFileDialog::getOpenFileName
    (
        this,
        QStringLiteral("导入 GProfile 配置"),
        QString(),
        QStringLiteral("JSON 文件 (*.json)")
    );

    if (filePath.isEmpty())
    {
        return;
    }

    QString errorMessage;
    const GProfile profile = GProfile::loadFromFile(filePath, &errorMessage);

    if (!errorMessage.isEmpty())
    {
        QMessageBox::warning(this, QStringLiteral("导入失败"), errorMessage);
        return;
    }

    applyProfile(profile);
}

void GProfileDialog::exportProfileToFile()
{
    updateCurrentEntityTypeBlock();
    updateCurrentLayerRule();
    updateCurrentColorRule();
    const GProfile profile = collectProfile();

    const QString defaultFileName = profile.profileName().trimmed().isEmpty()
        ? QStringLiteral("gprofile.json")
        : QStringLiteral("%1.json").arg(profile.profileName().trimmed());

    const QString filePath = QFileDialog::getSaveFileName
    (
        this,
        QStringLiteral("导出 GProfile 配置"),
        defaultFileName,
        QStringLiteral("JSON 文件 (*.json)")
    );

    if (filePath.isEmpty())
    {
        return;
    }

    QString errorMessage;

    if (!profile.saveToFile(filePath, &errorMessage))
    {
        QMessageBox::warning(this, QStringLiteral("导出失败"), errorMessage);
        return;
    }

    QMessageBox::information(this, QStringLiteral("导出完成"), QStringLiteral("配置已导出到: %1").arg(filePath));
}

void GProfileDialog::resetToDefaultProfile()
{
    applyProfile(GProfile::createDefaultLaserProfile());
}

QString GProfileDialog::currentEntityTypeKey() const
{
    return m_entityTypeComboBox->currentData().toString();
}

QString GProfileDialog::currentLayerKey() const
{
    QListWidgetItem* item = m_layerRuleListWidget->currentItem();
    return item != nullptr ? item->data(Qt::UserRole).toString() : QString();
}

QString GProfileDialog::currentColorKey() const
{
    QListWidgetItem* item = m_colorRuleListWidget->currentItem();
    return item != nullptr ? item->data(Qt::UserRole).toString() : QString();
}

QStringList GProfileDialog::supportedEntityTypes()
{
    return
    {
        QStringLiteral("LINE"),
        QStringLiteral("ARC"),
        QStringLiteral("CIRCLE"),
        QStringLiteral("ELLIPSE"),
        QStringLiteral("POLYLINE"),
        QStringLiteral("LWPOLYLINE"),
        QStringLiteral("POINT")
    };
}

QStringList GProfileDialog::supportedDefaultColorKeys()
{
    return builtInColorKeys();
}

QString GProfileDialog::displayNameForEntityType(const QString& entityTypeKey)
{
    const QString normalizedKey = GProfile::normalizeEntityTypeKey(entityTypeKey);

    if (normalizedKey == QStringLiteral("LINE"))
    {
        return QStringLiteral("直线 (LINE)");
    }

    if (normalizedKey == QStringLiteral("ARC"))
    {
        return QStringLiteral("圆弧 (ARC)");
    }

    if (normalizedKey == QStringLiteral("CIRCLE"))
    {
        return QStringLiteral("圆 (CIRCLE)");
    }

    if (normalizedKey == QStringLiteral("ELLIPSE"))
    {
        return QStringLiteral("椭圆 (ELLIPSE)");
    }

    if (normalizedKey == QStringLiteral("POLYLINE"))
    {
        return QStringLiteral("多段线 (POLYLINE)");
    }

    if (normalizedKey == QStringLiteral("LWPOLYLINE"))
    {
        return QStringLiteral("轻量多段线 (LWPOLYLINE)");
    }

    if (normalizedKey == QStringLiteral("POINT"))
    {
        return QStringLiteral("点 (POINT)");
    }

    return normalizedKey;
}

QString GProfileDialog::displayNameForColorKey(const QString& colorKey)
{
    const QString normalizedKey = GProfile::normalizeColorKey(colorKey);

    if (normalizedKey == QStringLiteral("BYLAYER"))
    {
        return QStringLiteral("随层 (BYLAYER)");
    }

    if (normalizedKey == QStringLiteral("BYBLOCK"))
    {
        return QStringLiteral("随块 (BYBLOCK)");
    }

    if (normalizedKey == QStringLiteral("ACI:1"))
    {
        return QStringLiteral("1 红 (ACI:1)");
    }

    if (normalizedKey == QStringLiteral("ACI:2"))
    {
        return QStringLiteral("2 黄 (ACI:2)");
    }

    if (normalizedKey == QStringLiteral("ACI:3"))
    {
        return QStringLiteral("3 绿 (ACI:3)");
    }

    if (normalizedKey == QStringLiteral("ACI:4"))
    {
        return QStringLiteral("4 青 (ACI:4)");
    }

    if (normalizedKey == QStringLiteral("ACI:5"))
    {
        return QStringLiteral("5 蓝 (ACI:5)");
    }

    if (normalizedKey == QStringLiteral("ACI:6"))
    {
        return QStringLiteral("6 洋红 (ACI:6)");
    }

    if (normalizedKey == QStringLiteral("ACI:7"))
    {
        return QStringLiteral("7 白 (ACI:7)");
    }

    if (normalizedKey == QStringLiteral("ACI:8"))
    {
        return QStringLiteral("8 灰 (ACI:8)");
    }

    if (normalizedKey == QStringLiteral("ACI:9"))
    {
        return QStringLiteral("9 浅灰 (ACI:9)");
    }

    return QStringLiteral("真彩色 %1").arg(normalizedKey);
}

bool GProfileDialog::hasMeaningfulContent(const GProfileCodeBlock& codeBlock)
{
    return !codeBlock.header.trimmed().isEmpty()
        || !codeBlock.footer.trimmed().isEmpty()
        || !codeBlock.comment.trimmed().isEmpty();
}
