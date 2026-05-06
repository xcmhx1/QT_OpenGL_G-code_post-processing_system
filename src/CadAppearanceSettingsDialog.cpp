// 实现 CadAppearanceSettingsDialog 模块，对应自定义外观编辑流程。
#include "pch.h"

#include "CadAppearanceSettingsDialog.h"

#include <QColorDialog>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QPushButton>
#include <QScrollArea>
#include <QVBoxLayout>

namespace
{
    QColor* themeColorByKey(AppThemeColors& theme, const QString& key)
    {
        if (key == QStringLiteral("windowBackground")) return &theme.windowBackground;
        if (key == QStringLiteral("panelBackground")) return &theme.panelBackground;
        if (key == QStringLiteral("surfaceBackground")) return &theme.surfaceBackground;
        if (key == QStringLiteral("surfaceAltBackground")) return &theme.surfaceAltBackground;
        if (key == QStringLiteral("borderColor")) return &theme.borderColor;
        if (key == QStringLiteral("textPrimaryColor")) return &theme.textPrimaryColor;
        if (key == QStringLiteral("textSecondaryColor")) return &theme.textSecondaryColor;
        if (key == QStringLiteral("accentColor")) return &theme.accentColor;
        if (key == QStringLiteral("accentTextColor")) return &theme.accentTextColor;
        if (key == QStringLiteral("hoverBackgroundColor")) return &theme.hoverBackgroundColor;
        if (key == QStringLiteral("pressedBackgroundColor")) return &theme.pressedBackgroundColor;
        if (key == QStringLiteral("viewerBackgroundColor")) return &theme.viewerBackgroundColor;
        if (key == QStringLiteral("viewerGridColor")) return &theme.viewerGridColor;
        if (key == QStringLiteral("processLabelFillColor")) return &theme.processLabelFillColor;
        if (key == QStringLiteral("processLabelBorderColor")) return &theme.processLabelBorderColor;
        if (key == QStringLiteral("processLabelTextColor")) return &theme.processLabelTextColor;
        if (key == QStringLiteral("selectedProcessLabelFillColor")) return &theme.selectedProcessLabelFillColor;
        if (key == QStringLiteral("selectedProcessLabelBorderColor")) return &theme.selectedProcessLabelBorderColor;
        if (key == QStringLiteral("selectedProcessLabelTextColor")) return &theme.selectedProcessLabelTextColor;
        if (key == QStringLiteral("selectedBasePointColor")) return &theme.selectedBasePointColor;
        if (key == QStringLiteral("selectedControlPointColor")) return &theme.selectedControlPointColor;
        return nullptr;
    }

    QString buttonStyleForColor(const QColor& color)
    {
        const QColor textColor = color.lightness() >= 128 ? QColor(18, 22, 26) : QColor(244, 247, 250);
        return QStringLiteral("QPushButton { background-color: %1; color: %2; border: 1px solid #7A828C; padding: 4px 8px; }")
            .arg(color.name(QColor::HexRgb))
            .arg(textColor.name(QColor::HexRgb));
    }
}

CadAppearanceSettingsDialog::CadAppearanceSettingsDialog(AppThemeMode baseMode, const AppThemeColors& initialTheme, QWidget* parent)
    : QDialog(parent)
    , m_baseMode(baseMode == AppThemeMode::Dark ? AppThemeMode::Dark : AppThemeMode::Light)
    , m_theme(initialTheme)
{
    setWindowTitle(QStringLiteral("自定义外观"));
    setMinimumSize(520, 620);
    buildUi();
    updateColorButtons();
}

AppThemeMode CadAppearanceSettingsDialog::baseMode() const
{
    return m_baseMode;
}

AppThemeColors CadAppearanceSettingsDialog::themeColors() const
{
    AppThemeColors theme = m_theme;
    finalizeAppThemePalette(theme);
    return theme;
}

void CadAppearanceSettingsDialog::buildUi()
{
    QVBoxLayout* rootLayout = new QVBoxLayout(this);
    rootLayout->setContentsMargins(12, 12, 12, 12);
    rootLayout->setSpacing(10);

    m_baseModeComboBox = new QComboBox(this);
    m_baseModeComboBox->addItem(QStringLiteral("从浅色模式继承"), static_cast<int>(AppThemeMode::Light));
    m_baseModeComboBox->addItem(QStringLiteral("从深色模式继承"), static_cast<int>(AppThemeMode::Dark));
    m_baseModeComboBox->setCurrentIndex(m_baseMode == AppThemeMode::Dark ? 1 : 0);
    rootLayout->addWidget(m_baseModeComboBox);

    QScrollArea* scrollArea = new QScrollArea(this);
    scrollArea->setWidgetResizable(true);
    rootLayout->addWidget(scrollArea, 1);

    QWidget* container = new QWidget(scrollArea);
    QVBoxLayout* containerLayout = new QVBoxLayout(container);
    containerLayout->setContentsMargins(0, 0, 0, 0);
    containerLayout->setSpacing(10);
    scrollArea->setWidget(container);

    QGroupBox* uiGroup = new QGroupBox(QStringLiteral("界面颜色"), container);
    QFormLayout* uiLayout = new QFormLayout(uiGroup);
    uiLayout->addRow(QStringLiteral("窗口背景"), addColorButton(uiGroup, QStringLiteral("窗口背景"), QStringLiteral("windowBackground")));
    uiLayout->addRow(QStringLiteral("面板背景"), addColorButton(uiGroup, QStringLiteral("面板背景"), QStringLiteral("panelBackground")));
    uiLayout->addRow(QStringLiteral("表面背景"), addColorButton(uiGroup, QStringLiteral("表面背景"), QStringLiteral("surfaceBackground")));
    uiLayout->addRow(QStringLiteral("次级表面"), addColorButton(uiGroup, QStringLiteral("次级表面"), QStringLiteral("surfaceAltBackground")));
    uiLayout->addRow(QStringLiteral("边框"), addColorButton(uiGroup, QStringLiteral("边框"), QStringLiteral("borderColor")));
    uiLayout->addRow(QStringLiteral("主文本"), addColorButton(uiGroup, QStringLiteral("主文本"), QStringLiteral("textPrimaryColor")));
    uiLayout->addRow(QStringLiteral("次文本"), addColorButton(uiGroup, QStringLiteral("次文本"), QStringLiteral("textSecondaryColor")));
    uiLayout->addRow(QStringLiteral("强调色"), addColorButton(uiGroup, QStringLiteral("强调色"), QStringLiteral("accentColor")));
    uiLayout->addRow(QStringLiteral("强调文本"), addColorButton(uiGroup, QStringLiteral("强调文本"), QStringLiteral("accentTextColor")));
    uiLayout->addRow(QStringLiteral("悬停背景"), addColorButton(uiGroup, QStringLiteral("悬停背景"), QStringLiteral("hoverBackgroundColor")));
    uiLayout->addRow(QStringLiteral("按下背景"), addColorButton(uiGroup, QStringLiteral("按下背景"), QStringLiteral("pressedBackgroundColor")));
    containerLayout->addWidget(uiGroup);

    QGroupBox* viewerGroup = new QGroupBox(QStringLiteral("视图颜色"), container);
    QFormLayout* viewerLayout = new QFormLayout(viewerGroup);
    viewerLayout->addRow(QStringLiteral("视图背景"), addColorButton(viewerGroup, QStringLiteral("视图背景"), QStringLiteral("viewerBackgroundColor")));
    viewerLayout->addRow(QStringLiteral("网格颜色"), addColorButton(viewerGroup, QStringLiteral("网格颜色"), QStringLiteral("viewerGridColor")));
    viewerLayout->addRow(QStringLiteral("基点手柄"), addColorButton(viewerGroup, QStringLiteral("基点手柄"), QStringLiteral("selectedBasePointColor")));
    viewerLayout->addRow(QStringLiteral("控制点手柄"), addColorButton(viewerGroup, QStringLiteral("控制点手柄"), QStringLiteral("selectedControlPointColor")));
    containerLayout->addWidget(viewerGroup);

    QGroupBox* processGroup = new QGroupBox(QStringLiteral("机加工标注颜色"), container);
    QFormLayout* processLayout = new QFormLayout(processGroup);
    processLayout->addRow(QStringLiteral("顺序填充"), addColorButton(processGroup, QStringLiteral("顺序填充"), QStringLiteral("processLabelFillColor")));
    processLayout->addRow(QStringLiteral("顺序边框"), addColorButton(processGroup, QStringLiteral("顺序边框"), QStringLiteral("processLabelBorderColor")));
    processLayout->addRow(QStringLiteral("顺序文本"), addColorButton(processGroup, QStringLiteral("顺序文本"), QStringLiteral("processLabelTextColor")));
    processLayout->addRow(QStringLiteral("选中填充"), addColorButton(processGroup, QStringLiteral("选中填充"), QStringLiteral("selectedProcessLabelFillColor")));
    processLayout->addRow(QStringLiteral("选中边框"), addColorButton(processGroup, QStringLiteral("选中边框"), QStringLiteral("selectedProcessLabelBorderColor")));
    processLayout->addRow(QStringLiteral("选中文本"), addColorButton(processGroup, QStringLiteral("选中文本"), QStringLiteral("selectedProcessLabelTextColor")));
    containerLayout->addWidget(processGroup);
    containerLayout->addStretch(1);

    QDialogButtonBox* buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    buttonBox->button(QDialogButtonBox::Ok)->setText(QStringLiteral("应用"));
    buttonBox->button(QDialogButtonBox::Cancel)->setText(QStringLiteral("取消"));
    rootLayout->addWidget(buttonBox);

    connect(buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
    connect
    (
        m_baseModeComboBox,
        QOverload<int>::of(&QComboBox::currentIndexChanged),
        this,
        [this](int index)
        {
            rebuildFromBaseMode(static_cast<AppThemeMode>(m_baseModeComboBox->itemData(index).toInt()));
        }
    );
}

void CadAppearanceSettingsDialog::rebuildFromBaseMode(AppThemeMode mode)
{
    m_baseMode = mode == AppThemeMode::Dark ? AppThemeMode::Dark : AppThemeMode::Light;
    m_theme = buildAppThemeColors(m_baseMode);
    updateColorButtons();
}

void CadAppearanceSettingsDialog::updateColorButtons()
{
    for (auto it = m_colorButtons.begin(); it != m_colorButtons.end(); ++it)
    {
        QColor* color = themeColorByKey(m_theme, it.key());

        if (color == nullptr || it.value() == nullptr)
        {
            continue;
        }

        it.value()->setText(color->name(QColor::HexArgb).toUpper());
        it.value()->setStyleSheet(buttonStyleForColor(*color));
    }
}

QPushButton* CadAppearanceSettingsDialog::addColorButton(QWidget* parent, const QString& label, const QString& key)
{
    QPushButton* button = new QPushButton(parent);
    button->setMinimumWidth(132);
    m_colorButtons.insert(key, button);
    connect
    (
        button,
        &QPushButton::clicked,
        this,
        [this, key, label]()
        {
            QColor* color = themeColorByKey(m_theme, key);

            if (color == nullptr)
            {
                return;
            }

            const QColor selectedColor = QColorDialog::getColor(*color, this, QStringLiteral("选择%1").arg(label), QColorDialog::ShowAlphaChannel);

            if (!selectedColor.isValid())
            {
                return;
            }

            *color = selectedColor;
            updateColorButtons();
        }
    );
    return button;
}
