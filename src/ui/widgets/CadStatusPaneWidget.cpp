// 实现 CadStatusPaneWidget 模块，对应头文件中声明的主要行为和协作流程。
// 状态栏模块，负责显示当前鼠标世界坐标和后续状态位预留区域。
#include "platform/pch.h"

#include "ui/widgets/CadStatusPaneWidget.h"

#include <QAction>
#include <QBoxLayout>
#include <QFont>
#include <QFrame>
#include <QStringList>

CadStatusPaneWidget::CadStatusPaneWidget(QWidget* parent)
    : QWidget(parent)
{
    setObjectName("CadStatusPaneWidget");
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    setFixedHeight(40);

    QHBoxLayout* layout = new QHBoxLayout(this);
    layout->setContentsMargins(8, 4, 8, 4);
    layout->setSpacing(8);

    QFrame* coordinateFrame = new QFrame(this);
    coordinateFrame->setObjectName("CoordinateBlock");
    QHBoxLayout* coordinateLayout = new QHBoxLayout(coordinateFrame);
    coordinateLayout->setContentsMargins(8, 3, 8, 3);
    coordinateLayout->setSpacing(6);

    QLabel* coordinateTitleLabel = new QLabel(QStringLiteral("坐标"), coordinateFrame);
    QFont titleFont = coordinateTitleLabel->font();
    titleFont.setPointSize(9);
    titleFont.setBold(true);
    coordinateTitleLabel->setFont(titleFont);
    coordinateTitleLabel->setProperty("statusTitle", true);

    m_coordinateValueLabel = new QLabel(coordinateFrame);
    QFont valueFont = m_coordinateValueLabel->font();
    valueFont.setPointSize(9);
    m_coordinateValueLabel->setFont(valueFont);
    m_coordinateValueLabel->setMinimumWidth(238);
    m_coordinateValueLabel->setProperty("statusValue", true);

    coordinateLayout->addWidget(coordinateTitleLabel);
    coordinateLayout->addWidget(m_coordinateValueLabel);

    QFrame* entityTypeFrame = new QFrame(this);
    entityTypeFrame->setObjectName("EntityTypeBlock");
    QHBoxLayout* entityTypeLayout = new QHBoxLayout(entityTypeFrame);
    entityTypeLayout->setContentsMargins(8, 3, 8, 3);
    entityTypeLayout->setSpacing(6);

    QLabel* entityTypeTitleLabel = new QLabel(QStringLiteral("图元"), entityTypeFrame);
    entityTypeTitleLabel->setFont(titleFont);
    entityTypeTitleLabel->setProperty("statusTitle", true);

    m_entityTypeValueLabel = new QLabel(entityTypeFrame);
    m_entityTypeValueLabel->setFont(valueFont);
    m_entityTypeValueLabel->setMinimumWidth(145);
    m_entityTypeValueLabel->setMaximumWidth(300);
    m_entityTypeValueLabel->setProperty("statusValue", true);
    m_entityTypeValueLabel->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
    m_entityTypeValueLabel->setText(QStringLiteral("无"));

    entityTypeLayout->addWidget(entityTypeTitleLabel);
    entityTypeLayout->addWidget(m_entityTypeValueLabel);

    QWidget* snapContainer = new QWidget(this);
    QHBoxLayout* snapLayout = new QHBoxLayout(snapContainer);
    snapLayout->setContentsMargins(0, 0, 0, 0);
    snapLayout->setSpacing(0);

    m_snapSettingsButton = new QToolButton(snapContainer);
    m_snapSettingsButton->setText(QStringLiteral("捕捉设置"));
    m_snapSettingsButton->setPopupMode(QToolButton::InstantPopup);
    m_snapSettingsButton->setCursor(Qt::PointingHandCursor);
    m_snapSettingsButton->setToolButtonStyle(Qt::ToolButtonTextOnly);
    m_snapSettingsButton->setProperty("snapToggle", true);
    m_snapSettingsButton->setMinimumHeight(28);

    m_snapSettingsMenu = new QMenu(m_snapSettingsButton);
    m_snapSettingsMenu->setObjectName("SnapSettingsMenu");

    QAction* objectSnapTitle = m_snapSettingsMenu->addSection(QStringLiteral("对象捕捉"));
    objectSnapTitle->setEnabled(false);
    m_basePointSnapAction = m_snapSettingsMenu->addAction(QStringLiteral("基点"));
    m_basePointSnapAction->setCheckable(true);
    m_controlPointSnapAction = m_snapSettingsMenu->addAction(QStringLiteral("控制点"));
    m_controlPointSnapAction->setCheckable(true);
    m_endpointSnapAction = m_snapSettingsMenu->addAction(QStringLiteral("端点"));
    m_endpointSnapAction->setCheckable(true);
    m_midpointSnapAction = m_snapSettingsMenu->addAction(QStringLiteral("中点"));
    m_midpointSnapAction->setCheckable(true);
    m_centerSnapAction = m_snapSettingsMenu->addAction(QStringLiteral("圆心/中心"));
    m_centerSnapAction->setCheckable(true);
    m_intersectionSnapAction = m_snapSettingsMenu->addAction(QStringLiteral("交点"));
    m_intersectionSnapAction->setCheckable(true);
    m_snapSettingsMenu->addSeparator();

    QAction* gridSnapTitle = m_snapSettingsMenu->addSection(QStringLiteral("网格捕捉"));
    gridSnapTitle->setEnabled(false);
    m_gridSnapAction = m_snapSettingsMenu->addAction(QStringLiteral("网格"));
    m_gridSnapAction->setCheckable(true);

    m_snapSettingsButton->setMenu(m_snapSettingsMenu);

    snapLayout->addWidget(m_snapSettingsButton);

    m_orthoButton = new QToolButton(this);
    m_orthoButton->setCheckable(true);
    m_orthoButton->setCursor(Qt::PointingHandCursor);
    m_orthoButton->setToolButtonStyle(Qt::ToolButtonTextOnly);
    m_orthoButton->setProperty("snapToggle", true);
    m_orthoButton->setMinimumHeight(28);

    m_polarButton = new QToolButton(this);
    m_polarButton->setCheckable(true);
    m_polarButton->setCursor(Qt::PointingHandCursor);
    m_polarButton->setToolButtonStyle(Qt::ToolButtonTextOnly);
    m_polarButton->setProperty("snapToggle", true);
    m_polarButton->setMinimumHeight(28);

    layout->addWidget(coordinateFrame);
    layout->addWidget(entityTypeFrame);
    layout->addSpacing(8);
    layout->addWidget(snapContainer);
    layout->addWidget(m_orthoButton);
    layout->addWidget(m_polarButton);
    layout->addStretch(1);

    connect(m_basePointSnapAction, &QAction::toggled, this, &CadStatusPaneWidget::basePointSnapToggled);
    connect(m_controlPointSnapAction, &QAction::toggled, this, &CadStatusPaneWidget::controlPointSnapToggled);
    connect(m_gridSnapAction, &QAction::toggled, this, &CadStatusPaneWidget::gridSnapToggled);
    connect(m_endpointSnapAction, &QAction::toggled, this, &CadStatusPaneWidget::endpointSnapToggled);
    connect(m_midpointSnapAction, &QAction::toggled, this, &CadStatusPaneWidget::midpointSnapToggled);
    connect(m_centerSnapAction, &QAction::toggled, this, &CadStatusPaneWidget::centerSnapToggled);
    connect(m_intersectionSnapAction, &QAction::toggled, this, &CadStatusPaneWidget::intersectionSnapToggled);
    connect
    (
        m_orthoButton,
        &QToolButton::toggled,
        this,
        [this](bool enabled)
        {
            refreshDraftingModeButtons();

            if (!m_applyingDraftingModes)
            {
                emit orthoToggled(enabled);
            }
        }
    );
    connect
    (
        m_polarButton,
        &QToolButton::toggled,
        this,
        [this](bool enabled)
        {
            refreshDraftingModeButtons();

            if (!m_applyingDraftingModes)
            {
                emit polarTrackingToggled(enabled);
            }
        }
    );

    const auto updateSummary = [this]()
    {
        refreshSnapSummary();

        if (!m_applyingSnapMask)
        {
            emit snapOptionMaskChanged(snapOptionMask());
        }
    };

    for (QAction* action :
        {
            m_basePointSnapAction,
            m_controlPointSnapAction,
            m_gridSnapAction,
            m_endpointSnapAction,
            m_midpointSnapAction,
            m_centerSnapAction,
            m_intersectionSnapAction
        })
    {
        connect(action, &QAction::toggled, this, updateSummary);
    }

    setTheme(buildAppThemeColors(AppThemeMode::Light));
    refreshSnapSummary();
    refreshDraftingModeButtons();

    setWorldPosition(QVector3D());
}

void CadStatusPaneWidget::setWorldPosition(const QVector3D& worldPos)
{
    m_coordinateValueLabel->setText
    (
        QStringLiteral("X: %1   Y: %2   Z: %3")
        .arg(worldPos.x(), 0, 'f', 3)
        .arg(worldPos.y(), 0, 'f', 3)
        .arg(worldPos.z(), 0, 'f', 3)
    );
}

void CadStatusPaneWidget::setEntityTypeText(const QString& text)
{
    if (m_entityTypeValueLabel == nullptr)
    {
        return;
    }

    const QString normalizedText = text.trimmed().isEmpty() ? QStringLiteral("无") : text.trimmed();
    m_entityTypeValueLabel->setText(normalizedText);
    m_entityTypeValueLabel->setToolTip(normalizedText);
}

void CadStatusPaneWidget::setTheme(const AppThemeColors& theme)
{
    setStyleSheet
    (
        QStringLiteral
        (
            "#CadStatusPaneWidget {"
            "background-color: %1;"
            "border-top: 1px solid %2;"
            "}"
            "#CoordinateBlock {"
            "background-color: %3;"
            "border: 1px solid %4;"
            "border-radius: 4px;"
            "}"
            "#EntityTypeBlock {"
            "background-color: %3;"
            "border: 1px solid %4;"
            "border-radius: 4px;"
            "}"
            "#CadStatusPaneWidget QLabel {"
            "color: %5;"
            "}"
            "#CadStatusPaneWidget QLabel[statusTitle=\"true\"] { color: %10; }"
            "#CadStatusPaneWidget QLabel[statusValue=\"true\"] { font-weight: 500; }"
            "QPushButton[snapToggle=\"true\"] {"
            "background-color: %6;"
            "color: %5;"
            "border: 1px solid %2;"
            "border-radius: 4px;"
            "padding: 3px 9px;"
            "}"
            "QPushButton[snapToggle=\"true\"]:hover {"
            "background-color: %7;"
            "}"
            "QToolButton[snapToggle=\"true\"] {"
            "background-color: %6;"
            "color: %5;"
            "border: 1px solid %2;"
            "border-radius: 4px;"
            "padding: 3px 10px;"
            "font-weight: 500;"
            "}"
            "QToolButton[snapToggle=\"true\"]:hover {"
            "background-color: %7;"
            "}"
            "QToolButton[snapToggle=\"true\"]:checked {"
            "background-color: %8;"
            "color: %9;"
            "border: 1px solid %8;"
            "font-weight: 600;"
            "}"
            "QPushButton[snapToggle=\"true\"]:checked {"
            "background-color: %8;"
            "color: %9;"
            "border: 1px solid %8;"
            "font-weight: 600;"
            "}"
            "#SnapSettingsMenu {"
            "background-color: %3;"
            "color: %5;"
            "border: 1px solid %4;"
            "}"
            "#SnapSettingsMenu::item {"
            "padding: 5px 18px 5px 22px;"
            "}"
            "#SnapSettingsMenu::item:selected {"
            "background-color: %7;"
            "}"
        )
        .arg(theme.panelBackground.name())
        .arg(theme.borderColor.name())
        .arg(theme.surfaceBackground.name())
        .arg(theme.borderStrongColor.name())
        .arg(theme.textPrimaryColor.name())
        .arg(theme.surfaceAltBackground.name())
        .arg(theme.hoverBackgroundColor.name())
        .arg(theme.accentColor.name())
        .arg(theme.accentTextColor.name())
        .arg(theme.textSecondaryColor.name())
    );
}

quint32 CadStatusPaneWidget::snapOptionMask() const
{
    quint32 mask = 0u;

    if (m_basePointSnapAction != nullptr && m_basePointSnapAction->isChecked())
    {
        mask |= BasePointSnapBit;
    }

    if (m_controlPointSnapAction != nullptr && m_controlPointSnapAction->isChecked())
    {
        mask |= ControlPointSnapBit;
    }

    if (m_endpointSnapAction != nullptr && m_endpointSnapAction->isChecked())
    {
        mask |= EndpointSnapBit;
    }

    if (m_midpointSnapAction != nullptr && m_midpointSnapAction->isChecked())
    {
        mask |= MidpointSnapBit;
    }

    if (m_centerSnapAction != nullptr && m_centerSnapAction->isChecked())
    {
        mask |= CenterSnapBit;
    }

    if (m_intersectionSnapAction != nullptr && m_intersectionSnapAction->isChecked())
    {
        mask |= IntersectionSnapBit;
    }

    if (m_gridSnapAction != nullptr && m_gridSnapAction->isChecked())
    {
        mask |= GridSnapBit;
    }

    return mask;
}

void CadStatusPaneWidget::setSnapOptionMask(quint32 mask)
{
    const quint32 normalizedMask = mask & allSnapOptionMask();
    m_applyingSnapMask = true;

    if (m_basePointSnapAction != nullptr)
    {
        m_basePointSnapAction->setChecked((normalizedMask & BasePointSnapBit) != 0u);
    }

    if (m_controlPointSnapAction != nullptr)
    {
        m_controlPointSnapAction->setChecked((normalizedMask & ControlPointSnapBit) != 0u);
    }

    if (m_endpointSnapAction != nullptr)
    {
        m_endpointSnapAction->setChecked((normalizedMask & EndpointSnapBit) != 0u);
    }

    if (m_midpointSnapAction != nullptr)
    {
        m_midpointSnapAction->setChecked((normalizedMask & MidpointSnapBit) != 0u);
    }

    if (m_centerSnapAction != nullptr)
    {
        m_centerSnapAction->setChecked((normalizedMask & CenterSnapBit) != 0u);
    }

    if (m_intersectionSnapAction != nullptr)
    {
        m_intersectionSnapAction->setChecked((normalizedMask & IntersectionSnapBit) != 0u);
    }

    if (m_gridSnapAction != nullptr)
    {
        m_gridSnapAction->setChecked((normalizedMask & GridSnapBit) != 0u);
    }

    m_applyingSnapMask = false;
    refreshSnapSummary();
    emit snapOptionMaskChanged(snapOptionMask());
}

void CadStatusPaneWidget::setOrthoEnabled(bool enabled)
{
    if (m_orthoButton == nullptr)
    {
        return;
    }

    m_applyingDraftingModes = true;
    m_orthoButton->setChecked(enabled);
    m_applyingDraftingModes = false;
    refreshDraftingModeButtons();
}

void CadStatusPaneWidget::setPolarTrackingEnabled(bool enabled)
{
    if (m_polarButton == nullptr)
    {
        return;
    }

    m_applyingDraftingModes = true;
    m_polarButton->setChecked(enabled);
    m_applyingDraftingModes = false;
    refreshDraftingModeButtons();
}

void CadStatusPaneWidget::refreshSnapSummary()
{
    if (m_snapSettingsButton == nullptr)
    {
        return;
    }

    QStringList enabledOptions;

    if (m_basePointSnapAction != nullptr && m_basePointSnapAction->isChecked())
    {
        enabledOptions.push_back(QStringLiteral("基点"));
    }

    if (m_controlPointSnapAction != nullptr && m_controlPointSnapAction->isChecked())
    {
        enabledOptions.push_back(QStringLiteral("控制点"));
    }

    if (m_endpointSnapAction != nullptr && m_endpointSnapAction->isChecked())
    {
        enabledOptions.push_back(QStringLiteral("端点"));
    }

    if (m_midpointSnapAction != nullptr && m_midpointSnapAction->isChecked())
    {
        enabledOptions.push_back(QStringLiteral("中点"));
    }

    if (m_centerSnapAction != nullptr && m_centerSnapAction->isChecked())
    {
        enabledOptions.push_back(QStringLiteral("圆心"));
    }

    if (m_intersectionSnapAction != nullptr && m_intersectionSnapAction->isChecked())
    {
        enabledOptions.push_back(QStringLiteral("交点"));
    }

    if (m_gridSnapAction != nullptr && m_gridSnapAction->isChecked())
    {
        enabledOptions.push_back(QStringLiteral("网格"));
    }

    if (enabledOptions.isEmpty())
    {
        m_snapSettingsButton->setToolTip(QStringLiteral("未启用任何捕捉选项"));
        return;
    }

    m_snapSettingsButton->setToolTip(QStringLiteral("已启用: %1").arg(enabledOptions.join(QStringLiteral(" | "))));
}

void CadStatusPaneWidget::refreshDraftingModeButtons()
{
    if (m_orthoButton != nullptr)
    {
        m_orthoButton->setText(m_orthoButton->isChecked() ? QStringLiteral("正交 开") : QStringLiteral("正交 关"));
        m_orthoButton->setToolTip(QStringLiteral("正交约束（F8）：将下一点锁定到水平或垂直方向"));
    }

    if (m_polarButton != nullptr)
    {
        m_polarButton->setText(m_polarButton->isChecked() ? QStringLiteral("极轴 15°") : QStringLiteral("极轴 关"));
        m_polarButton->setToolTip(QStringLiteral("极轴追踪（F10）：将下一点吸附到 15° 增量方向；正交开启时正交优先"));
    }
}
