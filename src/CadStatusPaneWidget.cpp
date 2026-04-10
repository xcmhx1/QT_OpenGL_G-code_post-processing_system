// 实现 CadStatusPaneWidget 模块，对应头文件中声明的主要行为和协作流程。
// 状态栏模块，负责显示当前鼠标世界坐标和后续状态位预留区域。
#include "pch.h"

#include "CadStatusPaneWidget.h"

#include <QBoxLayout>
#include <QFont>
#include <QFrame>

CadStatusPaneWidget::CadStatusPaneWidget(QWidget* parent)
    : QWidget(parent)
{
    setObjectName("CadStatusPaneWidget");
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    setFixedHeight(44);

    QHBoxLayout* layout = new QHBoxLayout(this);
    layout->setContentsMargins(10, 4, 10, 4);
    layout->setSpacing(12);

    QFrame* coordinateFrame = new QFrame(this);
    coordinateFrame->setObjectName("CoordinateBlock");
    QHBoxLayout* coordinateLayout = new QHBoxLayout(coordinateFrame);
    coordinateLayout->setContentsMargins(10, 4, 10, 4);
    coordinateLayout->setSpacing(8);

    QLabel* coordinateTitleLabel = new QLabel(QStringLiteral("坐标"), coordinateFrame);
    QFont titleFont = coordinateTitleLabel->font();
    titleFont.setPointSize(12);
    titleFont.setBold(true);
    coordinateTitleLabel->setFont(titleFont);

    m_coordinateValueLabel = new QLabel(coordinateFrame);
    QFont valueFont = m_coordinateValueLabel->font();
    valueFont.setPointSize(12);
    m_coordinateValueLabel->setFont(valueFont);
    m_coordinateValueLabel->setMinimumWidth(280);

    coordinateLayout->addWidget(coordinateTitleLabel);
    coordinateLayout->addWidget(m_coordinateValueLabel);

    QFrame* snapFrame = new QFrame(this);
    snapFrame->setObjectName("SnapBlock");
    QHBoxLayout* snapLayout = new QHBoxLayout(snapFrame);
    snapLayout->setContentsMargins(10, 4, 10, 4);
    snapLayout->setSpacing(8);

    QLabel* snapTitleLabel = new QLabel(QStringLiteral("吸附"), snapFrame);
    snapTitleLabel->setFont(titleFont);

    m_basePointSnapButton = new QPushButton(QStringLiteral("基点"), snapFrame);
    m_controlPointSnapButton = new QPushButton(QStringLiteral("控制点"), snapFrame);
    m_gridSnapButton = new QPushButton(QStringLiteral("网格"), snapFrame);

    for (QPushButton* button : { m_basePointSnapButton, m_controlPointSnapButton, m_gridSnapButton })
    {
        button->setCheckable(true);
        button->setFont(valueFont);
        button->setCursor(Qt::PointingHandCursor);
        button->setProperty("snapToggle", true);
        button->setMinimumHeight(28);
    }

    snapLayout->addWidget(snapTitleLabel);
    snapLayout->addWidget(m_basePointSnapButton);
    snapLayout->addWidget(m_controlPointSnapButton);
    snapLayout->addWidget(m_gridSnapButton);

    QLabel* orthoReservedLabel = new QLabel(QStringLiteral("正交: 预留"), this);
    orthoReservedLabel->setFont(valueFont);

    QLabel* polarReservedLabel = new QLabel(QStringLiteral("极轴: 预留"), this);
    polarReservedLabel->setFont(valueFont);

    layout->addWidget(coordinateFrame);
    layout->addSpacing(8);
    layout->addWidget(snapFrame);
    layout->addWidget(orthoReservedLabel);
    layout->addWidget(polarReservedLabel);
    layout->addStretch(1);

    connect(m_basePointSnapButton, &QPushButton::toggled, this, &CadStatusPaneWidget::basePointSnapToggled);
    connect(m_controlPointSnapButton, &QPushButton::toggled, this, &CadStatusPaneWidget::controlPointSnapToggled);
    connect(m_gridSnapButton, &QPushButton::toggled, this, &CadStatusPaneWidget::gridSnapToggled);

    setTheme(buildAppThemeColors(AppThemeMode::Light));

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
            "#SnapBlock {"
            "background-color: %3;"
            "border: 1px solid %4;"
            "border-radius: 4px;"
            "}"
            "#CadStatusPaneWidget QLabel {"
            "color: %5;"
            "}"
            "QPushButton[snapToggle=\"true\"] {"
            "background-color: %6;"
            "color: %5;"
            "border: 1px solid %2;"
            "border-radius: 4px;"
            "padding: 3px 10px;"
            "}"
            "QPushButton[snapToggle=\"true\"]:hover {"
            "background-color: %7;"
            "}"
            "QPushButton[snapToggle=\"true\"]:checked {"
            "background-color: %8;"
            "color: %9;"
            "border: 1px solid %8;"
            "font-weight: 600;"
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
    );
}
