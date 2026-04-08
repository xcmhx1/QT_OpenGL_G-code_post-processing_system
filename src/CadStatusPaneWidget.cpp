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

    QLabel* snapReservedLabel = new QLabel(QStringLiteral("吸附: 预留"), this);
    snapReservedLabel->setFont(valueFont);

    QLabel* orthoReservedLabel = new QLabel(QStringLiteral("正交: 预留"), this);
    orthoReservedLabel->setFont(valueFont);

    QLabel* polarReservedLabel = new QLabel(QStringLiteral("极轴: 预留"), this);
    polarReservedLabel->setFont(valueFont);

    layout->addWidget(coordinateFrame);
    layout->addSpacing(8);
    layout->addWidget(snapReservedLabel);
    layout->addWidget(orthoReservedLabel);
    layout->addWidget(polarReservedLabel);
    layout->addStretch(1);

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
            "#CadStatusPaneWidget QLabel {"
            "color: %5;"
            "}"
        )
        .arg(theme.panelBackground.name())
        .arg(theme.borderColor.name())
        .arg(theme.surfaceBackground.name())
        .arg(theme.borderStrongColor.name())
        .arg(theme.textPrimaryColor.name())
    );
}
