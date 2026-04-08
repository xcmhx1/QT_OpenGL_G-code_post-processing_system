#include "pch.h"

#include "CadToolPanelWidget.h"

#include <QComboBox>
#include <QFormLayout>
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QIcon>
#include <QLabel>
#include <QLayout>
#include <QMenu>
#include <QAction>
#include <QPainter>
#include <QPainterPath>
#include <QSizePolicy>
#include <QToolButton>
#include <QVBoxLayout>

namespace
{
    constexpr int kColorByLayer = 256;
    constexpr int kColorTrueColor = -1;
    constexpr int kPanelHeight = 112;
    constexpr int kComboHeight = 24;
    constexpr int kRibbonButtonWidth = 58;
    constexpr int kRibbonButtonHeight = 40;
    constexpr int kFooterHeight = 18;
    constexpr int kFooterReserveWidth = 16;
    constexpr int kDividerHeight = 78;
    constexpr int kLauncherSize = 12;
    constexpr int kRibbonIconSize = 16;

    void addColorOption(QComboBox* comboBox, const QString& text, int colorIndex)
    {
        comboBox->addItem(text, colorIndex);
    }

    QString cssRgb(const QColor& color)
    {
        return QStringLiteral("rgb(%1, %2, %3)").arg(color.red()).arg(color.green()).arg(color.blue());
    }

    QIcon buildRibbonIcon(DrawType drawType, const QColor& strokeColor)
    {
        QPixmap pixmap(kRibbonIconSize, kRibbonIconSize);
        pixmap.fill(Qt::transparent);

        QPainter painter(&pixmap);
        painter.setRenderHint(QPainter::Antialiasing, true);
        QPen pen(strokeColor);
        pen.setWidthF(1.5);
        pen.setCapStyle(Qt::RoundCap);
        pen.setJoinStyle(Qt::RoundJoin);
        painter.setPen(pen);
        painter.setBrush(Qt::NoBrush);

        switch (drawType)
        {
        case DrawType::Point:
            painter.setBrush(strokeColor);
            painter.drawEllipse(QPointF(9.0, 9.0), 2.1, 2.1);
            break;
        case DrawType::Line:
            painter.drawLine(QPointF(3.5, 13.5), QPointF(14.5, 4.5));
            break;
        case DrawType::Circle:
            painter.drawEllipse(QRectF(3.5, 3.5, 11.0, 11.0));
            break;
        case DrawType::Arc:
        {
            painter.drawArc(QRectF(3.0, 3.0, 12.0, 12.0), 35 * 16, 235 * 16);
            QPainterPath arrowHead;
            arrowHead.moveTo(13.8, 5.4);
            arrowHead.lineTo(14.9, 2.9);
            arrowHead.lineTo(11.9, 3.7);
            arrowHead.closeSubpath();
            painter.fillPath(arrowHead, pen.color());
            break;
        }
        case DrawType::Ellipse:
            painter.drawEllipse(QRectF(2.5, 5.0, 13.0, 8.0));
            break;
        case DrawType::Polyline:
        case DrawType::LWPolyline:
        {
            QPainterPath polylinePath;
            polylinePath.moveTo(3.0, 12.5);
            polylinePath.lineTo(7.0, 5.5);
            polylinePath.lineTo(11.0, 9.5);
            polylinePath.lineTo(15.0, 4.5);
            painter.drawPath(polylinePath);
            painter.setBrush(strokeColor);
            painter.drawEllipse(QPointF(3.0, 12.5), 1.2, 1.2);
            painter.drawEllipse(QPointF(7.0, 5.5), 1.2, 1.2);
            painter.drawEllipse(QPointF(11.0, 9.5), 1.2, 1.2);
            painter.drawEllipse(QPointF(15.0, 4.5), 1.2, 1.2);
            break;
        }
        default:
            break;
        }

        return QIcon(pixmap);
    }

    QIcon buildMoveIcon(const QColor& strokeColor)
    {
        QPixmap pixmap(kRibbonIconSize, kRibbonIconSize);
        pixmap.fill(Qt::transparent);

        QPainter painter(&pixmap);
        painter.setRenderHint(QPainter::Antialiasing, true);
        QPen pen(strokeColor);
        pen.setWidthF(1.4);
        pen.setCapStyle(Qt::RoundCap);
        pen.setJoinStyle(Qt::RoundJoin);
        painter.setPen(pen);

        painter.drawLine(QPointF(9.0, 3.0), QPointF(9.0, 15.0));
        painter.drawLine(QPointF(3.0, 9.0), QPointF(15.0, 9.0));

        QPainterPath arrowHead;
        arrowHead.moveTo(9.0, 2.0);
        arrowHead.lineTo(7.0, 4.5);
        arrowHead.lineTo(11.0, 4.5);
        arrowHead.closeSubpath();
        painter.fillPath(arrowHead, pen.color());
        arrowHead = QPainterPath();
        arrowHead.moveTo(9.0, 16.0);
        arrowHead.lineTo(7.0, 13.5);
        arrowHead.lineTo(11.0, 13.5);
        arrowHead.closeSubpath();
        painter.fillPath(arrowHead, pen.color());
        arrowHead = QPainterPath();
        arrowHead.moveTo(2.0, 9.0);
        arrowHead.lineTo(4.5, 7.0);
        arrowHead.lineTo(4.5, 11.0);
        arrowHead.closeSubpath();
        painter.fillPath(arrowHead, pen.color());
        arrowHead = QPainterPath();
        arrowHead.moveTo(16.0, 9.0);
        arrowHead.lineTo(13.5, 7.0);
        arrowHead.lineTo(13.5, 11.0);
        arrowHead.closeSubpath();
        painter.fillPath(arrowHead, pen.color());

        return QIcon(pixmap);
    }

    QIcon buildColorChipIcon(const QColor& color)
    {
        constexpr int kChipSize = 12;

        QPixmap pixmap(kChipSize, kChipSize);
        pixmap.fill(Qt::transparent);

        QPainter painter(&pixmap);
        painter.setRenderHint(QPainter::Antialiasing, true);
        painter.setPen(QPen(QColor(132, 138, 145), 1.0));
        painter.setBrush(color);
        painter.drawRect(QRectF(1.0, 1.0, kChipSize - 2.0, kChipSize - 2.0));
        return QIcon(pixmap);
    }

}

CadToolPanelWidget::CadToolPanelWidget(QWidget* parent)
    : QWidget(parent)
{
    setObjectName(QStringLiteral("cadToolPanelRoot"));
    buildUi();
    applyTheme();
}

void CadToolPanelWidget::setLayerNames(const QStringList& layerNames, const QMap<QString, QColor>& layerColors)
{
    const QString currentLayerName = m_layerComboBox->currentText().trimmed().isEmpty()
        ? QStringLiteral("0")
        : m_layerComboBox->currentText().trimmed();

    const QString currentPropertyLayerName = m_propertyLayerComboBox->currentText().trimmed().isEmpty()
        ? QStringLiteral("0")
        : m_propertyLayerComboBox->currentText().trimmed();

    m_updatingUi = true;
    m_layerColors = layerColors;
    m_layerComboBox->clear();
    m_propertyLayerComboBox->clear();
    m_layerComboBox->addItems(layerNames);
    m_propertyLayerComboBox->addItems(layerNames);
    updateLayerComboIcons();
    m_updatingUi = false;

    setActiveLayerName(!currentPropertyLayerName.isEmpty() ? currentPropertyLayerName : currentLayerName);
}

void CadToolPanelWidget::setLayerStatusText(const QString& text)
{
    m_layerStatusLabel->setText(text);
}

void CadToolPanelWidget::setPropertyStatusText(const QString& text)
{
    m_propertyStatusLabel->setText(text);
}

void CadToolPanelWidget::setActiveLayerName(const QString& layerName)
{
    const QString normalizedLayerName = layerName.trimmed().isEmpty() ? QStringLiteral("0") : layerName.trimmed();
    m_updatingUi = true;
    const int layerIndex = m_layerComboBox->findText(normalizedLayerName);
    const int propertyLayerIndex = m_propertyLayerComboBox->findText(normalizedLayerName);

    if (layerIndex >= 0)
    {
        m_layerComboBox->setCurrentIndex(layerIndex);
    }

    if (propertyLayerIndex >= 0)
    {
        m_propertyLayerComboBox->setCurrentIndex(propertyLayerIndex);
    }

    m_updatingUi = false;
}

void CadToolPanelWidget::setActiveColorState(const QColor& color, int colorIndex, const QColor& byLayerColor)
{
    m_updatingUi = true;
    updateColorComboIcons(color, byLayerColor);
    setComboCurrentByData(m_colorComboBox, colorIndex);
    m_updatingUi = false;
}

void CadToolPanelWidget::setMoveEnabled(bool enabled)
{
    m_moveButton->setEnabled(enabled);
}

void CadToolPanelWidget::setTheme(const AppThemeColors& theme)
{
    m_theme = theme;
    applyTheme();
}

void CadToolPanelWidget::buildUi()
{
    m_drawMoreMenu = new QMenu(this);
    m_drawPointAction = m_drawMoreMenu->addAction(QStringLiteral("点"));
    connect(m_drawPointAction, &QAction::triggered, this, [this]() { emit drawRequested(DrawType::Point); });

    QHBoxLayout* rootLayout = new QHBoxLayout(this);
    rootLayout->setContentsMargins(0, 0, 0, 0);
    rootLayout->setSpacing(0);

    rootLayout->addWidget(buildPanelFrame(QStringLiteral("绘图"), buildDrawPanel(), -1, m_drawMoreMenu), 0, Qt::AlignLeft | Qt::AlignTop);
    rootLayout->addWidget(buildDivider(), 0, Qt::AlignLeft | Qt::AlignVCenter);
    rootLayout->addWidget(buildPanelFrame(QStringLiteral("修改"), buildModifyPanel()), 0, Qt::AlignLeft | Qt::AlignTop);
    rootLayout->addWidget(buildDivider(), 0, Qt::AlignLeft | Qt::AlignVCenter);
    rootLayout->addWidget(buildPanelFrame(QStringLiteral("图层"), buildLayerPanel(), 176), 0, Qt::AlignLeft | Qt::AlignTop);
    rootLayout->addWidget(buildDivider(), 0, Qt::AlignLeft | Qt::AlignVCenter);
    rootLayout->addWidget(buildPanelFrame(QStringLiteral("特性"), buildPropertyPanel(), 226), 0, Qt::AlignLeft | Qt::AlignTop);
    rootLayout->addStretch(1);
}

void CadToolPanelWidget::applyTheme()
{
    setStyleSheet
    (
        QStringLiteral
        (
            "#cadToolPanelRoot { background: transparent; }"
            "QLabel { color: %1; }"
            "QLabel[panelTitle=\"true\"] {"
            " color: %2;"
            " font-size: 11px;"
            " padding-bottom: 1px;"
            "}"
            "QToolButton[panelLauncher=\"true\"] {"
            " border: none;"
            " padding: 0px;"
            " margin: 0px;"
            " color: %2;"
            " background: transparent;"
            "}"
            "QToolButton[panelLauncher=\"true\"]:hover {"
            " background: %3;"
            " color: %1;"
            "}"
            "QToolButton[ribbonButton=\"true\"] {"
            " border: 1px solid transparent;"
            " border-radius: 2px;"
            " padding: 2px 3px 2px 3px;"
            " background: transparent;"
            " color: %1;"
            " font-size: 9px;"
            "}"
            "QToolButton[ribbonButton=\"true\"]:hover {"
            " border-color: %4;"
            " background: %5;"
            "}"
            "QToolButton[ribbonButton=\"true\"]:pressed {"
            " border-color: %6;"
            " background: %3;"
            "}"
            "QComboBox {"
            " background-color: %7;"
            " color: %1;"
            " border: 1px solid %4;"
            " border-radius: 2px;"
            " padding: 1px 22px 1px 6px;"
            "}"
            "QComboBox:hover {"
            " border-color: %6;"
            "}"
            "QComboBox::drop-down {"
            " subcontrol-origin: padding;"
            " subcontrol-position: top right;"
            " width: 18px;"
            " border: none;"
            " background: transparent;"
            "}"
            "QComboBox QAbstractItemView {"
            " background-color: %7;"
            " color: %1;"
            " border: 1px solid %4;"
            " selection-background-color: %8;"
            " selection-color: %9;"
            "}"
        )
        .arg(cssRgb(m_theme.textPrimaryColor))
        .arg(cssRgb(m_theme.textSecondaryColor))
        .arg(cssRgb(m_theme.pressedBackgroundColor))
        .arg(cssRgb(m_theme.borderColor))
        .arg(cssRgb(m_theme.hoverBackgroundColor))
        .arg(cssRgb(m_theme.borderStrongColor))
        .arg(cssRgb(m_theme.surfaceBackground))
        .arg(cssRgb(m_theme.accentColor))
        .arg(cssRgb(m_theme.accentTextColor))
    );

    for (QFrame* divider : m_dividers)
    {
        if (divider != nullptr)
        {
            divider->setStyleSheet(QStringLiteral("background-color: %1;").arg(cssRgb(m_theme.borderColor)));
        }
    }

    for (QToolButton* button : m_drawButtons)
    {
        if (button == nullptr)
        {
            continue;
        }

        button->setIcon(buildRibbonIcon(static_cast<DrawType>(button->property("drawTypeId").toInt()), m_theme.accentColor));
    }

    if (m_moveButton != nullptr)
    {
        m_moveButton->setIcon(buildMoveIcon(m_theme.accentColor));
    }

    if (m_drawPointAction != nullptr)
    {
        m_drawPointAction->setIcon(buildRibbonIcon(DrawType::Point, m_theme.accentColor));
    }

    if (m_drawMoreMenu != nullptr)
    {
        m_drawMoreMenu->setStyleSheet
        (
            QStringLiteral
            (
                "QMenu {"
                " background-color: %1;"
                " color: %2;"
                " border: 1px solid %3;"
                " padding: 4px 0px;"
                "}"
                "QMenu::item {"
                " padding: 5px 20px 5px 24px;"
                "}"
                "QMenu::item:selected {"
                " background-color: %4;"
                " color: %5;"
                "}"
                "QMenu::separator {"
                " height: 1px;"
                " margin: 4px 8px;"
                " background: %3;"
                "}"
            )
            .arg(cssRgb(m_theme.surfaceBackground))
            .arg(cssRgb(m_theme.textPrimaryColor))
            .arg(cssRgb(m_theme.borderColor))
            .arg(cssRgb(m_theme.accentColor))
            .arg(cssRgb(m_theme.accentTextColor))
        );
    }
}

QWidget* CadToolPanelWidget::buildPanelFrame(const QString& title, QWidget* contentWidget, int preferredWidth, QMenu* launcherMenu)
{
    QWidget* panel = new QWidget(this);
    panel->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    panel->setMinimumHeight(kPanelHeight);
    panel->setMaximumHeight(kPanelHeight);

    QVBoxLayout* layout = new QVBoxLayout(panel);
    layout->setContentsMargins(6, 4, 6, 0);
    layout->setSpacing(0);
    layout->addWidget(contentWidget, 1);

    QWidget* footerWidget = new QWidget(panel);
    footerWidget->setFixedHeight(kFooterHeight);
    QHBoxLayout* footerLayout = new QHBoxLayout(footerWidget);
    footerLayout->setContentsMargins(0, 0, 0, 0);
    footerLayout->setSpacing(0);

    QWidget* titleCluster = new QWidget(footerWidget);
    QHBoxLayout* titleClusterLayout = new QHBoxLayout(titleCluster);
    titleClusterLayout->setContentsMargins(0, 0, 0, 0);
    titleClusterLayout->setSpacing(1);

    QLabel* titleLabel = new QLabel(title, titleCluster);
    titleLabel->setAlignment(Qt::AlignHCenter | Qt::AlignBottom);
    titleLabel->setProperty("panelTitle", true);
    titleClusterLayout->addWidget(titleLabel, 0, Qt::AlignVCenter);

    QToolButton* launcherButton = new QToolButton(titleCluster);
    launcherButton->setProperty("panelLauncher", true);
    launcherButton->setArrowType(Qt::DownArrow);
    launcherButton->setAutoRaise(true);
    launcherButton->setFixedSize(kLauncherSize, kLauncherSize);
    launcherButton->setEnabled(launcherMenu != nullptr);

    if (launcherMenu != nullptr)
    {
        launcherButton->setMenu(launcherMenu);
        launcherButton->setPopupMode(QToolButton::InstantPopup);
    }

    titleClusterLayout->addWidget(launcherButton, 0, Qt::AlignVCenter);

    footerLayout->addStretch(1);
    footerLayout->addWidget(titleCluster, 0, Qt::AlignHCenter | Qt::AlignBottom);
    footerLayout->addStretch(1);

    layout->addWidget(footerWidget, 0, Qt::AlignBottom);

    const int resolvedWidth = preferredWidth > 0
        ? preferredWidth
        : std::max(contentWidget->sizeHint().width(), titleCluster->sizeHint().width())
            + layout->contentsMargins().left() + layout->contentsMargins().right();

    panel->setMinimumWidth(resolvedWidth);
    panel->setMaximumWidth(resolvedWidth);

    if (launcherMenu != nullptr)
    {
        launcherMenu->setMinimumWidth(resolvedWidth);
    }

    return panel;
}

QWidget* CadToolPanelWidget::buildDivider()
{
    QFrame* divider = new QFrame();
    divider->setFixedWidth(1);
    divider->setFixedHeight(kDividerHeight);
    divider->setFrameShape(QFrame::NoFrame);
    m_dividers.push_back(divider);
    return divider;
}

QWidget* CadToolPanelWidget::buildDrawPanel()
{
    QWidget* panel = new QWidget(this);
    QGridLayout* layout = new QGridLayout(panel);
    layout->setContentsMargins(1, 4, 1, 2);
    layout->setHorizontalSpacing(3);
    layout->setVerticalSpacing(3);
    layout->setSizeConstraint(QLayout::SetFixedSize);

    addDrawButton(panel, QStringLiteral("直线"), DrawType::Line, 0, 0);
    addDrawButton(panel, QStringLiteral("圆"), DrawType::Circle, 0, 1);
    addDrawButton(panel, QStringLiteral("圆弧"), DrawType::Arc, 0, 2);
    addDrawButton(panel, QStringLiteral("椭圆"), DrawType::Ellipse, 1, 0);
    addDrawButton(panel, QStringLiteral("多段线"), DrawType::Polyline, 1, 1);
    addDrawButton(panel, QStringLiteral("轻量线"), DrawType::LWPolyline, 1, 2);
    layout->setAlignment(Qt::AlignLeft | Qt::AlignTop);
    return panel;
}

QWidget* CadToolPanelWidget::buildModifyPanel()
{
    QWidget* panel = new QWidget(this);
    QHBoxLayout* layout = new QHBoxLayout(panel);
    layout->setContentsMargins(1, 4, 1, 2);
    layout->setSpacing(0);
    layout->setSizeConstraint(QLayout::SetFixedSize);

    m_moveButton = new QToolButton(panel);
    m_moveButton->setProperty("ribbonButton", true);
    m_moveButton->setText(QStringLiteral("移动"));
    m_moveButton->setIcon(buildMoveIcon(m_theme.accentColor));
    m_moveButton->setIconSize(QSize(kRibbonIconSize, kRibbonIconSize));
    m_moveButton->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    m_moveButton->setFixedSize(56, kRibbonButtonHeight);
    m_moveButton->setEnabled(false);
    connect(m_moveButton, &QToolButton::clicked, this, &CadToolPanelWidget::moveRequested);

    layout->addWidget(m_moveButton);
    layout->setAlignment(Qt::AlignLeft | Qt::AlignTop);
    return panel;
}

QWidget* CadToolPanelWidget::buildLayerPanel()
{
    QWidget* panel = new QWidget(this);
    QVBoxLayout* layout = new QVBoxLayout(panel);
    layout->setContentsMargins(2, 4, 2, 2);
    layout->setSpacing(3);

    m_layerStatusLabel = new QLabel(QStringLiteral("当前默认绘图图层"), panel);
    m_layerStatusLabel->setWordWrap(false);
    m_layerStatusLabel->setFixedHeight(16);
    m_layerStatusLabel->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    layout->addWidget(m_layerStatusLabel);

    m_layerComboBox = new QComboBox(panel);
    m_layerComboBox->setEditable(false);
    m_layerComboBox->setFixedHeight(kComboHeight);
    layout->addWidget(m_layerComboBox);
    layout->addStretch(1);

    connect
    (
        m_layerComboBox,
        QOverload<int>::of(&QComboBox::activated),
        this,
        [this](int)
        {
            commitLayerChange(m_layerComboBox);
        }
    );

    return panel;
}

QWidget* CadToolPanelWidget::buildPropertyPanel()
{
    QWidget* panel = new QWidget(this);
    QVBoxLayout* layout = new QVBoxLayout(panel);
    layout->setContentsMargins(2, 4, 2, 2);
    layout->setSpacing(3);

    m_propertyStatusLabel = new QLabel(QStringLiteral("当前默认绘图特性"), panel);
    m_propertyStatusLabel->setWordWrap(false);
    m_propertyStatusLabel->setFixedHeight(16);
    m_propertyStatusLabel->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    layout->addWidget(m_propertyStatusLabel);

    QWidget* layerRowWidget = new QWidget(panel);
    QHBoxLayout* layerRowLayout = new QHBoxLayout(layerRowWidget);
    layerRowLayout->setContentsMargins(0, 0, 0, 0);
    layerRowLayout->setSpacing(4);

    QLabel* layerLabel = new QLabel(QStringLiteral("图层"), layerRowWidget);
    layerLabel->setFixedWidth(28);
    layerRowLayout->addWidget(layerLabel, 0, Qt::AlignVCenter);

    m_propertyLayerComboBox = new QComboBox(layerRowWidget);
    m_propertyLayerComboBox->setEditable(false);
    m_propertyLayerComboBox->setFixedHeight(kComboHeight);
    layerRowLayout->addWidget(m_propertyLayerComboBox, 1);

    layout->addWidget(layerRowWidget);

    connect
    (
        m_propertyLayerComboBox,
        QOverload<int>::of(&QComboBox::activated),
        this,
        [this](int)
        {
            commitLayerChange(m_propertyLayerComboBox);
        }
    );

    QWidget* colorRowWidget = new QWidget(panel);
    QHBoxLayout* colorRowLayout = new QHBoxLayout(colorRowWidget);
    colorRowLayout->setContentsMargins(0, 0, 0, 0);
    colorRowLayout->setSpacing(4);

    QLabel* colorLabel = new QLabel(QStringLiteral("颜色"), colorRowWidget);
    colorLabel->setFixedWidth(28);
    colorRowLayout->addWidget(colorLabel, 0, Qt::AlignVCenter);

    m_colorComboBox = new QComboBox(colorRowWidget);
    addColorOption(m_colorComboBox, QStringLiteral("ByLayer"), kColorByLayer);
    addColorOption(m_colorComboBox, QStringLiteral("红"), 1);
    addColorOption(m_colorComboBox, QStringLiteral("黄"), 2);
    addColorOption(m_colorComboBox, QStringLiteral("绿"), 3);
    addColorOption(m_colorComboBox, QStringLiteral("青"), 4);
    addColorOption(m_colorComboBox, QStringLiteral("蓝"), 5);
    addColorOption(m_colorComboBox, QStringLiteral("洋红"), 6);
    addColorOption(m_colorComboBox, QStringLiteral("白"), 7);
    addColorOption(m_colorComboBox, QStringLiteral("灰"), 8);
    addColorOption(m_colorComboBox, QStringLiteral("浅灰"), 9);
    addColorOption(m_colorComboBox, QStringLiteral("真彩色"), kColorTrueColor);
    m_colorComboBox->setFixedHeight(kComboHeight);
    colorRowLayout->addWidget(m_colorComboBox, 1);

    layout->addWidget(colorRowWidget);
    layout->addStretch(1);

    connect
    (
        m_colorComboBox,
        QOverload<int>::of(&QComboBox::currentIndexChanged),
        this,
        [this](int index)
        {
            if (m_updatingUi || index < 0)
            {
                return;
            }

            emit colorChangeRequested(m_colorComboBox->itemData(index).toInt());
        }
    );

    return panel;
}

void CadToolPanelWidget::addDrawButton(QWidget* parent, const QString& text, DrawType drawType, int row, int column)
{
    QGridLayout* layout = qobject_cast<QGridLayout*>(parent->layout());

    if (layout == nullptr)
    {
        return;
    }

    QToolButton* button = new QToolButton(parent);
    button->setProperty("ribbonButton", true);
    button->setProperty("drawTypeId", static_cast<int>(drawType));
    button->setText(text);
    button->setIcon(buildRibbonIcon(drawType, m_theme.accentColor));
    button->setIconSize(QSize(kRibbonIconSize, kRibbonIconSize));
    button->setFixedSize(kRibbonButtonWidth, kRibbonButtonHeight);
    button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    button->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    connect(button, &QToolButton::clicked, this, [this, drawType]() { emit drawRequested(drawType); });
    layout->addWidget(button, row, column);
    m_drawButtons.push_back(button);
}

void CadToolPanelWidget::commitLayerChange(QComboBox* comboBox)
{
    if (m_updatingUi || comboBox == nullptr)
    {
        return;
    }

    const QString layerName = comboBox->currentText().trimmed().isEmpty()
        ? QStringLiteral("0")
        : comboBox->currentText().trimmed();

    emit layerChangeRequested(layerName);
}

void CadToolPanelWidget::updateLayerComboIcons()
{
    const auto applyIcons =
        [this](QComboBox* comboBox)
        {
            if (comboBox == nullptr)
            {
                return;
            }

            for (int index = 0; index < comboBox->count(); ++index)
            {
                const QString layerName = comboBox->itemText(index).trimmed();
                const QColor color = m_layerColors.value(layerName, QColor(Qt::white));
                comboBox->setItemIcon(index, buildColorChipIcon(color));
            }
        };

    applyIcons(m_layerComboBox);
    applyIcons(m_propertyLayerComboBox);
}

void CadToolPanelWidget::updateColorComboIcons(const QColor& activeColor, const QColor& byLayerColor)
{
    if (m_colorComboBox == nullptr)
    {
        return;
    }

    const struct
    {
        int colorIndex;
        QColor color;
    } colorSpecs[] =
    {
        { kColorByLayer, byLayerColor.isValid() ? byLayerColor : QColor(Qt::white) },
        { 1, QColor(255, 0, 0) },
        { 2, QColor(255, 255, 0) },
        { 3, QColor(0, 255, 0) },
        { 4, QColor(0, 255, 255) },
        { 5, QColor(0, 0, 255) },
        { 6, QColor(255, 0, 255) },
        { 7, QColor(255, 255, 255) },
        { 8, QColor(128, 128, 128) },
        { 9, QColor(192, 192, 192) },
        { kColorTrueColor, activeColor.isValid() ? activeColor : QColor(Qt::white) }
    };

    for (const auto& colorSpec : colorSpecs)
    {
        const int index = m_colorComboBox->findData(colorSpec.colorIndex);

        if (index >= 0)
        {
            m_colorComboBox->setItemIcon(index, buildColorChipIcon(colorSpec.color));
        }
    }
}

void CadToolPanelWidget::setComboCurrentByData(QComboBox* comboBox, int value)
{
    if (comboBox == nullptr)
    {
        return;
    }

    const int index = comboBox->findData(value);

    if (index >= 0)
    {
        comboBox->setCurrentIndex(index);
        return;
    }

    const int trueColorIndex = comboBox->findData(kColorTrueColor);
    comboBox->setCurrentIndex(trueColorIndex);
}
