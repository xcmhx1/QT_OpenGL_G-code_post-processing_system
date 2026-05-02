#include "pch.h"

#include "CadToolPanelWidget.h"

#include <QCheckBox>
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
#include <QTabBar>
#include <QTabWidget>
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
        case DrawType::Xline:
        {
            painter.drawLine(QPointF(2.5, 13.5), QPointF(13.5, 2.5));
            QPainterPath headA;
            headA.moveTo(1.7, 14.4);
            headA.lineTo(2.9, 11.7);
            headA.lineTo(4.4, 13.1);
            headA.closeSubpath();
            painter.fillPath(headA, pen.color());
            QPainterPath headB;
            headB.moveTo(14.3, 1.6);
            headB.lineTo(11.6, 2.8);
            headB.lineTo(13.0, 4.3);
            headB.closeSubpath();
            painter.fillPath(headB, pen.color());
            break;
        }
        case DrawType::Rectangle:
            painter.drawRect(QRectF(3.5, 4.5, 10.0, 8.0));
            break;
        case DrawType::Polygon:
        {
            QPainterPath polygonPath;
            polygonPath.moveTo(9.0, 2.8);
            polygonPath.lineTo(14.2, 6.0);
            polygonPath.lineTo(12.9, 12.3);
            polygonPath.lineTo(5.1, 12.3);
            polygonPath.lineTo(3.8, 6.0);
            polygonPath.closeSubpath();
            painter.drawPath(polygonPath);
            break;
        }
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

    QIcon buildDeleteIcon(const QColor& strokeColor)
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
        painter.drawLine(QPointF(5.0, 5.0), QPointF(13.0, 13.0));
        painter.drawLine(QPointF(13.0, 5.0), QPointF(5.0, 13.0));
        painter.drawRect(QRectF(3.5, 3.5, 11.0, 11.0));
        return QIcon(pixmap);
    }

    QIcon buildRotateIcon(const QColor& strokeColor)
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
        painter.drawArc(QRectF(3.0, 3.0, 10.0, 10.0), 35 * 16, 270 * 16);
        painter.drawEllipse(QPointF(8.0, 8.0), 1.2, 1.2);

        QPainterPath arrowHead;
        arrowHead.moveTo(12.8, 4.5);
        arrowHead.lineTo(14.8, 4.4);
        arrowHead.lineTo(13.6, 6.2);
        arrowHead.closeSubpath();
        painter.fillPath(arrowHead, pen.color());
        return QIcon(pixmap);
    }

    QIcon buildCopyIcon(const QColor& strokeColor)
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
        painter.drawRect(QRectF(3.0, 5.0, 7.0, 7.0));
        painter.drawRect(QRectF(6.0, 2.0, 7.0, 7.0));
        return QIcon(pixmap);
    }

    QIcon buildScaleIcon(const QColor& strokeColor)
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
        painter.drawRect(QRectF(4.0, 4.0, 5.0, 5.0));
        painter.drawRect(QRectF(8.0, 8.0, 5.0, 5.0));
        painter.drawLine(QPointF(9.5, 9.5), QPointF(14.5, 14.5));

        QPainterPath arrowHead;
        arrowHead.moveTo(14.8, 14.8);
        arrowHead.lineTo(12.2, 14.3);
        arrowHead.lineTo(14.3, 12.2);
        arrowHead.closeSubpath();
        painter.fillPath(arrowHead, pen.color());
        return QIcon(pixmap);
    }

    QIcon buildArrayIcon(const QColor& strokeColor)
    {
        QPixmap pixmap(kRibbonIconSize, kRibbonIconSize);
        pixmap.fill(Qt::transparent);

        QPainter painter(&pixmap);
        painter.setRenderHint(QPainter::Antialiasing, true);
        QPen pen(strokeColor);
        pen.setWidthF(1.2);
        pen.setCapStyle(Qt::RoundCap);
        pen.setJoinStyle(Qt::RoundJoin);
        painter.setPen(pen);

        for (int row = 0; row < 3; ++row)
        {
            for (int column = 0; column < 3; ++column)
            {
                painter.drawRect(QRectF(2.5 + column * 4.0, 2.5 + row * 4.0, 3.0, 3.0));
            }
        }

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

void CadToolPanelWidget::setModifyActionsEnabled(bool enabled)
{
    if (m_moveButton != nullptr)
    {
        m_moveButton->setEnabled(enabled);
    }

    if (m_deleteButton != nullptr)
    {
        m_deleteButton->setEnabled(enabled);
    }

    if (m_rotateButton != nullptr)
    {
        m_rotateButton->setEnabled(enabled);
    }

    if (m_copyButton != nullptr)
    {
        m_copyButton->setEnabled(enabled);
    }

    if (m_scaleButton != nullptr)
    {
        m_scaleButton->setEnabled(enabled);
    }

    if (m_arrayButton != nullptr)
    {
        m_arrayButton->setEnabled(enabled);
    }

    if (m_rectangularArrayAction != nullptr)
    {
        m_rectangularArrayAction->setEnabled(enabled);
    }

    if (m_circularArrayAction != nullptr)
    {
        m_circularArrayAction->setEnabled(enabled);
    }

    if (m_mirrorAction != nullptr)
    {
        m_mirrorAction->setEnabled(enabled);
    }

    if (m_offsetAction != nullptr)
    {
        m_offsetAction->setEnabled(enabled);
    }

    if (m_trimAction != nullptr)
    {
        m_trimAction->setEnabled(enabled);
    }

    if (m_extendAction != nullptr)
    {
        m_extendAction->setEnabled(enabled);
    }

    if (m_joinAction != nullptr)
    {
        m_joinAction->setEnabled(enabled);
    }

    if (m_filletAction != nullptr)
    {
        m_filletAction->setEnabled(enabled);
    }

    if (m_chamferAction != nullptr)
    {
        m_chamferAction->setEnabled(enabled);
    }
}

void CadToolPanelWidget::setTheme(const AppThemeColors& theme)
{
    m_theme = theme;
    applyTheme();
}

void CadToolPanelWidget::setGCodeModeSelection(GCodeModeSelection selection)
{
    if (m_gcodeModeComboBox == nullptr)
    {
        return;
    }

    m_updatingUi = true;
    m_gcodeModeComboBox->setCurrentIndex(static_cast<int>(selection));
    m_updatingUi = false;
}

void CadToolPanelWidget::setAvailableProfiles(const QList<QPair<QString, QString>>& profiles)
{
    if (m_profileComboBox == nullptr)
    {
        return;
    }

    const QString currentProfileId = m_profileComboBox->currentData().toString();
    m_updatingUi = true;
    m_profileComboBox->clear();

    for (const QPair<QString, QString>& profile : profiles)
    {
        m_profileComboBox->addItem(profile.second, profile.first);
    }

    const int currentIndex = m_profileComboBox->findData(currentProfileId);

    if (currentIndex >= 0)
    {
        m_profileComboBox->setCurrentIndex(currentIndex);
    }

    m_updatingUi = false;
}

void CadToolPanelWidget::setCurrentProfileSelection(const QString& profileId)
{
    if (m_profileComboBox == nullptr)
    {
        return;
    }

    const int index = m_profileComboBox->findData(profileId);

    if (index < 0)
    {
        return;
    }

    m_updatingUi = true;
    m_profileComboBox->setCurrentIndex(index);
    m_updatingUi = false;
}

void CadToolPanelWidget::setAutoDeduplicateEnabled(bool enabled)
{
    if (m_autoDeduplicateCheckBox == nullptr)
    {
        return;
    }

    m_updatingUi = true;
    m_autoDeduplicateCheckBox->setChecked(enabled);
    m_updatingUi = false;
}

bool CadToolPanelWidget::autoDeduplicateEnabled() const
{
    return m_autoDeduplicateCheckBox != nullptr && m_autoDeduplicateCheckBox->isChecked();
}

void CadToolPanelWidget::setUseDxfFileNameEnabled(bool enabled)
{
    if (m_useDxfFileNameCheckBox == nullptr)
    {
        return;
    }

    m_updatingUi = true;
    m_useDxfFileNameCheckBox->setChecked(enabled);
    m_updatingUi = false;
}

bool CadToolPanelWidget::useDxfFileNameEnabled() const
{
    return m_useDxfFileNameCheckBox != nullptr && m_useDxfFileNameCheckBox->isChecked();
}

void CadToolPanelWidget::setUseDefaultImportPathEnabled(bool enabled)
{
    if (m_useDefaultImportPathCheckBox == nullptr)
    {
        return;
    }

    m_updatingUi = true;
    m_useDefaultImportPathCheckBox->setChecked(enabled);
    m_updatingUi = false;
}

bool CadToolPanelWidget::useDefaultImportPathEnabled() const
{
    return m_useDefaultImportPathCheckBox != nullptr && m_useDefaultImportPathCheckBox->isChecked();
}

void CadToolPanelWidget::setUseDefaultExportPathEnabled(bool enabled)
{
    if (m_useDefaultExportPathCheckBox == nullptr)
    {
        return;
    }

    m_updatingUi = true;
    m_useDefaultExportPathCheckBox->setChecked(enabled);
    m_updatingUi = false;
}

bool CadToolPanelWidget::useDefaultExportPathEnabled() const
{
    return m_useDefaultExportPathCheckBox != nullptr && m_useDefaultExportPathCheckBox->isChecked();
}

void CadToolPanelWidget::buildUi()
{
    m_drawMoreMenu = new QMenu(this);
    m_drawPointAction = m_drawMoreMenu->addAction(QStringLiteral("点"));
    connect(m_drawPointAction, &QAction::triggered, this, [this]() { emit drawRequested(DrawType::Point); });
    m_drawXlineAction = m_drawMoreMenu->addAction(QStringLiteral("构造线"));
    connect(m_drawXlineAction, &QAction::triggered, this, [this]() { emit drawRequested(DrawType::Xline); });
    m_drawRectangleAction = m_drawMoreMenu->addAction(QStringLiteral("矩形"));
    connect(m_drawRectangleAction, &QAction::triggered, this, [this]() { emit drawRequested(DrawType::Rectangle); });
    m_drawPolygonAction = m_drawMoreMenu->addAction(QStringLiteral("多边形"));
    connect(m_drawPolygonAction, &QAction::triggered, this, [this]() { emit drawRequested(DrawType::Polygon); });

    m_modifyMoreMenu = new QMenu(this);
    m_rectangularArrayAction = m_modifyMoreMenu->addAction(QStringLiteral("矩形阵列"));
    connect(m_rectangularArrayAction, &QAction::triggered, this, [this]() { emit arrayRequested(); });
    m_circularArrayAction = m_modifyMoreMenu->addAction(QStringLiteral("环形阵列"));
    connect(m_circularArrayAction, &QAction::triggered, this, [this]() { emit circularArrayRequested(); });
    m_modifyMoreMenu->addSeparator();
    m_mirrorAction = m_modifyMoreMenu->addAction(QStringLiteral("镜像"));
    connect(m_mirrorAction, &QAction::triggered, this, [this]() { emit mirrorRequested(); });
    m_offsetAction = m_modifyMoreMenu->addAction(QStringLiteral("偏移"));
    connect(m_offsetAction, &QAction::triggered, this, [this]() { emit offsetRequested(); });
    m_trimAction = m_modifyMoreMenu->addAction(QStringLiteral("修剪"));
    connect(m_trimAction, &QAction::triggered, this, [this]() { emit trimRequested(); });
    m_extendAction = m_modifyMoreMenu->addAction(QStringLiteral("延申"));
    connect(m_extendAction, &QAction::triggered, this, [this]() { emit extendRequested(); });
    m_joinAction = m_modifyMoreMenu->addAction(QStringLiteral("合并"));
    connect(m_joinAction, &QAction::triggered, this, [this]() { emit joinRequested(); });
    m_filletAction = m_modifyMoreMenu->addAction(QStringLiteral("圆角"));
    connect(m_filletAction, &QAction::triggered, this, [this]() { emit filletRequested(); });
    m_chamferAction = m_modifyMoreMenu->addAction(QStringLiteral("直角（倒角）"));
    connect(m_chamferAction, &QAction::triggered, this, [this]() { emit chamferRequested(); });
    setModifyActionsEnabled(false);

    QVBoxLayout* rootLayout = new QVBoxLayout(this);
    rootLayout->setContentsMargins(0, 0, 0, 0);
    rootLayout->setSpacing(0);

    m_tabWidget = new QTabWidget(this);
    m_tabWidget->setDocumentMode(true);
    m_tabWidget->tabBar()->setExpanding(false);
    m_tabWidget->tabBar()->setUsesScrollButtons(false);
    rootLayout->addWidget(m_tabWidget);

    QWidget* defaultTab = new QWidget(m_tabWidget);
    QHBoxLayout* defaultLayout = new QHBoxLayout(defaultTab);
    defaultLayout->setContentsMargins(0, 0, 0, 0);
    defaultLayout->setSpacing(0);
    defaultLayout->addWidget(buildPanelFrame(QStringLiteral("绘图"), buildDrawPanel(), -1, m_drawMoreMenu), 0, Qt::AlignLeft | Qt::AlignTop);
    defaultLayout->addWidget(buildDivider(), 0, Qt::AlignLeft | Qt::AlignVCenter);
    defaultLayout->addWidget(buildPanelFrame(QStringLiteral("修改"), buildModifyPanel(), -1, m_modifyMoreMenu), 0, Qt::AlignLeft | Qt::AlignTop);
    defaultLayout->addWidget(buildDivider(), 0, Qt::AlignLeft | Qt::AlignVCenter);
    defaultLayout->addWidget(buildPanelFrame(QStringLiteral("图层"), buildLayerPanel(), 176), 0, Qt::AlignLeft | Qt::AlignTop);
    defaultLayout->addWidget(buildDivider(), 0, Qt::AlignLeft | Qt::AlignVCenter);
    defaultLayout->addWidget(buildPanelFrame(QStringLiteral("特性"), buildPropertyPanel(), 226), 0, Qt::AlignLeft | Qt::AlignTop);
    defaultLayout->addStretch(1);
    m_tabWidget->addTab(defaultTab, QStringLiteral("默认"));

    m_tabWidget->addTab(buildMachiningPanel(), QStringLiteral("机加工"));
}

void CadToolPanelWidget::applyTheme()
{
    setStyleSheet
    (
        QStringLiteral
        (
            "#cadToolPanelRoot { background: transparent; }"
            "QTabWidget::pane { border: none; background: transparent; margin-top: 2px; }"
            "QTabBar::tab { background: %7; color: %2; border: 1px solid %4; border-bottom: none; padding: 4px 8px; min-width: 46px; font-size: 10px; }"
            "QTabBar::tab:selected { background: %5; color: %9; }"
            "QTabBar::tab:!selected { margin-top: 2px; }"
            "QLabel { color: %1; font-size: 10px; }"
            "QLabel[panelTitle=\"true\"] {"
            " color: %2;"
            " font-size: 12px;"
            " padding-bottom: 1px;"
            "}"
            "QLabel[machiningFieldLabel=\"true\"] {"
            " color: %2;"
            " font-size: 11px;"
            " font-weight: 500;"
            "}"
            "QCheckBox[machiningOption=\"true\"] {"
            " color: %2;"
            " font-size: 10px;"
            " spacing: 4px;"
            " padding: 0px 3px 0px 0px;"
            "}"
            "QCheckBox[machiningOption=\"true\"]::indicator {"
            " width: 13px;"
            " height: 13px;"
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
            "QToolButton[machiningButton=\"true\"] {"
            " border: 1px solid %4;"
            " border-radius: 2px;"
            " padding: 3px 10px;"
            " background: %7;"
            " color: %1;"
            " font-size: 11px;"
            " text-align: left;"
            "}"
            "QToolButton[machiningButton=\"true\"]:hover {"
            " border-color: %6;"
            " background: %5;"
            "}"
            "QToolButton[machiningButton=\"true\"]:pressed {"
            " border-color: %6;"
            " background: %3;"
            "}"
            "QComboBox {"
            " background-color: %7;"
            " color: %1;"
            " border: 1px solid %4;"
            " border-radius: 2px;"
            " padding: 1px 22px 1px 6px;"
            " font-size: 11px;"
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

    if (m_deleteButton != nullptr)
    {
        m_deleteButton->setIcon(buildDeleteIcon(m_theme.accentColor));
    }

    if (m_rotateButton != nullptr)
    {
        m_rotateButton->setIcon(buildRotateIcon(m_theme.accentColor));
    }

    if (m_copyButton != nullptr)
    {
        m_copyButton->setIcon(buildCopyIcon(m_theme.accentColor));
    }

    if (m_scaleButton != nullptr)
    {
        m_scaleButton->setIcon(buildScaleIcon(m_theme.accentColor));
    }

    if (m_arrayButton != nullptr)
    {
        m_arrayButton->setIcon(buildArrayIcon(m_theme.accentColor));
    }

    if (m_drawPointAction != nullptr)
    {
        m_drawPointAction->setIcon(buildRibbonIcon(DrawType::Point, m_theme.accentColor));
    }

    if (m_drawXlineAction != nullptr)
    {
        m_drawXlineAction->setIcon(buildRibbonIcon(DrawType::Xline, m_theme.accentColor));
    }

    if (m_drawRectangleAction != nullptr)
    {
        m_drawRectangleAction->setIcon(buildRibbonIcon(DrawType::Rectangle, m_theme.accentColor));
    }

    if (m_drawPolygonAction != nullptr)
    {
        m_drawPolygonAction->setIcon(buildRibbonIcon(DrawType::Polygon, m_theme.accentColor));
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

QWidget* CadToolPanelWidget::buildPanelFrame(const QString& title, QWidget* contentWidget, int preferredWidth, QMenu* launcherMenu, bool flexibleWidth)
{
    QWidget* panel = new QWidget(this);
    panel->setSizePolicy(flexibleWidth ? QSizePolicy::Preferred : QSizePolicy::Fixed, QSizePolicy::Fixed);
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

    if (flexibleWidth)
    {
        panel->setMinimumWidth(std::max(112, resolvedWidth - 24));
        panel->setMaximumWidth(QWIDGETSIZE_MAX);
    }
    else
    {
        panel->setMinimumWidth(resolvedWidth);
        panel->setMaximumWidth(resolvedWidth);
    }

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
    QGridLayout* layout = new QGridLayout(panel);
    layout->setContentsMargins(1, 4, 1, 2);
    layout->setHorizontalSpacing(3);
    layout->setVerticalSpacing(3);
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

    m_deleteButton = new QToolButton(panel);
    m_deleteButton->setProperty("ribbonButton", true);
    m_deleteButton->setText(QStringLiteral("删除"));
    m_deleteButton->setIcon(buildDeleteIcon(m_theme.accentColor));
    m_deleteButton->setIconSize(QSize(kRibbonIconSize, kRibbonIconSize));
    m_deleteButton->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    m_deleteButton->setFixedSize(56, kRibbonButtonHeight);
    m_deleteButton->setEnabled(false);
    connect(m_deleteButton, &QToolButton::clicked, this, &CadToolPanelWidget::deleteRequested);

    m_rotateButton = new QToolButton(panel);
    m_rotateButton->setProperty("ribbonButton", true);
    m_rotateButton->setText(QStringLiteral("旋转"));
    m_rotateButton->setIcon(buildRotateIcon(m_theme.accentColor));
    m_rotateButton->setIconSize(QSize(kRibbonIconSize, kRibbonIconSize));
    m_rotateButton->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    m_rotateButton->setFixedSize(56, kRibbonButtonHeight);
    m_rotateButton->setEnabled(false);
    connect(m_rotateButton, &QToolButton::clicked, this, &CadToolPanelWidget::rotateRequested);

    m_copyButton = new QToolButton(panel);
    m_copyButton->setProperty("ribbonButton", true);
    m_copyButton->setText(QStringLiteral("复制"));
    m_copyButton->setIcon(buildCopyIcon(m_theme.accentColor));
    m_copyButton->setIconSize(QSize(kRibbonIconSize, kRibbonIconSize));
    m_copyButton->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    m_copyButton->setFixedSize(56, kRibbonButtonHeight);
    m_copyButton->setEnabled(false);
    connect(m_copyButton, &QToolButton::clicked, this, &CadToolPanelWidget::copyRequested);

    m_scaleButton = new QToolButton(panel);
    m_scaleButton->setProperty("ribbonButton", true);
    m_scaleButton->setText(QStringLiteral("缩放"));
    m_scaleButton->setIcon(buildScaleIcon(m_theme.accentColor));
    m_scaleButton->setIconSize(QSize(kRibbonIconSize, kRibbonIconSize));
    m_scaleButton->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    m_scaleButton->setFixedSize(56, kRibbonButtonHeight);
    m_scaleButton->setEnabled(false);
    connect(m_scaleButton, &QToolButton::clicked, this, &CadToolPanelWidget::scaleRequested);

    m_arrayButton = new QToolButton(panel);
    m_arrayButton->setProperty("ribbonButton", true);
    m_arrayButton->setText(QStringLiteral("阵列"));
    m_arrayButton->setIcon(buildArrayIcon(m_theme.accentColor));
    m_arrayButton->setIconSize(QSize(kRibbonIconSize, kRibbonIconSize));
    m_arrayButton->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    m_arrayButton->setFixedSize(56, kRibbonButtonHeight);
    m_arrayButton->setEnabled(false);
    connect(m_arrayButton, &QToolButton::clicked, this, &CadToolPanelWidget::arrayRequested);

    layout->addWidget(m_moveButton, 0, 0);
    layout->addWidget(m_deleteButton, 0, 1);
    layout->addWidget(m_scaleButton, 0, 2);
    layout->addWidget(m_rotateButton, 1, 0);
    layout->addWidget(m_copyButton, 1, 1);
    layout->addWidget(m_arrayButton, 1, 2);
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

QWidget* CadToolPanelWidget::buildMachiningPanel()
{
    QWidget* panel = new QWidget(this);
    QHBoxLayout* rootLayout = new QHBoxLayout(panel);
    rootLayout->setContentsMargins(4, 0, 4, 0);
    rootLayout->setSpacing(0);
    rootLayout->setAlignment(Qt::AlignLeft | Qt::AlignTop);

    auto buildMachiningButton =
        [panel](const QString& text)
        {
            QToolButton* button = new QToolButton(panel);
            button->setText(text);
            button->setToolButtonStyle(Qt::ToolButtonTextOnly);
            button->setProperty("machiningButton", true);
            button->setMinimumHeight(30);
            button->setMinimumWidth(92);
            button->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::Fixed);
            return button;
        };

    m_importFileButton = buildMachiningButton(QStringLiteral("文件导入"));
    m_exportGCodeButton = buildMachiningButton(QStringLiteral("G代码导出"));
    m_deduplicateButton = buildMachiningButton(QStringLiteral("去重"));
    m_autoDeduplicateCheckBox = new QCheckBox(QStringLiteral("自动"), panel);
    m_autoDeduplicateCheckBox->setProperty("machiningOption", true);
    m_autoDeduplicateCheckBox->setToolTip(QStringLiteral("勾选后，导出G代码前会先自动执行一次去重"));
    m_useDxfFileNameCheckBox = new QCheckBox(QStringLiteral("使用dxf文件名"), panel);
    m_useDxfFileNameCheckBox->setProperty("machiningOption", true);
    m_useDxfFileNameCheckBox->setToolTip(QStringLiteral("勾选后，导出G代码时会优先使用当前DXF文件名作为输出文件名"));
    m_useDefaultExportPathCheckBox = new QCheckBox(QStringLiteral("使用默认导出路径"), panel);
    m_useDefaultExportPathCheckBox->setProperty("machiningOption", true);
    m_useDefaultExportPathCheckBox->setToolTip(QStringLiteral("勾选后，导出G代码时会默认定位到上次导出目录"));
    m_sortKeepDirectionButton = buildMachiningButton(QStringLiteral("排序(保留方向)"));
    m_smartSortButton = buildMachiningButton(QStringLiteral("智能排序"));
    m_profileSettingsButton = buildMachiningButton(QStringLiteral("G代码配置"));

    QWidget* importPanel = new QWidget(panel);
    QGridLayout* importLayout = new QGridLayout(importPanel);
    importLayout->setContentsMargins(1, 6, 1, 2);
    importLayout->setHorizontalSpacing(6);
    importLayout->setVerticalSpacing(6);
    importLayout->addWidget(m_importFileButton, 0, 0, 1, 2);
    importLayout->addWidget(m_exportGCodeButton, 1, 0, 1, 2);
    importLayout->setColumnStretch(0, 1);
    importLayout->setColumnStretch(1, 1);
    importLayout->setAlignment(Qt::AlignLeft | Qt::AlignTop);

    QWidget* sortPanel = new QWidget(panel);
    QGridLayout* sortLayout = new QGridLayout(sortPanel);
    sortLayout->setContentsMargins(1, 6, 1, 2);
    sortLayout->setHorizontalSpacing(6);
    sortLayout->setVerticalSpacing(6);
    sortLayout->addWidget(m_sortKeepDirectionButton, 0, 0);
    sortLayout->addWidget(m_smartSortButton, 1, 0);
    sortLayout->setColumnStretch(0, 1);
    sortLayout->setAlignment(Qt::AlignLeft | Qt::AlignTop);

    QWidget* featurePanel = new QWidget(panel);
    QGridLayout* featureLayout = new QGridLayout(featurePanel);
    featureLayout->setContentsMargins(1, 6, 1, 2);
    featureLayout->setHorizontalSpacing(6);
    featureLayout->setVerticalSpacing(6);
    featureLayout->addWidget(m_autoDeduplicateCheckBox, 0, 0, Qt::AlignLeft | Qt::AlignVCenter);
    featureLayout->addWidget(m_deduplicateButton, 0, 1);
    featureLayout->addWidget(m_useDefaultExportPathCheckBox, 1, 0, 1, 2, Qt::AlignLeft | Qt::AlignVCenter);
    featureLayout->addWidget(m_useDxfFileNameCheckBox, 2, 0, 1, 2, Qt::AlignLeft | Qt::AlignVCenter);
    featureLayout->setColumnStretch(1, 1);
    featureLayout->setAlignment(Qt::AlignLeft | Qt::AlignTop);

    QWidget* configPanel = new QWidget(panel);
    QVBoxLayout* configLayout = new QVBoxLayout(configPanel);
    configLayout->setContentsMargins(2, 6, 2, 2);
    configLayout->setSpacing(6);

    QWidget* profileRow = new QWidget(configPanel);
    QHBoxLayout* profileLayout = new QHBoxLayout(profileRow);
    profileLayout->setContentsMargins(0, 0, 0, 0);
    profileLayout->setSpacing(6);

    QLabel* profileLabel = new QLabel(QStringLiteral("当前配置"), profileRow);
    profileLabel->setProperty("machiningFieldLabel", true);
    profileLabel->setMinimumWidth(54);
    m_profileComboBox = new QComboBox(profileRow);
    m_profileComboBox->setEditable(false);
    m_profileComboBox->setFixedHeight(kComboHeight);
    m_profileComboBox->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    profileLayout->addWidget(profileLabel, 0);
    profileLayout->addWidget(m_profileComboBox, 1);
    configLayout->addWidget(profileRow);

    QWidget* modeRow = new QWidget(configPanel);
    QHBoxLayout* modeLayout = new QHBoxLayout(modeRow);
    modeLayout->setContentsMargins(0, 0, 0, 0);
    modeLayout->setSpacing(6);

    QLabel* modeLabel = new QLabel(QStringLiteral("G代码模式"), modeRow);
    modeLabel->setProperty("machiningFieldLabel", true);
    modeLabel->setMinimumWidth(54);
    m_gcodeModeComboBox = new QComboBox(modeRow);
    m_gcodeModeComboBox->setEditable(false);
    m_gcodeModeComboBox->setFixedHeight(kComboHeight);
    m_gcodeModeComboBox->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    m_gcodeModeComboBox->addItem(QStringLiteral("自动"), static_cast<int>(GCodeModeSelection::Auto));
    m_gcodeModeComboBox->addItem(QStringLiteral("3轴"), static_cast<int>(GCodeModeSelection::ThreeAxis));
    m_gcodeModeComboBox->addItem(QStringLiteral("4轴(绕A)"), static_cast<int>(GCodeModeSelection::FourAxisAroundA));
    modeLayout->addWidget(modeLabel, 0);
    modeLayout->addWidget(m_gcodeModeComboBox, 1);
    configLayout->addWidget(modeRow);
    configLayout->addStretch(1);

    QWidget* profileSettingsPanel = new QWidget(panel);
    QVBoxLayout* profileSettingsLayout = new QVBoxLayout(profileSettingsPanel);
    profileSettingsLayout->setContentsMargins(2, 6, 2, 2);
    profileSettingsLayout->setSpacing(6);
    profileSettingsLayout->addWidget(m_profileSettingsButton);
    profileSettingsLayout->addStretch(1);

    rootLayout->addWidget(buildPanelFrame(QStringLiteral("导入导出"), importPanel, 182, nullptr, false), 0);
    rootLayout->addWidget(buildDivider(), 0, Qt::AlignLeft | Qt::AlignVCenter);
    rootLayout->addWidget(buildPanelFrame(QStringLiteral("排序"), sortPanel, 132, nullptr, false), 0);
    rootLayout->addWidget(buildDivider(), 0, Qt::AlignLeft | Qt::AlignVCenter);
    rootLayout->addWidget(buildPanelFrame(QStringLiteral("功能"), featurePanel, 148, nullptr, false), 0);
    rootLayout->addWidget(buildDivider(), 0, Qt::AlignLeft | Qt::AlignVCenter);
    rootLayout->addWidget(buildPanelFrame(QStringLiteral("配置"), configPanel, 196, nullptr, false), 0);
    rootLayout->addWidget(buildDivider(), 0, Qt::AlignLeft | Qt::AlignVCenter);
    rootLayout->addWidget(buildPanelFrame(QStringLiteral("G代码配置"), profileSettingsPanel, 128, nullptr, false), 0);
    rootLayout->addStretch(1);

    connect(m_importFileButton, &QToolButton::clicked, this, [this]() { emit importFileRequested(); });
    connect(m_exportGCodeButton, &QToolButton::clicked, this, [this]() { emit exportGCodeRequested(); });
    connect(m_deduplicateButton, &QToolButton::clicked, this, [this]() { emit deduplicateRequested(); });
    connect(m_autoDeduplicateCheckBox, &QCheckBox::toggled, this, [this](bool checked)
    {
        if (!m_updatingUi)
        {
            emit autoDeduplicateOptionChanged(checked);
        }
    });
    connect(m_useDxfFileNameCheckBox, &QCheckBox::toggled, this, [this](bool checked)
    {
        if (!m_updatingUi)
        {
            emit useDxfFileNameOptionChanged(checked);
        }
    });
    connect(m_useDefaultExportPathCheckBox, &QCheckBox::toggled, this, [this](bool checked)
    {
        if (!m_updatingUi)
        {
            emit useDefaultExportPathOptionChanged(checked);
        }
    });
    connect(m_sortKeepDirectionButton, &QToolButton::clicked, this, [this]() { emit sortKeepDirectionRequested(); });
    connect(m_smartSortButton, &QToolButton::clicked, this, [this]() { emit smartSortRequested(); });
    connect(m_profileSettingsButton, &QToolButton::clicked, this, [this]() { emit profileSettingsRequested(); });
    connect
    (
        m_profileComboBox,
        &QComboBox::currentIndexChanged,
        this,
        [this](int index)
        {
            if (m_updatingUi || index < 0)
            {
                return;
            }

            emit profileSelectionChanged(m_profileComboBox->itemData(index).toString());
        }
    );
    connect
    (
        m_gcodeModeComboBox,
        &QComboBox::currentIndexChanged,
        this,
        [this](int index)
        {
            if (m_updatingUi || index < 0)
            {
                return;
            }

            emit gcodeModeSelectionChanged(static_cast<GCodeModeSelection>(index));
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
