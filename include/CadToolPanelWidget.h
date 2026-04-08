#pragma once

#include "AppTheme.h"
#include "DrawStateMachine.h"

#include <QColor>
#include <QMap>
#include <QStringList>
#include <QVector>
#include <QWidget>

class QAction;
class QComboBox;
class QFrame;
class QLabel;
class QMenu;
class QToolButton;

class CadToolPanelWidget : public QWidget
{
    Q_OBJECT

public:
    explicit CadToolPanelWidget(QWidget* parent = nullptr);

    void setLayerNames(const QStringList& layerNames, const QMap<QString, QColor>& layerColors);
    void setLayerStatusText(const QString& text);
    void setPropertyStatusText(const QString& text);
    void setActiveLayerName(const QString& layerName);
    void setActiveColorState(const QColor& color, int colorIndex, const QColor& byLayerColor);
    void setMoveEnabled(bool enabled);
    void setTheme(const AppThemeColors& theme);

signals:
    void drawRequested(DrawType drawType);
    void moveRequested();
    void layerChangeRequested(const QString& layerName);
    void colorChangeRequested(int colorIndex);

private:
    void buildUi();
    void applyTheme();
    QWidget* buildPanelFrame(const QString& title, QWidget* contentWidget, int preferredWidth = -1, QMenu* launcherMenu = nullptr);
    QWidget* buildDivider();
    QWidget* buildDrawPanel();
    QWidget* buildModifyPanel();
    QWidget* buildLayerPanel();
    QWidget* buildPropertyPanel();
    void addDrawButton(QWidget* parent, const QString& text, DrawType drawType, int row, int column);
    void commitLayerChange(QComboBox* comboBox);
    void updateLayerComboIcons();
    void updateColorComboIcons(const QColor& activeColor, const QColor& byLayerColor);
    void setComboCurrentByData(QComboBox* comboBox, int value);

private:
    QToolButton* m_moveButton = nullptr;
    QLabel* m_layerStatusLabel = nullptr;
    QLabel* m_propertyStatusLabel = nullptr;
    QComboBox* m_layerComboBox = nullptr;
    QComboBox* m_propertyLayerComboBox = nullptr;
    QComboBox* m_colorComboBox = nullptr;
    QMenu* m_drawMoreMenu = nullptr;
    QAction* m_drawPointAction = nullptr;
    QMap<QString, QColor> m_layerColors;
    QVector<QToolButton*> m_drawButtons;
    QVector<QFrame*> m_dividers;
    AppThemeColors m_theme = buildAppThemeColors(AppThemeMode::Light);
    bool m_updatingUi = false;
};
