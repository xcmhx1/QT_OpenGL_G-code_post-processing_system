#pragma once

#include "DrawStateMachine.h"

#include <QColor>
#include <QStringList>
#include <QWidget>

class QComboBox;
class QGroupBox;
class QLabel;
class QToolButton;

class CadToolPanelWidget : public QWidget
{
    Q_OBJECT

public:
    explicit CadToolPanelWidget(QWidget* parent = nullptr);

    void setLayerNames(const QStringList& layerNames);
    void setLayerStatusText(const QString& text);
    void setPropertyStatusText(const QString& text);
    void setActiveLayerName(const QString& layerName);
    void setActiveColorState(const QColor& color, int colorIndex);
    void setMoveEnabled(bool enabled);

signals:
    void drawRequested(DrawType drawType);
    void moveRequested();
    void layerChangeRequested(const QString& layerName);
    void colorChangeRequested(int colorIndex);

private:
    void buildUi();
    QGroupBox* buildPanelFrame(const QString& title, QWidget* contentWidget, int preferredWidth);
    QWidget* buildDrawPanel();
    QWidget* buildModifyPanel();
    QWidget* buildLayerPanel();
    QWidget* buildPropertyPanel();
    void addDrawButton(QWidget* parent, const QString& text, DrawType drawType, int row, int column);
    void commitLayerChange(QComboBox* comboBox);
    void updateColorSwatch(const QColor& color);
    void setComboCurrentByData(QComboBox* comboBox, int value);

private:
    QToolButton* m_moveButton = nullptr;
    QLabel* m_layerStatusLabel = nullptr;
    QLabel* m_propertyStatusLabel = nullptr;
    QComboBox* m_layerComboBox = nullptr;
    QComboBox* m_propertyLayerComboBox = nullptr;
    QComboBox* m_colorComboBox = nullptr;
    QLabel* m_colorSwatchLabel = nullptr;
    bool m_updatingUi = false;
};
