#pragma once

#include "AppTheme.h"
#include "DrawStateMachine.h"

#include <QColor>
#include <QMap>
#include <QStringList>
#include <QVector>
#include <QWidget>

class QAction;
class QCheckBox;
class QComboBox;
class QFrame;
class QLabel;
class QMenu;
class QTabBar;
class QTabWidget;
class QToolButton;

class CadToolPanelWidget : public QWidget
{
    Q_OBJECT

public:
    enum class GCodeModeSelection
    {
        Auto,
        ThreeAxis,
        FourAxisAroundA
    };

    explicit CadToolPanelWidget(QWidget* parent = nullptr);

    void setLayerNames(const QStringList& layerNames, const QMap<QString, QColor>& layerColors);
    void setLayerStatusText(const QString& text);
    void setPropertyStatusText(const QString& text);
    void setActiveLayerName(const QString& layerName);
    void setActiveColorState(const QColor& color, int colorIndex, const QColor& byLayerColor);
    void setMoveEnabled(bool enabled);
    void setModifyActionsEnabled(bool enabled);
    void setTheme(const AppThemeColors& theme);
    void setGCodeModeSelection(GCodeModeSelection selection);
    void setAvailableProfiles(const QList<QPair<QString, QString>>& profiles);
    void setCurrentProfileSelection(const QString& profileId);
    void setAutoDeduplicateEnabled(bool enabled);
    bool autoDeduplicateEnabled() const;
    void setUseDxfFileNameEnabled(bool enabled);
    bool useDxfFileNameEnabled() const;
    void setUseDefaultImportPathEnabled(bool enabled);
    bool useDefaultImportPathEnabled() const;
    void setUseDefaultExportPathEnabled(bool enabled);
    bool useDefaultExportPathEnabled() const;
    void setProcessVisualsVisible(bool enabled);
    bool processVisualsVisible() const;

signals:
    void drawRequested(DrawType drawType);
    void moveRequested();
    void deleteRequested();
    void rotateRequested();
    void copyRequested();
    void scaleRequested();
    void arrayRequested();
    void circularArrayRequested();
    void mirrorRequested();
    void offsetRequested();
    void trimRequested();
    void extendRequested();
    void joinRequested();
    void filletRequested();
    void chamferRequested();
    void layerChangeRequested(const QString& layerName);
    void colorChangeRequested(int colorIndex);
    void importFileRequested();
    void exportGCodeRequested();
    void deduplicateRequested();
    void sortKeepDirectionRequested();
    void smartSortRequested();
    void gcodeModeSelectionChanged(CadToolPanelWidget::GCodeModeSelection selection);
    void profileSelectionChanged(const QString& profileId);
    void profileSettingsRequested();
    void autoDeduplicateOptionChanged(bool enabled);
    void useDxfFileNameOptionChanged(bool enabled);
    void useDefaultImportPathOptionChanged(bool enabled);
    void useDefaultExportPathOptionChanged(bool enabled);
    void processVisualsVisibleChanged(bool enabled);

private:
    void buildUi();
    void applyTheme();
    QWidget* buildPanelFrame(const QString& title, QWidget* contentWidget, int preferredWidth = -1, QMenu* launcherMenu = nullptr, bool flexibleWidth = false);
    QWidget* buildDivider();
    QWidget* buildDrawPanel();
    QWidget* buildModifyPanel();
    QWidget* buildLayerPanel();
    QWidget* buildPropertyPanel();
    QWidget* buildDisplayPanel();
    QWidget* buildMachiningPanel();
    void addDrawButton(QWidget* parent, const QString& text, DrawType drawType, int row, int column);
    void commitLayerChange(QComboBox* comboBox);
    void updateLayerComboIcons();
    void updateColorComboIcons(const QColor& activeColor, const QColor& byLayerColor);
    void setComboCurrentByData(QComboBox* comboBox, int value);

private:
    QToolButton* m_moveButton = nullptr;
    QToolButton* m_deleteButton = nullptr;
    QToolButton* m_rotateButton = nullptr;
    QToolButton* m_copyButton = nullptr;
    QToolButton* m_scaleButton = nullptr;
    QToolButton* m_arrayButton = nullptr;
    QLabel* m_layerStatusLabel = nullptr;
    QLabel* m_propertyStatusLabel = nullptr;
    QComboBox* m_layerComboBox = nullptr;
    QComboBox* m_propertyLayerComboBox = nullptr;
    QComboBox* m_colorComboBox = nullptr;
    QComboBox* m_gcodeModeComboBox = nullptr;
    QComboBox* m_profileComboBox = nullptr;
    QCheckBox* m_autoDeduplicateCheckBox = nullptr;
    QCheckBox* m_useDxfFileNameCheckBox = nullptr;
    QCheckBox* m_useDefaultImportPathCheckBox = nullptr;
    QCheckBox* m_useDefaultExportPathCheckBox = nullptr;
    QCheckBox* m_processVisualsCheckBox = nullptr;
    QMenu* m_drawMoreMenu = nullptr;
    QAction* m_drawPointAction = nullptr;
    QAction* m_drawXlineAction = nullptr;
    QAction* m_drawRectangleAction = nullptr;
    QAction* m_drawPolygonAction = nullptr;
    QMenu* m_modifyMoreMenu = nullptr;
    QAction* m_rectangularArrayAction = nullptr;
    QAction* m_circularArrayAction = nullptr;
    QAction* m_mirrorAction = nullptr;
    QAction* m_offsetAction = nullptr;
    QAction* m_trimAction = nullptr;
    QAction* m_extendAction = nullptr;
    QAction* m_joinAction = nullptr;
    QAction* m_filletAction = nullptr;
    QAction* m_chamferAction = nullptr;
    QToolButton* m_importFileButton = nullptr;
    QToolButton* m_exportGCodeButton = nullptr;
    QToolButton* m_deduplicateButton = nullptr;
    QToolButton* m_sortKeepDirectionButton = nullptr;
    QToolButton* m_smartSortButton = nullptr;
    QToolButton* m_profileSettingsButton = nullptr;
    QTabWidget* m_tabWidget = nullptr;
    QMap<QString, QColor> m_layerColors;
    QVector<QToolButton*> m_drawButtons;
    QVector<QFrame*> m_dividers;
    AppThemeColors m_theme = buildAppThemeColors(AppThemeMode::Light);
    bool m_updatingUi = false;
};
