#pragma once

#include "AppTheme.h"
#include "CadCommandLineWidget.h"
#include "CadEditer.h"
#include "CadDocument.h"
#include "CadStatusPaneWidget.h"
#include "CadToolPanelWidget.h"

#include <QtWidgets/QMainWindow>

#include "ui_Gcode_postprocessing_system.h"

QT_BEGIN_NAMESPACE
namespace Ui { class Gcode_postprocessing_systemClass; }
QT_END_NAMESPACE

class QAction;

class Gcode_postprocessing_system : public QMainWindow
{
    Q_OBJECT

public:
    Gcode_postprocessing_system(QWidget* parent = nullptr);
    ~Gcode_postprocessing_system();

private:
    void initializeThemeMenu();
    void applyTheme(AppThemeMode mode);
    AppThemeMode loadThemeMode() const;
    void saveThemeMode(AppThemeMode mode) const;
    void initializeToolPanel();
    void syncToolPanelState();
    void applyDefaultDrawingProperties();
    QString activeLayerName() const;
    QColor activeColor() const;
    int activeColorIndex() const;
    bool importCadFile(const QString& filePath);
    bool importDxfFile(const QString& filePath);
    bool importBitmapFile(const QString& filePath);
    bool exportGCode();
    bool toggleSelectedEntityReverse();
    bool sortEntitiesByCurrentDirection();
    bool assignSelectedEntityProcessOrder();
    bool smartSortEntities();

private:
    Ui::Gcode_postprocessing_systemClass* ui = nullptr;
    CadCommandLineWidget* m_commandLineWidget = nullptr;
    CadStatusPaneWidget* m_statusPaneWidget = nullptr;
    CadToolPanelWidget* m_toolPanelWidget = nullptr;
    QAction* m_lightThemeAction = nullptr;
    QAction* m_darkThemeAction = nullptr;
    CadEditer m_editer;
    CadDocument m_document;
    QString m_currentLayerName = QStringLiteral("0");
    QColor m_currentColor = QColor(Qt::white);
    int m_currentColorIndex = 256;
    AppThemeMode m_themeMode = AppThemeMode::Light;
};
