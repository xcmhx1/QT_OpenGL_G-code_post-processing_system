#pragma once

#include "AppTheme.h"
#include "CadCommandLineWidget.h"
#include "CadEditer.h"
#include "CadDocument.h"
#include "CadStatusPaneWidget.h"
#include "CadToolPanelWidget.h"
#include "GGenerator.h"
#include "GProfile.h"

#include <QtWidgets/QMainWindow>
#include <QtGlobal>

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
    enum class GCodeGenerationPreference
    {
        Auto,
        Force2D,
        Force3D
    };

private:
    void initializeThemeMenu();
    void openProfileSettingsDialog();
    void applyTheme(AppThemeMode mode);
    AppThemeMode loadThemeMode() const;
    void saveThemeMode(AppThemeMode mode) const;
    quint32 loadSnapOptionMask() const;
    void saveSnapOptionMask(quint32 mask) const;
    GCodeGenerationPreference loadGenerationPreference() const;
    void saveGenerationPreference(GCodeGenerationPreference preference) const;
    GGenerator::GenerationMode resolveGenerationMode() const;
    void initializeToolPanel();
    void syncToolPanelState();
    void applyDefaultDrawingProperties();
    QString activeLayerName() const;
    QColor activeColor() const;
    int activeColorIndex() const;
    bool importCadFile(const QString& filePath);
    bool importDxfFile(const QString& filePath);
    bool importBitmapFile(const QString& filePath);
    bool saveCurrentDocument();
    bool exportDxfDocument(bool safeMode = false);
    bool exportGCodeDocument();
    bool writeDocumentToDxf(const QString& filePath, bool updateCurrentPath, bool safeMode = false);
    QString ensureDxfSuffix(const QString& filePath) const;
    QString defaultDxfPathForCurrentDocument() const;
    bool toggleSelectedEntityReverse();
    bool deleteSelectedEntity();
    bool copySelectedEntity();
    bool rotateSelectedEntity();
    bool scaleSelectedEntity();
    bool arraySelectedEntity();
    bool sortEntitiesByCurrentDirection();
    bool assignSelectedEntityProcessOrder();
    bool smartSortEntities();
    bool sortEntitiesByCurrentDirection3D();
    bool smartSortEntities3D();

private:
    Ui::Gcode_postprocessing_systemClass* ui = nullptr;
    CadCommandLineWidget* m_commandLineWidget = nullptr;
    CadStatusPaneWidget* m_statusPaneWidget = nullptr;
    CadToolPanelWidget* m_toolPanelWidget = nullptr;
    QAction* m_lightThemeAction = nullptr;
    QAction* m_darkThemeAction = nullptr;
    QAction* m_profileSettingsAction = nullptr;
    QAction* m_generationModeAutoAction = nullptr;
    QAction* m_generationMode2DAction = nullptr;
    QAction* m_generationMode3DAction = nullptr;
    CadEditer m_editer;
    CadDocument m_document;
    GProfile m_activeProfile = GProfile::createDefaultLaserProfile();
    QString m_currentLayerName = QStringLiteral("0");
    QColor m_currentColor = QColor(Qt::white);
    int m_currentColorIndex = 256;
    AppThemeMode m_themeMode = AppThemeMode::Light;
    GCodeGenerationPreference m_generationPreference = GCodeGenerationPreference::Auto;
    QString m_currentDocumentPath;
};
