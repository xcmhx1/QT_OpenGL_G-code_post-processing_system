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
#include <QMap>

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
    bool loadAutoDeduplicateOnExport() const;
    void saveAutoDeduplicateOnExport(bool enabled) const;
    bool loadUseDxfFileNameOnExport() const;
    void saveUseDxfFileNameOnExport(bool enabled) const;
    bool loadUseDefaultImportPath() const;
    void saveUseDefaultImportPath(bool enabled) const;
    bool loadUseDefaultExportPath() const;
    void saveUseDefaultExportPath(bool enabled) const;
    QString loadLastImportDirectory() const;
    void saveLastImportDirectory(const QString& filePath) const;
    QString loadLastGCodeExportPath() const;
    void saveLastGCodeExportPath(const QString& filePath) const;
    void applyGenerationPreference(GCodeGenerationPreference preference);
    GGenerator::GenerationMode resolveGenerationMode() const;
    void loadAvailableProfiles();
    void refreshAvailableProfilesUi();
    bool applyLoadedProfileById(const QString& profileId, bool announceChange = true);
    QString runtimeProfileDirectoryPath() const;
    QString loadSelectedProfileId() const;
    void saveSelectedProfileId(const QString& profileId) const;
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
    bool exportGCode();
    bool exportGCode(GGenerator::GenerationMode generationMode, const QString& modeDisplayName);
    QString defaultImportPath() const;
    QString defaultGCodeExportPathForCurrentDocument() const;
    bool prepareDocumentForGCodeExport(GGenerator::GenerationMode generationMode);
    bool sortEntitiesByCurrentMode(bool smartSort);
    bool writeDocumentToDxf(const QString& filePath, bool updateCurrentPath, bool safeMode = false);
    QString ensureDxfSuffix(const QString& filePath) const;
    QString defaultDxfPathForCurrentDocument() const;
    QString generationModeDisplayName(GGenerator::GenerationMode generationMode) const;
    bool toggleSelectedEntityReverse();
    bool deleteSelectedEntity();
    bool copySelectedEntity();
    bool rotateSelectedEntity();
    bool scaleSelectedEntity();
    bool arraySelectedEntity();
    bool circularArraySelectedEntity();
    bool mirrorSelectedEntities();
    bool offsetSelectedEntity();
    bool trimSelectedEntity();
    bool extendSelectedEntity();
    bool joinSelectedEntities();
    bool filletSelectedEntities();
    bool chamferSelectedEntities();
    bool removeDuplicateEntities();
    bool sortEntitiesByCurrentDirection();
    bool assignSelectedEntityProcessOrder();
    bool smartSortEntities();
    bool sortEntitiesByCurrentDirection3D();
    bool smartSortEntities3D();
    bool hasCompleteProcessOrderForExport(GGenerator::GenerationMode generationMode) const;

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
    QMap<QString, GProfile> m_loadedProfiles;
    QMap<QString, QString> m_loadedProfileNames;
    QStringList m_loadedProfileOrder;
    QString m_activeProfileId = QStringLiteral("builtin:3axis");
    int m_sessionImportedProfileSerial = 0;
};
