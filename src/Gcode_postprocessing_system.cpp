#include "pch.h"

#include "Gcode_postprocessing_system.h"
#include "CadItem.h"
#include "GProfileDialog.h"

#include <QActionGroup>
#include <QApplication>
#include <QCoreApplication>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QInputDialog>
#include <QKeySequence>
#include <QMap>
#include <QMessageBox>
#include <QMenu>
#include <QMenuBar>
#include <QSettings>
#include <QStatusBar>
#include <QStyleFactory>
#include <QStandardPaths>
#include <QSet>
#include <QToolBar>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace
{
    constexpr const char* kBuiltinThreeAxisProfileId = "builtin:3axis";
    constexpr const char* kBuiltinFourAxisProfileId = "builtin:4axis";
    constexpr int kColorByLayer = 256;

    QColor colorFromAci(int colorIndex)
    {
        static const QRgb aciStandardColors[] =
        {
            qRgb(0, 0, 0),
            qRgb(255, 0, 0),
            qRgb(255, 255, 0),
            qRgb(0, 255, 0),
            qRgb(0, 255, 255),
            qRgb(0, 0, 255),
            qRgb(255, 0, 255),
            qRgb(255, 255, 255),
            qRgb(128, 128, 128),
            qRgb(192, 192, 192)
        };

        if (colorIndex >= 1 && colorIndex <= 9)
        {
            return QColor(aciStandardColors[colorIndex]);
        }

        if (colorIndex == 0)
        {
            return QColor(Qt::white);
        }

        return QColor();
    }

    QColor colorFromTrueColor(int color24)
    {
        if (color24 < 0)
        {
            return QColor();
        }

        return QColor((color24 >> 16) & 0xFF, (color24 >> 8) & 0xFF, color24 & 0xFF);
    }

    QString entityLayerName(const CadItem* item)
    {
        if (item == nullptr || item->m_nativeEntity == nullptr)
        {
            return QStringLiteral("0");
        }

        const QString layerName = QString::fromUtf8(item->m_nativeEntity->layer.c_str()).trimmed();
        return layerName.isEmpty() ? QStringLiteral("0") : layerName;
    }

    int entityColorIndex(const CadItem* item)
    {
        if (item == nullptr || item->m_nativeEntity == nullptr)
        {
            return kColorByLayer;
        }

        return item->m_nativeEntity->color24 != -1
            ? -1
            : item->m_nativeEntity->color;
    }

    QColor entityDisplayColor(const CadDocument& document, const CadItem* item)
    {
        if (item == nullptr || item->m_nativeEntity == nullptr)
        {
            return QColor(Qt::white);
        }

        const QColor trueColor = colorFromTrueColor(item->m_nativeEntity->color24);

        if (trueColor.isValid())
        {
            return trueColor;
        }

        if (item->m_nativeEntity->color == kColorByLayer)
        {
            return document.layerColor(entityLayerName(item), item->m_color);
        }

        const QColor aciColor = colorFromAci(item->m_nativeEntity->color);
        return aciColor.isValid() ? aciColor : item->m_color;
    }

}

Gcode_postprocessing_system::Gcode_postprocessing_system(QWidget* parent)
    : QMainWindow(parent)
    , ui(new Ui::Gcode_postprocessing_systemClass())
{
    ui->setupUi(this);
    loadAvailableProfiles();

    m_commandLineWidget = new CadCommandLineWidget(this);
    m_statusPaneWidget = new CadStatusPaneWidget(this);

    if (QVBoxLayout* centralLayout = qobject_cast<QVBoxLayout*>(ui->centralWidget->layout()))
    {
        centralLayout->addWidget(m_commandLineWidget);
        centralLayout->addWidget(m_statusPaneWidget);
    }

    m_editer.setDocument(&m_document);
    ui->openGLWidget->setEditer(&m_editer);
    ui->openGLWidget->setDocument(&m_document);
    ui->openGLWidget->refreshCommandPrompt();

    connect(ui->openGLWidget, &CadViewer::hoveredWorldPositionChanged, m_statusPaneWidget, &CadStatusPaneWidget::setWorldPosition);
    connect(m_statusPaneWidget, &CadStatusPaneWidget::basePointSnapToggled, ui->openGLWidget, &CadViewer::setBasePointSnapEnabled);
    connect(m_statusPaneWidget, &CadStatusPaneWidget::controlPointSnapToggled, ui->openGLWidget, &CadViewer::setControlPointSnapEnabled);
    connect(m_statusPaneWidget, &CadStatusPaneWidget::gridSnapToggled, ui->openGLWidget, &CadViewer::setGridSnapEnabled);
    connect(m_statusPaneWidget, &CadStatusPaneWidget::endpointSnapToggled, ui->openGLWidget, &CadViewer::setEndpointSnapEnabled);
    connect(m_statusPaneWidget, &CadStatusPaneWidget::midpointSnapToggled, ui->openGLWidget, &CadViewer::setMidpointSnapEnabled);
    connect(m_statusPaneWidget, &CadStatusPaneWidget::centerSnapToggled, ui->openGLWidget, &CadViewer::setCenterSnapEnabled);
    connect(m_statusPaneWidget, &CadStatusPaneWidget::intersectionSnapToggled, ui->openGLWidget, &CadViewer::setIntersectionSnapEnabled);
    connect(m_statusPaneWidget, &CadStatusPaneWidget::orthoToggled, ui->openGLWidget, &CadViewer::setOrthoEnabled);
    connect(m_statusPaneWidget, &CadStatusPaneWidget::polarTrackingToggled, ui->openGLWidget, &CadViewer::setPolarTrackingEnabled);
    connect(ui->openGLWidget, &CadViewer::orthoEnabledChanged, m_statusPaneWidget, &CadStatusPaneWidget::setOrthoEnabled);
    connect(ui->openGLWidget, &CadViewer::polarTrackingEnabledChanged, m_statusPaneWidget, &CadStatusPaneWidget::setPolarTrackingEnabled);
    connect
    (
        m_statusPaneWidget,
        &CadStatusPaneWidget::snapOptionMaskChanged,
        this,
        [this](quint32 mask)
        {
            saveSnapOptionMask(mask);
        }
    );

    m_statusPaneWidget->setSnapOptionMask(loadSnapOptionMask());
    connect(ui->openGLWidget, &CadViewer::commandPromptChanged, m_commandLineWidget, &CadCommandLineWidget::setPrompt);
    connect(ui->openGLWidget, &CadViewer::commandMessageAppended, m_commandLineWidget, &CadCommandLineWidget::appendMessage);
    connect
    (
        ui->openGLWidget,
        &CadViewer::fileDropRequested,
        this,
        [this](const QString& filePath)
        {
            importCadFile(filePath);
        }
    );

    connect
    (
        ui->action_File_Import_Dxf,
        &QAction::triggered,
        this,
        [this]()
        {
            const QString filePath = QFileDialog::getOpenFileName
            (
                this,
                QStringLiteral("导入文件"),
                defaultImportPath(),
                QStringLiteral("支持文件 (*.dxf *.dwg *.bmp *.png *.jpg *.jpeg);;CAD 文件 (*.dxf *.dwg);;位图文件 (*.bmp *.png *.jpg *.jpeg)")
            );

            if (filePath.isEmpty())
            {
                return;
            }

            importCadFile(filePath);
        }
    );

    QAction* importDxfOnlyAction = new QAction(QStringLiteral("导入DXF..."), this);
    QAction* importDwgOnlyAction = new QAction(QStringLiteral("导入DWG..."), this);
    ui->menuFile->insertAction(ui->action_File_Import_Image, importDxfOnlyAction);
    ui->menuFile->insertAction(ui->action_File_Import_Image, importDwgOnlyAction);

    connect
    (
        importDxfOnlyAction,
        &QAction::triggered,
        this,
        [this]()
        {
            const QString filePath = QFileDialog::getOpenFileName
            (
                this,
                QStringLiteral("导入DXF"),
                defaultImportPath(),
                QStringLiteral("DXF 文件 (*.dxf)")
            );

            if (!filePath.isEmpty())
            {
                importDxfFile(filePath);
            }
        }
    );

    connect
    (
        importDwgOnlyAction,
        &QAction::triggered,
        this,
        [this]()
        {
            const QString filePath = QFileDialog::getOpenFileName
            (
                this,
                QStringLiteral("导入DWG"),
                defaultImportPath(),
                QStringLiteral("DWG 文件 (*.dwg)")
            );

            if (!filePath.isEmpty())
            {
                importDxfFile(filePath);
            }
        }
    );

    connect
    (
        ui->action_File_Import_Image,
        &QAction::triggered,
        this,
        [this]()
        {
            const QString filePath = QFileDialog::getOpenFileName
            (
                this,
                QStringLiteral("导入图片"),
                defaultImportPath(),
                QStringLiteral("位图文件 (*.bmp *.png *.jpg *.jpeg)")
            );

            if (filePath.isEmpty())
            {
                return;
            }

            importBitmapFile(filePath);
        }
    );

    ui->action_FileExport->setText(QStringLiteral("保存文件"));
    ui->action_FileExport->setShortcut(QKeySequence::Save);
    ui->action_FileExport->setShortcutContext(Qt::ApplicationShortcut);
    ui->menuFile->insertAction(ui->action_File_Export_G, ui->action_FileExport);

    QAction* exportDxfAction = new QAction(QStringLiteral("导出为DXF..."), this);
    QAction* exportSafeDxfAction = new QAction(QStringLiteral("导出为DXF（安全模式）..."), this);
    QAction* mirrorAction = new QAction(QStringLiteral("镜像..."), this);
    QAction* offsetAction = new QAction(QStringLiteral("偏移..."), this);
    QAction* rectangularArrayAction = new QAction(QStringLiteral("矩形阵列..."), this);
    QAction* circularArrayAction = new QAction(QStringLiteral("环形阵列..."), this);
    QAction* trimAction = new QAction(QStringLiteral("修剪..."), this);
    QAction* extendAction = new QAction(QStringLiteral("延申..."), this);
    QAction* joinAction = new QAction(QStringLiteral("合并..."), this);
    QAction* filletAction = new QAction(QStringLiteral("圆角..."), this);
    QAction* chamferAction = new QAction(QStringLiteral("直角（倒角）..."), this);

    ui->menuFile->insertAction(ui->action_File_Export_G, exportDxfAction);
    ui->menuFile->insertAction(ui->action_File_Export_G, exportSafeDxfAction);
    ui->menuFile->insertSeparator(ui->action_File_Export_G);
    ui->menuEdit->addSeparator();
    ui->menuEdit->addAction(mirrorAction);
    ui->menuEdit->addAction(offsetAction);
    ui->menuEdit->addAction(rectangularArrayAction);
    ui->menuEdit->addAction(circularArrayAction);
    ui->menuEdit->addAction(trimAction);
    ui->menuEdit->addAction(extendAction);
    ui->menuEdit->addAction(joinAction);
    ui->menuEdit->addAction(filletAction);
    ui->menuEdit->addAction(chamferAction);

    ui->action_File_Export_G->setText(QStringLiteral("导出G代码"));

    connect(ui->action_FileExport, &QAction::triggered, this, [this]() { saveCurrentDocument(); });
    connect(exportDxfAction, &QAction::triggered, this, [this]() { exportDxfDocument(); });
    connect(exportSafeDxfAction, &QAction::triggered, this, [this]() { exportDxfDocument(true); });
    connect(ui->action_File_Export_G, &QAction::triggered, this, [this]() { exportGCode(); });
    connect(ui->action_Edit_ReversePeocess, &QAction::triggered, this, [this]() { toggleSelectedEntityReverse(); });
    connect(mirrorAction, &QAction::triggered, this, [this]() { mirrorSelectedEntities(); });
    connect(offsetAction, &QAction::triggered, this, [this]() { offsetSelectedEntity(); });
    connect(rectangularArrayAction, &QAction::triggered, this, [this]() { arraySelectedEntity(); });
    connect(circularArrayAction, &QAction::triggered, this, [this]() { circularArraySelectedEntity(); });
    connect(trimAction, &QAction::triggered, this, [this]() { trimSelectedEntity(); });
    connect(extendAction, &QAction::triggered, this, [this]() { extendSelectedEntity(); });
    connect(joinAction, &QAction::triggered, this, [this]() { joinSelectedEntities(); });
    connect(filletAction, &QAction::triggered, this, [this]() { filletSelectedEntities(); });
    connect(chamferAction, &QAction::triggered, this, [this]() { chamferSelectedEntities(); });
    connect(ui->action_Sort_2D_Assign, &QAction::triggered, this, [this]() { sortEntitiesByCurrentMode(false); });
    connect(ui->action_Sort_2D_Smart, &QAction::triggered, this, [this]() { sortEntitiesByCurrentMode(true); });

    ui->action_Sort_3D_Assign->setVisible(false);
    ui->action_Sort_3D_Smart->setVisible(false);

    m_generationPreference = loadGenerationPreference();
    initializeThemeMenu();
    initializeToolPanel();
    applyDefaultDrawingProperties();
    applyTheme(loadThemeMode());
    syncToolPanelState();
}

Gcode_postprocessing_system::~Gcode_postprocessing_system()
{
    delete ui;
}

void Gcode_postprocessing_system::initializeThemeMenu()
{
    ui->menuSet->setTitle(QStringLiteral("用户设置"));
    ui->menuSort->setTitle(QStringLiteral("排序"));
    ui->action_Sort_2D_Assign->setText(QStringLiteral("排序（保留方向）"));
    ui->action_Sort_2D_Smart->setText(QStringLiteral("智能排序"));

    QMenu* generationMenu = ui->menuSet->addMenu(QStringLiteral("G代码模式"));
    QActionGroup* generationModeActionGroup = new QActionGroup(this);
    generationModeActionGroup->setExclusive(true);

    m_generationModeAutoAction = generationMenu->addAction(QStringLiteral("自动"));
    m_generationModeAutoAction->setCheckable(true);
    generationModeActionGroup->addAction(m_generationModeAutoAction);

    m_generationMode2DAction = generationMenu->addAction(QStringLiteral("3轴"));
    m_generationMode2DAction->setCheckable(true);
    generationModeActionGroup->addAction(m_generationMode2DAction);

    m_generationMode3DAction = generationMenu->addAction(QStringLiteral("4轴(绕A)"));
    m_generationMode3DAction->setCheckable(true);
    generationModeActionGroup->addAction(m_generationMode3DAction);

    m_generationModeAutoAction->setChecked(m_generationPreference == GCodeGenerationPreference::Auto);
    m_generationMode2DAction->setChecked(m_generationPreference == GCodeGenerationPreference::Force2D);
    m_generationMode3DAction->setChecked(m_generationPreference == GCodeGenerationPreference::Force3D);

    connect
    (
        m_generationModeAutoAction,
        &QAction::triggered,
        this,
        [this]()
        {
            applyGenerationPreference(GCodeGenerationPreference::Auto);
        }
    );
    connect
    (
        m_generationMode2DAction,
        &QAction::triggered,
        this,
        [this]()
        {
            applyGenerationPreference(GCodeGenerationPreference::Force2D);
        }
    );
    connect
    (
        m_generationMode3DAction,
        &QAction::triggered,
        this,
        [this]()
        {
            applyGenerationPreference(GCodeGenerationPreference::Force3D);
        }
    );

    ui->menuSet->addSeparator();
    QMenu* themeMenu = ui->menuSet->addMenu(QStringLiteral("主题"));
    QActionGroup* themeActionGroup = new QActionGroup(this);
    themeActionGroup->setExclusive(true);

    m_lightThemeAction = themeMenu->addAction(QStringLiteral("浅色模式"));
    m_lightThemeAction->setCheckable(true);
    themeActionGroup->addAction(m_lightThemeAction);

    m_darkThemeAction = themeMenu->addAction(QStringLiteral("深色模式"));
    m_darkThemeAction->setCheckable(true);
    themeActionGroup->addAction(m_darkThemeAction);

    connect(m_lightThemeAction, &QAction::triggered, this, [this]() { applyTheme(AppThemeMode::Light); });
    connect(m_darkThemeAction, &QAction::triggered, this, [this]() { applyTheme(AppThemeMode::Dark); });

    ui->menuSet->addSeparator();
    m_profileSettingsAction = ui->menuSet->addAction(QStringLiteral("G代码配置..."));
    connect(m_profileSettingsAction, &QAction::triggered, this, [this]() { openProfileSettingsDialog(); });
}

void Gcode_postprocessing_system::openProfileSettingsDialog()
{
    QMap<QString, QColor> layerColors;

    for (const QString& layerName : m_document.layerNames())
    {
        layerColors.insert(layerName, m_document.layerColor(layerName, QColor(Qt::white)));
    }

    GProfileDialog dialog
    (
        m_activeProfile,
        m_document.layerNames(),
        layerColors,
        buildAppThemeColors(m_themeMode),
        this
    );

    if (dialog.exec() != QDialog::Accepted)
    {
        return;
    }

    const GProfile updatedProfile = dialog.profile();
    const QString importedProfilePath = dialog.importedProfilePath().trimmed();

    if (!importedProfilePath.isEmpty())
    {
        const QString runtimeDirectory = runtimeProfileDirectoryPath();
        const QFileInfo importedFileInfo(importedProfilePath);
        const QString absoluteImportedPath = importedFileInfo.absoluteFilePath();
        const QString displayName = updatedProfile.profileName().trimmed().isEmpty()
            ? importedFileInfo.completeBaseName()
            : updatedProfile.profileName().trimmed();

        if (QFileInfo(absoluteImportedPath).dir().absolutePath().compare(runtimeDirectory, Qt::CaseInsensitive) == 0)
        {
            const QString profileId = QStringLiteral("file:%1").arg(QDir::toNativeSeparators(absoluteImportedPath));
            m_loadedProfiles.insert(profileId, updatedProfile);
            m_loadedProfileNames.insert(profileId, displayName);

            if (!m_loadedProfileOrder.contains(profileId))
            {
                m_loadedProfileOrder.append(profileId);
            }

            m_activeProfile = updatedProfile;
            m_activeProfileId = profileId;
        }
        else
        {
            const QString profileId = QStringLiteral("session:%1").arg(++m_sessionImportedProfileSerial);
            m_loadedProfiles.insert(profileId, updatedProfile);
            m_loadedProfileNames.insert(profileId, displayName);
            m_loadedProfileOrder.append(profileId);
            m_activeProfile = updatedProfile;
            m_activeProfileId = profileId;
        }
    }
    else
    {
        m_activeProfile = updatedProfile;

        if (!m_activeProfileId.isEmpty() && m_loadedProfiles.contains(m_activeProfileId))
        {
            m_loadedProfiles[m_activeProfileId] = updatedProfile;
            const QString displayName = updatedProfile.profileName().trimmed().isEmpty()
                ? m_loadedProfileNames.value(m_activeProfileId, QStringLiteral("未命名配置"))
                : updatedProfile.profileName().trimmed();
            m_loadedProfileNames[m_activeProfileId] = displayName;
        }
    }

    refreshAvailableProfilesUi();
    saveSelectedProfileId(m_activeProfileId);

    const QString profileName = m_activeProfile.profileName().trimmed().isEmpty()
        ? QStringLiteral("未命名配置")
        : m_activeProfile.profileName().trimmed();

    statusBar()->showMessage(QStringLiteral("当前 G 代码配置已更新为: %1").arg(profileName), 4000);
}

void Gcode_postprocessing_system::loadAvailableProfiles()
{
    m_loadedProfiles.clear();
    m_loadedProfileNames.clear();
    m_loadedProfileOrder.clear();
    m_sessionImportedProfileSerial = 0;

    const QString builtinThreeAxisProfileId = QString::fromLatin1(kBuiltinThreeAxisProfileId);
    const QString builtinFourAxisProfileId = QString::fromLatin1(kBuiltinFourAxisProfileId);

    m_loadedProfiles.insert(builtinThreeAxisProfileId, GProfile::createDefaultLaserProfile());
    m_loadedProfileNames.insert(builtinThreeAxisProfileId, QStringLiteral("内置3轴默认"));
    m_loadedProfileOrder.append(builtinThreeAxisProfileId);

    m_loadedProfiles.insert(builtinFourAxisProfileId, GProfile::createDefaultRotaryProfile());
    m_loadedProfileNames.insert(builtinFourAxisProfileId, QStringLiteral("内置4轴默认"));
    m_loadedProfileOrder.append(builtinFourAxisProfileId);

    const QDir runtimeDirectory(runtimeProfileDirectoryPath());
    const QFileInfoList profileFiles = runtimeDirectory.entryInfoList(QStringList() << QStringLiteral("*.json"), QDir::Files | QDir::Readable, QDir::Name);

    for (const QFileInfo& profileFileInfo : profileFiles)
    {
        QString errorMessage;
        const GProfile profile = GProfile::loadFromFile(profileFileInfo.absoluteFilePath(), &errorMessage);

        if (!errorMessage.trimmed().isEmpty())
        {
            continue;
        }

        const QString profileId = QStringLiteral("file:%1").arg(QDir::toNativeSeparators(profileFileInfo.absoluteFilePath()));
        const QString displayName = profile.profileName().trimmed().isEmpty()
            ? profileFileInfo.completeBaseName()
            : profile.profileName().trimmed();

        m_loadedProfiles.insert(profileId, profile);
        m_loadedProfileNames.insert(profileId, displayName);
        m_loadedProfileOrder.append(profileId);
    }

    const QString preferredProfileId = loadSelectedProfileId();

    if (!preferredProfileId.isEmpty() && m_loadedProfiles.contains(preferredProfileId))
    {
        m_activeProfileId = preferredProfileId;
        m_activeProfile = m_loadedProfiles.value(preferredProfileId, GProfile::createDefaultLaserProfile());
        return;
    }

    m_activeProfileId = builtinThreeAxisProfileId;
    m_activeProfile = m_loadedProfiles.value(m_activeProfileId, GProfile::createDefaultLaserProfile());
}

void Gcode_postprocessing_system::refreshAvailableProfilesUi()
{
    if (m_toolPanelWidget == nullptr)
    {
        return;
    }

    QList<QPair<QString, QString>> profiles;
    profiles.reserve(m_loadedProfileOrder.size());

    for (const QString& profileId : m_loadedProfileOrder)
    {
        profiles.append(qMakePair(profileId, m_loadedProfileNames.value(profileId, QStringLiteral("未命名配置"))));
    }

    m_toolPanelWidget->setAvailableProfiles(profiles);
    m_toolPanelWidget->setCurrentProfileSelection(m_activeProfileId);
}

bool Gcode_postprocessing_system::applyLoadedProfileById(const QString& profileId, bool announceChange)
{
    if (profileId.trimmed().isEmpty() || !m_loadedProfiles.contains(profileId))
    {
        return false;
    }

    m_activeProfileId = profileId;
    m_activeProfile = m_loadedProfiles.value(profileId, GProfile::createDefaultLaserProfile());
    saveSelectedProfileId(m_activeProfileId);
    refreshAvailableProfilesUi();

    if (announceChange)
    {
        statusBar()->showMessage
        (
            QStringLiteral("当前 G 代码配置已切换为: %1").arg(m_loadedProfileNames.value(profileId, QStringLiteral("未命名配置"))),
            4000
        );
    }

    return true;
}

QString Gcode_postprocessing_system::runtimeProfileDirectoryPath() const
{
    return QCoreApplication::applicationDirPath();
}

QString Gcode_postprocessing_system::loadSelectedProfileId() const
{
    QSettings settings(QStringLiteral("GCodePostProcessingSystem"), QStringLiteral("GCodePostProcessingSystem"));
    return settings.value(QStringLiteral("gcode/selectedProfileId"), QString::fromLatin1(kBuiltinThreeAxisProfileId)).toString().trimmed();
}

void Gcode_postprocessing_system::saveSelectedProfileId(const QString& profileId) const
{
    QSettings settings(QStringLiteral("GCodePostProcessingSystem"), QStringLiteral("GCodePostProcessingSystem"));
    settings.setValue(QStringLiteral("gcode/selectedProfileId"), profileId.trimmed());
}

void Gcode_postprocessing_system::applyTheme(AppThemeMode mode)
{
    m_themeMode = mode;
    const AppThemeColors theme = buildAppThemeColors(mode);

    qApp->setStyle(QStyleFactory::create(QStringLiteral("Fusion")));
    qApp->setPalette(theme.palette);

    setStyleSheet
    (
        QStringLiteral
        (
            "QMainWindow { background-color: %1; color: %2; }"
            "QWidget#centralWidget { background-color: %1; }"
            "QMenuBar { background-color: %3; color: %2; border-bottom: 1px solid %4; }"
            "QMenuBar::item { background: transparent; padding: 4px 10px; }"
            "QMenuBar::item:selected { background: %5; }"
            "QToolBar { background-color: %3; border: none; border-bottom: 1px solid %4; spacing: 0px; }"
            "QStatusBar { background-color: %3; color: %2; border-top: 1px solid %4; }"
            "QStatusBar::item { border: none; }"
        )
        .arg(theme.windowBackground.name())
        .arg(theme.textPrimaryColor.name())
        .arg(theme.panelBackground.name())
        .arg(theme.borderColor.name())
        .arg(theme.hoverBackgroundColor.name())
    );

    if (m_commandLineWidget != nullptr)
    {
        m_commandLineWidget->setTheme(theme);
    }

    if (m_statusPaneWidget != nullptr)
    {
        m_statusPaneWidget->setTheme(theme);
    }

    if (m_toolPanelWidget != nullptr)
    {
        m_toolPanelWidget->setTheme(theme);
    }

    if (ui->openGLWidget != nullptr)
    {
        ui->openGLWidget->setTheme(theme);
    }

    if (m_lightThemeAction != nullptr)
    {
        m_lightThemeAction->setChecked(mode == AppThemeMode::Light);
    }

    if (m_darkThemeAction != nullptr)
    {
        m_darkThemeAction->setChecked(mode == AppThemeMode::Dark);
    }

    saveThemeMode(mode);
}

AppThemeMode Gcode_postprocessing_system::loadThemeMode() const
{
    QSettings settings(QStringLiteral("GCodePostProcessingSystem"), QStringLiteral("GCodePostProcessingSystem"));
    const QString themeValue = settings.value(QStringLiteral("ui/themeMode"), QStringLiteral("light")).toString().trimmed().toLower();
    return themeValue == QStringLiteral("dark") ? AppThemeMode::Dark : AppThemeMode::Light;
}

void Gcode_postprocessing_system::saveThemeMode(AppThemeMode mode) const
{
    QSettings settings(QStringLiteral("GCodePostProcessingSystem"), QStringLiteral("GCodePostProcessingSystem"));
    settings.setValue(QStringLiteral("ui/themeMode"), mode == AppThemeMode::Dark ? QStringLiteral("dark") : QStringLiteral("light"));
}

quint32 Gcode_postprocessing_system::loadSnapOptionMask() const
{
    QSettings settings(QStringLiteral("GCodePostProcessingSystem"), QStringLiteral("GCodePostProcessingSystem"));
    const quint32 defaultMask = CadStatusPaneWidget::defaultSnapOptionMask();
    bool converted = false;
    const quint32 storedMask = settings.value(QStringLiteral("ui/snapModeMask"), defaultMask).toUInt(&converted);

    if (!converted)
    {
        return defaultMask;
    }

    return storedMask & CadStatusPaneWidget::allSnapOptionMask();
}

void Gcode_postprocessing_system::saveSnapOptionMask(quint32 mask) const
{
    QSettings settings(QStringLiteral("GCodePostProcessingSystem"), QStringLiteral("GCodePostProcessingSystem"));
    settings.setValue
    (
        QStringLiteral("ui/snapModeMask"),
        static_cast<uint>(mask & CadStatusPaneWidget::allSnapOptionMask())
    );
}

Gcode_postprocessing_system::GCodeGenerationPreference Gcode_postprocessing_system::loadGenerationPreference() const
{
    QSettings settings(QStringLiteral("GCodePostProcessingSystem"), QStringLiteral("GCodePostProcessingSystem"));
    const QString modeValue = settings.value(QStringLiteral("gcode/outputMode"), QStringLiteral("auto")).toString().trimmed().toLower();

    if (modeValue == QStringLiteral("2d"))
    {
        return GCodeGenerationPreference::Force2D;
    }

    if (modeValue == QStringLiteral("3d"))
    {
        return GCodeGenerationPreference::Force3D;
    }

    return GCodeGenerationPreference::Auto;
}

bool Gcode_postprocessing_system::loadAutoDeduplicateOnExport() const
{
    QSettings settings(QStringLiteral("GCodePostProcessingSystem"), QStringLiteral("GCodePostProcessingSystem"));
    return settings.value(QStringLiteral("gcode/autoDeduplicateOnExport"), false).toBool();
}

void Gcode_postprocessing_system::saveAutoDeduplicateOnExport(bool enabled) const
{
    QSettings settings(QStringLiteral("GCodePostProcessingSystem"), QStringLiteral("GCodePostProcessingSystem"));
    settings.setValue(QStringLiteral("gcode/autoDeduplicateOnExport"), enabled);
}

bool Gcode_postprocessing_system::loadUseDxfFileNameOnExport() const
{
    QSettings settings(QStringLiteral("GCodePostProcessingSystem"), QStringLiteral("GCodePostProcessingSystem"));
    return settings.value(QStringLiteral("gcode/useDxfFileNameOnExport"), false).toBool();
}

void Gcode_postprocessing_system::saveUseDxfFileNameOnExport(bool enabled) const
{
    QSettings settings(QStringLiteral("GCodePostProcessingSystem"), QStringLiteral("GCodePostProcessingSystem"));
    settings.setValue(QStringLiteral("gcode/useDxfFileNameOnExport"), enabled);
}

bool Gcode_postprocessing_system::loadUseDefaultImportPath() const
{
    QSettings settings(QStringLiteral("GCodePostProcessingSystem"), QStringLiteral("GCodePostProcessingSystem"));
    return settings.value(QStringLiteral("ui/useDefaultImportPath"), true).toBool();
}

void Gcode_postprocessing_system::saveUseDefaultImportPath(bool enabled) const
{
    QSettings settings(QStringLiteral("GCodePostProcessingSystem"), QStringLiteral("GCodePostProcessingSystem"));
    settings.setValue(QStringLiteral("ui/useDefaultImportPath"), enabled);
}

bool Gcode_postprocessing_system::loadUseDefaultExportPath() const
{
    QSettings settings(QStringLiteral("GCodePostProcessingSystem"), QStringLiteral("GCodePostProcessingSystem"));
    return settings.value(QStringLiteral("gcode/useDefaultExportPath"), true).toBool();
}

void Gcode_postprocessing_system::saveUseDefaultExportPath(bool enabled) const
{
    QSettings settings(QStringLiteral("GCodePostProcessingSystem"), QStringLiteral("GCodePostProcessingSystem"));
    settings.setValue(QStringLiteral("gcode/useDefaultExportPath"), enabled);
}

void Gcode_postprocessing_system::saveGenerationPreference(GCodeGenerationPreference preference) const
{
    QSettings settings(QStringLiteral("GCodePostProcessingSystem"), QStringLiteral("GCodePostProcessingSystem"));
    QString modeValue = QStringLiteral("auto");

    if (preference == GCodeGenerationPreference::Force2D)
    {
        modeValue = QStringLiteral("2d");
    }
    else if (preference == GCodeGenerationPreference::Force3D)
    {
        modeValue = QStringLiteral("3d");
    }

    settings.setValue(QStringLiteral("gcode/outputMode"), modeValue);
}

void Gcode_postprocessing_system::applyGenerationPreference(GCodeGenerationPreference preference)
{
    m_generationPreference = preference;
    saveGenerationPreference(m_generationPreference);

    if (m_generationModeAutoAction != nullptr)
    {
        m_generationModeAutoAction->setChecked(m_generationPreference == GCodeGenerationPreference::Auto);
    }

    if (m_generationMode2DAction != nullptr)
    {
        m_generationMode2DAction->setChecked(m_generationPreference == GCodeGenerationPreference::Force2D);
    }

    if (m_generationMode3DAction != nullptr)
    {
        m_generationMode3DAction->setChecked(m_generationPreference == GCodeGenerationPreference::Force3D);
    }

    if (m_toolPanelWidget != nullptr)
    {
        switch (m_generationPreference)
        {
        case GCodeGenerationPreference::Auto:
            m_toolPanelWidget->setGCodeModeSelection(CadToolPanelWidget::GCodeModeSelection::Auto);
            break;
        case GCodeGenerationPreference::Force2D:
            m_toolPanelWidget->setGCodeModeSelection(CadToolPanelWidget::GCodeModeSelection::ThreeAxis);
            break;
        case GCodeGenerationPreference::Force3D:
            m_toolPanelWidget->setGCodeModeSelection(CadToolPanelWidget::GCodeModeSelection::FourAxisAroundA);
            break;
        default:
            break;
        }
    }

    const QString modeText = m_generationPreference == GCodeGenerationPreference::Force3D
        ? QStringLiteral("4轴(绕A)")
        : (m_generationPreference == GCodeGenerationPreference::Force2D
            ? QStringLiteral("3轴")
            : QStringLiteral("自动"));
    statusBar()->showMessage(QStringLiteral("G 代码输出模式已切换为%1").arg(modeText), 3000);
}


void Gcode_postprocessing_system::initializeToolPanel()
{
    m_toolPanelWidget = new CadToolPanelWidget(this);
    m_toolPanelWidget->setAutoDeduplicateEnabled(loadAutoDeduplicateOnExport());
    m_toolPanelWidget->setUseDxfFileNameEnabled(loadUseDxfFileNameOnExport());
    m_toolPanelWidget->setUseDefaultImportPathEnabled(loadUseDefaultImportPath());
    m_toolPanelWidget->setUseDefaultExportPathEnabled(loadUseDefaultExportPath());
    ui->mainToolBar->setMovable(false);
    ui->mainToolBar->setFloatable(false);
    ui->mainToolBar->addWidget(m_toolPanelWidget);

    connect(&m_document, &CadDocument::sceneChanged, this, [this]() { syncToolPanelState(); });
    connect(ui->openGLWidget, &CadViewer::selectedEntityChanged, this, [this](CadItem*) { syncToolPanelState(); });

    connect
    (
        m_toolPanelWidget,
        &CadToolPanelWidget::drawRequested,
        this,
        [this](DrawType drawType)
        {
            applyDefaultDrawingProperties();
            ui->openGLWidget->startDrawing(drawType);
        }
    );

    connect
    (
        m_toolPanelWidget,
        &CadToolPanelWidget::moveRequested,
        this,
        [this]()
        {
            if (!ui->openGLWidget->startMoveSelected())
            {
                statusBar()->showMessage(QStringLiteral("请先选择一个图元再执行移动"), 3000);
            }
        }
    );

    connect(m_toolPanelWidget, &CadToolPanelWidget::deleteRequested, this, [this]() { deleteSelectedEntity(); });
    connect(m_toolPanelWidget, &CadToolPanelWidget::copyRequested, this, [this]() { copySelectedEntity(); });
    connect(m_toolPanelWidget, &CadToolPanelWidget::rotateRequested, this, [this]() { rotateSelectedEntity(); });
    connect(m_toolPanelWidget, &CadToolPanelWidget::scaleRequested, this, [this]() { scaleSelectedEntity(); });
    connect(m_toolPanelWidget, &CadToolPanelWidget::arrayRequested, this, [this]() { arraySelectedEntity(); });
    connect(m_toolPanelWidget, &CadToolPanelWidget::circularArrayRequested, this, [this]() { circularArraySelectedEntity(); });
    connect(m_toolPanelWidget, &CadToolPanelWidget::mirrorRequested, this, [this]() { mirrorSelectedEntities(); });
    connect(m_toolPanelWidget, &CadToolPanelWidget::offsetRequested, this, [this]() { offsetSelectedEntity(); });
    connect(m_toolPanelWidget, &CadToolPanelWidget::trimRequested, this, [this]() { trimSelectedEntity(); });
    connect(m_toolPanelWidget, &CadToolPanelWidget::extendRequested, this, [this]() { extendSelectedEntity(); });
    connect(m_toolPanelWidget, &CadToolPanelWidget::joinRequested, this, [this]() { joinSelectedEntities(); });
    connect(m_toolPanelWidget, &CadToolPanelWidget::filletRequested, this, [this]() { filletSelectedEntities(); });
    connect(m_toolPanelWidget, &CadToolPanelWidget::chamferRequested, this, [this]() { chamferSelectedEntities(); });
    connect(m_toolPanelWidget, &CadToolPanelWidget::importFileRequested, this, [this]() { ui->action_File_Import_Dxf->trigger(); });
    connect(m_toolPanelWidget, &CadToolPanelWidget::exportGCodeRequested, this, [this]() { ui->action_File_Export_G->trigger(); });
    connect(m_toolPanelWidget, &CadToolPanelWidget::deduplicateRequested, this, [this]() { removeDuplicateEntities(); });
    connect(m_toolPanelWidget, &CadToolPanelWidget::autoDeduplicateOptionChanged, this, [this](bool enabled) { saveAutoDeduplicateOnExport(enabled); });
    connect(m_toolPanelWidget, &CadToolPanelWidget::useDxfFileNameOptionChanged, this, [this](bool enabled) { saveUseDxfFileNameOnExport(enabled); });
    connect(m_toolPanelWidget, &CadToolPanelWidget::useDefaultImportPathOptionChanged, this, [this](bool enabled) { saveUseDefaultImportPath(enabled); });
    connect(m_toolPanelWidget, &CadToolPanelWidget::useDefaultExportPathOptionChanged, this, [this](bool enabled) { saveUseDefaultExportPath(enabled); });
    connect(m_toolPanelWidget, &CadToolPanelWidget::sortKeepDirectionRequested, this, [this]() { ui->action_Sort_2D_Assign->trigger(); });
    connect(m_toolPanelWidget, &CadToolPanelWidget::smartSortRequested, this, [this]() { ui->action_Sort_2D_Smart->trigger(); });
    connect
    (
        m_toolPanelWidget,
        &CadToolPanelWidget::profileSelectionChanged,
        this,
        [this](const QString& profileId)
        {
            applyLoadedProfileById(profileId);
        }
    );
    connect(m_toolPanelWidget, &CadToolPanelWidget::profileSettingsRequested, this, [this]() { openProfileSettingsDialog(); });
    connect
    (
        m_toolPanelWidget,
        &CadToolPanelWidget::gcodeModeSelectionChanged,
        this,
        [this](CadToolPanelWidget::GCodeModeSelection selection)
        {
            switch (selection)
            {
            case CadToolPanelWidget::GCodeModeSelection::Auto:
                applyGenerationPreference(GCodeGenerationPreference::Auto);
                break;
            case CadToolPanelWidget::GCodeModeSelection::ThreeAxis:
                applyGenerationPreference(GCodeGenerationPreference::Force2D);
                break;
            case CadToolPanelWidget::GCodeModeSelection::FourAxisAroundA:
                applyGenerationPreference(GCodeGenerationPreference::Force3D);
                break;
            default:
                break;
            }
        }
    );

    connect
    (
        m_toolPanelWidget,
        &CadToolPanelWidget::layerChangeRequested,
        this,
        [this](const QString& layerName)
        {
            const QString normalizedLayerName = layerName.trimmed().isEmpty() ? QStringLiteral("0") : layerName.trimmed();
            const QVector<CadItem*> selectedItems = ui->openGLWidget->selectedEntities();

            if (!selectedItems.isEmpty())
            {
                int changedCount = 0;

                for (CadItem* item : selectedItems)
                {
                    if (item != nullptr && m_editer.changeEntityLayer(item, normalizedLayerName))
                    {
                        ++changedCount;
                    }
                }

                if (changedCount > 0)
                {
                    statusBar()->showMessage
                    (
                        QStringLiteral("已将 %1 个图元图层更新为 %2").arg(changedCount).arg(normalizedLayerName),
                        3000
                    );
                }

                return;
            }

            m_currentLayerName = normalizedLayerName;

            if (m_document.ensureLayerExists(m_currentLayerName))
            {
                m_document.notifySceneChanged();
            }

            applyDefaultDrawingProperties();
            syncToolPanelState();
        }
    );

    connect
    (
        m_toolPanelWidget,
        &CadToolPanelWidget::colorChangeRequested,
        this,
        [this](int colorIndex)
        {
            const QVector<CadItem*> selectedItems = ui->openGLWidget->selectedEntities();

            if (!selectedItems.isEmpty())
            {
                int changedCount = 0;

                for (CadItem* item : selectedItems)
                {
                    if (item == nullptr)
                    {
                        continue;
                    }

                    const QColor targetColor = colorIndex == kColorByLayer
                        ? m_document.layerColor(entityLayerName(item), entityDisplayColor(m_document, item))
                        : (colorIndex < 0 ? entityDisplayColor(m_document, item) : colorFromAci(colorIndex));

                    if (m_editer.changeEntityColor(item, targetColor, colorIndex))
                    {
                        ++changedCount;
                    }
                }

                if (changedCount > 0)
                {
                    statusBar()->showMessage(QStringLiteral("已更新 %1 个图元颜色").arg(changedCount), 3000);
                }

                return;
            }

            m_currentColorIndex = colorIndex;

            if (colorIndex == kColorByLayer)
            {
                m_currentColor = m_document.layerColor(m_currentLayerName, QColor(Qt::white));
            }
            else if (colorIndex >= 0)
            {
                m_currentColor = colorFromAci(colorIndex);
            }

            applyDefaultDrawingProperties();
            syncToolPanelState();
        }
    );

    switch (m_generationPreference)
    {
    case GCodeGenerationPreference::Auto:
        m_toolPanelWidget->setGCodeModeSelection(CadToolPanelWidget::GCodeModeSelection::Auto);
        break;
    case GCodeGenerationPreference::Force2D:
        m_toolPanelWidget->setGCodeModeSelection(CadToolPanelWidget::GCodeModeSelection::ThreeAxis);
        break;
    case GCodeGenerationPreference::Force3D:
        m_toolPanelWidget->setGCodeModeSelection(CadToolPanelWidget::GCodeModeSelection::FourAxisAroundA);
        break;
    default:
        break;
    }

    refreshAvailableProfilesUi();
}

void Gcode_postprocessing_system::syncToolPanelState()
{
    if (m_toolPanelWidget == nullptr)
    {
        return;
    }

    const QStringList layerNames = m_document.layerNames();
    QMap<QString, QColor> layerColors;

    for (const QString& layerName : layerNames)
    {
        layerColors.insert(layerName, m_document.layerColor(layerName, QColor(Qt::white)));
    }

    m_toolPanelWidget->setLayerNames(layerNames, layerColors);

    CadItem* selectedItem = ui->openGLWidget->selectedEntity();

    if (selectedItem != nullptr)
    {
        m_toolPanelWidget->setModifyActionsEnabled(true);
        m_toolPanelWidget->setLayerStatusText(QStringLiteral("当前选中图元图层"));
        m_toolPanelWidget->setPropertyStatusText(QStringLiteral("当前选中图元特性"));
        m_toolPanelWidget->setActiveLayerName(entityLayerName(selectedItem));
        m_toolPanelWidget->setActiveColorState
        (
            entityDisplayColor(m_document, selectedItem),
            entityColorIndex(selectedItem),
            m_document.layerColor(entityLayerName(selectedItem), QColor(Qt::white))
        );
        return;
    }

    if (m_currentLayerName.trimmed().isEmpty())
    {
        m_currentLayerName = layerNames.isEmpty() ? QStringLiteral("0") : layerNames.front();
    }

    if (m_currentColorIndex == kColorByLayer)
    {
        m_currentColor = m_document.layerColor(m_currentLayerName, QColor(Qt::white));
    }
    else if (m_currentColorIndex >= 0)
    {
        m_currentColor = colorFromAci(m_currentColorIndex);
    }

    m_toolPanelWidget->setModifyActionsEnabled(false);
    m_toolPanelWidget->setLayerStatusText(QStringLiteral("当前默认绘图图层"));
    m_toolPanelWidget->setPropertyStatusText(QStringLiteral("当前默认绘图特性"));
    m_toolPanelWidget->setActiveLayerName(m_currentLayerName);
    m_toolPanelWidget->setActiveColorState
    (
        m_currentColor,
        m_currentColorIndex,
        m_document.layerColor(m_currentLayerName, QColor(Qt::white))
    );
}

void Gcode_postprocessing_system::applyDefaultDrawingProperties()
{
    if (m_currentLayerName.trimmed().isEmpty())
    {
        m_currentLayerName = QStringLiteral("0");
    }

    if (m_currentColorIndex == kColorByLayer)
    {
        m_currentColor = m_document.layerColor(m_currentLayerName, QColor(Qt::white));
    }
    else if (m_currentColorIndex >= 0)
    {
        m_currentColor = colorFromAci(m_currentColorIndex);
    }

    ui->openGLWidget->setDefaultDrawingProperties(m_currentLayerName, m_currentColor, m_currentColorIndex);
}

QString Gcode_postprocessing_system::activeLayerName() const
{
    CadItem* selectedItem = ui->openGLWidget->selectedEntity();
    return selectedItem != nullptr ? entityLayerName(selectedItem) : m_currentLayerName;
}

QColor Gcode_postprocessing_system::activeColor() const
{
    CadItem* selectedItem = ui->openGLWidget->selectedEntity();
    return selectedItem != nullptr ? entityDisplayColor(m_document, selectedItem) : m_currentColor;
}

int Gcode_postprocessing_system::activeColorIndex() const
{
    CadItem* selectedItem = ui->openGLWidget->selectedEntity();
    return selectedItem != nullptr ? entityColorIndex(selectedItem) : m_currentColorIndex;
}
