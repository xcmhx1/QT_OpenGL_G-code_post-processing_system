#include "platform/pch.h"

#include "desktop/Gcode_postprocessing_system.h"
#include "ui/dialogs/CadAppearanceSettingsDialog.h"
#include "ui/dialogs/CadHelpDialog.h"
#include "cad/items/CadItem.h"
#include "ui/dialogs/GProfileDialog.h"
#include "ui/dialogs/GProfileManagerDialog.h"
#include "infrastructure/config/GProfilePathStore.h"
#include "ui/widgets/MachiningSettingsWidget.h"

#include <QActionGroup>
#include <QApplication>
#include <QCoreApplication>
#include <QDir>
#include <QDockWidget>
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
#include <QVariant>
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

    QString entityTypeDisplayName(DRW::ETYPE type)
    {
        switch (type)
        {
        case DRW::ETYPE::POINT:
            return QStringLiteral("POINT 点");
        case DRW::ETYPE::LINE:
            return QStringLiteral("LINE 直线");
        case DRW::ETYPE::XLINE:
            return QStringLiteral("XLINE 构造线");
        case DRW::ETYPE::RAY:
            return QStringLiteral("RAY 射线");
        case DRW::ETYPE::CIRCLE:
            return QStringLiteral("CIRCLE 圆");
        case DRW::ETYPE::ARC:
            return QStringLiteral("ARC 圆弧");
        case DRW::ETYPE::ELLIPSE:
            return QStringLiteral("ELLIPSE 椭圆");
        case DRW::ETYPE::POLYLINE:
            return QStringLiteral("POLYLINE 多段线");
        case DRW::ETYPE::LWPOLYLINE:
            return QStringLiteral("LWPOLYLINE 轻量多段线");
        case DRW::ETYPE::SPLINE:
            return QStringLiteral("SPLINE 样条曲线");
        default:
            return QStringLiteral("UNKNOWN 未知");
        }
    }

    QString selectedEntityTypeSummary(const QVector<CadItem*>& items)
    {
        if (items.isEmpty())
        {
            return QStringLiteral("无");
        }

        if (items.size() == 1 && items.first() != nullptr)
        {
            return entityTypeDisplayName(items.first()->m_type);
        }

        QMap<QString, int> typeCounts;

        for (const CadItem* item : items)
        {
            if (item == nullptr)
            {
                continue;
            }

            const QString typeName = entityTypeDisplayName(item->m_type);
            typeCounts[typeName] = typeCounts.value(typeName) + 1;
        }

        QStringList parts;

        for (auto iterator = typeCounts.cbegin(); iterator != typeCounts.cend(); ++iterator)
        {
            parts.push_back(QStringLiteral("%1 x%2").arg(iterator.key()).arg(iterator.value()));
        }

        return QStringLiteral("已选 %1: %2").arg(items.size()).arg(parts.join(QStringLiteral(", ")));
    }

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

    struct ThemeColorSetting
    {
        const char* key = nullptr;
        QColor AppThemeColors::* member = nullptr;
    };

    const ThemeColorSetting* themeColorSettings(int* count = nullptr)
    {
        static const ThemeColorSetting settings[] =
        {
            { "windowBackground", &AppThemeColors::windowBackground },
            { "panelBackground", &AppThemeColors::panelBackground },
            { "surfaceBackground", &AppThemeColors::surfaceBackground },
            { "surfaceAltBackground", &AppThemeColors::surfaceAltBackground },
            { "borderColor", &AppThemeColors::borderColor },
            { "textPrimaryColor", &AppThemeColors::textPrimaryColor },
            { "textSecondaryColor", &AppThemeColors::textSecondaryColor },
            { "accentColor", &AppThemeColors::accentColor },
            { "accentTextColor", &AppThemeColors::accentTextColor },
            { "hoverBackgroundColor", &AppThemeColors::hoverBackgroundColor },
            { "pressedBackgroundColor", &AppThemeColors::pressedBackgroundColor },
            { "viewerBackgroundColor", &AppThemeColors::viewerBackgroundColor },
            { "viewerGridColor", &AppThemeColors::viewerGridColor },
            { "processLabelFillColor", &AppThemeColors::processLabelFillColor },
            { "processLabelBorderColor", &AppThemeColors::processLabelBorderColor },
            { "processLabelTextColor", &AppThemeColors::processLabelTextColor },
            { "selectedProcessLabelFillColor", &AppThemeColors::selectedProcessLabelFillColor },
            { "selectedProcessLabelBorderColor", &AppThemeColors::selectedProcessLabelBorderColor },
            { "selectedProcessLabelTextColor", &AppThemeColors::selectedProcessLabelTextColor },
            { "selectedBasePointColor", &AppThemeColors::selectedBasePointColor },
            { "selectedControlPointColor", &AppThemeColors::selectedControlPointColor }
        };

        if (count != nullptr)
        {
            *count = static_cast<int>(sizeof(settings) / sizeof(settings[0]));
        }

        return settings;
    }

    QString themeModeToSettingValue(AppThemeMode mode)
    {
        if (mode == AppThemeMode::Dark)
        {
            return QStringLiteral("dark");
        }

        if (mode == AppThemeMode::Custom)
        {
            return QStringLiteral("custom");
        }

        return QStringLiteral("light");
    }

    AppThemeMode settingValueToThemeMode(const QString& value)
    {
        const QString normalizedValue = value.trimmed().toLower();

        if (normalizedValue == QStringLiteral("dark"))
        {
            return AppThemeMode::Dark;
        }

        if (normalizedValue == QStringLiteral("custom"))
        {
            return AppThemeMode::Custom;
        }

        return AppThemeMode::Light;
    }

}

Gcode_postprocessing_system::Gcode_postprocessing_system(QWidget* parent)
    : QMainWindow(parent)
    , ui(new Ui::Gcode_postprocessing_systemClass())
{
    ui->setupUi(this);
    m_branding = AppBranding::load();
    m_license = AppLicense::load();
    applyBranding();
    loadAvailableProfiles();

    m_commandLineWidget = new CadCommandLineWidget(this);
    m_statusPaneWidget = new CadStatusPaneWidget(this);

    if (QVBoxLayout* centralLayout = qobject_cast<QVBoxLayout*>(ui->centralWidget->layout()))
    {
        centralLayout->setContentsMargins(0, 0, 0, 0);
        centralLayout->setSpacing(0);
        centralLayout->addWidget(m_commandLineWidget);
        centralLayout->addWidget(m_statusPaneWidget);
    }

    m_editer.setDocument(&m_document);
    m_editer.setProcessState(&m_processState);
    ui->openGLWidget->setEditer(&m_editer);
    ui->openGLWidget->setDocument(&m_document);
    ui->openGLWidget->setDocumentProcessState(&m_processState);
    ui->openGLWidget->setProcessPresentation(nullptr);
    ui->openGLWidget->refreshCommandPrompt();

    connect(ui->openGLWidget, &CadViewer::hoveredWorldPositionChanged, m_statusPaneWidget, &CadStatusPaneWidget::setWorldPosition);
    const auto updateStatusEntityType = [this]()
    {
        if (m_statusPaneWidget != nullptr)
        {
            m_statusPaneWidget->setEntityTypeText(selectedEntityTypeSummary(ui->openGLWidget->selectedEntities()));
        }
    };
    connect(ui->openGLWidget, &CadViewer::selectedEntityChanged, this, [updateStatusEntityType](CadItem*) { updateStatusEntityType(); });
    connect(ui->openGLWidget, &CadViewer::processDirectionToggleRequested, this,
        [this](cadcam::geometry::EntityId processEntityId)
        {
            const auto state = m_processState.stateOrDefault(processEntityId);
            const auto direction = state.overrideData.direction == cadcam::process::DirectionPreference::Reverse
                ? cadcam::process::DirectionPreference::Forward
                : cadcam::process::DirectionPreference::Reverse;
            if (m_processState.setDirection(processEntityId, direction))
            {
                invalidateCurrentProcessPlan();
                ui->openGLWidget->appendCommandMessage(QStringLiteral("已更新加工方向偏好，请重新排序。"));
            }
        });
    connect(&m_document, &CadDocument::sceneChanged, this, updateStatusEntityType);
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
    updateStatusEntityType();
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
    QAction* assignRotaryEndCutAction = new QAction(QStringLiteral("指定为加工断面"), this);
    QAction* assignWasteRotaryEndCutAction = new QAction(QStringLiteral("指定为废面"), this);
    QAction* recognizeRotaryTubeSectionAction = new QAction(QStringLiteral("识别方管垂直截面（外轮廓）"), this);
    QAction* removeInternalPathsAction = new QAction(QStringLiteral("去除内部线条"), this);
    QAction* restoreInternalPathsAction = new QAction(QStringLiteral("恢复内部线条"), this);
    QAction* clearSelectedRotaryEndCutAssignmentsAction = new QAction(QStringLiteral("清除选中加工断面指定"), this);
    QAction* clearRotaryEndCutAssignmentsAction = new QAction(QStringLiteral("清除全部加工断面指定"), this);

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
    ui->menuSort->addSeparator();
    ui->menuSort->addAction(assignRotaryEndCutAction);
    ui->menuSort->addAction(assignWasteRotaryEndCutAction);
    ui->menuSort->addSeparator();
    ui->menuSort->addAction(recognizeRotaryTubeSectionAction);
    ui->menuSort->addAction(removeInternalPathsAction);
    ui->menuSort->addAction(restoreInternalPathsAction);
    ui->menuSort->addAction(clearSelectedRotaryEndCutAssignmentsAction);
    ui->menuSort->addAction(clearRotaryEndCutAssignmentsAction);

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
    connect(assignRotaryEndCutAction, &QAction::triggered, this, [this]() { smartAssignSelectedRotaryEndCut(); });
    connect(assignWasteRotaryEndCutAction, &QAction::triggered, this, [this]() { assignSelectedWasteEndCut(); });
    connect(recognizeRotaryTubeSectionAction, &QAction::triggered, this, [this]() { recognizeRotaryTubeSection(); });
    connect(removeInternalPathsAction, &QAction::triggered, this, [this]() { removeInternalMachiningPaths(); });
    connect(restoreInternalPathsAction, &QAction::triggered, this, [this]() { restoreInternalMachiningPaths(); });
    connect(clearSelectedRotaryEndCutAssignmentsAction, &QAction::triggered, this, [this]() { clearSelectedRotaryEndCutAssignments(); });
    connect(clearRotaryEndCutAssignmentsAction, &QAction::triggered, this, [this]() { clearRotaryEndCutAssignments(); });
    connect(ui->action_Sort_2D_Assign, &QAction::triggered, this, [this]() { sortEntitiesByCurrentMode(false); });
    connect(ui->action_Sort_2D_Smart, &QAction::triggered, this, [this]() { sortEntitiesByCurrentMode(true); });

    ui->action_Sort_3D_Assign->setVisible(false);
    ui->action_Sort_3D_Smart->setVisible(false);

    m_generationPreference = loadGenerationPreference();

    if (m_generationPreference == GCodeGenerationPreference::Force3D && !m_license.allows(AppFeature::FourAxisExport))
    {
        m_generationPreference = GCodeGenerationPreference::Auto;
    }

    initializeThemeMenu();
    initializeHelpMenu();
    initializeMachiningSettingsDock();
    initializeToolPanel();

    QSettings windowSettings(QStringLiteral("GCodePostProcessingSystem"), QStringLiteral("GCodePostProcessingSystem"));
    restoreGeometry(windowSettings.value(QStringLiteral("ui/mainWindowGeometry")).toByteArray());
    const QByteArray savedWindowState = windowSettings.value(QStringLiteral("ui/mainWindowState")).toByteArray();

    if (savedWindowState.isEmpty() || !restoreState(savedWindowState))
    {
        addDockWidget(Qt::RightDockWidgetArea, m_machiningSettingsDock);
        m_machiningSettingsDock->show();
    }

    applyDefaultDrawingProperties();
    applyTheme(loadThemeMode());
    syncToolPanelState();
}

Gcode_postprocessing_system::~Gcode_postprocessing_system()
{
    QSettings settings(QStringLiteral("GCodePostProcessingSystem"), QStringLiteral("GCodePostProcessingSystem"));
    settings.setValue(QStringLiteral("ui/mainWindowGeometry"), saveGeometry());
    settings.setValue(QStringLiteral("ui/mainWindowState"), saveState());
    delete ui;
}

void Gcode_postprocessing_system::showMachiningContextMenu(const QPoint& globalPos)
{
    QMenu menu(this);
    QAction* recognizeSectionAction = menu.addAction(QStringLiteral("截面识别"));
    QMenu* processSectionMenu = menu.addMenu(QStringLiteral("加工断面"));
    QAction* toggleProcessSectionAction = processSectionMenu->addAction(QStringLiteral("加工断面指定/恢复"));
    QAction* recognizeAllProcessSectionsAction = processSectionMenu->addAction(QStringLiteral("所有断面识别"));
    QAction* restoreAllProcessSectionsAction = processSectionMenu->addAction(QStringLiteral("所有断面恢复"));
    QMenu* internalLineMenu = menu.addMenu(QStringLiteral("内部线条"));
    QAction* toggleInternalLineAction = internalLineMenu->addAction(QStringLiteral("内部线条指定/恢复"));
    QAction* recognizeAllInternalLinesAction = internalLineMenu->addAction(QStringLiteral("所有线条识别"));
    QAction* restoreAllInternalLinesAction = internalLineMenu->addAction(QStringLiteral("所有线条恢复"));
    menu.addSeparator();
    QAction* clearAllAction = menu.addAction(QStringLiteral("清空所有面线"));

    const bool documentEmpty = m_document.m_entities.empty();
    const bool hasSelection = !ui->openGLWidget->selectedEntities().isEmpty();
    recognizeSectionAction->setEnabled(!documentEmpty && hasSelection);
    toggleProcessSectionAction->setEnabled(!documentEmpty && hasSelection);
    toggleInternalLineAction->setEnabled(!documentEmpty && hasSelection);
    processSectionMenu->setEnabled(!documentEmpty);
    internalLineMenu->setEnabled(!documentEmpty);
    clearAllAction->setEnabled(!documentEmpty);

    connect(recognizeSectionAction, &QAction::triggered, this, [this]() { recognizeRotaryTubeSection(); });
    connect(toggleProcessSectionAction, &QAction::triggered, this, [this]() { toggleSelectedRotaryEndCutAssignment(); });
    connect(recognizeAllProcessSectionsAction, &QAction::triggered, this, [this]() { recognizeAllRotaryEndCuts(); });
    connect(restoreAllProcessSectionsAction, &QAction::triggered, this, [this]() { clearRotaryEndCutAssignments(); });
    connect(toggleInternalLineAction, &QAction::triggered, this, [this]() { toggleSelectedInternalPathAssignment(); });
    connect(recognizeAllInternalLinesAction, &QAction::triggered, this, [this]() { removeInternalMachiningPaths(); });
    connect(restoreAllInternalLinesAction, &QAction::triggered, this, [this]() { restoreInternalMachiningPaths(); });
    connect(clearAllAction, &QAction::triggered, this, [this]() { clearAllMachiningFaceAndLineAssignments(); });
    menu.exec(globalPos);
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
        ui->openGLWidget,
        &CadViewer::machiningContextMenuRequested,
        this,
        &Gcode_postprocessing_system::showMachiningContextMenu
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
    QMenu* appearanceMenu = ui->menuSet->addMenu(QStringLiteral("外观设置"));
    QMenu* themeMenu = appearanceMenu->addMenu(QStringLiteral("主题"));
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

    appearanceMenu->addSeparator();
    m_customAppearanceAction = appearanceMenu->addAction(QStringLiteral("自定义外观..."));
    connect(m_customAppearanceAction, &QAction::triggered, this, [this]() { openAppearanceSettingsDialog(); });

    ui->menuSet->addSeparator();
    m_profileSettingsAction = ui->menuSet->addAction(QStringLiteral("G代码配置..."));
    connect(m_profileSettingsAction, &QAction::triggered, this, [this]() { openProfileSettingsDialog(); });
}

void Gcode_postprocessing_system::initializeHelpMenu()
{
    QMenu* helpMenu = menuBar()->addMenu(QStringLiteral("帮助"));

    auto addHelpAction =
        [this, helpMenu](const QString& title, CadHelpSection section)
        {
            QAction* action = helpMenu->addAction(title);
            connect(action, &QAction::triggered, this, [this, section]() { openHelpDialog(section); });
        };

    addHelpAction(QStringLiteral("快速上手"), CadHelpSection::QuickStart);
    helpMenu->addSeparator();
    addHelpAction(QStringLiteral("快捷命令"), CadHelpSection::Shortcuts);
    addHelpAction(QStringLiteral("绘图教程"), CadHelpSection::Drawing);
    addHelpAction(QStringLiteral("修改教程"), CadHelpSection::Editing);
    addHelpAction(QStringLiteral("机加工 / G代码"), CadHelpSection::Machining);
    addHelpAction(QStringLiteral("位图导入"), CadHelpSection::BitmapImport);
    addHelpAction(QStringLiteral("外观与显示"), CadHelpSection::Appearance);
    helpMenu->addSeparator();
    addHelpAction(QStringLiteral("关于"), CadHelpSection::About);
}

void Gcode_postprocessing_system::openHelpDialog(CadHelpSection section)
{
    CadHelpDialog dialog(this);
    dialog.setCurrentSection(section);
    dialog.exec();
}

void Gcode_postprocessing_system::applyBranding()
{
    QCoreApplication::setApplicationName(m_branding.applicationName());
    QCoreApplication::setOrganizationName(m_branding.companyName());

    QString title = m_branding.applicationName();

    if (!m_branding.windowTitleSuffix().trimmed().isEmpty())
    {
        title += QStringLiteral(" - %1").arg(m_branding.windowTitleSuffix().trimmed());
    }

    title += QStringLiteral(" [%1]").arg(m_license.editionName());
    setWindowTitle(title);

    const QIcon icon = m_branding.icon();

    if (!icon.isNull())
    {
        setWindowIcon(icon);
        qApp->setWindowIcon(icon);
    }

    statusBar()->showMessage(m_license.statusText(), 5000);
}

bool Gcode_postprocessing_system::ensureFeatureAvailable(AppFeature feature, const QString& actionName)
{
    if (m_license.allows(feature))
    {
        return true;
    }

    const QString message = QStringLiteral("%1 属于 Pro 功能。\n\n当前授权：%2\n机器码：%3\n\n请将机器码发给软件提供方，获取 license.dat 后放到程序目录。")
        .arg(actionName, m_license.statusText(), AppLicense::currentMachineId());

    QMessageBox::information(this, QStringLiteral("需要授权"), message);

    if (ui != nullptr && ui->openGLWidget != nullptr)
    {
        ui->openGLWidget->appendCommandMessage(QStringLiteral("%1 未执行：需要 Pro 授权。").arg(actionName));
        ui->openGLWidget->refreshCommandPrompt();
    }

    return false;
}

void Gcode_postprocessing_system::openAppearanceSettingsDialog()
{
    if (!ensureFeatureAvailable(AppFeature::CustomAppearance, QStringLiteral("自定义外观")))
    {
        return;
    }

    AppThemeMode baseMode = AppThemeMode::Light;
    AppThemeColors initialTheme = m_themeMode == AppThemeMode::Custom
        ? loadCustomThemeColors(&baseMode)
        : m_themeColors;

    if (m_themeMode == AppThemeMode::Light || m_themeMode == AppThemeMode::Dark)
    {
        baseMode = m_themeMode;
        initialTheme = buildAppThemeColors(m_themeMode);
    }

    CadAppearanceSettingsDialog dialog(baseMode, initialTheme, this);

    if (dialog.exec() != QDialog::Accepted)
    {
        return;
    }

    saveCustomThemeColors(dialog.baseMode(), dialog.themeColors());
    applyTheme(AppThemeMode::Custom);
    statusBar()->showMessage(QStringLiteral("自定义外观已应用"), 3000);
}

void Gcode_postprocessing_system::openProfileSettingsDialog()
{
    if (!ensureFeatureAvailable(AppFeature::ProfileSettings, QStringLiteral("G代码配置")))
    {
        return;
    }

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
        m_themeColors,
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
        const QFileInfo importedFileInfo(importedProfilePath);
        const QString absoluteImportedPath = importedFileInfo.absoluteFilePath();
        const QString displayName = updatedProfile.profileName().trimmed().isEmpty()
            ? importedFileInfo.completeBaseName()
            : updatedProfile.profileName().trimmed();

        GProfilePathStore::recordDirectory(importedFileInfo.absolutePath());
        const QString profileId = GProfilePathStore::profileIdForFile(absoluteImportedPath);
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
    invalidateCurrentProcessPlan();
    saveSelectedProfileId(m_activeProfileId);

    const QString profileName = m_activeProfile.profileName().trimmed().isEmpty()
        ? QStringLiteral("未命名配置")
        : m_activeProfile.profileName().trimmed();

    statusBar()->showMessage(QStringLiteral("当前 G 代码配置已更新为: %1").arg(profileName), 4000);
}

void Gcode_postprocessing_system::openProfileManagerDialog()
{
    if (!ensureFeatureAvailable(AppFeature::ProfileSettings, QStringLiteral("G代码配置管理")))
    {
        return;
    }

    GProfileManagerDialog dialog(m_activeProfileId, m_themeColors, this);
    const QString previousProfileId = m_activeProfileId;
    const GProfile previousProfile = m_activeProfile;
    const QString previousProfileName = m_loadedProfileNames.value(previousProfileId);
    const int result = dialog.exec();
    const QString selectedProfilePath = dialog.selectedProfilePath();

    loadAvailableProfiles();

    if ((result != QDialog::Accepted || selectedProfilePath.isEmpty())
        && m_loadedProfiles.contains(previousProfileId))
    {
        m_activeProfileId = previousProfileId;
        m_activeProfile = previousProfile;
        m_loadedProfiles[previousProfileId] = previousProfile;

        if (!previousProfileName.isEmpty())
        {
            m_loadedProfileNames[previousProfileId] = previousProfileName;
        }

        saveSelectedProfileId(previousProfileId);
    }

    refreshAvailableProfilesUi();

    if (result == QDialog::Accepted && !selectedProfilePath.isEmpty())
    {
        const QString profileId = GProfilePathStore::profileIdForFile(selectedProfilePath);

        if (!applyLoadedProfileById(profileId))
        {
            QMessageBox::warning(this, QStringLiteral("配置不可用"), QStringLiteral("所选配置文件无法加载。"));
        }
    }
}

void Gcode_postprocessing_system::loadAvailableProfiles()
{
    m_loadedProfiles.clear();
    m_loadedProfileNames.clear();
    m_loadedProfileOrder.clear();

    const QString builtinThreeAxisProfileId = QString::fromLatin1(kBuiltinThreeAxisProfileId);
    const QString builtinFourAxisProfileId = QString::fromLatin1(kBuiltinFourAxisProfileId);

    m_loadedProfiles.insert(builtinThreeAxisProfileId, GProfile::createDefaultLaserProfile());
    m_loadedProfileNames.insert(builtinThreeAxisProfileId, QStringLiteral("内置3轴默认"));
    m_loadedProfileOrder.append(builtinThreeAxisProfileId);

    m_loadedProfiles.insert(builtinFourAxisProfileId, GProfile::createDefaultRotaryProfile());
    m_loadedProfileNames.insert(builtinFourAxisProfileId, QStringLiteral("内置4轴默认"));
    m_loadedProfileOrder.append(builtinFourAxisProfileId);

    QSet<QString> loadedFilePaths;

    for (const QString& directoryPath : GProfilePathStore::directories())
    {
        const QDir directory(directoryPath);

        if (!directory.exists())
        {
            continue;
        }

        const QFileInfoList profileFiles = directory.entryInfoList
        (
            QStringList() << QStringLiteral("*.json"),
            QDir::Files | QDir::Readable,
            QDir::Name
        );

        for (const QFileInfo& profileFileInfo : profileFiles)
        {
            const QString normalizedFilePath = QDir::cleanPath(profileFileInfo.absoluteFilePath());
            const QString filePathKey = normalizedFilePath.toCaseFolded();

            if (loadedFilePaths.contains(filePathKey))
            {
                continue;
            }

            loadedFilePaths.insert(filePathKey);
            QString errorMessage;
            const GProfile profile = GProfile::loadFromFile(normalizedFilePath, &errorMessage);

            if (!errorMessage.trimmed().isEmpty())
            {
                continue;
            }

            const QString profileId = GProfilePathStore::profileIdForFile(normalizedFilePath);
            const QString displayName = profile.profileName().trimmed().isEmpty()
                ? profileFileInfo.completeBaseName()
                : profile.profileName().trimmed();

            m_loadedProfiles.insert(profileId, profile);
            m_loadedProfileNames.insert(profileId, displayName);
            m_loadedProfileOrder.append(profileId);
        }
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
    saveSelectedProfileId(m_activeProfileId);
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
    invalidateCurrentProcessPlan();
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
    AppThemeColors theme = mode == AppThemeMode::Custom
        ? loadCustomThemeColors()
        : buildAppThemeColors(mode);
    applyThemeColors(theme, mode);
}

void Gcode_postprocessing_system::applyThemeColors(const AppThemeColors& theme, AppThemeMode mode)
{
    m_themeMode = mode;
    m_themeColors = theme;

    qApp->setStyle(QStyleFactory::create(QStringLiteral("Fusion")));
    qApp->setPalette(theme.palette);
    QFont appFont(QStringLiteral("Microsoft YaHei UI"));
    appFont.setPointSize(9);
    qApp->setFont(appFont);

    setStyleSheet
    (
        QStringLiteral
        (
            "QMainWindow, QDialog, QMessageBox { background-color: %1; color: %2; }"
            "QWidget#centralWidget { background-color: %1; }"
            "QMenuBar { background-color: %3; color: %2; border: none; border-bottom: 1px solid %4; padding: 2px 6px; }"
            "QMenuBar::item { background: transparent; border-radius: 4px; padding: 5px 11px; margin: 1px; }"
            "QMenuBar::item:selected { background: %5; }"
            "QMenuBar::item:pressed { background: %7; }"
            "QMenu { background-color: %6; color: %2; border: 1px solid %4; padding: 5px; }"
            "QMenu::item { border-radius: 3px; padding: 6px 30px 6px 24px; }"
            "QMenu::item:selected { background-color: %5; }"
            "QMenu::item:disabled { color: palette(mid); }"
            "QMenu::separator { height: 1px; background: %4; margin: 5px 8px; }"
            "QMenu::icon { left: 7px; }"
            "QToolBar { background-color: %3; border: none; border-bottom: 1px solid %4; spacing: 0px; padding: 0px; }"
            "QToolTip { background-color: %6; color: %2; border: 1px solid %4; padding: 5px 7px; }"
            "QDialog QLabel { color: %2; }"
            "QDialog QGroupBox { border: 1px solid %4; border-radius: 5px; margin-top: 12px; padding: 12px 8px 8px 8px; font-weight: 600; }"
            "QDialog QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0px 5px; color: %2; }"
            "QDialog QPushButton { min-height: 27px; min-width: 72px; background-color: %6; color: %2; border: 1px solid %4; border-radius: 4px; padding: 2px 12px; }"
            "QDialog QPushButton:hover { background-color: %5; border-color: %8; }"
            "QDialog QPushButton:pressed { background-color: %7; }"
            "QDialog QPushButton:default { background-color: %8; color: %9; border-color: %8; font-weight: 600; }"
            "QDialog QPushButton:disabled { color: palette(mid); background-color: %7; }"
            "QDialog QLineEdit, QDialog QTextEdit, QDialog QPlainTextEdit, QDialog QSpinBox, QDialog QDoubleSpinBox, QDialog QComboBox { min-height: 25px; background-color: %6; color: %2; border: 1px solid %4; border-radius: 4px; padding: 2px 6px; selection-background-color: %8; selection-color: %9; }"
            "QDialog QLineEdit:focus, QDialog QTextEdit:focus, QDialog QPlainTextEdit:focus, QDialog QSpinBox:focus, QDialog QDoubleSpinBox:focus, QDialog QComboBox:focus { border-color: %8; }"
            "QDialog QLabel[imagePreview=\"true\"] { background-color: %7; color: %2; border: 1px solid %4; border-radius: 5px; }"
            "QDialog QTabWidget::pane { border: 1px solid %4; border-radius: 4px; background: %6; top: -1px; }"
            "QDialog QTabBar::tab { background: %7; color: %2; border: 1px solid %4; padding: 7px 14px; margin-right: 2px; }"
            "QDialog QTabBar::tab:selected { background: %6; border-bottom-color: %6; color: %8; }"
            "QAbstractItemView { background-color: %6; alternate-background-color: %7; color: %2; border: 1px solid %4; outline: none; selection-background-color: %8; selection-color: %9; }"
            "QAbstractItemView::item { min-height: 24px; padding: 2px 5px; }"
            "QHeaderView::section { background-color: %7; color: %2; border: none; border-right: 1px solid %4; border-bottom: 1px solid %4; padding: 6px 8px; font-weight: 600; }"
            "QScrollBar:vertical { background: %7; width: 10px; margin: 0px; }"
            "QScrollBar::handle:vertical { background: %4; min-height: 26px; border-radius: 4px; margin: 2px; }"
            "QScrollBar::handle:vertical:hover { background: %8; }"
            "QScrollBar:horizontal { background: %7; height: 10px; margin: 0px; }"
            "QScrollBar::handle:horizontal { background: %4; min-width: 26px; border-radius: 4px; margin: 2px; }"
            "QScrollBar::handle:horizontal:hover { background: %8; }"
            "QScrollBar::add-line, QScrollBar::sub-line { width: 0px; height: 0px; }"
            "QStatusBar { background-color: %3; color: %2; border-top: 1px solid %4; padding: 1px 6px; }"
            "QStatusBar::item { border: none; }"
        )
        .arg(theme.windowBackground.name())
        .arg(theme.textPrimaryColor.name())
        .arg(theme.panelBackground.name())
        .arg(theme.borderColor.name())
        .arg(theme.hoverBackgroundColor.name())
        .arg(theme.surfaceBackground.name())
        .arg(theme.surfaceAltBackground.name())
        .arg(theme.accentColor.name())
        .arg(theme.accentTextColor.name())
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

    QActionGroup* themeActionGroup = m_lightThemeAction != nullptr ? m_lightThemeAction->actionGroup() : nullptr;

    if (themeActionGroup != nullptr)
    {
        themeActionGroup->setExclusive(false);
    }

    if (m_lightThemeAction != nullptr)
    {
        m_lightThemeAction->setChecked(mode == AppThemeMode::Light);
    }

    if (m_darkThemeAction != nullptr)
    {
        m_darkThemeAction->setChecked(mode == AppThemeMode::Dark);
    }

    if (themeActionGroup != nullptr)
    {
        themeActionGroup->setExclusive(true);
    }

    saveThemeMode(mode);
}

AppThemeMode Gcode_postprocessing_system::loadThemeMode() const
{
    QSettings settings(QStringLiteral("GCodePostProcessingSystem"), QStringLiteral("GCodePostProcessingSystem"));
    const QString themeValue = settings.value(QStringLiteral("ui/themeMode"), QStringLiteral("light")).toString().trimmed().toLower();
    return settingValueToThemeMode(themeValue);
}

void Gcode_postprocessing_system::saveThemeMode(AppThemeMode mode) const
{
    QSettings settings(QStringLiteral("GCodePostProcessingSystem"), QStringLiteral("GCodePostProcessingSystem"));
    settings.setValue(QStringLiteral("ui/themeMode"), themeModeToSettingValue(mode));
}

AppThemeColors Gcode_postprocessing_system::loadCustomThemeColors(AppThemeMode* baseMode) const
{
    QSettings settings(QStringLiteral("GCodePostProcessingSystem"), QStringLiteral("GCodePostProcessingSystem"));
    const AppThemeMode storedBaseMode = settingValueToThemeMode
    (
        settings.value(QStringLiteral("ui/customThemeBaseMode"), QStringLiteral("light")).toString()
    ) == AppThemeMode::Dark ? AppThemeMode::Dark : AppThemeMode::Light;

    if (baseMode != nullptr)
    {
        *baseMode = storedBaseMode;
    }

    AppThemeColors theme = buildAppThemeColors(storedBaseMode);
    int settingCount = 0;
    const ThemeColorSetting* colorSettings = themeColorSettings(&settingCount);

    for (int index = 0; index < settingCount; ++index)
    {
        const QString key = QStringLiteral("ui/customTheme/%1").arg(QString::fromLatin1(colorSettings[index].key));
        const QVariant value = settings.value(key);

        if (value.canConvert<QColor>())
        {
            const QColor color = value.value<QColor>();

            if (color.isValid())
            {
                theme.*(colorSettings[index].member) = color;
            }
        }
    }

    theme.dark = storedBaseMode == AppThemeMode::Dark;
    finalizeAppThemePalette(theme);
    return theme;
}

void Gcode_postprocessing_system::saveCustomThemeColors(AppThemeMode baseMode, const AppThemeColors& theme) const
{
    QSettings settings(QStringLiteral("GCodePostProcessingSystem"), QStringLiteral("GCodePostProcessingSystem"));
    const AppThemeMode storedBaseMode = baseMode == AppThemeMode::Dark ? AppThemeMode::Dark : AppThemeMode::Light;
    settings.setValue(QStringLiteral("ui/customThemeBaseMode"), themeModeToSettingValue(storedBaseMode));

    int settingCount = 0;
    const ThemeColorSetting* colorSettings = themeColorSettings(&settingCount);

    for (int index = 0; index < settingCount; ++index)
    {
        const QString key = QStringLiteral("ui/customTheme/%1").arg(QString::fromLatin1(colorSettings[index].key));
        settings.setValue(key, theme.*(colorSettings[index].member));
    }
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

bool Gcode_postprocessing_system::loadProcessVisualsVisible() const
{
    QSettings settings(QStringLiteral("GCodePostProcessingSystem"), QStringLiteral("GCodePostProcessingSystem"));
    return settings.value(QStringLiteral("ui/processVisualsVisible"), true).toBool();
}

void Gcode_postprocessing_system::saveProcessVisualsVisible(bool enabled) const
{
    QSettings settings(QStringLiteral("GCodePostProcessingSystem"), QStringLiteral("GCodePostProcessingSystem"));
    settings.setValue(QStringLiteral("ui/processVisualsVisible"), enabled);
}

bool Gcode_postprocessing_system::loadDisplayOption(const QString& key, bool defaultValue) const
{
    QSettings settings(QStringLiteral("GCodePostProcessingSystem"), QStringLiteral("GCodePostProcessingSystem"));
    return settings.value(QStringLiteral("ui/display/") + key, defaultValue).toBool();
}

void Gcode_postprocessing_system::saveDisplayOption(const QString& key, bool enabled) const
{
    QSettings settings(QStringLiteral("GCodePostProcessingSystem"), QStringLiteral("GCodePostProcessingSystem"));
    settings.setValue(QStringLiteral("ui/display/") + key, enabled);
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

bool Gcode_postprocessing_system::loadAutoDeduplicateOnImport() const
{
    QSettings settings(QStringLiteral("GCodePostProcessingSystem"), QStringLiteral("GCodePostProcessingSystem"));
    const QString newKey = QStringLiteral("dxf/autoDeduplicateOnImport");

    if (settings.contains(newKey))
    {
        return settings.value(newKey, false).toBool();
    }

    const bool migratedValue = settings.value(QStringLiteral("gcode/autoDeduplicateOnExport"), false).toBool();
    settings.setValue(newKey, migratedValue);
    return migratedValue;
}

void Gcode_postprocessing_system::saveAutoDeduplicateOnImport(bool enabled) const
{
    QSettings settings(QStringLiteral("GCodePostProcessingSystem"), QStringLiteral("GCodePostProcessingSystem"));
    settings.setValue(QStringLiteral("dxf/autoDeduplicateOnImport"), enabled);
}

bool Gcode_postprocessing_system::loadAutoRecognizeRotaryTubeSectionOnImport() const
{
    QSettings settings(QStringLiteral("GCodePostProcessingSystem"), QStringLiteral("GCodePostProcessingSystem"));
    return settings.value(QStringLiteral("dxf/autoRecognizeRotaryTubeSectionOnImport"), false).toBool();
}

void Gcode_postprocessing_system::saveAutoRecognizeRotaryTubeSectionOnImport(bool enabled) const
{
    QSettings settings(QStringLiteral("GCodePostProcessingSystem"), QStringLiteral("GCodePostProcessingSystem"));
    settings.setValue(QStringLiteral("dxf/autoRecognizeRotaryTubeSectionOnImport"), enabled);
}

bool Gcode_postprocessing_system::loadAutoRecognizeRotaryEndCutsOnImport() const
{
    QSettings settings(QStringLiteral("GCodePostProcessingSystem"), QStringLiteral("GCodePostProcessingSystem"));
    return settings.value(QStringLiteral("dxf/autoRecognizeRotaryEndCutsOnImport"), false).toBool();
}

void Gcode_postprocessing_system::saveAutoRecognizeRotaryEndCutsOnImport(bool enabled) const
{
    QSettings settings(QStringLiteral("GCodePostProcessingSystem"), QStringLiteral("GCodePostProcessingSystem"));
    settings.setValue(QStringLiteral("dxf/autoRecognizeRotaryEndCutsOnImport"), enabled);
}

bool Gcode_postprocessing_system::loadAutoRemoveInternalPathsOnImport() const
{
    QSettings settings(QStringLiteral("GCodePostProcessingSystem"), QStringLiteral("GCodePostProcessingSystem"));
    return settings.value(QStringLiteral("dxf/autoRemoveInternalPathsOnImport"), false).toBool();
}

void Gcode_postprocessing_system::saveAutoRemoveInternalPathsOnImport(bool enabled) const
{
    QSettings settings(QStringLiteral("GCodePostProcessingSystem"), QStringLiteral("GCodePostProcessingSystem"));
    settings.setValue(QStringLiteral("dxf/autoRemoveInternalPathsOnImport"), enabled);
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
    if (preference == GCodeGenerationPreference::Force3D && !ensureFeatureAvailable(AppFeature::FourAxisExport, QStringLiteral("4轴(绕A) G代码导出")))
    {
        preference = m_generationPreference == GCodeGenerationPreference::Force3D
            ? GCodeGenerationPreference::Auto
            : m_generationPreference;
    }

    m_generationPreference = preference;
    invalidateCurrentProcessPlan();
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


void Gcode_postprocessing_system::initializeMachiningSettingsDock()
{
    m_machiningSettingsDock = new QDockWidget(QStringLiteral("加工设置"), this);
    m_machiningSettingsDock->setObjectName(QStringLiteral("machiningSettingsDock"));
    m_machiningSettingsDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    m_machiningSettingsDock->setFeatures
    (
        QDockWidget::DockWidgetClosable
        | QDockWidget::DockWidgetMovable
        | QDockWidget::DockWidgetFloatable
    );
    m_machiningSettingsDock->setMinimumWidth(260);
    m_machiningSettingsDock->resize(310, height());

    m_machiningSettingsWidget = new MachiningSettingsWidget(m_machiningSettingsDock);
    m_machiningSettingsDock->setWidget(m_machiningSettingsWidget);
    addDockWidget(Qt::RightDockWidgetArea, m_machiningSettingsDock);

    QMenu* viewMenu = menuBar()->addMenu(QStringLiteral("视图"));
    QAction* settingsAction = m_machiningSettingsDock->toggleViewAction();
    settingsAction->setText(QStringLiteral("加工设置"));
    viewMenu->addAction(settingsAction);

    connect(m_machiningSettingsWidget, &MachiningSettingsWidget::autoDeduplicateOnImportChanged, this, [this](bool enabled)
    {
        saveAutoDeduplicateOnImport(enabled);
        QMetaObject::invokeMethod(this, [this]() { syncMachiningSettingsState(); }, Qt::QueuedConnection);
    });
    connect(m_machiningSettingsWidget, &MachiningSettingsWidget::autoRecognizeRotaryTubeSectionOnImportChanged, this, [this](bool enabled)
    {
        saveAutoRecognizeRotaryTubeSectionOnImport(enabled);
        QMetaObject::invokeMethod(this, [this]() { syncMachiningSettingsState(); }, Qt::QueuedConnection);
    });
    connect(m_machiningSettingsWidget, &MachiningSettingsWidget::autoRecognizeRotaryEndCutsOnImportChanged, this, [this](bool enabled)
    {
        saveAutoRecognizeRotaryEndCutsOnImport(enabled);
        QMetaObject::invokeMethod(this, [this]() { syncMachiningSettingsState(); }, Qt::QueuedConnection);
    });
    connect(m_machiningSettingsWidget, &MachiningSettingsWidget::autoRemoveInternalPathsOnImportChanged, this, [this](bool enabled)
    {
        saveAutoRemoveInternalPathsOnImport(enabled);
        QMetaObject::invokeMethod(this, [this]() { syncMachiningSettingsState(); }, Qt::QueuedConnection);
    });
    connect(m_machiningSettingsWidget, &MachiningSettingsWidget::useDefaultExportDirectoryChanged, this, [this](bool enabled)
    {
        saveUseDefaultExportPath(enabled);
        QMetaObject::invokeMethod(this, [this]() { syncMachiningSettingsState(); }, Qt::QueuedConnection);
    });
    connect(m_machiningSettingsWidget, &MachiningSettingsWidget::useDxfFileNameChanged, this, [this](bool enabled)
    {
        saveUseDxfFileNameOnExport(enabled);
        QMetaObject::invokeMethod(this, [this]() { syncMachiningSettingsState(); }, Qt::QueuedConnection);
    });
    connect
    (
        m_machiningSettingsWidget,
        &MachiningSettingsWidget::manualRotaryTubeSectionRequested,
        this,
        [this](double yLength, double zWidth, double cornerRadius)
        {
            const double centerX = m_rotaryTubeSectionModel.valid
                ? m_rotaryTubeSectionModel.centerX : 0.0;
            const cadcam::geometry::Vector2d effectiveCenter =
                m_rotaryTubeSectionModel.effectiveCenter();
            RotaryTubeSectionModel manualModel =
                RotaryTubeGeometryAnalyzer::buildManualSectionModel
                (
                    yLength,
                    zWidth,
                    cornerRadius,
                    centerX,
                    effectiveCenter.x,
                    effectiveCenter.y,
                    m_document.contentRevision()
                );

            if (!manualModel.valid)
            {
                const QString message = manualModel.errorMessage.isEmpty()
                    ? QStringLiteral("手动方管截面参数无效。")
                    : manualModel.errorMessage;
                ui->openGLWidget->appendCommandMessage(message);
                statusBar()->showMessage(message, 5000);
                return;
            }

            manualModel.setAutomaticCenter(m_rotaryTubeSectionModel.automaticCenter);
            manualModel.setUserCenter(m_rotaryTubeSectionModel.userCenter);
            m_rotaryTubeSectionModel = std::move(manualModel);
            invalidateProcessOrdersAfterEndCutChange();
            syncToolPanelState();
            syncMachiningSettingsState();
            const QString message = QStringLiteral
                ("已应用手动方管截面：Y 长 %1 mm，Z 宽 %2 mm，圆角半径 %3 mm。")
                .arg(yLength, 0, 'f', 3)
                .arg(zWidth, 0, 'f', 3)
                .arg(cornerRadius, 0, 'f', 3);
            ui->openGLWidget->appendCommandMessage(message);
            statusBar()->showMessage(message, 5000);
        }
    );

    syncMachiningSettingsState();
}

void Gcode_postprocessing_system::initializeToolPanel()
{
    m_toolPanelWidget = new CadToolPanelWidget(this);
    const bool processVisualsVisible = loadProcessVisualsVisible();
    m_toolPanelWidget->setProcessVisualsVisible(processVisualsVisible);
    ui->openGLWidget->setProcessVisualsVisible(processVisualsVisible);
    const bool processDirectionVisible = loadDisplayOption(QStringLiteral("processDirection"));
    const bool processOrderVisible = loadDisplayOption(QStringLiteral("processOrder"));
    const bool rotaryEndCutsVisible = loadDisplayOption(QStringLiteral("rotaryEndCuts"));
    const bool excludedEntitiesDimmed = loadDisplayOption(QStringLiteral("excludedEntitiesDimmed"));
    const bool backgroundGridVisible = loadDisplayOption(QStringLiteral("backgroundGrid"));
    m_toolPanelWidget->setProcessDirectionVisible(processDirectionVisible);
    m_toolPanelWidget->setProcessOrderVisible(processOrderVisible);
    m_toolPanelWidget->setRotaryEndCutsVisible(rotaryEndCutsVisible);
    m_toolPanelWidget->setExcludedEntitiesDimmed(excludedEntitiesDimmed);
    m_toolPanelWidget->setBackgroundGridVisible(backgroundGridVisible);
    ui->openGLWidget->setProcessDirectionVisible(processDirectionVisible);
    ui->openGLWidget->setProcessOrderVisible(processOrderVisible);
    ui->openGLWidget->setRotaryEndCutsVisible(rotaryEndCutsVisible);
    ui->openGLWidget->setExcludedEntitiesDimmed(excludedEntitiesDimmed);
    ui->openGLWidget->setBackgroundGridVisible(backgroundGridVisible);
    ui->mainToolBar->setMovable(false);
    ui->mainToolBar->setFloatable(false);
    ui->mainToolBar->addWidget(m_toolPanelWidget);

    connect(&m_document, &CadDocument::sceneChanged, this, [this]()
    {
        if (m_currentProcessPlan.has_value()
            && (m_currentProcessPlan->contentRevision != m_document.contentRevision()
                || m_currentProcessPlan->processStateRevision != m_processState.revision()))
        {
            invalidateCurrentProcessPlan();
        }
        syncToolPanelState();
    });
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
    connect(m_toolPanelWidget, &CadToolPanelWidget::recognizeRotaryTubeSectionRequested, this, [this]() { recognizeRotaryTubeSection(true); });
    connect(m_toolPanelWidget, &CadToolPanelWidget::recognizeRotaryEndCutsRequested, this, [this]() { recognizeAllRotaryEndCuts(true); });
    connect(m_toolPanelWidget, &CadToolPanelWidget::removeInternalPathsRequested, this, [this]() { removeInternalMachiningPaths(true); });
    connect(m_toolPanelWidget, &CadToolPanelWidget::machiningSettingsRequested, this, [this]()
    {
        if (m_machiningSettingsDock != nullptr)
        {
            m_machiningSettingsDock->show();
            m_machiningSettingsDock->raise();
        }
    });
    connect
    (
        m_toolPanelWidget,
        &CadToolPanelWidget::processVisualsVisibleChanged,
        this,
        [this](bool enabled)
        {
            ui->openGLWidget->setProcessVisualsVisible(enabled);
            saveProcessVisualsVisible(enabled);
        }
    );
    connect(m_toolPanelWidget, &CadToolPanelWidget::processDirectionVisibleChanged, this, [this](bool enabled)
    {
        ui->openGLWidget->setProcessDirectionVisible(enabled);
        saveDisplayOption(QStringLiteral("processDirection"), enabled);
    });
    connect(m_toolPanelWidget, &CadToolPanelWidget::processOrderVisibleChanged, this, [this](bool enabled)
    {
        ui->openGLWidget->setProcessOrderVisible(enabled);
        saveDisplayOption(QStringLiteral("processOrder"), enabled);
    });
    connect(m_toolPanelWidget, &CadToolPanelWidget::rotaryEndCutsVisibleChanged, this, [this](bool enabled)
    {
        ui->openGLWidget->setRotaryEndCutsVisible(enabled);
        saveDisplayOption(QStringLiteral("rotaryEndCuts"), enabled);
    });
    connect(m_toolPanelWidget, &CadToolPanelWidget::excludedEntitiesDimmedChanged, this, [this](bool enabled)
    {
        ui->openGLWidget->setExcludedEntitiesDimmed(enabled);
        saveDisplayOption(QStringLiteral("excludedEntitiesDimmed"), enabled);
    });
    connect(m_toolPanelWidget, &CadToolPanelWidget::backgroundGridVisibleChanged, this, [this](bool enabled)
    {
        ui->openGLWidget->setBackgroundGridVisible(enabled);
        saveDisplayOption(QStringLiteral("backgroundGrid"), enabled);
    });
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
    connect(m_toolPanelWidget, &CadToolPanelWidget::profileManagerRequested, this, [this]() { openProfileManagerDialog(); });
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

    syncMachiningSettingsState();

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

void Gcode_postprocessing_system::syncMachiningSettingsState()
{
    if (m_machiningSettingsWidget == nullptr)
    {
        return;
    }

    const bool recognizeEndCuts = loadAutoRecognizeRotaryEndCutsOnImport();
    const bool removeInternalPaths = loadAutoRemoveInternalPathsOnImport();
    bool recognizeSection = loadAutoRecognizeRotaryTubeSectionOnImport();

    if (!recognizeSection && (recognizeEndCuts || removeInternalPaths))
    {
        recognizeSection = true;
        saveAutoRecognizeRotaryTubeSectionOnImport(true);
    }

    m_machiningSettingsWidget->setAutomaticOptions
    (
        loadAutoDeduplicateOnImport(),
        recognizeSection,
        recognizeEndCuts,
        removeInternalPaths
    );
    m_machiningSettingsWidget->setExportOptions
    (
        loadUseDefaultExportPath(),
        loadUseDxfFileNameOnExport()
    );
    m_machiningSettingsWidget->setRotaryTubeSectionProperties
    (
        m_rotaryTubeSectionModel.valid,
        m_rotaryTubeSectionModel.yLength,
        m_rotaryTubeSectionModel.zWidth,
        m_rotaryTubeSectionModel.cornerRadius,
        m_rotaryTubeSectionModel.roundedCornerCount,
        m_rotaryTubeSectionModel.manuallyConfigured
    );

    QSet<int> rotaryEndCutIds;
    int internalPathCount = 0;

    for (const std::unique_ptr<CadItem>& entity : m_document.m_entities)
    {
        if (entity == nullptr)
        {
            continue;
        }

        const auto state = m_processState.stateOrDefault(entity->m_entityId);
        if (state.overrideData.boundaryRole == cadcam::planning::BoundaryRole::Break
            && state.overrideData.boundaryPairId >= 0)
        {
            rotaryEndCutIds.insert(state.overrideData.boundaryPairId);
        }

        if (state.effectiveInternalExclusion())
        {
            ++internalPathCount;
        }
    }

    m_machiningSettingsWidget->setRotaryEndCutCount(rotaryEndCutIds.size());
    m_machiningSettingsWidget->setInternalPathCount(internalPathCount);
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
