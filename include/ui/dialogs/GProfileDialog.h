#pragma once

#include "desktop/AppTheme.h"
#include "infrastructure/config/GProfile.h"

#include <QDialog>
#include <QColor>
#include <QMap>
#include <QStringList>

class QComboBox;
class QCheckBox;
class QDialogButtonBox;
class QDoubleSpinBox;
class QLineEdit;
class QListWidget;
class QListWidgetItem;
class QPlainTextEdit;
class QTabWidget;

class GProfileDialog : public QDialog
{
public:
    explicit GProfileDialog
    (
        const GProfile& profile,
        const QStringList& availableLayerNames,
        const QMap<QString, QColor>& availableLayerColors,
        const AppThemeColors& theme,
        QWidget* parent = nullptr
    );

    GProfile profile() const;
    QString importedProfilePath() const;

protected:
    void accept() override;

private:
    void buildUi();
    void applyTheme();
    void applyProfile(const GProfile& profile);
    GProfile collectProfile() const;

    void refreshEntityTypeList(const QString& preferredEntityTypeKey = QString());
    void loadSelectedEntityTypeBlock();
    void updateCurrentEntityTypeBlock();

    void refreshLayerRuleList(const QString& preferredLayerKey = QString());
    void loadSelectedLayerRule();
    void updateCurrentLayerRule();
    void addLayerRule();
    void removeSelectedLayerRule();

    void refreshColorRuleList(const QString& preferredColorKey = QString());
    void loadSelectedColorRule();
    void updateCurrentColorRule();
    void addCustomColorRule();
    void removeSelectedColorRule();

    void importProfileFromFile();
    void exportProfileToFile();
    void resetToDefaultLaserProfile();
    void resetToDefaultRotaryProfile();

    QString currentEntityTypeKey() const;
    QString currentLayerKey() const;
    QString currentColorKey() const;
    static QStringList supportedEntityTypes();
    static QStringList supportedDefaultColorKeys();
    static QString displayNameForEntityType(const QString& entityTypeKey);
    static QString displayNameForColorKey(const QString& colorKey);
    static bool hasMeaningfulContent(const GProfileCodeBlock& codeBlock);

private:
    bool m_updatingUi = false;
    GProfile m_profile;
    QStringList m_availableLayerNames;
    QMap<QString, QColor> m_availableLayerColors;
    AppThemeColors m_theme = buildAppThemeColors(AppThemeMode::Light);
    QMap<QString, GProfileCodeBlock> m_entityTypeBlocks;
    QMap<QString, GProfileCodeBlock> m_layerBlocks;
    QMap<QString, GProfileCodeBlock> m_colorBlocks;
    QString m_importedProfilePath;

    QLineEdit* m_profileNameEdit = nullptr;
    QPlainTextEdit* m_fileHeaderEdit = nullptr;
    QPlainTextEdit* m_fileFooterEdit = nullptr;
    QPlainTextEdit* m_fileCommentEdit = nullptr;
    QDoubleSpinBox* m_rotationSafetyClearanceSpinBox = nullptr;
    QDoubleSpinBox* m_sameZoneTransferClearanceSpinBox = nullptr;
    QCheckBox* m_coordinatedTransferCheckBox = nullptr;
    QDoubleSpinBox* m_rotaryPlaneZOffsetSpinBox = nullptr;
    QDoubleSpinBox* m_rotaryOvercutDistanceSpinBox = nullptr;
    QCheckBox* m_lazyRotationProcessingCheckBox = nullptr;
    QComboBox* m_perimeterSweepDirectionComboBox = nullptr;
    QComboBox* m_longitudinalSweepDirectionComboBox = nullptr;

    QListWidget* m_entityTypeListWidget = nullptr;
    QPlainTextEdit* m_entityTypeHeaderEdit = nullptr;
    QPlainTextEdit* m_entityTypeFooterEdit = nullptr;
    QPlainTextEdit* m_entityTypeCommentEdit = nullptr;

    QListWidget* m_layerRuleListWidget = nullptr;
    QLineEdit* m_layerKeyEdit = nullptr;
    QPlainTextEdit* m_layerHeaderEdit = nullptr;
    QPlainTextEdit* m_layerFooterEdit = nullptr;
    QPlainTextEdit* m_layerCommentEdit = nullptr;

    QListWidget* m_colorRuleListWidget = nullptr;
    QLineEdit* m_colorKeyEdit = nullptr;
    QPlainTextEdit* m_colorHeaderEdit = nullptr;
    QPlainTextEdit* m_colorFooterEdit = nullptr;
    QPlainTextEdit* m_colorCommentEdit = nullptr;
    QDialogButtonBox* m_buttonBox = nullptr;
};
