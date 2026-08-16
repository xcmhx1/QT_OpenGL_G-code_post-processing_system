#pragma once

#include <QWidget>

class QCheckBox;
class QDoubleSpinBox;
class QLabel;
class QPushButton;
class QTimer;

class MachiningSettingsWidget : public QWidget
{
    Q_OBJECT

public:
    explicit MachiningSettingsWidget(QWidget* parent = nullptr);

    void setAutomaticOptions(bool deduplicate, bool recognizeSection, bool recognizeEndCuts, bool removeInternalPaths);
    void setExportOptions(bool useDefaultExportDirectory, bool useDxfFileName);
    void setRotaryTubeSectionProperties
    (
        bool recognized,
        double yLength,
        double zWidth,
        double cornerRadius,
        int roundedCornerCount,
        bool manuallyConfigured,
        double centerX,
        double centerY,
        double centerZ
    );
    void setRotaryEndCutCount(int count);

signals:
    void autoDeduplicateOnImportChanged(bool enabled);
    void autoRecognizeRotaryTubeSectionOnImportChanged(bool enabled);
    void autoRecognizeRotaryEndCutsOnImportChanged(bool enabled);
    void autoRemoveInternalPathsOnImportChanged(bool enabled);
    void useDefaultExportDirectoryChanged(bool enabled);
    void useDxfFileNameChanged(bool enabled);
    void manualRotaryTubeSectionRequested(double yLength, double zWidth, double cornerRadius);

private:
    void updateAutomaticOptionDependencies();
    void refreshSectionStatusStyle();

private:
    QCheckBox* m_autoDeduplicateCheckBox = nullptr;
    QCheckBox* m_autoRecognizeSectionCheckBox = nullptr;
    QCheckBox* m_autoRecognizeEndCutsCheckBox = nullptr;
    QCheckBox* m_autoRemoveInternalPathsCheckBox = nullptr;
    QCheckBox* m_useDefaultExportDirectoryCheckBox = nullptr;
    QCheckBox* m_useDxfFileNameCheckBox = nullptr;
    QLabel* m_sectionStatusValue = nullptr;
    QDoubleSpinBox* m_yLengthInput = nullptr;
    QDoubleSpinBox* m_zWidthInput = nullptr;
    QDoubleSpinBox* m_cornerRadiusInput = nullptr;
    QPushButton* m_applyManualSectionButton = nullptr;
    QLabel* m_roundedCornerCountValue = nullptr;
    QLabel* m_centerLineValue = nullptr;
    QLabel* m_rotaryEndCutCountValue = nullptr;
    QTimer* m_sectionBlinkTimer = nullptr;
    bool m_sectionRecognized = false;
    bool m_sectionBlinkVisible = true;
    bool m_updatingUi = false;
};
