#include "platform/pch.h"

#include "ui/widgets/MachiningSettingsWidget.h"

#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QLabel>
#include <QPushButton>
#include <QScrollArea>
#include <QTimer>
#include <QVBoxLayout>

namespace
{
    QLabel* createStatusValue(QWidget* parent)
    {
        QLabel* label = new QLabel(QStringLiteral("--"), parent);
        label->setTextInteractionFlags(Qt::TextSelectableByMouse);
        return label;
    }

    QDoubleSpinBox* createSectionInput(QWidget* parent)
    {
        QDoubleSpinBox* input = new QDoubleSpinBox(parent);
        input->setDecimals(3);
        input->setRange(0.0, 1000000.0);
        input->setSingleStep(1.0);
        input->setSuffix(QStringLiteral(" mm"));
        input->setSpecialValueText(QStringLiteral("--"));
        return input;
    }
}

MachiningSettingsWidget::MachiningSettingsWidget(QWidget* parent)
    : QWidget(parent)
{
    setMinimumWidth(260);

    QVBoxLayout* rootLayout = new QVBoxLayout(this);
    rootLayout->setContentsMargins(0, 0, 0, 0);

    QScrollArea* scrollArea = new QScrollArea(this);
    scrollArea->setWidgetResizable(true);
    scrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    rootLayout->addWidget(scrollArea);

    QWidget* content = new QWidget(scrollArea);
    QVBoxLayout* contentLayout = new QVBoxLayout(content);
    contentLayout->setContentsMargins(10, 10, 10, 10);
    contentLayout->setSpacing(10);

    QGroupBox* automaticGroup = new QGroupBox(QStringLiteral("自动处理"), content);
    QVBoxLayout* automaticLayout = new QVBoxLayout(automaticGroup);
    m_autoDeduplicateCheckBox = new QCheckBox(QStringLiteral("导入后自动去重"), automaticGroup);
    m_autoDeduplicateCheckBox->setToolTip(QStringLiteral("导入 DXF/DWG 后删除完全重叠的同类型图元。"));
    m_autoRecognizeSectionCheckBox = new QCheckBox(QStringLiteral("导入后自动识别方管截面"), automaticGroup);
    m_autoRecognizeSectionCheckBox->setToolTip(QStringLiteral("从文档中的多个候选截面中选择最可靠的方管垂直截面。"));
    m_autoRecognizeEndCutsCheckBox = new QCheckBox(QStringLiteral("自动识别加工断面"), automaticGroup);
    m_autoRecognizeEndCutsCheckBox->setToolTip(QStringLiteral("方管截面识别成功后，自动搜索并标记全部有效加工断面。"));
    m_autoRemoveInternalPathsCheckBox = new QCheckBox(QStringLiteral("自动清理内部线条"), automaticGroup);
    m_autoRemoveInternalPathsCheckBox->setToolTip(QStringLiteral(
        "方管截面识别成功后，自动删除与内缩窗口相交的内部线图元，删除可撤销。"));
    m_autoRecognizeEndCutsCheckBox->setContentsMargins(18, 0, 0, 0);
    m_autoRemoveInternalPathsCheckBox->setContentsMargins(18, 0, 0, 0);
    automaticLayout->addWidget(m_autoDeduplicateCheckBox);
    automaticLayout->addWidget(m_autoRecognizeSectionCheckBox);
    automaticLayout->addWidget(m_autoRecognizeEndCutsCheckBox);
    automaticLayout->addWidget(m_autoRemoveInternalPathsCheckBox);
    contentLayout->addWidget(automaticGroup);

    QGroupBox* exportGroup = new QGroupBox(QStringLiteral("导出设置"), content);
    QVBoxLayout* exportLayout = new QVBoxLayout(exportGroup);
    m_useDefaultExportDirectoryCheckBox = new QCheckBox(QStringLiteral("使用默认导出目录"), exportGroup);
    m_useDxfFileNameCheckBox = new QCheckBox(QStringLiteral("使用 DXF 文件名"), exportGroup);
    exportLayout->addWidget(m_useDefaultExportDirectoryCheckBox);
    exportLayout->addWidget(m_useDxfFileNameCheckBox);
    contentLayout->addWidget(exportGroup);

    QGroupBox* statusGroup = new QGroupBox(QStringLiteral("方管识别状态"), content);
    QFormLayout* statusLayout = new QFormLayout(statusGroup);
    statusLayout->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
    m_sectionStatusValue = createStatusValue(statusGroup);
    m_yLengthInput = createSectionInput(statusGroup);
    m_zWidthInput = createSectionInput(statusGroup);
    m_cornerRadiusInput = createSectionInput(statusGroup);
    m_cornerRadiusInput->setSingleStep(0.1);
    m_applyManualSectionButton = new QPushButton(QStringLiteral("应用手动截面"), statusGroup);
    m_applyManualSectionButton->setToolTip
        (QStringLiteral("手动设置优先用于后续内部线识别、加工断面、排序和导出。"));
    m_roundedCornerCountValue = createStatusValue(statusGroup);
    m_rotaryEndCutCountValue = createStatusValue(statusGroup);
    statusLayout->addRow(QStringLiteral("截面状态："), m_sectionStatusValue);
    statusLayout->addRow(QStringLiteral("Y 长："), m_yLengthInput);
    statusLayout->addRow(QStringLiteral("Z 宽："), m_zWidthInput);
    statusLayout->addRow(QStringLiteral("圆角半径："), m_cornerRadiusInput);
    statusLayout->addRow(QString(), m_applyManualSectionButton);
    statusLayout->addRow(QStringLiteral("圆角数量："), m_roundedCornerCountValue);
    statusLayout->addRow(QStringLiteral("加工断面："), m_rotaryEndCutCountValue);
    contentLayout->addWidget(statusGroup);
    contentLayout->addStretch(1);
    scrollArea->setWidget(content);

    m_sectionBlinkTimer = new QTimer(this);
    m_sectionBlinkTimer->setInterval(650);
    connect(m_sectionBlinkTimer, &QTimer::timeout, this, [this]()
    {
        if (m_sectionRecognized)
        {
            m_sectionBlinkTimer->stop();
            return;
        }

        m_sectionBlinkVisible = !m_sectionBlinkVisible;
        refreshSectionStatusStyle();
    });
    m_sectionBlinkTimer->start();

    connect(m_autoDeduplicateCheckBox, &QCheckBox::toggled, this, [this](bool enabled)
    {
        if (!m_updatingUi) emit autoDeduplicateOnImportChanged(enabled);
    });
    connect(m_autoRecognizeSectionCheckBox, &QCheckBox::toggled, this, [this](bool enabled)
    {
        updateAutomaticOptionDependencies();

        if (!m_updatingUi) emit autoRecognizeRotaryTubeSectionOnImportChanged(enabled);
    });
    connect(m_autoRecognizeEndCutsCheckBox, &QCheckBox::toggled, this, [this](bool enabled)
    {
        if (enabled && !m_autoRecognizeSectionCheckBox->isChecked())
        {
            m_autoRecognizeSectionCheckBox->setChecked(true);
        }

        if (!m_updatingUi) emit autoRecognizeRotaryEndCutsOnImportChanged(enabled);
    });
    connect(m_autoRemoveInternalPathsCheckBox, &QCheckBox::toggled, this, [this](bool enabled)
    {
        if (enabled && !m_autoRecognizeSectionCheckBox->isChecked())
        {
            m_autoRecognizeSectionCheckBox->setChecked(true);
        }

        if (!m_updatingUi) emit autoRemoveInternalPathsOnImportChanged(enabled);
    });
    connect(m_useDefaultExportDirectoryCheckBox, &QCheckBox::toggled, this, [this](bool enabled)
    {
        if (!m_updatingUi) emit useDefaultExportDirectoryChanged(enabled);
    });
    connect(m_useDxfFileNameCheckBox, &QCheckBox::toggled, this, [this](bool enabled)
    {
        if (!m_updatingUi) emit useDxfFileNameChanged(enabled);
    });
    connect(m_applyManualSectionButton, &QPushButton::clicked, this, [this]()
    {
        emit manualRotaryTubeSectionRequested
        (
            m_yLengthInput->value(),
            m_zWidthInput->value(),
            m_cornerRadiusInput->value()
        );
    });
}

void MachiningSettingsWidget::setAutomaticOptions
(
    bool deduplicate,
    bool recognizeSection,
    bool recognizeEndCuts,
    bool removeInternalPaths
)
{
    m_updatingUi = true;
    recognizeSection = recognizeSection || recognizeEndCuts || removeInternalPaths;
    m_autoDeduplicateCheckBox->setChecked(deduplicate);
    m_autoRecognizeSectionCheckBox->setChecked(recognizeSection);
    m_autoRecognizeEndCutsCheckBox->setChecked(recognizeEndCuts);
    m_autoRemoveInternalPathsCheckBox->setChecked(removeInternalPaths);
    updateAutomaticOptionDependencies();
    m_updatingUi = false;
}

void MachiningSettingsWidget::setExportOptions(bool useDefaultExportDirectory, bool useDxfFileName)
{
    m_updatingUi = true;
    m_useDefaultExportDirectoryCheckBox->setChecked(useDefaultExportDirectory);
    m_useDxfFileNameCheckBox->setChecked(useDxfFileName);
    m_updatingUi = false;
}

void MachiningSettingsWidget::setRotaryTubeSectionProperties
(
    bool recognized,
    double yLength,
    double zWidth,
    double cornerRadius,
    int roundedCornerCount,
    bool manuallyConfigured
)
{
    m_updatingUi = true;
    m_sectionRecognized = recognized;
    m_sectionBlinkVisible = true;
    m_sectionStatusValue->setText(recognized
        ? (manuallyConfigured ? QStringLiteral("已手动设置") : QStringLiteral("已识别"))
        : QStringLiteral("未识别"));

    if (recognized)
    {
        m_sectionBlinkTimer->stop();
        m_yLengthInput->setValue(yLength);
        m_zWidthInput->setValue(zWidth);
        m_cornerRadiusInput->setValue(cornerRadius);
        m_roundedCornerCountValue->setText(QString::number(std::clamp(roundedCornerCount, 0, 4)));
    }
    else
    {
        m_yLengthInput->setValue(0.0);
        m_zWidthInput->setValue(0.0);
        m_cornerRadiusInput->setValue(0.0);
        m_roundedCornerCountValue->setText(QStringLiteral("--"));

        if (!m_sectionBlinkTimer->isActive())
        {
            m_sectionBlinkTimer->start();
        }
    }

    m_updatingUi = false;
    refreshSectionStatusStyle();
}

void MachiningSettingsWidget::setRotaryEndCutCount(int count)
{
    m_rotaryEndCutCountValue->setText(count > 0
        ? QStringLiteral("已识别 %1 个").arg(count)
        : QStringLiteral("未识别"));
}

void MachiningSettingsWidget::updateAutomaticOptionDependencies()
{
    const bool sectionEnabled = m_autoRecognizeSectionCheckBox->isChecked();

    if (!sectionEnabled)
    {
        m_autoRecognizeEndCutsCheckBox->setChecked(false);
        m_autoRemoveInternalPathsCheckBox->setChecked(false);
    }

    m_autoRecognizeEndCutsCheckBox->setEnabled(sectionEnabled);
    m_autoRemoveInternalPathsCheckBox->setEnabled(sectionEnabled);
}

void MachiningSettingsWidget::refreshSectionStatusStyle()
{
    if (m_sectionRecognized)
    {
        m_sectionStatusValue->setStyleSheet(QString());
        return;
    }

    m_sectionStatusValue->setStyleSheet(m_sectionBlinkVisible
        ? QStringLiteral("color: #d32f2f; font-weight: 600;")
        : QStringLiteral("color: rgba(211, 47, 47, 90); font-weight: 600;"));
}
