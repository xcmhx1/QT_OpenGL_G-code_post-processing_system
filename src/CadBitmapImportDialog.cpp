// 实现 CadBitmapImportDialog 模块，对应头文件中声明的主要行为和协作流程。
// 位图导入对话框模块，负责导入参数配置、预处理预览和导入方式选择。
#include "pch.h"

#include "CadBitmapImportDialog.h"

#include <QCheckBox>
#include <QColorDialog>
#include <QColor>
#include <QDialogButtonBox>
#include <QFileInfo>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QResizeEvent>
#include <QSpinBox>
#include <QSplitter>
#include <QVBoxLayout>
#include <QComboBox>
#include <QDoubleSpinBox>

CadBitmapImportDialog::CadBitmapImportDialog(const QString& filePath, QWidget* parent)
    : QDialog(parent)
    , m_filePath(filePath)
{
    setWindowTitle(QStringLiteral("位图导入: %1").arg(QFileInfo(filePath).fileName()));
    setMinimumSize(1180, 760);
    resize(1280, 820);

    buildUi();
    connectSignals();

    if (!CadBitmapVectorizer::loadBitmapImage(filePath, m_sourceImage, &m_errorMessage))
    {
        m_summaryLabel->setText(m_errorMessage);
        updatePreviewLabel(m_sourcePreviewLabel, QImage(), QStringLiteral("原图加载失败"));
        updatePreviewLabel(m_processedPreviewLabel, QImage(), QStringLiteral("无法生成预览"));
        m_buttonBox->button(QDialogButtonBox::Ok)->setEnabled(false);
        return;
    }

    refreshPreview();
}

bool CadBitmapImportDialog::isReady() const
{
    return !m_sourceImage.empty();
}

const QString& CadBitmapImportDialog::errorMessage() const
{
    return m_errorMessage;
}

const cv::Mat& CadBitmapImportDialog::sourceImage() const
{
    return m_sourceImage;
}

CadBitmapImportOptions CadBitmapImportDialog::options() const
{
    return collectOptions();
}

void CadBitmapImportDialog::resizeEvent(QResizeEvent* event)
{
    QDialog::resizeEvent(event);
    updatePreviewLabel(m_sourcePreviewLabel, m_sourcePreviewImage, QStringLiteral("原图预览"));
    updatePreviewLabel(m_processedPreviewLabel, m_processedPreviewImage, QStringLiteral("处理后预览"));
}

void CadBitmapImportDialog::buildUi()
{
    QVBoxLayout* rootLayout = new QVBoxLayout(this);
    rootLayout->setContentsMargins(10, 10, 10, 10);
    rootLayout->setSpacing(8);

    QLabel* tipLabel = new QLabel(QStringLiteral("导入流程: 位图预处理 -> 轮廓提取 -> CAD 图元拟合 -> 写入文档"), this);
    rootLayout->addWidget(tipLabel);

    QSplitter* splitter = new QSplitter(Qt::Horizontal, this);
    rootLayout->addWidget(splitter, 1);

    QWidget* previewPanel = new QWidget(splitter);
    QVBoxLayout* previewLayout = new QVBoxLayout(previewPanel);
    previewLayout->setContentsMargins(0, 0, 0, 0);
    previewLayout->setSpacing(8);

    QLabel* previewTitle = new QLabel(QStringLiteral("预处理预览"), previewPanel);
    previewLayout->addWidget(previewTitle);

    QHBoxLayout* previewImagesLayout = new QHBoxLayout();
    previewImagesLayout->setSpacing(8);

    m_sourcePreviewLabel = new QLabel(previewPanel);
    m_sourcePreviewLabel->setMinimumSize(480, 320);
    m_sourcePreviewLabel->setAlignment(Qt::AlignCenter);
    m_sourcePreviewLabel->setStyleSheet("background-color: rgb(24, 27, 31); border: 1px solid rgb(70, 75, 82);");

    m_processedPreviewLabel = new QLabel(previewPanel);
    m_processedPreviewLabel->setMinimumSize(480, 320);
    m_processedPreviewLabel->setAlignment(Qt::AlignCenter);
    m_processedPreviewLabel->setStyleSheet("background-color: rgb(24, 27, 31); border: 1px solid rgb(70, 75, 82);");

    previewImagesLayout->addWidget(m_sourcePreviewLabel, 1);
    previewImagesLayout->addWidget(m_processedPreviewLabel, 1);
    previewLayout->addLayout(previewImagesLayout, 1);

    m_summaryLabel = new QLabel(QStringLiteral("请配置位图预处理参数。"), previewPanel);
    m_summaryLabel->setWordWrap(true);
    previewLayout->addWidget(m_summaryLabel);

    QWidget* settingsPanel = new QWidget(splitter);
    QVBoxLayout* settingsPanelLayout = new QVBoxLayout(settingsPanel);
    settingsPanelLayout->setContentsMargins(0, 0, 0, 0);
    settingsPanelLayout->setSpacing(10);

    QWidget* settingsContainer = new QWidget(settingsPanel);
    QVBoxLayout* settingsLayout = new QVBoxLayout(settingsContainer);
    settingsLayout->setContentsMargins(0, 0, 0, 0);
    settingsLayout->setSpacing(10);
    settingsPanelLayout->addWidget(settingsContainer);

    QGroupBox* importGroup = new QGroupBox(QStringLiteral("导入方式"), settingsContainer);
    QFormLayout* importLayout = new QFormLayout(importGroup);
    m_importModeCombo = new QComboBox(importGroup);
    m_importModeCombo->addItem(QStringLiteral("替换当前文档"), static_cast<int>(CadBitmapImportMode::ReplaceDocument));
    m_importModeCombo->addItem(QStringLiteral("追加到当前文档"), static_cast<int>(CadBitmapImportMode::AppendToDocument));
    importLayout->addRow(QStringLiteral("写入策略"), m_importModeCombo);

    m_autoFitSceneCheckBox = new QCheckBox(QStringLiteral("导入完成后自动适配视图"), importGroup);
    m_autoFitSceneCheckBox->setChecked(true);
    importLayout->addRow(QStringLiteral("视图"), m_autoFitSceneCheckBox);
    settingsLayout->addWidget(importGroup);

    QGroupBox* placementGroup = new QGroupBox(QStringLiteral("放置参数"), settingsContainer);
    QFormLayout* placementLayout = new QFormLayout(placementGroup);

    m_insertXSpinBox = new QDoubleSpinBox(placementGroup);
    m_insertXSpinBox->setRange(-1000000.0, 1000000.0);
    m_insertXSpinBox->setDecimals(3);
    m_insertXSpinBox->setSingleStep(1.0);
    placementLayout->addRow(QStringLiteral("插入点 X"), m_insertXSpinBox);

    m_insertYSpinBox = new QDoubleSpinBox(placementGroup);
    m_insertYSpinBox->setRange(-1000000.0, 1000000.0);
    m_insertYSpinBox->setDecimals(3);
    m_insertYSpinBox->setSingleStep(1.0);
    placementLayout->addRow(QStringLiteral("插入点 Y"), m_insertYSpinBox);

    m_layerLineEdit = new QLineEdit(QStringLiteral("BITMAP_IMPORT"), placementGroup);
    placementLayout->addRow(QStringLiteral("目标图层"), m_layerLineEdit);

    m_colorButton = new QPushButton(QStringLiteral("选择颜色"), placementGroup);
    placementLayout->addRow(QStringLiteral("图元颜色"), m_colorButton);
    settingsLayout->addWidget(placementGroup);

    QGroupBox* preprocessGroup = new QGroupBox(QStringLiteral("预处理"), settingsContainer);
    QFormLayout* preprocessLayout = new QFormLayout(preprocessGroup);
    m_operatorCombo = new QComboBox(preprocessGroup);
    m_operatorCombo->addItem(QStringLiteral("Otsu 二值"), static_cast<int>(CadBitmapPreprocessOperator::OtsuThreshold));
    m_operatorCombo->addItem(QStringLiteral("固定阈值"), static_cast<int>(CadBitmapPreprocessOperator::FixedThreshold));
    m_operatorCombo->addItem(QStringLiteral("自适应阈值"), static_cast<int>(CadBitmapPreprocessOperator::AdaptiveThreshold));
    m_operatorCombo->addItem(QStringLiteral("Canny 边缘"), static_cast<int>(CadBitmapPreprocessOperator::CannyEdges));
    preprocessLayout->addRow(QStringLiteral("算子选择"), m_operatorCombo);

    m_blurCheckBox = new QCheckBox(QStringLiteral("启用高斯模糊"), preprocessGroup);
    m_blurCheckBox->setChecked(true);
    preprocessLayout->addRow(QStringLiteral("平滑"), m_blurCheckBox);

    m_blurKernelSpinBox = new QSpinBox(preprocessGroup);
    m_blurKernelSpinBox->setRange(1, 31);
    m_blurKernelSpinBox->setSingleStep(2);
    m_blurKernelSpinBox->setValue(5);
    preprocessLayout->addRow(QStringLiteral("模糊核"), m_blurKernelSpinBox);

    m_invertCheckBox = new QCheckBox(QStringLiteral("反相前景"), preprocessGroup);
    m_invertCheckBox->setChecked(true);
    preprocessLayout->addRow(QStringLiteral("极性"), m_invertCheckBox);

    m_thresholdSpinBox = new QSpinBox(preprocessGroup);
    m_thresholdSpinBox->setRange(0, 255);
    m_thresholdSpinBox->setValue(140);
    preprocessLayout->addRow(QStringLiteral("固定阈值"), m_thresholdSpinBox);

    m_adaptiveBlockSizeSpinBox = new QSpinBox(preprocessGroup);
    m_adaptiveBlockSizeSpinBox->setRange(3, 101);
    m_adaptiveBlockSizeSpinBox->setSingleStep(2);
    m_adaptiveBlockSizeSpinBox->setValue(31);
    preprocessLayout->addRow(QStringLiteral("自适应块"), m_adaptiveBlockSizeSpinBox);

    m_adaptiveCSpinBox = new QDoubleSpinBox(preprocessGroup);
    m_adaptiveCSpinBox->setRange(-50.0, 50.0);
    m_adaptiveCSpinBox->setDecimals(1);
    m_adaptiveCSpinBox->setSingleStep(0.5);
    m_adaptiveCSpinBox->setValue(5.0);
    preprocessLayout->addRow(QStringLiteral("自适应 C"), m_adaptiveCSpinBox);

    m_cannyLowSpinBox = new QSpinBox(preprocessGroup);
    m_cannyLowSpinBox->setRange(0, 500);
    m_cannyLowSpinBox->setValue(50);
    preprocessLayout->addRow(QStringLiteral("Canny 低阈"), m_cannyLowSpinBox);

    m_cannyHighSpinBox = new QSpinBox(preprocessGroup);
    m_cannyHighSpinBox->setRange(0, 500);
    m_cannyHighSpinBox->setValue(150);
    preprocessLayout->addRow(QStringLiteral("Canny 高阈"), m_cannyHighSpinBox);

    m_morphologyCombo = new QComboBox(preprocessGroup);
    m_morphologyCombo->addItem(QStringLiteral("不处理"), static_cast<int>(CadBitmapMorphologyOperator::None));
    m_morphologyCombo->addItem(QStringLiteral("开运算"), static_cast<int>(CadBitmapMorphologyOperator::Open));
    m_morphologyCombo->addItem(QStringLiteral("闭运算"), static_cast<int>(CadBitmapMorphologyOperator::Close));
    m_morphologyCombo->addItem(QStringLiteral("膨胀"), static_cast<int>(CadBitmapMorphologyOperator::Dilate));
    m_morphologyCombo->addItem(QStringLiteral("腐蚀"), static_cast<int>(CadBitmapMorphologyOperator::Erode));
    m_morphologyCombo->setCurrentIndex(2);
    preprocessLayout->addRow(QStringLiteral("形态学"), m_morphologyCombo);

    m_morphologyKernelSpinBox = new QSpinBox(preprocessGroup);
    m_morphologyKernelSpinBox->setRange(1, 31);
    m_morphologyKernelSpinBox->setSingleStep(2);
    m_morphologyKernelSpinBox->setValue(3);
    preprocessLayout->addRow(QStringLiteral("形态学核"), m_morphologyKernelSpinBox);
    settingsLayout->addWidget(preprocessGroup);

    QGroupBox* fittingGroup = new QGroupBox(QStringLiteral("图元拟合"), settingsContainer);
    QFormLayout* fittingLayout = new QFormLayout(fittingGroup);
    m_contourModeCombo = new QComboBox(fittingGroup);
    m_contourModeCombo->addItem(QStringLiteral("仅外轮廓"), static_cast<int>(CadBitmapContourMode::ExternalOnly));
    m_contourModeCombo->addItem(QStringLiteral("全部轮廓"), static_cast<int>(CadBitmapContourMode::AllContours));
    fittingLayout->addRow(QStringLiteral("轮廓模式"), m_contourModeCombo);

    m_fitStrategyCombo = new QComboBox(fittingGroup);
    m_fitStrategyCombo->addItem(QStringLiteral("优先规则图元"), static_cast<int>(CadBitmapFitStrategy::PreferPrimitives));
    m_fitStrategyCombo->addItem(QStringLiteral("全部折线"), static_cast<int>(CadBitmapFitStrategy::PolylineOnly));
    fittingLayout->addRow(QStringLiteral("拟合策略"), m_fitStrategyCombo);

    m_scaleSpinBox = new QDoubleSpinBox(fittingGroup);
    m_scaleSpinBox->setRange(0.001, 10000.0);
    m_scaleSpinBox->setDecimals(4);
    m_scaleSpinBox->setSingleStep(0.1);
    m_scaleSpinBox->setValue(1.0);
    fittingLayout->addRow(QStringLiteral("像素比例"), m_scaleSpinBox);

    m_approxEpsilonSpinBox = new QDoubleSpinBox(fittingGroup);
    m_approxEpsilonSpinBox->setRange(0.1, 100.0);
    m_approxEpsilonSpinBox->setDecimals(2);
    m_approxEpsilonSpinBox->setSingleStep(0.2);
    m_approxEpsilonSpinBox->setValue(2.5);
    fittingLayout->addRow(QStringLiteral("折线容差"), m_approxEpsilonSpinBox);

    m_minContourAreaSpinBox = new QDoubleSpinBox(fittingGroup);
    m_minContourAreaSpinBox->setRange(0.0, 100000.0);
    m_minContourAreaSpinBox->setDecimals(1);
    m_minContourAreaSpinBox->setSingleStep(5.0);
    m_minContourAreaSpinBox->setValue(24.0);
    fittingLayout->addRow(QStringLiteral("最小面积"), m_minContourAreaSpinBox);

    m_minLineLengthSpinBox = new QDoubleSpinBox(fittingGroup);
    m_minLineLengthSpinBox->setRange(0.0, 10000.0);
    m_minLineLengthSpinBox->setDecimals(2);
    m_minLineLengthSpinBox->setSingleStep(0.5);
    m_minLineLengthSpinBox->setValue(4.0);
    fittingLayout->addRow(QStringLiteral("最小线长"), m_minLineLengthSpinBox);

    m_lineFitToleranceSpinBox = new QDoubleSpinBox(fittingGroup);
    m_lineFitToleranceSpinBox->setRange(0.1, 100.0);
    m_lineFitToleranceSpinBox->setDecimals(2);
    m_lineFitToleranceSpinBox->setSingleStep(0.1);
    m_lineFitToleranceSpinBox->setValue(1.6);
    fittingLayout->addRow(QStringLiteral("线段容差"), m_lineFitToleranceSpinBox);

    m_arcFitToleranceSpinBox = new QDoubleSpinBox(fittingGroup);
    m_arcFitToleranceSpinBox->setRange(0.1, 100.0);
    m_arcFitToleranceSpinBox->setDecimals(2);
    m_arcFitToleranceSpinBox->setSingleStep(0.1);
    m_arcFitToleranceSpinBox->setValue(1.4);
    fittingLayout->addRow(QStringLiteral("圆弧容差"), m_arcFitToleranceSpinBox);

    m_minArcAngleSpinBox = new QDoubleSpinBox(fittingGroup);
    m_minArcAngleSpinBox->setRange(1.0, 180.0);
    m_minArcAngleSpinBox->setDecimals(1);
    m_minArcAngleSpinBox->setSingleStep(1.0);
    m_minArcAngleSpinBox->setValue(18.0);
    fittingLayout->addRow(QStringLiteral("最小圆弧角"), m_minArcAngleSpinBox);

    m_maxEntityCountSpinBox = new QSpinBox(fittingGroup);
    m_maxEntityCountSpinBox->setRange(1, 200000);
    m_maxEntityCountSpinBox->setValue(5000);
    fittingLayout->addRow(QStringLiteral("最大实体数"), m_maxEntityCountSpinBox);
    settingsLayout->addWidget(fittingGroup);

    settingsLayout->addStretch(1);

    splitter->addWidget(previewPanel);
    splitter->addWidget(settingsPanel);
    splitter->setStretchFactor(0, 3);
    splitter->setStretchFactor(1, 2);

    m_buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    m_buttonBox->button(QDialogButtonBox::Ok)->setText(QStringLiteral("导入"));
    m_buttonBox->button(QDialogButtonBox::Cancel)->setText(QStringLiteral("取消"));
    connect(m_buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(m_buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
    rootLayout->addWidget(m_buttonBox);

    updateColorButton();
}

void CadBitmapImportDialog::connectSignals()
{
    const auto refreshLambda = [this]()
    {
        refreshPreview();
    };

    connect(m_importModeCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, refreshLambda);
    connect(m_operatorCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, refreshLambda);
    connect(m_morphologyCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, refreshLambda);
    connect(m_contourModeCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, refreshLambda);
    connect(m_fitStrategyCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, refreshLambda);
    connect(m_invertCheckBox, &QCheckBox::checkStateChanged, this, refreshLambda);
    connect(m_blurCheckBox, &QCheckBox::checkStateChanged, this, refreshLambda);
    connect(m_blurKernelSpinBox, qOverload<int>(&QSpinBox::valueChanged), this, refreshLambda);
    connect(m_thresholdSpinBox, qOverload<int>(&QSpinBox::valueChanged), this, refreshLambda);
    connect(m_adaptiveBlockSizeSpinBox, qOverload<int>(&QSpinBox::valueChanged), this, refreshLambda);
    connect(m_adaptiveCSpinBox, qOverload<double>(&QDoubleSpinBox::valueChanged), this, refreshLambda);
    connect(m_cannyLowSpinBox, qOverload<int>(&QSpinBox::valueChanged), this, refreshLambda);
    connect(m_cannyHighSpinBox, qOverload<int>(&QSpinBox::valueChanged), this, refreshLambda);
    connect(m_morphologyKernelSpinBox, qOverload<int>(&QSpinBox::valueChanged), this, refreshLambda);
    connect(m_scaleSpinBox, qOverload<double>(&QDoubleSpinBox::valueChanged), this, refreshLambda);
    connect(m_insertXSpinBox, qOverload<double>(&QDoubleSpinBox::valueChanged), this, refreshLambda);
    connect(m_insertYSpinBox, qOverload<double>(&QDoubleSpinBox::valueChanged), this, refreshLambda);
    connect(m_approxEpsilonSpinBox, qOverload<double>(&QDoubleSpinBox::valueChanged), this, refreshLambda);
    connect(m_minContourAreaSpinBox, qOverload<double>(&QDoubleSpinBox::valueChanged), this, refreshLambda);
    connect(m_minLineLengthSpinBox, qOverload<double>(&QDoubleSpinBox::valueChanged), this, refreshLambda);
    connect(m_lineFitToleranceSpinBox, qOverload<double>(&QDoubleSpinBox::valueChanged), this, refreshLambda);
    connect(m_arcFitToleranceSpinBox, qOverload<double>(&QDoubleSpinBox::valueChanged), this, refreshLambda);
    connect(m_minArcAngleSpinBox, qOverload<double>(&QDoubleSpinBox::valueChanged), this, refreshLambda);
    connect(m_maxEntityCountSpinBox, qOverload<int>(&QSpinBox::valueChanged), this, refreshLambda);
    connect(m_autoFitSceneCheckBox, &QCheckBox::checkStateChanged, this, refreshLambda);
    connect(m_layerLineEdit, &QLineEdit::textChanged, this, refreshLambda);
    connect
    (
        m_colorButton,
        &QPushButton::clicked,
        this,
        [this]()
        {
            const QColor selectedColor = QColorDialog::getColor(m_selectedColor, this, QStringLiteral("选择导入图元颜色"));

            if (!selectedColor.isValid())
            {
                return;
            }

            m_selectedColor = selectedColor;
            updateColorButton();
            refreshPreview();
        }
    );
}

void CadBitmapImportDialog::refreshPreview()
{
    if (m_sourceImage.empty())
    {
        return;
    }

    QString errorMessage;
    const CadBitmapPreviewData previewData = CadBitmapVectorizer::buildPreview(m_sourceImage, collectOptions(), &errorMessage);
    m_errorMessage = errorMessage;
    m_sourcePreviewImage = previewData.sourcePreview;
    m_processedPreviewImage = previewData.processedPreview;

    updatePreviewLabel(m_sourcePreviewLabel, m_sourcePreviewImage, QStringLiteral("原图预览"));
    updatePreviewLabel(m_processedPreviewLabel, m_processedPreviewImage, QStringLiteral("处理后预览"));

    if (!errorMessage.isEmpty())
    {
        m_summaryLabel->setText(errorMessage);
        return;
    }

    m_summaryLabel->setText(previewData.summaryText);
}

void CadBitmapImportDialog::updatePreviewLabel(QLabel* label, const QImage& image, const QString& fallbackText)
{
    if (label == nullptr)
    {
        return;
    }

    if (image.isNull())
    {
        label->setPixmap(QPixmap());
        label->setText(fallbackText);
        return;
    }

    const QSize targetSize = label->size() - QSize(12, 12);
    const QPixmap pixmap = QPixmap::fromImage(image).scaled(targetSize, Qt::KeepAspectRatio, Qt::SmoothTransformation);
    label->setText(QString());
    label->setPixmap(pixmap);
}

void CadBitmapImportDialog::updateColorButton()
{
    if (m_colorButton == nullptr)
    {
        return;
    }

    m_colorButton->setText(m_selectedColor.name(QColor::HexRgb).toUpper());
    m_colorButton->setStyleSheet
    (
        QStringLiteral
        (
            "QPushButton { background-color: %1; color: %2; border: 1px solid rgb(92, 98, 108); padding: 4px 8px; }"
        )
        .arg(m_selectedColor.name(QColor::HexRgb))
        .arg(m_selectedColor.lightness() >= 128 ? QStringLiteral("#111111") : QStringLiteral("#F4F6F8"))
    );
}

CadBitmapImportOptions CadBitmapImportDialog::collectOptions() const
{
    CadBitmapImportOptions importOptions;
    importOptions.filePath = m_filePath;
    importOptions.importMode = static_cast<CadBitmapImportMode>(m_importModeCombo->currentData().toInt());
    importOptions.preprocessOperator = static_cast<CadBitmapPreprocessOperator>(m_operatorCombo->currentData().toInt());
    importOptions.morphologyOperator = static_cast<CadBitmapMorphologyOperator>(m_morphologyCombo->currentData().toInt());
    importOptions.contourMode = static_cast<CadBitmapContourMode>(m_contourModeCombo->currentData().toInt());
    importOptions.fitStrategy = static_cast<CadBitmapFitStrategy>(m_fitStrategyCombo->currentData().toInt());
    importOptions.invert = m_invertCheckBox->isChecked();
    importOptions.blurEnabled = m_blurCheckBox->isChecked();
    importOptions.blurKernelSize = m_blurKernelSpinBox->value();
    importOptions.thresholdValue = m_thresholdSpinBox->value();
    importOptions.adaptiveBlockSize = m_adaptiveBlockSizeSpinBox->value();
    importOptions.adaptiveC = m_adaptiveCSpinBox->value();
    importOptions.cannyLowThreshold = m_cannyLowSpinBox->value();
    importOptions.cannyHighThreshold = m_cannyHighSpinBox->value();
    importOptions.morphologyKernelSize = m_morphologyKernelSpinBox->value();
    importOptions.scale = m_scaleSpinBox->value();
    importOptions.insertOffsetX = m_insertXSpinBox->value();
    importOptions.insertOffsetY = m_insertYSpinBox->value();
    importOptions.approxEpsilon = m_approxEpsilonSpinBox->value();
    importOptions.minContourArea = m_minContourAreaSpinBox->value();
    importOptions.minLineLength = m_minLineLengthSpinBox->value();
    importOptions.lineFitTolerance = m_lineFitToleranceSpinBox->value();
    importOptions.arcFitTolerance = m_arcFitToleranceSpinBox->value();
    importOptions.minArcAngleDegrees = m_minArcAngleSpinBox->value();
    importOptions.maxEntityCount = m_maxEntityCountSpinBox->value();
    importOptions.layerName = m_layerLineEdit->text().trimmed().isEmpty() ? QStringLiteral("BITMAP_IMPORT") : m_layerLineEdit->text().trimmed();
    importOptions.entityColor = m_selectedColor;
    importOptions.autoFitScene = m_autoFitSceneCheckBox->isChecked();
    return importOptions;
}
