// 声明 CadBitmapImportDialog 模块，对外暴露位图导入预处理配置界面。
// 位图导入对话框模块，负责预览图像预处理结果并收集导入参数。
#pragma once

#include "CadBitmapVectorizer.h"

#include <QDialog>
#include <QImage>
#include <QString>

#include <opencv2/core/mat.hpp>

class QCheckBox;
class QComboBox;
class QDialogButtonBox;
class QDoubleSpinBox;
class QLabel;
class QSpinBox;

class CadBitmapImportDialog : public QDialog
{
public:
    explicit CadBitmapImportDialog(const QString& filePath, QWidget* parent = nullptr);

    bool isReady() const;
    const QString& errorMessage() const;
    const cv::Mat& sourceImage() const;
    CadBitmapImportOptions options() const;

protected:
    void resizeEvent(QResizeEvent* event) override;

private:
    void buildUi();
    void connectSignals();
    void refreshPreview();
    void updatePreviewLabel(QLabel* label, const QImage& image, const QString& fallbackText);
    CadBitmapImportOptions collectOptions() const;

private:
    QString m_filePath;
    QString m_errorMessage;
    cv::Mat m_sourceImage;
    QImage m_sourcePreviewImage;
    QImage m_processedPreviewImage;
    QLabel* m_sourcePreviewLabel = nullptr;
    QLabel* m_processedPreviewLabel = nullptr;
    QLabel* m_summaryLabel = nullptr;
    QComboBox* m_importModeCombo = nullptr;
    QComboBox* m_operatorCombo = nullptr;
    QComboBox* m_morphologyCombo = nullptr;
    QComboBox* m_contourModeCombo = nullptr;
    QComboBox* m_fitStrategyCombo = nullptr;
    QCheckBox* m_invertCheckBox = nullptr;
    QCheckBox* m_blurCheckBox = nullptr;
    QSpinBox* m_blurKernelSpinBox = nullptr;
    QSpinBox* m_thresholdSpinBox = nullptr;
    QSpinBox* m_adaptiveBlockSizeSpinBox = nullptr;
    QDoubleSpinBox* m_adaptiveCSpinBox = nullptr;
    QSpinBox* m_cannyLowSpinBox = nullptr;
    QSpinBox* m_cannyHighSpinBox = nullptr;
    QSpinBox* m_morphologyKernelSpinBox = nullptr;
    QDoubleSpinBox* m_scaleSpinBox = nullptr;
    QDoubleSpinBox* m_approxEpsilonSpinBox = nullptr;
    QDoubleSpinBox* m_minContourAreaSpinBox = nullptr;
    QSpinBox* m_maxEntityCountSpinBox = nullptr;
    QDialogButtonBox* m_buttonBox = nullptr;
};
