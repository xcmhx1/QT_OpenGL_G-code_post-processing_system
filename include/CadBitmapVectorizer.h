// 声明 CadBitmapVectorizer 模块，对外暴露位图预处理与 CAD 图元拟合能力。
// 位图导入模块，负责把 bmp/jpeg/png 等栅格图像转换为当前项目支持的 CAD 实体。
#pragma once

#include "libdxfrw/drw_entities.h"

#include <memory>
#include <vector>

#include <QColor>
#include <QImage>
#include <QString>

#include <opencv2/core/mat.hpp>

enum class CadBitmapImportMode
{
    ReplaceDocument,
    AppendToDocument
};

enum class CadBitmapPreprocessOperator
{
    OtsuThreshold,
    FixedThreshold,
    AdaptiveThreshold,
    CannyEdges
};

enum class CadBitmapMorphologyOperator
{
    None,
    Open,
    Close,
    Dilate,
    Erode
};

enum class CadBitmapContourMode
{
    ExternalOnly,
    AllContours
};

enum class CadBitmapFitStrategy
{
    PreferPrimitives,
    PolylineOnly
};

struct CadBitmapImportOptions
{
    QString filePath;
    CadBitmapImportMode importMode = CadBitmapImportMode::ReplaceDocument;
    CadBitmapPreprocessOperator preprocessOperator = CadBitmapPreprocessOperator::OtsuThreshold;
    CadBitmapMorphologyOperator morphologyOperator = CadBitmapMorphologyOperator::Close;
    CadBitmapContourMode contourMode = CadBitmapContourMode::ExternalOnly;
    CadBitmapFitStrategy fitStrategy = CadBitmapFitStrategy::PreferPrimitives;
    bool invert = true;
    bool blurEnabled = true;
    int blurKernelSize = 5;
    int thresholdValue = 140;
    int adaptiveBlockSize = 31;
    double adaptiveC = 5.0;
    int cannyLowThreshold = 50;
    int cannyHighThreshold = 150;
    int morphologyKernelSize = 3;
    double scale = 1.0;
    double approxEpsilon = 2.5;
    double minContourArea = 24.0;
    int maxEntityCount = 5000;
    QColor entityColor = QColor(255, 255, 255);
};

struct CadBitmapPreviewData
{
    QImage sourcePreview;
    QImage processedPreview;
    QString summaryText;
};

struct CadBitmapImportResult
{
    std::vector<std::unique_ptr<DRW_Entity>> entities;
    QString summaryText;
    int contourCount = 0;
    int importedEntityCount = 0;
    int pointCount = 0;
    int lineCount = 0;
    int circleCount = 0;
    int ellipseCount = 0;
    int polylineCount = 0;
};

class CadBitmapVectorizer
{
public:
    static bool loadBitmapImage(const QString& filePath, cv::Mat& image, QString* errorMessage = nullptr);
    static CadBitmapPreviewData buildPreview(const cv::Mat& sourceImage, const CadBitmapImportOptions& options, QString* errorMessage = nullptr);
    static bool vectorize(const cv::Mat& sourceImage, const CadBitmapImportOptions& options, CadBitmapImportResult& result, QString* errorMessage = nullptr);
};
