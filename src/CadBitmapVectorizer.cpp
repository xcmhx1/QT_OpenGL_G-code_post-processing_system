// 实现 CadBitmapVectorizer 模块，对应头文件中声明的主要行为和协作流程。
// 位图导入模块，负责图像预处理、轮廓提取和 CAD 实体拟合。
#include "pch.h"

#include "CadBitmapVectorizer.h"

#include <QFile>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <cmath>

namespace
{
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kCircleCircularityThreshold = 0.82;
    constexpr double kCircleErrorRatio = 0.10;
    constexpr double kEllipseResidualThreshold = 0.25;
    constexpr double kEllipseRoundRatioThreshold = 0.92;

    int oddKernelSize(int value, int minimumValue = 3)
    {
        value = std::max(minimumValue, value);

        if ((value % 2) == 0)
        {
            ++value;
        }

        return value;
    }

    int colorToTrueColor(const QColor& color)
    {
        return (color.red() << 16) | (color.green() << 8) | color.blue();
    }

    void applyEntityColor(DRW_Entity* entity, const QColor& color)
    {
        if (entity == nullptr)
        {
            return;
        }

        entity->color = DRW::ColorByLayer;
        entity->color24 = colorToTrueColor(color);
    }

    cv::Point2f toCadPoint(const cv::Point2f& imagePoint, int imageHeight, double scale)
    {
        return cv::Point2f
        (
            static_cast<float>(imagePoint.x * scale),
            static_cast<float>((static_cast<double>(imageHeight) - imagePoint.y) * scale)
        );
    }

    QImage matToQImage(const cv::Mat& image)
    {
        if (image.empty())
        {
            return QImage();
        }

        if (image.type() == CV_8UC1)
        {
            QImage qImage(image.cols, image.rows, QImage::Format_Grayscale8);

            for (int row = 0; row < image.rows; ++row)
            {
                std::memcpy(qImage.scanLine(row), image.ptr(row), static_cast<size_t>(image.cols));
            }

            return qImage;
        }

        cv::Mat rgbImage;
        cv::cvtColor(image, rgbImage, cv::COLOR_BGR2RGB);

        QImage qImage
        (
            rgbImage.data,
            rgbImage.cols,
            rgbImage.rows,
            static_cast<int>(rgbImage.step),
            QImage::Format_RGB888
        );

        return qImage.copy();
    }

    bool buildBinaryMask(const cv::Mat& sourceImage, const CadBitmapImportOptions& options, cv::Mat& mask, QString* errorMessage)
    {
        if (sourceImage.empty())
        {
            if (errorMessage != nullptr)
            {
                *errorMessage = QStringLiteral("位图数据为空，无法生成预处理结果。");
            }

            return false;
        }

        cv::Mat grayImage;
        cv::cvtColor(sourceImage, grayImage, cv::COLOR_BGR2GRAY);

        if (options.blurEnabled)
        {
            const int blurKernelSize = oddKernelSize(options.blurKernelSize);
            cv::GaussianBlur(grayImage, grayImage, cv::Size(blurKernelSize, blurKernelSize), 0.0);
        }

        switch (options.preprocessOperator)
        {
        case CadBitmapPreprocessOperator::OtsuThreshold:
            cv::threshold(grayImage, mask, 0.0, 255.0, cv::THRESH_BINARY | cv::THRESH_OTSU);
            break;

        case CadBitmapPreprocessOperator::FixedThreshold:
            cv::threshold(grayImage, mask, options.thresholdValue, 255.0, cv::THRESH_BINARY);
            break;

        case CadBitmapPreprocessOperator::AdaptiveThreshold:
        {
            const int adaptiveBlockSize = oddKernelSize(options.adaptiveBlockSize);
            cv::adaptiveThreshold
            (
                grayImage,
                mask,
                255.0,
                cv::ADAPTIVE_THRESH_GAUSSIAN_C,
                cv::THRESH_BINARY,
                adaptiveBlockSize,
                options.adaptiveC
            );
            break;
        }

        case CadBitmapPreprocessOperator::CannyEdges:
            cv::Canny(grayImage, mask, options.cannyLowThreshold, options.cannyHighThreshold);
            break;
        }

        if (options.invert)
        {
            cv::bitwise_not(mask, mask);
        }

        if (options.morphologyOperator != CadBitmapMorphologyOperator::None)
        {
            const int morphologyKernelSize = oddKernelSize(options.morphologyKernelSize);
            const cv::Mat kernel = cv::getStructuringElement
            (
                cv::MORPH_RECT,
                cv::Size(morphologyKernelSize, morphologyKernelSize)
            );

            switch (options.morphologyOperator)
            {
            case CadBitmapMorphologyOperator::Open:
                cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
                break;

            case CadBitmapMorphologyOperator::Close:
                cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
                break;

            case CadBitmapMorphologyOperator::Dilate:
                cv::dilate(mask, mask, kernel);
                break;

            case CadBitmapMorphologyOperator::Erode:
                cv::erode(mask, mask, kernel);
                break;

            case CadBitmapMorphologyOperator::None:
                break;
            }
        }

        return true;
    }

    std::unique_ptr<DRW_Entity> createPointEntity(const cv::Point2f& imagePoint, int imageHeight, const CadBitmapImportOptions& options)
    {
        const cv::Point2f cadPoint = toCadPoint(imagePoint, imageHeight, options.scale);
        auto entity = std::make_unique<DRW_Point>();
        entity->basePoint.x = cadPoint.x;
        entity->basePoint.y = cadPoint.y;
        entity->basePoint.z = 0.0;
        applyEntityColor(entity.get(), options.entityColor);
        return entity;
    }

    std::unique_ptr<DRW_Entity> createLineEntity(const cv::Point2f& imageStartPoint, const cv::Point2f& imageEndPoint, int imageHeight, const CadBitmapImportOptions& options)
    {
        const cv::Point2f startPoint = toCadPoint(imageStartPoint, imageHeight, options.scale);
        const cv::Point2f endPoint = toCadPoint(imageEndPoint, imageHeight, options.scale);

        if (cv::norm(endPoint - startPoint) <= 1.0e-6)
        {
            return nullptr;
        }

        auto entity = std::make_unique<DRW_Line>();
        entity->basePoint.x = startPoint.x;
        entity->basePoint.y = startPoint.y;
        entity->basePoint.z = 0.0;
        entity->secPoint.x = endPoint.x;
        entity->secPoint.y = endPoint.y;
        entity->secPoint.z = 0.0;
        applyEntityColor(entity.get(), options.entityColor);
        return entity;
    }

    std::unique_ptr<DRW_Entity> createCircleEntity(const cv::Point2f& imageCenter, float radius, int imageHeight, const CadBitmapImportOptions& options)
    {
        if (radius <= 1.0f)
        {
            return nullptr;
        }

        const cv::Point2f centerPoint = toCadPoint(imageCenter, imageHeight, options.scale);
        auto entity = std::make_unique<DRW_Circle>();
        entity->basePoint.x = centerPoint.x;
        entity->basePoint.y = centerPoint.y;
        entity->basePoint.z = 0.0;
        entity->radious = radius * options.scale;
        entity->extPoint.x = 0.0;
        entity->extPoint.y = 0.0;
        entity->extPoint.z = 1.0;
        applyEntityColor(entity.get(), options.entityColor);
        return entity;
    }

    std::unique_ptr<DRW_Entity> createEllipseEntity(const cv::RotatedRect& ellipse, int imageHeight, const CadBitmapImportOptions& options)
    {
        double majorAxisLength = ellipse.size.width * 0.5;
        double minorAxisLength = ellipse.size.height * 0.5;
        double angleDegrees = ellipse.angle;

        if (majorAxisLength < minorAxisLength)
        {
            std::swap(majorAxisLength, minorAxisLength);
            angleDegrees += 90.0;
        }

        if (majorAxisLength <= 1.0 || minorAxisLength <= 1.0)
        {
            return nullptr;
        }

        const cv::Point2f centerPoint = toCadPoint(ellipse.center, imageHeight, options.scale);
        const double angleRadians = angleDegrees * kPi / 180.0;
        auto entity = std::make_unique<DRW_Ellipse>();
        entity->basePoint.x = centerPoint.x;
        entity->basePoint.y = centerPoint.y;
        entity->basePoint.z = 0.0;
        entity->secPoint.x = std::cos(angleRadians) * majorAxisLength * options.scale;
        entity->secPoint.y = -std::sin(angleRadians) * majorAxisLength * options.scale;
        entity->secPoint.z = 0.0;
        entity->ratio = minorAxisLength / majorAxisLength;
        entity->staparam = 0.0;
        entity->endparam = 2.0 * kPi;
        entity->extPoint.x = 0.0;
        entity->extPoint.y = 0.0;
        entity->extPoint.z = 1.0;
        applyEntityColor(entity.get(), options.entityColor);
        return entity;
    }

    std::unique_ptr<DRW_Entity> createPolylineEntity(const std::vector<cv::Point>& imagePoints, bool closed, int imageHeight, const CadBitmapImportOptions& options)
    {
        if (imagePoints.empty())
        {
            return nullptr;
        }

        auto entity = std::make_unique<DRW_LWPolyline>();
        entity->flags = closed ? 1 : 0;
        entity->elevation = 0.0;

        for (const cv::Point& point : imagePoints)
        {
            const cv::Point2f cadPoint = toCadPoint(cv::Point2f(static_cast<float>(point.x), static_cast<float>(point.y)), imageHeight, options.scale);
            std::shared_ptr<DRW_Vertex2D> vertex = std::make_shared<DRW_Vertex2D>();
            vertex->x = cadPoint.x;
            vertex->y = cadPoint.y;
            vertex->bulge = 0.0;
            entity->vertlist.push_back(vertex);
        }

        entity->vertexnum = static_cast<int>(entity->vertlist.size());
        applyEntityColor(entity.get(), options.entityColor);
        return entity;
    }

    bool tryCreateCircleEntity
    (
        const std::vector<cv::Point>& contour,
        int imageHeight,
        const CadBitmapImportOptions& options,
        std::unique_ptr<DRW_Entity>& entity
    )
    {
        if (contour.size() < 5)
        {
            return false;
        }

        const double area = std::abs(cv::contourArea(contour));
        const double perimeter = cv::arcLength(contour, true);

        if (area <= 1.0 || perimeter <= 1.0)
        {
            return false;
        }

        const double circularity = 4.0 * kPi * area / (perimeter * perimeter);

        if (circularity < kCircleCircularityThreshold)
        {
            return false;
        }

        cv::Point2f center;
        float radius = 0.0f;
        cv::minEnclosingCircle(contour, center, radius);

        if (radius <= 1.0f)
        {
            return false;
        }

        double errorSum = 0.0;

        for (const cv::Point& point : contour)
        {
            errorSum += std::abs(cv::norm(cv::Point2f(static_cast<float>(point.x), static_cast<float>(point.y)) - center) - radius);
        }

        const double meanError = errorSum / static_cast<double>(contour.size());

        if (meanError > std::max(1.5, radius * kCircleErrorRatio))
        {
            return false;
        }

        entity = createCircleEntity(center, radius, imageHeight, options);
        return entity != nullptr;
    }

    bool tryCreateEllipseEntity
    (
        const std::vector<cv::Point>& contour,
        int imageHeight,
        const CadBitmapImportOptions& options,
        std::unique_ptr<DRW_Entity>& entity
    )
    {
        if (contour.size() < 5)
        {
            return false;
        }

        const cv::RotatedRect ellipse = cv::fitEllipse(contour);
        double majorAxisLength = ellipse.size.width * 0.5;
        double minorAxisLength = ellipse.size.height * 0.5;
        double angleDegrees = ellipse.angle;

        if (majorAxisLength < minorAxisLength)
        {
            std::swap(majorAxisLength, minorAxisLength);
            angleDegrees += 90.0;
        }

        if (majorAxisLength <= 1.0 || minorAxisLength <= 1.0)
        {
            return false;
        }

        const double minorToMajorRatio = minorAxisLength / majorAxisLength;

        if (minorToMajorRatio >= kEllipseRoundRatioThreshold)
        {
            return false;
        }

        const double angleRadians = angleDegrees * kPi / 180.0;
        const double cosAngle = std::cos(angleRadians);
        const double sinAngle = std::sin(angleRadians);
        double residualSum = 0.0;

        for (const cv::Point& point : contour)
        {
            const double translatedX = point.x - ellipse.center.x;
            const double translatedY = point.y - ellipse.center.y;
            const double localX = translatedX * cosAngle + translatedY * sinAngle;
            const double localY = -translatedX * sinAngle + translatedY * cosAngle;
            const double ellipseValue = (localX * localX) / (majorAxisLength * majorAxisLength)
                + (localY * localY) / (minorAxisLength * minorAxisLength);
            residualSum += std::abs(1.0 - ellipseValue);
        }

        const double meanResidual = residualSum / static_cast<double>(contour.size());

        if (meanResidual > kEllipseResidualThreshold)
        {
            return false;
        }

        entity = createEllipseEntity(ellipse, imageHeight, options);
        return entity != nullptr;
    }

    QString buildResultSummary(const CadBitmapImportResult& result)
    {
        return QStringLiteral
        (
            "位图拟合完成: 轮廓 %1, 实体 %2, 圆 %3, 椭圆 %4, 折线 %5, 直线 %6, 点 %7"
        )
        .arg(result.contourCount)
        .arg(result.importedEntityCount)
        .arg(result.circleCount)
        .arg(result.ellipseCount)
        .arg(result.polylineCount)
        .arg(result.lineCount)
        .arg(result.pointCount);
    }
}

bool CadBitmapVectorizer::loadBitmapImage(const QString& filePath, cv::Mat& image, QString* errorMessage)
{
    QFile file(filePath);

    if (!file.open(QIODevice::ReadOnly))
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("无法打开位图文件: %1").arg(filePath);
        }

        return false;
    }

    const QByteArray fileData = file.readAll();
    const std::vector<uchar> encodedBytes(fileData.begin(), fileData.end());
    image = cv::imdecode(encodedBytes, cv::IMREAD_COLOR);

    if (image.empty())
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("OpenCV 无法解码该位图文件: %1").arg(filePath);
        }

        return false;
    }

    return true;
}

CadBitmapPreviewData CadBitmapVectorizer::buildPreview(const cv::Mat& sourceImage, const CadBitmapImportOptions& options, QString* errorMessage)
{
    CadBitmapPreviewData previewData;
    previewData.sourcePreview = matToQImage(sourceImage);

    cv::Mat mask;

    if (!buildBinaryMask(sourceImage, options, mask, errorMessage))
    {
        previewData.summaryText = errorMessage != nullptr ? *errorMessage : QStringLiteral("无法生成位图预处理预览。");
        return previewData;
    }

    previewData.processedPreview = matToQImage(mask);
    previewData.summaryText = QStringLiteral("预览尺寸: %1 x %2，当前比例: %3")
        .arg(sourceImage.cols)
        .arg(sourceImage.rows)
        .arg(options.scale, 0, 'f', 3);
    return previewData;
}

bool CadBitmapVectorizer::vectorize(const cv::Mat& sourceImage, const CadBitmapImportOptions& options, CadBitmapImportResult& result, QString* errorMessage)
{
    result = CadBitmapImportResult();

    cv::Mat mask;

    if (!buildBinaryMask(sourceImage, options, mask, errorMessage))
    {
        return false;
    }

    std::vector<std::vector<cv::Point>> contours;
    const int retrievalMode = options.contourMode == CadBitmapContourMode::ExternalOnly ? cv::RETR_EXTERNAL : cv::RETR_LIST;
    cv::findContours(mask.clone(), contours, retrievalMode, cv::CHAIN_APPROX_SIMPLE);
    result.contourCount = static_cast<int>(contours.size());

    for (const std::vector<cv::Point>& contour : contours)
    {
        if (result.importedEntityCount >= options.maxEntityCount)
        {
            break;
        }

        const double area = std::abs(cv::contourArea(contour));
        const double perimeter = cv::arcLength(contour, true);

        if (area < options.minContourArea && perimeter < std::max(8.0, options.minContourArea))
        {
            continue;
        }

        std::unique_ptr<DRW_Entity> entity;

        if (options.fitStrategy == CadBitmapFitStrategy::PreferPrimitives)
        {
            if (tryCreateCircleEntity(contour, sourceImage.rows, options, entity))
            {
                ++result.circleCount;
            }
            else if (tryCreateEllipseEntity(contour, sourceImage.rows, options, entity))
            {
                ++result.ellipseCount;
            }
        }

        if (entity == nullptr)
        {
            std::vector<cv::Point> approxPoints;
            cv::approxPolyDP(contour, approxPoints, std::max(0.5, options.approxEpsilon), true);

            if (approxPoints.size() <= 1)
            {
                entity = createPointEntity
                (
                    contour.empty() ? cv::Point2f() : cv::Point2f(static_cast<float>(contour.front().x), static_cast<float>(contour.front().y)),
                    sourceImage.rows,
                    options
                );

                if (entity != nullptr)
                {
                    ++result.pointCount;
                }
            }
            else if (approxPoints.size() == 2)
            {
                entity = createLineEntity
                (
                    cv::Point2f(static_cast<float>(approxPoints[0].x), static_cast<float>(approxPoints[0].y)),
                    cv::Point2f(static_cast<float>(approxPoints[1].x), static_cast<float>(approxPoints[1].y)),
                    sourceImage.rows,
                    options
                );

                if (entity != nullptr)
                {
                    ++result.lineCount;
                }
            }
            else
            {
                entity = createPolylineEntity(approxPoints, true, sourceImage.rows, options);

                if (entity != nullptr)
                {
                    ++result.polylineCount;
                }
            }
        }

        if (entity == nullptr)
        {
            continue;
        }

        result.entities.push_back(std::move(entity));
        ++result.importedEntityCount;
    }

    result.summaryText = buildResultSummary(result);

    if (result.importedEntityCount <= 0)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("位图处理完成，但没有生成可导入的 CAD 图元。请调整阈值、算子或拟合参数。");
        }

        return false;
    }

    return true;
}
