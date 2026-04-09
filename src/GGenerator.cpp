#include "pch.h"

#include "GGenerator.h"

#include "CadArcItem.h"
#include "CadCircleItem.h"
#include "CadDocument.h"
#include "CadEllipseItem.h"
#include "CadItem.h"
#include "CadLineItem.h"
#include "CadLWPolylineItem.h"
#include "CadPointItem.h"
#include "CadPolylineItem.h"

#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QTextStream>
#include <QVector3D>
#include <QWidget>

#include <algorithm>
#include <cmath>
#include <vector>

namespace
{
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kTwoPi = 6.28318530717958647692;
    constexpr double kCircleTolerance = 1.0e-8;
    constexpr int kFullEllipseSegments = 128;

    QString formatCoord(double value)
    {
        return QString::number(value, 'f', 5);
    }

    void writeTextBlock(QTextStream& stream, const QString& text)
    {
        if (text.trimmed().isEmpty())
        {
            return;
        }

        QString normalizedText = text;
        normalizedText.replace("\r\n", "\n");
        normalizedText.replace('\r', '\n');

        const QStringList lines = normalizedText.split('\n', Qt::KeepEmptyParts);

        for (const QString& line : lines)
        {
            if (line.trimmed().isEmpty())
            {
                continue;
            }

            stream << line << "\r\n";
        }
    }

    QString entityTypeKey(const CadItem* item)
    {
        if (item == nullptr)
        {
            return QString();
        }

        switch (item->m_type)
        {
        case DRW::ETYPE::LINE:
            return QStringLiteral("LINE");
        case DRW::ETYPE::ARC:
            return QStringLiteral("ARC");
        case DRW::ETYPE::CIRCLE:
            return QStringLiteral("CIRCLE");
        case DRW::ETYPE::ELLIPSE:
            return QStringLiteral("ELLIPSE");
        case DRW::ETYPE::POLYLINE:
            return QStringLiteral("POLYLINE");
        case DRW::ETYPE::LWPOLYLINE:
            return QStringLiteral("LWPOLYLINE");
        case DRW::ETYPE::POINT:
            return QStringLiteral("POINT");
        default:
            return QString();
        }
    }

    QString entityLayerKey(const CadItem* item)
    {
        if (item == nullptr || item->m_nativeEntity == nullptr)
        {
            return QString();
        }

        return GProfile::normalizeLayerKey(QString::fromUtf8(item->m_nativeEntity->layer.c_str()));
    }

    QString entityColorKey(const CadItem* item)
    {
        if (item == nullptr || item->m_nativeEntity == nullptr)
        {
            return QString();
        }

        if (item->m_nativeEntity->color24 >= 0)
        {
            return GProfile::colorKeyFromColor(QColor::fromRgb
            (
                (item->m_nativeEntity->color24 >> 16) & 0xFF,
                (item->m_nativeEntity->color24 >> 8) & 0xFF,
                item->m_nativeEntity->color24 & 0xFF
            ));
        }

        if (item->m_nativeEntity->color == DRW::ColorByLayer)
        {
            return QStringLiteral("BYLAYER");
        }

        if (item->m_nativeEntity->color == DRW::ColorByBlock)
        {
            return QStringLiteral("BYBLOCK");
        }

        return GProfile::colorKeyFromAci(item->m_nativeEntity->color);
    }

    QVector3D ellipsePointAt(const DRW_Ellipse* ellipse, double parameter)
    {
        if (ellipse == nullptr)
        {
            return QVector3D();
        }

        const QVector3D center(ellipse->basePoint.x, ellipse->basePoint.y, ellipse->basePoint.z);
        const QVector3D majorAxis(ellipse->secPoint.x, ellipse->secPoint.y, ellipse->secPoint.z);

        if (majorAxis.lengthSquared() <= 1.0e-12f || ellipse->ratio <= 0.0)
        {
            return QVector3D();
        }

        QVector3D normal(ellipse->extPoint.x, ellipse->extPoint.y, ellipse->extPoint.z);

        if (normal.lengthSquared() <= 1.0e-12f)
        {
            normal = QVector3D(0.0f, 0.0f, 1.0f);
        }
        else
        {
            normal.normalize();
        }

        QVector3D minorAxis = QVector3D::crossProduct(normal, majorAxis);

        if (minorAxis.lengthSquared() <= 1.0e-12f)
        {
            const QVector3D helper = std::abs(majorAxis.z()) < 0.999f
                ? QVector3D(0.0f, 0.0f, 1.0f)
                : QVector3D(0.0f, 1.0f, 0.0f);

            minorAxis = QVector3D::crossProduct(helper, majorAxis);
        }

        if (minorAxis.lengthSquared() <= 1.0e-12f)
        {
            return QVector3D();
        }

        minorAxis.normalize();
        minorAxis *= static_cast<float>(majorAxis.length() * ellipse->ratio);

        return center
            + majorAxis * static_cast<float>(std::cos(parameter))
            + minorAxis * static_cast<float>(std::sin(parameter));
    }

    double normalizeAnglePositive(double angle)
    {
        double normalized = std::fmod(angle, kTwoPi);

        if (normalized < 0.0)
        {
            normalized += kTwoPi;
        }

        return normalized;
    }

    bool isFullEllipsePath(const DRW_Ellipse* ellipse)
    {
        if (ellipse == nullptr)
        {
            return false;
        }

        const double span = ellipse->endparam - ellipse->staparam;
        return std::abs(span) < 1.0e-10
            || std::abs(std::abs(span) - kTwoPi) < 1.0e-10;
    }

    double effectiveCircleStartParameter(const CadCircleItem* item)
    {
        if (item != nullptr && item->m_hasCustomProcessStart)
        {
            return normalizeAnglePositive(item->m_processStartParameter);
        }

        return kPi * 0.5;
    }

    double effectiveClosedEllipseStartParameter(const CadEllipseItem* item)
    {
        if (item != nullptr && item->m_hasCustomProcessStart)
        {
            return item->m_processStartParameter;
        }

        return (item != nullptr && item->m_data != nullptr) ? item->m_data->staparam : 0.0;
    }

    size_t effectiveClosedPolylineStartIndex(const CadItem* item, size_t vertexCount)
    {
        if (vertexCount == 0)
        {
            return 0;
        }

        if (item != nullptr && item->m_hasCustomProcessStart)
        {
            const int rawIndex = static_cast<int>(std::llround(item->m_processStartParameter));
            const int normalized = ((rawIndex % static_cast<int>(vertexCount)) + static_cast<int>(vertexCount)) % static_cast<int>(vertexCount);
            return static_cast<size_t>(normalized);
        }

        return 0;
    }

    QVector<QVector3D> buildEllipsePolyline(const CadEllipseItem* item)
    {
        QVector<QVector3D> points;

        if (item == nullptr || item->m_data == nullptr)
        {
            return points;
        }

        double startParam = item->m_data->staparam;
        double endParam = item->m_data->endparam;

        if (isFullEllipsePath(item->m_data))
        {
            startParam = effectiveClosedEllipseStartParameter(item);
            endParam = startParam + kTwoPi;
        }

        while (endParam <= startParam)
        {
            endParam += kTwoPi;
        }

        const double span = endParam - startParam;
        const int segments = std::max(16, static_cast<int>(std::ceil(span / kTwoPi * kFullEllipseSegments)));
        points.reserve(segments + 1);

        for (int i = 0; i <= segments; ++i)
        {
            const double parameter = startParam + span * static_cast<double>(i) / static_cast<double>(segments);
            points.append(ellipsePointAt(item->m_data, parameter));
        }

        return points;
    }

    void writeRapidMove(QTextStream& stream, const QVector3D& point)
    {
        stream << "G00 X" << formatCoord(point.x()) << " Y" << formatCoord(point.y()) << "\r\n";
    }

    void writeLinearMove(QTextStream& stream, const QVector3D& point)
    {
        stream << "G01 X" << formatCoord(point.x()) << " Y" << formatCoord(point.y()) << "\r\n";
    }

    void writeBulgeSegment(QTextStream& stream, const QVector3D& startPoint, const QVector3D& endPoint, double bulge)
    {
        if (std::abs(bulge) < kCircleTolerance)
        {
            writeLinearMove(stream, endPoint);
            return;
        }

        const double dx = endPoint.x() - startPoint.x();
        const double dy = endPoint.y() - startPoint.y();
        const double chordLength = std::sqrt(dx * dx + dy * dy);

        if (chordLength <= kCircleTolerance)
        {
            return;
        }

        const double midpointX = (startPoint.x() + endPoint.x()) * 0.5;
        const double midpointY = (startPoint.y() + endPoint.y()) * 0.5;
        const double centerOffset = chordLength * (1.0 / bulge - bulge) * 0.25;
        const double centerX = midpointX - centerOffset * (dy / chordLength);
        const double centerY = midpointY + centerOffset * (dx / chordLength);
        const double i = centerX - startPoint.x();
        const double j = centerY - startPoint.y();
        const QString gCode = bulge > 0.0 ? QStringLiteral("G03") : QStringLiteral("G02");

        stream
            << gCode
            << " X" << formatCoord(endPoint.x())
            << " Y" << formatCoord(endPoint.y())
            << " I" << formatCoord(i)
            << " J" << formatCoord(j)
            << "\r\n";
    }

    QVector<CadItem*> collectOrderedItems(const CadDocument* document)
    {
        QVector<CadItem*> orderedItems;

        if (document == nullptr)
        {
            return orderedItems;
        }

        orderedItems.reserve(static_cast<int>(document->m_entities.size()));

        for (const std::unique_ptr<CadItem>& entity : document->m_entities)
        {
            if (entity != nullptr)
            {
                orderedItems.append(entity.get());
            }
        }

        std::stable_sort
        (
            orderedItems.begin(),
            orderedItems.end(),
            [](const CadItem* left, const CadItem* right)
            {
                const int leftOrder = left != nullptr ? left->m_processOrder : -1;
                const int rightOrder = right != nullptr ? right->m_processOrder : -1;

                if (leftOrder < 0 && rightOrder < 0)
                {
                    return false;
                }

                if (leftOrder < 0)
                {
                    return false;
                }

                if (rightOrder < 0)
                {
                    return true;
                }

                return leftOrder < rightOrder;
            }
        );

        return orderedItems;
    }

    bool writeLineEntity(QTextStream& stream, const CadLineItem* item)
    {
        if (item == nullptr || item->m_data == nullptr)
        {
            return false;
        }

        const QVector3D startPoint = item->m_isReverse
            ? QVector3D(item->m_data->secPoint.x, item->m_data->secPoint.y, item->m_data->secPoint.z)
            : QVector3D(item->m_data->basePoint.x, item->m_data->basePoint.y, item->m_data->basePoint.z);
        const QVector3D endPoint = item->m_isReverse
            ? QVector3D(item->m_data->basePoint.x, item->m_data->basePoint.y, item->m_data->basePoint.z)
            : QVector3D(item->m_data->secPoint.x, item->m_data->secPoint.y, item->m_data->secPoint.z);

        writeRapidMove(stream, startPoint);
        writeLinearMove(stream, endPoint);
        return true;
    }

    bool writeArcEntity(QTextStream& stream, const CadArcItem* item)
    {
        if (item == nullptr || item->m_data == nullptr || item->m_data->radious <= 0.0)
        {
            return false;
        }

        const double startAngle = item->m_isReverse ? item->m_data->endangle : item->m_data->staangle;
        const double endAngle = item->m_isReverse ? item->m_data->staangle : item->m_data->endangle;
        const QVector3D center(item->m_data->basePoint.x, item->m_data->basePoint.y, item->m_data->basePoint.z);
        const QVector3D startPoint
        (
            static_cast<float>(center.x() + std::cos(startAngle) * item->m_data->radious),
            static_cast<float>(center.y() + std::sin(startAngle) * item->m_data->radious),
            center.z()
        );
        const QVector3D endPoint
        (
            static_cast<float>(center.x() + std::cos(endAngle) * item->m_data->radious),
            static_cast<float>(center.y() + std::sin(endAngle) * item->m_data->radious),
            center.z()
        );

        writeRapidMove(stream, startPoint);

        stream
            << (item->m_isReverse ? "G02" : "G03")
            << " X" << formatCoord(endPoint.x())
            << " Y" << formatCoord(endPoint.y())
            << " I" << formatCoord(center.x() - startPoint.x())
            << " J" << formatCoord(center.y() - startPoint.y())
            << "\r\n";

        return true;
    }

    bool writeCircleEntity(QTextStream& stream, const CadCircleItem* item)
    {
        if (item == nullptr || item->m_data == nullptr || item->m_data->radious <= 0.0)
        {
            return false;
        }

        const QVector3D center(item->m_data->basePoint.x, item->m_data->basePoint.y, item->m_data->basePoint.z);
        const double startParameter = effectiveCircleStartParameter(item);
        const QVector3D startPoint
        (
            static_cast<float>(center.x() + std::cos(startParameter) * item->m_data->radious),
            static_cast<float>(center.y() + std::sin(startParameter) * item->m_data->radious),
            center.z()
        );

        writeRapidMove(stream, startPoint);

        stream
            << (item->m_isReverse ? "G02" : "G03")
            << " X" << formatCoord(startPoint.x())
            << " Y" << formatCoord(startPoint.y())
            << " I" << formatCoord(center.x() - startPoint.x())
            << " J" << formatCoord(center.y() - startPoint.y())
            << "\r\n";

        return true;
    }

    bool writeEllipseEntity(QTextStream& stream, const CadEllipseItem* item)
    {
        const QVector<QVector3D> sampledPoints = buildEllipsePolyline(item);

        if (sampledPoints.size() < 2)
        {
            return false;
        }

        if (item->m_isReverse)
        {
            writeRapidMove(stream, sampledPoints.back());

            for (int index = sampledPoints.size() - 2; index >= 0; --index)
            {
                writeLinearMove(stream, sampledPoints.at(index));
            }
        }
        else
        {
            writeRapidMove(stream, sampledPoints.front());

            for (int index = 1; index < sampledPoints.size(); ++index)
            {
                writeLinearMove(stream, sampledPoints.at(index));
            }
        }

        return true;
    }

    bool writePolylineEntity(QTextStream& stream, const CadPolylineItem* item)
    {
        if (item == nullptr || item->m_data == nullptr || item->m_data->vertlist.empty())
        {
            return false;
        }

        const bool isClosed = (item->m_data->flags & 1) != 0;
        const auto toVertex = [](const std::shared_ptr<DRW_Vertex>& vertex)
        {
            return QVector3D
            (
                static_cast<float>(vertex->basePoint.x),
                static_cast<float>(vertex->basePoint.y),
                static_cast<float>(vertex->basePoint.z)
            );
        };

        const int vertexCount = static_cast<int>(item->m_data->vertlist.size());
        const size_t startIndex = isClosed
            ? effectiveClosedPolylineStartIndex(item, static_cast<size_t>(vertexCount))
            : 0;
        const QVector3D startPoint = isClosed
            ? toVertex(item->m_data->vertlist.at(startIndex))
            : (item->m_isReverse
                ? toVertex(item->m_data->vertlist.back())
                : toVertex(item->m_data->vertlist.front()));

        writeRapidMove(stream, startPoint);

        if (isClosed)
        {
            if (item->m_isReverse)
            {
                for (int step = 0; step < vertexCount; ++step)
                {
                    const int currentIndex = (static_cast<int>(startIndex) - step + vertexCount) % vertexCount;
                    const int previousIndex = (currentIndex - 1 + vertexCount) % vertexCount;

                    writeBulgeSegment
                    (
                        stream,
                        toVertex(item->m_data->vertlist.at(currentIndex)),
                        toVertex(item->m_data->vertlist.at(previousIndex)),
                        -item->m_data->vertlist.at(previousIndex)->bulge
                    );
                }
            }
            else
            {
                for (int step = 0; step < vertexCount; ++step)
                {
                    const int currentIndex = (static_cast<int>(startIndex) + step) % vertexCount;
                    const int nextIndex = (currentIndex + 1) % vertexCount;

                    writeBulgeSegment
                    (
                        stream,
                        toVertex(item->m_data->vertlist.at(currentIndex)),
                        toVertex(item->m_data->vertlist.at(nextIndex)),
                        item->m_data->vertlist.at(currentIndex)->bulge
                    );
                }
            }
        }
        else if (item->m_isReverse)
        {
            for (int index = vertexCount - 1; index > 0; --index)
            {
                writeBulgeSegment
                (
                    stream,
                    toVertex(item->m_data->vertlist.at(index)),
                    toVertex(item->m_data->vertlist.at(index - 1)),
                    -item->m_data->vertlist.at(index - 1)->bulge
                );
            }
        }
        else
        {
            for (int index = 0; index < vertexCount - 1; ++index)
            {
                writeBulgeSegment
                (
                    stream,
                    toVertex(item->m_data->vertlist.at(index)),
                    toVertex(item->m_data->vertlist.at(index + 1)),
                    item->m_data->vertlist.at(index)->bulge
                );
            }
        }

        return true;
    }

    bool writeLWPolylineEntity(QTextStream& stream, const CadLWPolylineItem* item)
    {
        if (item == nullptr || item->m_data == nullptr || item->m_data->vertlist.empty())
        {
            return false;
        }

        const bool isClosed = (item->m_data->flags & 1) != 0;
        const float z = static_cast<float>(item->m_data->elevation);
        const auto toVertex = [z](const std::shared_ptr<DRW_Vertex2D>& vertex)
        {
            return QVector3D(static_cast<float>(vertex->x), static_cast<float>(vertex->y), z);
        };

        const int vertexCount = static_cast<int>(item->m_data->vertlist.size());
        const size_t startIndex = isClosed
            ? effectiveClosedPolylineStartIndex(item, static_cast<size_t>(vertexCount))
            : 0;
        const QVector3D startPoint = isClosed
            ? toVertex(item->m_data->vertlist.at(startIndex))
            : (item->m_isReverse
                ? toVertex(item->m_data->vertlist.back())
                : toVertex(item->m_data->vertlist.front()));

        writeRapidMove(stream, startPoint);

        if (isClosed)
        {
            if (item->m_isReverse)
            {
                for (int step = 0; step < vertexCount; ++step)
                {
                    const int currentIndex = (static_cast<int>(startIndex) - step + vertexCount) % vertexCount;
                    const int previousIndex = (currentIndex - 1 + vertexCount) % vertexCount;

                    writeBulgeSegment
                    (
                        stream,
                        toVertex(item->m_data->vertlist.at(currentIndex)),
                        toVertex(item->m_data->vertlist.at(previousIndex)),
                        -item->m_data->vertlist.at(previousIndex)->bulge
                    );
                }
            }
            else
            {
                for (int step = 0; step < vertexCount; ++step)
                {
                    const int currentIndex = (static_cast<int>(startIndex) + step) % vertexCount;
                    const int nextIndex = (currentIndex + 1) % vertexCount;

                    writeBulgeSegment
                    (
                        stream,
                        toVertex(item->m_data->vertlist.at(currentIndex)),
                        toVertex(item->m_data->vertlist.at(nextIndex)),
                        item->m_data->vertlist.at(currentIndex)->bulge
                    );
                }
            }
        }
        else if (item->m_isReverse)
        {
            for (int index = vertexCount - 1; index > 0; --index)
            {
                writeBulgeSegment
                (
                    stream,
                    toVertex(item->m_data->vertlist.at(index)),
                    toVertex(item->m_data->vertlist.at(index - 1)),
                    -item->m_data->vertlist.at(index - 1)->bulge
                );
            }
        }
        else
        {
            for (int index = 0; index < vertexCount - 1; ++index)
            {
                writeBulgeSegment
                (
                    stream,
                    toVertex(item->m_data->vertlist.at(index)),
                    toVertex(item->m_data->vertlist.at(index + 1)),
                    item->m_data->vertlist.at(index)->bulge
                );
            }
        }

        return true;
    }

    bool writeItemGeometry(QTextStream& stream, const CadItem* item)
    {
        if (item == nullptr)
        {
            return false;
        }

        switch (item->m_type)
        {
        case DRW::ETYPE::LINE:
            return writeLineEntity(stream, static_cast<const CadLineItem*>(item));
        case DRW::ETYPE::ARC:
            return writeArcEntity(stream, static_cast<const CadArcItem*>(item));
        case DRW::ETYPE::CIRCLE:
            return writeCircleEntity(stream, static_cast<const CadCircleItem*>(item));
        case DRW::ETYPE::ELLIPSE:
            return writeEllipseEntity(stream, static_cast<const CadEllipseItem*>(item));
        case DRW::ETYPE::POLYLINE:
            return writePolylineEntity(stream, static_cast<const CadPolylineItem*>(item));
        case DRW::ETYPE::LWPOLYLINE:
            return writeLWPolylineEntity(stream, static_cast<const CadLWPolylineItem*>(item));
        case DRW::ETYPE::POINT:
            return false;
        default:
            return false;
        }
    }
}

GGenerator::GGenerator()
    : m_defaultProfile(GProfile::createDefaultLaserProfile())
    , m_profile(&m_defaultProfile)
{
}

void GGenerator::setDocument(CadDocument* document)
{
    m_document = document;
}

CadDocument* GGenerator::document() const
{
    return m_document;
}

void GGenerator::setProfile(GProfile* profile)
{
    m_profile = profile != nullptr ? profile : &m_defaultProfile;
}

GProfile* GGenerator::profile() const
{
    return m_profile;
}

void GGenerator::setGenerationMode(GenerationMode generationMode)
{
    m_generationMode = generationMode;
}

GGenerator::GenerationMode GGenerator::generationMode() const
{
    return m_generationMode;
}

bool GGenerator::generate(QWidget* parent, QString* errorMessage) const
{
    const QString filePath = QFileDialog::getSaveFileName
    (
        parent,
        QStringLiteral("导出 G 代码"),
        QStringLiteral("output.nc"),
        QStringLiteral("NC 文件 (*.nc);;GCode 文件 (*.gcode);;文本文件 (*.txt)")
    );

    if (filePath.isEmpty())
    {
        if (errorMessage != nullptr)
        {
            errorMessage->clear();
        }

        return false;
    }

    QString resolvedPath = filePath;

    if (QFileInfo(resolvedPath).suffix().isEmpty())
    {
        resolvedPath.append(QStringLiteral(".nc"));
    }

    return generateToFile(resolvedPath, errorMessage);
}

bool GGenerator::generateToFile(const QString& filePath, QString* errorMessage) const
{
    if (m_generationMode != GenerationMode::Mode2D)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("当前仅实现纯 2D G 代码生成。");
        }

        return false;
    }

    if (m_document == nullptr)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("未设置文档，无法生成 G 代码。");
        }

        return false;
    }

    if (m_profile == nullptr)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("未设置 GProfile，无法生成 G 代码。");
        }

        return false;
    }

    QFile file(filePath);

    if (!file.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate))
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("无法写入 G 代码文件: %1").arg(filePath);
        }

        return false;
    }

    QTextStream stream(&file);
    stream.setEncoding(QStringConverter::Utf8);

    writeTextBlock(stream, m_profile->fileCode().header);

    const QVector<CadItem*> orderedItems = collectOrderedItems(m_document);

    for (CadItem* item : orderedItems)
    {
        if (item == nullptr || item->m_nativeEntity == nullptr)
        {
            continue;
        }

        const QString typeKey = entityTypeKey(item);
        const QString layerKey = entityLayerKey(item);
        const QString colorKey = entityColorKey(item);
        const GProfileCodeBlock typeCode = m_profile->entityTypeCode(typeKey);
        const GProfileCodeBlock layerCode = m_profile->layerCode(layerKey);
        const GProfileCodeBlock colorCode = m_profile->entityColorCode(colorKey);

        QString geometryText;
        QTextStream geometryStream(&geometryText);
        geometryStream.setEncoding(QStringConverter::Utf8);

        if (!writeItemGeometry(geometryStream, item))
        {
            continue;
        }

        if (!geometryText.isEmpty())
        {
            writeTextBlock(stream, layerCode.header);
            writeTextBlock(stream, colorCode.header);
            writeTextBlock(stream, typeCode.header);
            stream << geometryText;
            writeTextBlock(stream, typeCode.footer);
            writeTextBlock(stream, colorCode.footer);
            writeTextBlock(stream, layerCode.footer);
        }
    }

    writeTextBlock(stream, m_profile->fileCode().footer);
    file.close();

    if (errorMessage != nullptr)
    {
        errorMessage->clear();
    }

    return true;
}
