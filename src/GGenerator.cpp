#include "pch.h"

#include "GGenerator.h"
#include "application/nc/NcProgramService.h"
#include "infrastructure/nc/GCodePostProcessor.h"

#include "CadArcItem.h"
#include "CadCircleItem.h"
#include "CadDocument.h"
#include "CadEllipseItem.h"
#include "CadItem.h"
#include "CadOcsGeometry.h"
#include "CadLineItem.h"
#include "CadLWPolylineItem.h"
#include "CadPointItem.h"
#include "CadPolylineItem.h"
#include "CadSplineItem.h"

#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QDir>
#include <QDebug>
#include <QSettings>
#include <QStandardPaths>
#include <QTextStream>
#include <QVector3D>
#include <QWidget>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace
{
    constexpr double kTwoPi = 6.28318530717958647692;
    constexpr double kCircleTolerance = 1.0e-8;
    constexpr int kFullEllipseSegments = 128;
    constexpr double kControlPointTolerance = 1.0e-5;
    constexpr double kNoLiftPathConnectionTolerance = 1.0;
    QString formatCoord(double value)
    {
        return QString::number(value, 'f', 5);
    }

    QVector<QVector3D> buildEllipsePolyline(const CadEllipseItem* item);

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

    QString normalizeLineEndingsToCrLf(const QString& text)
    {
        QString normalizedText = text;
        normalizedText.replace("\r\n", "\n");
        normalizedText.replace('\r', '\n');
        normalizedText.replace('\n', "\r\n");
        return normalizedText;
    }

    bool isStandaloneMCode(const QString& line, const QString& paddedCode, const QString& shortCode)
    {
        const QString normalized = line.trimmed().toUpper();
        return normalized == paddedCode || normalized == shortCode;
    }

    QString removeRedundantLaserRestartPairs2D(const QString& program)
    {
        QString normalized = program;
        normalized.replace("\r\n", "\n");
        normalized.replace('\r', '\n');

        QStringList optimizedLines;
        const QStringList lines = normalized.split('\n', Qt::KeepEmptyParts);

        for (const QString& line : lines)
        {
            if (line.trimmed().isEmpty())
            {
                continue;
            }

            const bool startsLaser = isStandaloneMCode(line, QStringLiteral("M03"), QStringLiteral("M3"));
            const bool previousStopsLaser = !optimizedLines.isEmpty()
                && isStandaloneMCode(optimizedLines.constLast(), QStringLiteral("M05"), QStringLiteral("M5"));

            if (startsLaser && previousStopsLaser)
            {
                optimizedLines.removeLast();
                continue;
            }

            optimizedLines.push_back(line);
        }

        return optimizedLines.isEmpty()
            ? QString()
            : optimizedLines.join(QStringLiteral("\r\n")) + QStringLiteral("\r\n");
    }

    bool isRapidMoveLine(const QString& line)
    {
        const QString trimmedLine = line.trimmed().toUpper();
        return trimmedLine.startsWith(QStringLiteral("G00"))
            || trimmedLine.startsWith(QStringLiteral("G0 "));
    }

    void splitLeadingRapidMoves
    (
        const QString& geometryText,
        QString& outRapidPrefix,
        QString& outCuttingBody
    )
    {
        outRapidPrefix.clear();
        outCuttingBody.clear();

        QString normalizedText = geometryText;
        normalizedText.replace("\r\n", "\n");
        normalizedText.replace('\r', '\n');

        const QStringList lines = normalizedText.split('\n', Qt::KeepEmptyParts);
        bool bodyStarted = false;

        for (const QString& line : lines)
        {
            if (line.trimmed().isEmpty())
            {
                continue;
            }

            if (!bodyStarted && isRapidMoveLine(line))
            {
                outRapidPrefix += line;
                outRapidPrefix += QLatin1Char('\n');
                continue;
            }

            bodyStarted = true;
            outCuttingBody += line;
            outCuttingBody += QLatin1Char('\n');
        }
    }

    QString defaultNcPath()
    {
        QString baseDirectory = QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation);

        if (baseDirectory.trimmed().isEmpty())
        {
            baseDirectory = QDir::homePath();
        }

        return QDir(baseDirectory).filePath(QStringLiteral("output.nc"));
    }

    QString loadLastExportPath()
    {
        QSettings settings(QStringLiteral("GCodePostProcessingSystem"), QStringLiteral("GCodePostProcessingSystem"));
        return settings.value(QStringLiteral("gcode/lastExportPath"), QString()).toString().trimmed();
    }

    void saveLastExportPath(const QString& filePath)
    {
        QSettings settings(QStringLiteral("GCodePostProcessingSystem"), QStringLiteral("GCodePostProcessingSystem"));
        settings.setValue(QStringLiteral("gcode/lastExportPath"), filePath.trimmed());
    }

    QString resolveInitialExportPath(QWidget*)
    {
        const QString lastExportPath = loadLastExportPath();

        if (!lastExportPath.isEmpty())
        {
            const QFileInfo lastExportInfo(lastExportPath);
            const QString lastDirectoryPath = lastExportInfo.absolutePath();

            if (QDir(lastDirectoryPath).exists())
            {
                return lastExportPath;
            }

        }

        return defaultNcPath();
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
        case DRW::ETYPE::SPLINE:
            return QStringLiteral("SPLINE");
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
        return item != nullptr ? item->defaultProcessStartParameter() : M_PI_2;
    }

    double effectiveClosedEllipseStartParameter(const CadEllipseItem* item)
    {
        return item != nullptr ? item->defaultProcessStartParameter() : M_PI_2;
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

    void writeRapidMove3Axis(QTextStream& stream, const QVector3D& point)
    {
        stream
            << "G00 X" << formatCoord(point.x())
            << " Y" << formatCoord(point.y())
            << " Z" << formatCoord(point.z())
            << "\r\n";
    }

    void writeLinearMove3Axis(QTextStream& stream, const QVector3D& point)
    {
        stream
            << "G01 X" << formatCoord(point.x())
            << " Y" << formatCoord(point.y())
            << " Z" << formatCoord(point.z())
            << "\r\n";
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
            if (entity != nullptr && !entity->m_excludedFromProcessing)
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

        const DRW_Arc* arc = item->m_data;
        const double startAngle = item->m_isReverse ? arc->endangle : arc->staangle;
        const double endAngle = item->m_isReverse ? arc->staangle : arc->endangle;
        const QVector3D center = CadOcsGeometry::center(arc);
        const QVector3D startPoint = CadOcsGeometry::pointAt(arc, startAngle);
        const QVector3D endPoint = CadOcsGeometry::pointAt(arc, endAngle);
        const QVector3D normal = CadOcsGeometry::normal(arc->extPoint);
        constexpr float kPlaneTolerance = 1.0e-6f;

        auto arcCode = [item](float normalComponent)
            {
                const bool clockwise = (normalComponent < 0.0f) != item->m_isReverse;
                return clockwise ? QStringLiteral("G02") : QStringLiteral("G03");
            };

        if (std::abs(normal.x()) <= kPlaneTolerance && std::abs(normal.y()) <= kPlaneTolerance)
        {
            writeRapidMove(stream, startPoint);
            stream
                << arcCode(normal.z())
                << " X" << formatCoord(endPoint.x())
                << " Y" << formatCoord(endPoint.y())
                << " I" << formatCoord(center.x() - startPoint.x())
                << " J" << formatCoord(center.y() - startPoint.y())
                << "\r\n";
            return true;
        }

        if (std::abs(normal.x()) <= kPlaneTolerance && std::abs(normal.z()) <= kPlaneTolerance)
        {
            writeRapidMove3Axis(stream, startPoint);
            stream << "G18\r\n";
            stream
                // G18 uses the ZX orientation, opposite to the XZ screen order.
                << arcCode(-normal.y())
                << " X" << formatCoord(endPoint.x())
                << " Z" << formatCoord(endPoint.z())
                << " I" << formatCoord(center.x() - startPoint.x())
                << " K" << formatCoord(center.z() - startPoint.z())
                << "\r\n"
                << "G17\r\n";
            return true;
        }

        if (std::abs(normal.y()) <= kPlaneTolerance && std::abs(normal.z()) <= kPlaneTolerance)
        {
            writeRapidMove3Axis(stream, startPoint);
            stream << "G19\r\n";
            stream
                << arcCode(normal.x())
                << " Y" << formatCoord(endPoint.y())
                << " Z" << formatCoord(endPoint.z())
                << " J" << formatCoord(center.y() - startPoint.y())
                << " K" << formatCoord(center.z() - startPoint.z())
                << "\r\n"
                << "G17\r\n";
            return true;
        }

        const QVector<QVector3D>& vertices = item->m_geometry.vertices;
        if (vertices.size() < 2)
        {
            return false;
        }

        if (item->m_isReverse)
        {
            writeRapidMove3Axis(stream, vertices.constLast());
            for (int index = vertices.size() - 2; index >= 0; --index)
            {
                writeLinearMove3Axis(stream, vertices.at(index));
            }
        }
        else
        {
            writeRapidMove3Axis(stream, vertices.constFirst());
            for (int index = 1; index < vertices.size(); ++index)
            {
                writeLinearMove3Axis(stream, vertices.at(index));
            }
        }

        return true;
    }

    bool writeCircleEntity(QTextStream& stream, const CadCircleItem* item)
    {
        if (item == nullptr || item->m_data == nullptr || item->m_data->radious <= 0.0)
        {
            return false;
        }

        const DRW_Circle* circle = item->m_data;
        const QVector3D center = CadOcsGeometry::center(circle);
        const double startParameter = effectiveCircleStartParameter(item);
        const QVector3D startPoint = CadOcsGeometry::pointAt(circle, startParameter);
        const QVector3D normal = CadOcsGeometry::normal(circle->extPoint);
        constexpr float kPlaneTolerance = 1.0e-6f;

        if (std::abs(normal.x()) <= kPlaneTolerance && std::abs(normal.y()) <= kPlaneTolerance)
        {
            const bool clockwise = (normal.z() < 0.0f) != item->m_isReverse;

            writeRapidMove(stream, startPoint);

            stream
                << (clockwise ? "G02" : "G03")
                << " X" << formatCoord(startPoint.x())
                << " Y" << formatCoord(startPoint.y())
                << " I" << formatCoord(center.x() - startPoint.x())
                << " J" << formatCoord(center.y() - startPoint.y())
                << "\r\n";

            return true;
        }

        const QVector<QVector3D>& vertices = item->m_geometry.vertices;
        if (vertices.size() < 2)
        {
            return false;
        }

        if (item->m_isReverse)
        {
            writeRapidMove3Axis(stream, vertices.constLast());
            for (int index = vertices.size() - 2; index >= 0; --index)
            {
                writeLinearMove3Axis(stream, vertices.at(index));
            }
        }
        else
        {
            writeRapidMove3Axis(stream, vertices.constFirst());
            for (int index = 1; index < vertices.size(); ++index)
            {
                writeLinearMove3Axis(stream, vertices.at(index));
            }
        }

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

    bool writeSplineEntity(QTextStream& stream, const CadSplineItem* item)
    {
        if (item == nullptr)
        {
            return false;
        }
        CadSplineItem* writableItem = const_cast<CadSplineItem*>(item);
        writableItem->rebuildRawPathPoints3D();
        const std::vector<RawPathPoint3D>& points = writableItem->rawPathPoints3D();
        if (points.size() < 2U)
        {
            return false;
        }
        const auto toVector = [](const RawPathPoint3D& point)
        {
            return QVector3D
            (
                static_cast<float>(point.x),
                static_cast<float>(point.y),
                static_cast<float>(point.z)
            );
        };
        writeRapidMove3Axis(stream, toVector(points.front()));
        for (std::size_t index = 1U; index < points.size(); ++index)
        {
            writeLinearMove3Axis(stream, toVector(points[index]));
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
        case DRW::ETYPE::SPLINE:
            return writeSplineEntity(stream, static_cast<const CadSplineItem*>(item));
        case DRW::ETYPE::POINT:
            return false;
        default:
            return false;
        }
    }

    Diagnostic makeGeneratorDiagnostic
    (
        const OperationContext& context,
        DiagnosticCode code,
        DiagnosticSeverity severity,
        const QString& operation,
        const QString& stage,
        const QString& userMessage,
        const QString& technicalDetail = QString(),
        const QVariantMap& diagnosticContext = QVariantMap()
    )
    {
        Diagnostic diagnostic;
        diagnostic.code = code;
        diagnostic.severity = severity;
        diagnostic.component = QStringLiteral("GGenerator");
        diagnostic.operation = operation;
        diagnostic.stage = stage;
        diagnostic.userMessage = userMessage;
        diagnostic.technicalDetail = technicalDetail;
        diagnostic.correlationId = context.correlationId;
        diagnostic.context = diagnosticContext;
        return diagnostic;
    }

    QString legacyErrorMessage(const QVector<Diagnostic>& diagnostics)
    {
        for (const Diagnostic& diagnostic : diagnostics)
        {
            if (isErrorSeverity(diagnostic.severity))
            {
                return diagnostic.userMessage.trimmed().isEmpty()
                    ? diagnostic.technicalDetail
                    : diagnostic.userMessage;
            }
        }

        return diagnostics.isEmpty() ? QString() : diagnostics.constFirst().userMessage;
    }

    bool isGCodeGeometryType(DRW::ETYPE type)
    {
        switch (type)
        {
        case DRW::ETYPE::LINE:
        case DRW::ETYPE::CIRCLE:
        case DRW::ETYPE::ARC:
        case DRW::ETYPE::ELLIPSE:
        case DRW::ETYPE::POLYLINE:
        case DRW::ETYPE::LWPOLYLINE:
        case DRW::ETYPE::SPLINE:
            return true;
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

void GGenerator::setRotaryTubeCenter(double centerY, double centerZ, bool valid)
{
    m_rotaryTubeCenterY = centerY;
    m_rotaryTubeCenterZ = centerZ;
    m_rotaryTubeCenterValid = valid;
}

void GGenerator::setProcessPlan(const cadcam::planning::ProcessPlan* processPlan)
{
    m_processPlan = processPlan;
}

void GGenerator::setTubeSectionModel
(
    const std::optional<cadcam::machining::TubeSectionModel>& tubeSectionModel
)
{
    m_tubeSectionModel = tubeSectionModel;
}

bool GGenerator::generate(QWidget* parent, QString* errorMessage) const
{
    const QString initialPath = resolveInitialExportPath(parent);
    const QString filePath = QFileDialog::getSaveFileName
    (
        parent,
        QStringLiteral("导出 G 代码"),
        initialPath,
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

    const bool generated = generateToFile(resolvedPath, errorMessage);

    if (generated)
    {
        saveLastExportPath(resolvedPath);
    }

    return generated;
}

OperationResult<QString> GGenerator::buildRotaryProgramText(const OperationContext& context) const
{
    OperationResult<QString> result;
    const bool hasProcessableEntity = std::any_of
    (
        m_document->m_entities.cbegin(), m_document->m_entities.cend(),
        [](const std::unique_ptr<CadItem>& item)
        {
            return item != nullptr && item->m_nativeEntity != nullptr
                && isGCodeGeometryType(item->m_type);
        }
    );
    if (!hasProcessableEntity)
    {
        result.status = OperationStatus::InvalidInput;
        result.addDiagnostic(makeGeneratorDiagnostic
        (
            context,
            DiagnosticCode::InvalidGeometry,
            DiagnosticSeverity::Error,
            QStringLiteral("BuildRotaryProgramText"),
            QStringLiteral("ValidateDocument"),
            QStringLiteral("文档中没有可生成 G 代码的图元。")
        ));
        return result;
    }
    if (m_processPlan == nullptr)
    {
        result.status = OperationStatus::InvalidInput;
        result.addDiagnostic(makeGeneratorDiagnostic
        (
            context,
            DiagnosticCode::MachineTrajectoryInputInvalid,
            DiagnosticSeverity::Error,
            QStringLiteral("BuildRotaryProgramText"),
            QStringLiteral("ValidateProcessPlan"),
            QStringLiteral("当前没有有效的四轴加工计划，无法导出。")
        ));
        return result;
    }

    std::optional<cadcam::geometry::Vector2d> explicitCenter;
    if (m_rotaryTubeCenterValid)
        explicitCenter = cadcam::geometry::Vector2d{ m_rotaryTubeCenterY, m_rotaryTubeCenterZ };
    NcProgramService service;
    auto programResult = service.buildRotaryProgram
    (
        *m_document,
        *m_processPlan,
        m_tubeSectionModel,
        m_profile->rotaryAxisConfig(),
        context,
        explicitCenter
    );
    result.mergeDiagnostics(programResult);
    if (!programResult.succeeded() || !programResult.value.has_value())
    {
        result.status = programResult.status;
        return result;
    }

    const auto postProfile = cadcam::infrastructure::nc::makeGCodePostProcessorProfile(*m_profile);
    auto rendered = cadcam::infrastructure::nc::GCodePostProcessor::render
        (*programResult.value, postProfile, context);
    result.mergeDiagnostics(rendered);
    if (!rendered.succeeded() || !rendered.value.has_value())
    {
        result.status = rendered.status;
        return result;
    }
    result.status = programResult.status == OperationStatus::PartialSuccess
        ? OperationStatus::PartialSuccess
        : rendered.status;
    result.value = std::move(rendered.value);
    return result;
}

OperationResult<QString> GGenerator::buildProgramText(const OperationContext& context) const
{
    OperationResult<QString> result;
    if (m_document == nullptr || m_profile == nullptr)
    {
        result.status = OperationStatus::InvalidInput;
        result.addDiagnostic(makeGeneratorDiagnostic
        (
            context,
            m_document == nullptr ? DiagnosticCode::MissingDocument : DiagnosticCode::MissingProfile,
            DiagnosticSeverity::Error,
            QStringLiteral("BuildProgramText"),
            QStringLiteral("ValidateInput"),
            m_document == nullptr
                ? QStringLiteral("未设置文档，无法生成 G 代码。")
                : QStringLiteral("未设置 G 代码配置，无法生成 G 代码。")
        ));
        return result;
    }
    if (m_generationMode == GenerationMode::Mode3D) return buildRotaryProgramText(context);

    int processableItemCount = 0;
    int missingProcessOrderCount = 0;
    for (const auto& entity : m_document->m_entities)
    {
        if (entity == nullptr || entity->m_excludedFromProcessing
            || entity->m_nativeEntity == nullptr || !isGCodeGeometryType(entity->m_type)) continue;
        ++processableItemCount;
        if (entity->m_processOrder < 0) ++missingProcessOrderCount;
    }
    if (processableItemCount == 0 || missingProcessOrderCount > 0)
    {
        result.status = OperationStatus::InvalidInput;
        result.addDiagnostic(makeGeneratorDiagnostic
        (
            context,
            processableItemCount == 0 ? DiagnosticCode::InvalidGeometry : DiagnosticCode::MissingProcessOrder,
            DiagnosticSeverity::Error,
            QStringLiteral("BuildProgramText"),
            QStringLiteral("ValidateProcessPlan"),
            processableItemCount == 0
                ? QStringLiteral("当前文档没有可生成 G 代码的图元。")
                : QStringLiteral("部分图元缺少加工顺序，无法生成 G 代码。"),
            QString(),
            {
                { QStringLiteral("expectedCount"), processableItemCount },
                { QStringLiteral("missingCount"), missingProcessOrderCount }
            }
        ));
        return result;
    }

    const QVector<CadItem*> orderedItems = collectOrderedItems(m_document);
    QString programText;
    QTextStream stream(&programText);
    stream.setEncoding(QStringConverter::Utf8);
    writeTextBlock(stream, m_profile->fileCode().header);

    int writtenGeometryCount = 0;
    bool partialSuccess = false;
    for (CadItem* item : orderedItems)
    {
        if (item == nullptr || item->m_nativeEntity == nullptr) continue;
        QString geometryText;
        QTextStream geometryStream(&geometryText);
        geometryStream.setEncoding(QStringConverter::Utf8);
        if (!writeItemGeometry(geometryStream, item))
        {
            item->rebuildRawPathPoints3D();
            partialSuccess = true;
            result.addDiagnostic(makeGeneratorDiagnostic
            (
                context,
                item->rawPathPoints3D().empty() ? DiagnosticCode::EmptyPath : DiagnosticCode::InvalidGeometry,
                DiagnosticSeverity::Warning,
                QStringLiteral("BuildProgramText"),
                QStringLiteral("SerializeGeometry"),
                QStringLiteral("一个图元未生成加工代码，已跳过。")
            ));
            continue;
        }
        ++writtenGeometryCount;

        QString rapidPrefix;
        QString cuttingBody;
        splitLeadingRapidMoves(geometryText, rapidPrefix, cuttingBody);
        if (!rapidPrefix.isEmpty()) stream << normalizeLineEndingsToCrLf(rapidPrefix);
        if (!cuttingBody.trimmed().isEmpty())
        {
            const GProfileCodeBlock typeCode = m_profile->entityTypeCode(entityTypeKey(item));
            const GProfileCodeBlock layerCode = m_profile->layerCode(entityLayerKey(item));
            const GProfileCodeBlock colorCode = m_profile->entityColorCode(entityColorKey(item));
            writeTextBlock(stream, layerCode.header);
            writeTextBlock(stream, colorCode.header);
            writeTextBlock(stream, typeCode.header);
            stream << normalizeLineEndingsToCrLf(cuttingBody);
            writeTextBlock(stream, typeCode.footer);
            writeTextBlock(stream, colorCode.footer);
            writeTextBlock(stream, layerCode.footer);
        }
    }
    writeTextBlock(stream, m_profile->fileCode().footer);
    stream.flush();

    if (writtenGeometryCount != processableItemCount)
    {
        partialSuccess = true;
        result.addDiagnostic(makeGeneratorDiagnostic
        (
            context,
            DiagnosticCode::InternalInvariantViolation,
            DiagnosticSeverity::Warning,
            QStringLiteral("BuildProgramText"),
            QStringLiteral("VerifyOutput"),
            QStringLiteral("部分可加工图元未产生 G 代码。")
        ));
    }
    const QString optimized = removeRedundantLaserRestartPairs2D(programText);
    if (optimized.isEmpty())
    {
        result.status = OperationStatus::Failed;
        result.addDiagnostic(makeGeneratorDiagnostic
        (
            context,
            DiagnosticCode::OutputVerificationFailure,
            DiagnosticSeverity::Error,
            QStringLiteral("BuildProgramText"),
            QStringLiteral("VerifyOutput"),
            QStringLiteral("生成的 G 代码为空。")
        ));
        return result;
    }
    result.status = partialSuccess ? OperationStatus::PartialSuccess : OperationStatus::Success;
    result.value = optimized;
    return result;
}

OperationReport GGenerator::writeProgramText
(
    const QString& filePath,
    const QString& program,
    const OperationContext& context
) const
{
    OperationReport report;

    if (filePath.trimmed().isEmpty())
    {
        report.status = OperationStatus::InvalidInput;
        report.addDiagnostic(makeGeneratorDiagnostic
        (
            context,
            DiagnosticCode::InvalidArgument,
            DiagnosticSeverity::Error,
            QStringLiteral("WriteProgramText"),
            QStringLiteral("ValidateInput"),
            QStringLiteral("G 代码输出路径为空。"),
            QStringLiteral("filePath is empty")
        ));
        return report;
    }

    if (program.isEmpty())
    {
        report.status = OperationStatus::InvalidInput;
        report.addDiagnostic(makeGeneratorDiagnostic
        (
            context,
            DiagnosticCode::OutputVerificationFailure,
            DiagnosticSeverity::Error,
            QStringLiteral("WriteProgramText"),
            QStringLiteral("ValidateInput"),
            QStringLiteral("不能写入空的 G 代码程序。"),
            QStringLiteral("program is empty"),
            { { QStringLiteral("filePath"), filePath } }
        ));
        return report;
    }

    QFile file(filePath);
    // CRLF is already explicit in program. Text mode would turn it into CRCRLF on Windows.
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate))
    {
        report.status = OperationStatus::Failed;
        report.addDiagnostic(makeGeneratorDiagnostic
        (
            context,
            DiagnosticCode::FileOpenFailure,
            DiagnosticSeverity::Error,
            QStringLiteral("WriteProgramText"),
            QStringLiteral("OpenOutputFile"),
            QStringLiteral("无法打开 G 代码输出文件。"),
            file.errorString(),
            { { QStringLiteral("filePath"), filePath } }
        ));
        return report;
    }

    const QByteArray encodedProgram = program.toUtf8();
    const qint64 writtenLength = file.write(encodedProgram);
    file.close();

    if (writtenLength != encodedProgram.size())
    {
        report.status = OperationStatus::Failed;
        report.addDiagnostic(makeGeneratorDiagnostic
        (
            context,
            DiagnosticCode::FileWriteFailure,
            DiagnosticSeverity::Error,
            QStringLiteral("WriteProgramText"),
            QStringLiteral("WriteOutputFile"),
            QStringLiteral("G 代码文件写入不完整。"),
            QStringLiteral("QFile::write returned an unexpected byte count"),
            {
                { QStringLiteral("filePath"), filePath },
                { QStringLiteral("expectedCount"), encodedProgram.size() },
                { QStringLiteral("actualCount"), writtenLength }
            }
        ));
        return report;
    }

    report.status = OperationStatus::Success;
    report.value = std::monostate{};
    return report;
}

bool GGenerator::generateToFile(const QString& filePath, QString* errorMessage) const
{
    const OperationContext context = createOperationContext(QStringLiteral("generate-gcode"));
    const OperationResult<QString> buildResult = buildProgramText(context);

    if (!buildResult.succeeded() || !buildResult.value.has_value())
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = legacyErrorMessage(buildResult.diagnostics);
        }
        return false;
    }

    const OperationReport writeResult = writeProgramText(filePath, *buildResult.value, context);
    if (!writeResult.succeeded())
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = legacyErrorMessage(writeResult.diagnostics);
        }
        return false;
    }

    if (errorMessage != nullptr)
    {
        errorMessage->clear();
    }
    return true;
}
