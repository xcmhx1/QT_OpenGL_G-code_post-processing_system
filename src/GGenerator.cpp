#include "pch.h"

#include "GGenerator.h"

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

#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QHash>
#include <QDir>
#include <QDebug>
#include <QMessageBox>
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

    QString formatAngle(double value)
    {
        return QString::number(value, 'f', 5);
    }

    QString formatDebugValue(double value)
    {
        return QString::number(value, 'f', 6);
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

    QString removeRedundantLaserRestartPairs(const QString& program)
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

    QString resolveInitialExportPath(QWidget* parent)
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

            if (parent != nullptr)
            {
                QMessageBox::warning
                (
                    parent,
                    QStringLiteral("导出目录不可用"),
                    QStringLiteral("上次导出的目录已不存在：\n%1\n\n将改用其它默认目录。").arg(lastDirectoryPath)
                );
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

    void writeRapidMove4Axis(QTextStream& stream, double x, double y, double z, double aDeg)
    {
        stream
            << "G00 X" << formatCoord(x)
            << " Y" << formatCoord(y)
            << " Z" << formatCoord(z)
            << " A" << formatAngle(aDeg)
            << "\r\n";
    }

    void writeLinearMove4Axis(QTextStream& stream, double x, double y, double z, double aDeg)
    {
        stream
            << "G01 X" << formatCoord(x)
            << " Y" << formatCoord(y)
            << " Z" << formatCoord(z)
            << " A" << formatAngle(aDeg)
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

    bool areControlPointsCoincident
    (
        const ControlPoint4Axis& left,
        const ControlPoint4Axis& right,
        double tolerance = kControlPointTolerance
    )
    {
        const double dx = left.x - right.x;
        const double dy = left.y - right.y;
        const double dz = left.z - right.z;
        const double da = left.aDeg - right.aDeg;
        return dx * dx + dy * dy + dz * dz + da * da <= tolerance * tolerance;
    }

    double rawPathPointDistance(const RawPathPoint3D& left, const RawPathPoint3D& right)
    {
        const double dx = left.x - right.x;
        const double dy = left.y - right.y;
        const double dz = left.z - right.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    struct RotaryCornerCenter
    {
        bool valid = false;
        double y = 0.0;
        double z = 0.0;
    };

    struct RotarySectionBounds
    {
        bool valid = false;
        double minY = 0.0;
        double maxY = 0.0;
        double minZ = 0.0;
        double maxZ = 0.0;
        RotaryCornerCenter topRightCorner;
        RotaryCornerCenter topLeftCorner;
        RotaryCornerCenter bottomRightCorner;
        RotaryCornerCenter bottomLeftCorner;
    };

    double rotarySectionExactSideTolerance(const RotarySectionBounds& bounds)
    {
        if (!bounds.valid)
        {
            return kControlPointTolerance;
        }

        const double spanY = bounds.maxY - bounds.minY;
        const double spanZ = bounds.maxZ - bounds.minZ;
        return std::max(kControlPointTolerance, std::max(spanY, spanZ) * 1.0e-6);
    }

    void inferRoundedCornerCenters
    (
        const std::vector<RawPathPoint3D>& points,
        RotarySectionBounds& bounds
    )
    {
        if (!bounds.valid || points.empty())
        {
            return;
        }

        const double tolerance = rotarySectionExactSideTolerance(bounds);
        const double inf = std::numeric_limits<double>::infinity();
        double topMinY = inf;
        double topMaxY = -inf;
        double bottomMinY = inf;
        double bottomMaxY = -inf;
        double rightMinZ = inf;
        double rightMaxZ = -inf;
        double leftMinZ = inf;
        double leftMaxZ = -inf;

        for (const RawPathPoint3D& point : points)
        {
            if (std::abs(point.z - bounds.maxZ) <= tolerance)
            {
                topMinY = std::min(topMinY, point.y);
                topMaxY = std::max(topMaxY, point.y);
            }

            if (std::abs(point.z - bounds.minZ) <= tolerance)
            {
                bottomMinY = std::min(bottomMinY, point.y);
                bottomMaxY = std::max(bottomMaxY, point.y);
            }

            if (std::abs(point.y - bounds.maxY) <= tolerance)
            {
                rightMinZ = std::min(rightMinZ, point.z);
                rightMaxZ = std::max(rightMaxZ, point.z);
            }

            if (std::abs(point.y - bounds.minY) <= tolerance)
            {
                leftMinZ = std::min(leftMinZ, point.z);
                leftMaxZ = std::max(leftMaxZ, point.z);
            }
        }

        const auto hasValue = [](double value)
            {
                return std::isfinite(value);
            };

        const auto makeCorner =
            [tolerance](double centerY, double centerZ, double outerY, double outerZ)
            {
                RotaryCornerCenter center;

                if (!std::isfinite(centerY) || !std::isfinite(centerZ))
                {
                    return center;
                }

                if (std::abs(outerY - centerY) <= tolerance
                    || std::abs(outerZ - centerZ) <= tolerance)
                {
                    return center;
                }

                center.valid = true;
                center.y = centerY;
                center.z = centerZ;
                return center;
            };

        if (hasValue(topMaxY) && hasValue(rightMaxZ))
        {
            bounds.topRightCorner = makeCorner(topMaxY, rightMaxZ, bounds.maxY, bounds.maxZ);
        }

        if (hasValue(topMinY) && hasValue(leftMaxZ))
        {
            bounds.topLeftCorner = makeCorner(topMinY, leftMaxZ, bounds.minY, bounds.maxZ);
        }

        if (hasValue(bottomMaxY) && hasValue(rightMinZ))
        {
            bounds.bottomRightCorner = makeCorner(bottomMaxY, rightMinZ, bounds.maxY, bounds.minZ);
        }

        if (hasValue(bottomMinY) && hasValue(leftMinZ))
        {
            bounds.bottomLeftCorner = makeCorner(bottomMinY, leftMinZ, bounds.minY, bounds.minZ);
        }
    }

    bool computeRotarySectionBounds(const QVector<CadItem*>& orderedItems, RotarySectionBounds& bounds)
    {
        bounds = {};
        std::vector<RawPathPoint3D> allRawPoints;

        for (CadItem* item : orderedItems)
        {
            if (item == nullptr || item->m_type == DRW::ETYPE::POINT)
            {
                continue;
            }

            item->rebuildRawPathPoints3D();

            for (const RawPathPoint3D& point : item->rawPathPoints3D())
            {
                allRawPoints.push_back(point);

                if (!bounds.valid)
                {
                    bounds.valid = true;
                    bounds.minY = bounds.maxY = point.y;
                    bounds.minZ = bounds.maxZ = point.z;
                    continue;
                }

                bounds.minY = std::min(bounds.minY, point.y);
                bounds.maxY = std::max(bounds.maxY, point.y);
                bounds.minZ = std::min(bounds.minZ, point.z);
                bounds.maxZ = std::max(bounds.maxZ, point.z);
            }
        }

        inferRoundedCornerCenters(allRawPoints, bounds);

        return bounds.valid;
    }

    std::vector<RotaryCornerToolCenter> buildRotaryCornerToolCenters(const RotarySectionBounds& bounds)
    {
        std::vector<RotaryCornerToolCenter> result;

        if (!bounds.valid)
        {
            return result;
        }

        const auto appendCorner =
            [&result]
            (
                const RotaryCornerCenter& center,
                double radiusY,
                double radiusZ,
                int yDirection,
                int zDirection
            )
            {
                if (!center.valid || radiusY <= 0.0 || radiusZ <= 0.0)
                {
                    return;
                }

                const double radius = 0.5 * (radiusY + radiusZ);
                const double radiusMismatch = std::abs(radiusY - radiusZ);
                const double radialTolerance = std::max(0.01, radius * 0.01);

                // 只有接近等半径的角部才作为方管圆角处理，避免误改普通空间曲线。
                if (radiusMismatch > radialTolerance)
                {
                    return;
                }

                result.push_back
                (
                    { center.y, center.z, radius, radialTolerance, yDirection, zDirection }
                );
            };

        appendCorner
        (
            bounds.topRightCorner,
            bounds.maxY - bounds.topRightCorner.y,
            bounds.maxZ - bounds.topRightCorner.z,
            1,
            1
        );
        appendCorner
        (
            bounds.topLeftCorner,
            bounds.topLeftCorner.y - bounds.minY,
            bounds.maxZ - bounds.topLeftCorner.z,
            -1,
            1
        );
        appendCorner
        (
            bounds.bottomRightCorner,
            bounds.maxY - bounds.bottomRightCorner.y,
            bounds.bottomRightCorner.z - bounds.minZ,
            1,
            -1
        );
        appendCorner
        (
            bounds.bottomLeftCorner,
            bounds.bottomLeftCorner.y - bounds.minY,
            bounds.bottomLeftCorner.z - bounds.minZ,
            -1,
            -1
        );

        return result;
    }

    bool hasInitialMachinePoint(const GProfileRotaryAxisConfig& config)
    {
        return config.useInitialMachinePoint;
    }

    double unwrapAngleNear(double previousDeg, double currentDeg)
    {
        while (currentDeg - previousDeg > 180.0)
        {
            currentDeg -= 360.0;
        }

        while (currentDeg - previousDeg < -180.0)
        {
            currentDeg += 360.0;
        }

        return currentDeg;
    }

    void alignControlPointsToPreviousA(std::vector<ControlPoint4Axis>& controlPoints, double previousADeg)
    {
        if (controlPoints.empty())
        {
            return;
        }

        const double alignedStartA = unwrapAngleNear(previousADeg, controlPoints.front().aDeg);
        const double shift = alignedStartA - controlPoints.front().aDeg;

        if (std::abs(shift) <= 1.0e-9)
        {
            return;
        }

        for (ControlPoint4Axis& point : controlPoints)
        {
            point.aDeg += shift;
        }
    }

    void applyMachiningPlaneZOffset(std::vector<ControlPoint4Axis>& controlPoints, double zOffset)
    {
        if (controlPoints.empty() || std::abs(zOffset) <= 1.0e-9)
        {
            return;
        }

        for (ControlPoint4Axis& point : controlPoints)
        {
            point.z += zOffset;
        }
    }

    struct RotaryExportContext
    {
        double axisY = 0.0;
        double axisZ = 0.0;
        double tubeCenterY = 0.0;
        double tubeCenterZ = 0.0;
        double safeMachineZ = 0.0;
        RotarySectionBounds sectionBounds;
        std::vector<RotaryCornerToolCenter> cornerToolCenters;
    };

    bool computeRotaryJudgeCenter
    (
        const QVector<CadItem*>& orderedItems,
        double& outJudgeCenterY,
        double& outJudgeCenterZ
    )
    {
        bool hasPoint = false;
        double minY = 0.0;
        double maxY = 0.0;
        double minZ = 0.0;
        double maxZ = 0.0;

        auto includePoint = [&](double y, double z)
            {
                if (!hasPoint)
                {
                    minY = maxY = y;
                    minZ = maxZ = z;
                    hasPoint = true;
                    return;
                }

                minY = std::min(minY, y);
                maxY = std::max(maxY, y);
                minZ = std::min(minZ, z);
                maxZ = std::max(maxZ, z);
            };

        for (CadItem* item : orderedItems)
        {
            if (item == nullptr || item->m_nativeEntity == nullptr)
            {
                continue;
            }

            item->rebuildRawPathPoints3D();

            const std::vector<RawPathPoint3D>& rawPathPoints = item->rawPathPoints3D();

            if (!rawPathPoints.empty())
            {
                for (const RawPathPoint3D& point : rawPathPoints)
                {
                    includePoint(point.y, point.z);
                }

                continue;
            }

            for (const QVector3D& vertex : item->m_geometry.vertices)
            {
                includePoint(vertex.y(), vertex.z());
            }
        }

        if (!hasPoint)
        {
            outJudgeCenterY = 0.0;
            outJudgeCenterZ = 0.0;
            return false;
        }

        outJudgeCenterY = 0.5 * (minY + maxY);
        outJudgeCenterZ = 0.5 * (minZ + maxZ);
        return true;
    }

    void writeCommentLine(QTextStream& stream, const QString& comment)
    {
        if (!comment.trimmed().isEmpty())
        {
            stream << "(" << comment << ")\r\n";
        }
    }

    double computeSafeMachineZFromTubeCenter
    (
        const QVector<CadItem*>& orderedItems,
        double tubeCenterY,
        double tubeCenterZ,
        double extraRadialClearance,
        double* outMaxCollisionRadius = nullptr
    )
    {
        double maxCollisionRadius = 0.0;

        for (CadItem* item : orderedItems)
        {
            if (item == nullptr || item->m_nativeEntity == nullptr)
            {
                continue;
            }

            item->rebuildRawPathPoints3D();

            for (const RawPathPoint3D& point : item->rawPathPoints3D())
            {
                const double dy = point.y - tubeCenterY;
                const double dz = point.z - tubeCenterZ;
                const double radius = std::sqrt(dy * dy + dz * dz);
                maxCollisionRadius = std::max(maxCollisionRadius, radius);
            }
        }

        if (outMaxCollisionRadius != nullptr)
        {
            *outMaxCollisionRadius = maxCollisionRadius;
        }

        return tubeCenterZ + maxCollisionRadius + extraRadialClearance;
    }

    ControlPoint4Axis machineSafeApproachPoint
    (
        const ControlPoint4Axis& point,
        double safeMachineZ
    )
    {
        ControlPoint4Axis safePoint = point;
        safePoint.z = std::max(point.z, safeMachineZ);
        return safePoint;
    }

    bool writeControlPointPath4Axis
    (
        QTextStream& stream,
        const std::vector<ControlPoint4Axis>& controlPoints,
        const GProfileRotaryAxisConfig& config,
        double safeMachineZ,
        const ControlPoint4Axis* previousEndPoint,
        bool pathDirectlyContinuous,
        bool needsCuttingConnectionMove,
        ControlPoint4Axis* writtenEndPoint
    )
    {
        if (controlPoints.empty())
        {
            return false;
        }

        const ControlPoint4Axis& firstPoint = controlPoints.front();
        const bool hasPreviousEndPoint = previousEndPoint != nullptr;

        if (!hasPreviousEndPoint)
        {
            if (hasInitialMachinePoint(config))
            {
                ControlPoint4Axis initialPoint;
                initialPoint.x = config.initialMachineX;
                initialPoint.y = config.initialMachineY;
                initialPoint.z = config.initialMachineZ;
                initialPoint.aDeg = firstPoint.aDeg;

                if (!areControlPointsCoincident(initialPoint, firstPoint))
                {
                    writeRapidMove4Axis(stream, initialPoint.x, initialPoint.y, initialPoint.z, initialPoint.aDeg);
                }
            }

            if (safeMachineZ > firstPoint.z + kControlPointTolerance)
            {
                const ControlPoint4Axis approachPoint = machineSafeApproachPoint(firstPoint, safeMachineZ);
                writeRapidMove4Axis(stream, approachPoint.x, approachPoint.y, approachPoint.z, approachPoint.aDeg);

                if (!areControlPointsCoincident(approachPoint, firstPoint))
                {
                    writeRapidMove4Axis(stream, firstPoint.x, firstPoint.y, firstPoint.z, firstPoint.aDeg);
                }
            }
            else
            {
                writeRapidMove4Axis(stream, firstPoint.x, firstPoint.y, firstPoint.z, firstPoint.aDeg);
            }
        }
        else if (!pathDirectlyContinuous)
        {
            if (safeMachineZ > std::min(previousEndPoint->z, firstPoint.z) + kControlPointTolerance)
            {
                const ControlPoint4Axis departureSafePoint = machineSafeApproachPoint(*previousEndPoint, safeMachineZ);
                const ControlPoint4Axis approachPoint = machineSafeApproachPoint(firstPoint, safeMachineZ);

                if (!areControlPointsCoincident(*previousEndPoint, departureSafePoint))
                {
                    writeRapidMove4Axis
                    (
                        stream,
                        departureSafePoint.x,
                        departureSafePoint.y,
                        departureSafePoint.z,
                        departureSafePoint.aDeg
                    );
                }

                if (!areControlPointsCoincident(departureSafePoint, approachPoint))
                {
                    writeRapidMove4Axis(stream, approachPoint.x, approachPoint.y, approachPoint.z, approachPoint.aDeg);
                }

                if (!areControlPointsCoincident(approachPoint, firstPoint))
                {
                    writeRapidMove4Axis(stream, firstPoint.x, firstPoint.y, firstPoint.z, firstPoint.aDeg);
                }
            }
            else
            {
                writeRapidMove4Axis(stream, firstPoint.x, firstPoint.y, firstPoint.z, firstPoint.aDeg);
            }
        }
        else if (needsCuttingConnectionMove
            && !areControlPointsCoincident(*previousEndPoint, firstPoint))
        {
            writeLinearMove4Axis(stream, firstPoint.x, firstPoint.y, firstPoint.z, firstPoint.aDeg);
        }

        for (size_t index = 1; index < controlPoints.size(); ++index)
        {
            const ControlPoint4Axis& point = controlPoints[index];
            writeLinearMove4Axis(stream, point.x, point.y, point.z, point.aDeg);
        }

        if (writtenEndPoint != nullptr)
        {
            *writtenEndPoint = controlPoints.back();
        }

        return true;
    }

    bool writeItemGeometry4Axis
    (
        QTextStream& stream,
        const CadItem* item,
        const GProfileRotaryAxisConfig& config,
        const RotaryExportContext& exportContext,
        const ControlPoint4Axis* previousEndPoint,
        const RawPathPoint3D* previousRawEndPoint,
        ControlPoint4Axis* writtenEndPoint,
        RawPathPoint3D* writtenRawEndPoint,
        QString* errorMessage
    )
    {
        if (item == nullptr)
        {
            if (errorMessage != nullptr)
            {
                *errorMessage = QStringLiteral("空图元无法生成 4 轴 G 代码。");
            }

            return false;
        }

        if (item->m_type == DRW::ETYPE::POINT)
        {
            if (errorMessage != nullptr)
            {
                errorMessage->clear();
            }

            return false;
        }

        CadItem* writableItem = const_cast<CadItem*>(item);
        const bool rebuilt = writableItem->rebuildControlPoints4Axis
        (
            exportContext.axisY,
            exportContext.axisZ,
            exportContext.tubeCenterY,
            exportContext.tubeCenterZ,
            config.invertAAxisDirection,
            config.aAxisOffsetDegrees,
            config.keepContinuousAngle,
            errorMessage
        );

        if (!rebuilt)
        {
            return false;
        }

        std::vector<ControlPoint4Axis>& controlPoints = writableItem->controlPoints4AxisMutable();
        const std::vector<RawPathPoint3D>& rawPathPoints = writableItem->rawPathPoints3D();

        if (rawPathPoints.empty())
        {
            return false;
        }

        writableItem->applyRoundedCornerToolOrientation
        (
            exportContext.cornerToolCenters,
            exportContext.axisY,
            exportContext.axisZ,
            config.invertAAxisDirection,
            config.aAxisOffsetDegrees,
            config.keepContinuousAngle
        );

        applyMachiningPlaneZOffset(controlPoints, config.machiningPlaneZOffset);

        if (previousEndPoint != nullptr)
        {
            alignControlPointsToPreviousA(controlPoints, previousEndPoint->aDeg);
        }

        const double rawConnectionDistance = previousRawEndPoint != nullptr
            ? rawPathPointDistance(*previousRawEndPoint, rawPathPoints.front())
            : std::numeric_limits<double>::max();
        const bool pathDirectlyContinuous = previousRawEndPoint != nullptr
            && rawConnectionDistance <= kNoLiftPathConnectionTolerance;
        const bool needsCuttingConnectionMove = pathDirectlyContinuous
            && rawConnectionDistance > kControlPointTolerance;

        const bool written = writeControlPointPath4Axis
        (
            stream,
            controlPoints,
            config,
            exportContext.safeMachineZ,
            previousEndPoint,
            pathDirectlyContinuous,
            needsCuttingConnectionMove,
            writtenEndPoint
        );

        if (written && writtenRawEndPoint != nullptr)
        {
            *writtenRawEndPoint = rawPathPoints.back();
        }

        return written;
    }

    struct RotaryOvercutPath
    {
        std::vector<RawPathPoint3D> rawPoints;
        std::vector<ControlPoint4Axis> controlPoints;
        int itemCount = 0;
        bool directlyClosed = false;
        bool valid = true;
    };

    bool isDirectlyClosedItem(const CadItem* item)
    {
        if (item == nullptr || item->m_nativeEntity == nullptr)
        {
            return false;
        }

        switch (item->m_type)
        {
        case DRW::ETYPE::CIRCLE:
            return true;
        case DRW::ETYPE::ELLIPSE:
            return isFullEllipsePath(static_cast<const DRW_Ellipse*>(item->m_nativeEntity));
        case DRW::ETYPE::POLYLINE:
            return (static_cast<const DRW_Polyline*>(item->m_nativeEntity)->flags & 1) != 0;
        case DRW::ETYPE::LWPOLYLINE:
            return (static_cast<const DRW_LWPolyline*>(item->m_nativeEntity)->flags & 1) != 0;
        default:
            return false;
        }
    }

    void appendItemToOvercutPath(const CadItem* item, RotaryOvercutPath& path)
    {
        if (item == nullptr)
        {
            path.valid = false;
            return;
        }

        const std::vector<RawPathPoint3D>& rawPoints = item->rawPathPoints3D();
        const std::vector<ControlPoint4Axis>& controlPoints = item->controlPoints4Axis();
        if (rawPoints.empty() || rawPoints.size() != controlPoints.size())
        {
            path.valid = false;
            return;
        }

        ++path.itemCount;
        path.rawPoints.insert(path.rawPoints.end(), rawPoints.cbegin(), rawPoints.cend());
        path.controlPoints.insert(path.controlPoints.end(), controlPoints.cbegin(), controlPoints.cend());
        path.directlyClosed = path.itemCount == 1 && isDirectlyClosedItem(item);
    }

    ControlPoint4Axis interpolateControlPoint
    (
        const ControlPoint4Axis& start,
        const ControlPoint4Axis& end,
        double ratio
    )
    {
        return
        {
            start.x + (end.x - start.x) * ratio,
            start.y + (end.y - start.y) * ratio,
            start.z + (end.z - start.z) * ratio,
            start.aDeg + (end.aDeg - start.aDeg) * ratio
        };
    }

    RawPathPoint3D interpolateRawPathPoint
    (
        const RawPathPoint3D& start,
        const RawPathPoint3D& end,
        double ratio
    )
    {
        return
        {
            start.x + (end.x - start.x) * ratio,
            start.y + (end.y - start.y) * ratio,
            start.z + (end.z - start.z) * ratio
        };
    }

    bool writeRotaryOvercut
    (
        QTextStream& stream,
        const RotaryOvercutPath& path,
        double requestedDistance,
        const ControlPoint4Axis& currentEndPoint,
        ControlPoint4Axis& writtenEndPoint,
        RawPathPoint3D& writtenRawEndPoint
    )
    {
        if (requestedDistance <= kControlPointTolerance
            || !path.valid
            || path.rawPoints.size() < 2
            || path.rawPoints.size() != path.controlPoints.size())
        {
            return false;
        }

        const bool closed = path.directlyClosed
            || rawPathPointDistance(path.rawPoints.back(), path.rawPoints.front())
                <= kNoLiftPathConnectionTolerance;
        if (!closed)
        {
            return false;
        }

        double totalLength = 0.0;
        for (size_t index = 1; index < path.rawPoints.size(); ++index)
        {
            totalLength += rawPathPointDistance(path.rawPoints[index - 1], path.rawPoints[index]);
        }

        if (totalLength <= kControlPointTolerance)
        {
            return false;
        }

        const double overcutDistance = std::min(requestedDistance, totalLength);
        if (requestedDistance > totalLength + kControlPointTolerance)
        {
            qWarning().noquote() << QStringLiteral
            (
                "[四轴过切] 请求 %1 mm 超过闭环总长 %2 mm，已限制为一圈。"
            )
                .arg(requestedDistance, 0, 'f', 3)
                .arg(totalLength, 0, 'f', 3);
        }

        std::vector<ControlPoint4Axis> alignedControlPoints = path.controlPoints;
        alignControlPointsToPreviousA(alignedControlPoints, currentEndPoint.aDeg);

        ControlPoint4Axis currentControlPoint = currentEndPoint;
        RawPathPoint3D currentRawPoint = path.rawPoints.front();
        const ControlPoint4Axis& exactStart = alignedControlPoints.front();
        if (!areControlPointsCoincident(currentControlPoint, exactStart))
        {
            writeLinearMove4Axis(stream, exactStart.x, exactStart.y, exactStart.z, exactStart.aDeg);
        }
        currentControlPoint = exactStart;

        double remainingDistance = overcutDistance;
        for (size_t index = 1; index < path.rawPoints.size() && remainingDistance > kControlPointTolerance; ++index)
        {
            const RawPathPoint3D& rawStart = path.rawPoints[index - 1];
            const RawPathPoint3D& rawEnd = path.rawPoints[index];
            const double segmentLength = rawPathPointDistance(rawStart, rawEnd);
            if (segmentLength <= kControlPointTolerance)
            {
                continue;
            }

            if (remainingDistance >= segmentLength - kControlPointTolerance)
            {
                const ControlPoint4Axis& segmentEnd = alignedControlPoints[index];
                writeLinearMove4Axis(stream, segmentEnd.x, segmentEnd.y, segmentEnd.z, segmentEnd.aDeg);
                currentControlPoint = segmentEnd;
                currentRawPoint = rawEnd;
                remainingDistance -= segmentLength;
                continue;
            }

            const double ratio = remainingDistance / segmentLength;
            currentControlPoint = interpolateControlPoint
            (
                alignedControlPoints[index - 1],
                alignedControlPoints[index],
                ratio
            );
            currentRawPoint = interpolateRawPathPoint(rawStart, rawEnd, ratio);
            writeLinearMove4Axis
            (
                stream,
                currentControlPoint.x,
                currentControlPoint.y,
                currentControlPoint.z,
                currentControlPoint.aDeg
            );
            remainingDistance = 0.0;
        }

        writtenEndPoint = currentControlPoint;
        writtenRawEndPoint = currentRawPoint;
        return true;
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

bool GGenerator::generateToFile(const QString& filePath, QString* errorMessage) const
{
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

    // Line endings are emitted explicitly as CRLF. Text mode would translate
    // the '\n' again on Windows and produce CRCRLF, which appears as blank lines.
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate))
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("无法写入 G 代码文件: %1").arg(filePath);
        }

        return false;
    }

    QString programText;
    QTextStream stream(&programText);
    stream.setEncoding(QStringConverter::Utf8);

    writeTextBlock(stream, m_profile->fileCode().header);

    const QVector<CadItem*> orderedItems = collectOrderedItems(m_document);
    const GProfileRotaryAxisConfig& rotaryAxisConfig = m_profile->rotaryAxisConfig();
    RotaryExportContext rotaryExportContext;
    rotaryExportContext.axisY = rotaryAxisConfig.centerY;
    rotaryExportContext.axisZ = rotaryAxisConfig.centerZ;

    if (m_generationMode == GenerationMode::Mode3D)
    {
        computeRotarySectionBounds(orderedItems, rotaryExportContext.sectionBounds);
        rotaryExportContext.cornerToolCenters =
            buildRotaryCornerToolCenters(rotaryExportContext.sectionBounds);
    }

    if (m_rotaryTubeCenterValid)
    {
        rotaryExportContext.tubeCenterY = m_rotaryTubeCenterY;
        rotaryExportContext.tubeCenterZ = m_rotaryTubeCenterZ;
    }
    else if (rotaryExportContext.sectionBounds.valid)
    {
        rotaryExportContext.tubeCenterY = 0.5
            * (rotaryExportContext.sectionBounds.minY + rotaryExportContext.sectionBounds.maxY);
        rotaryExportContext.tubeCenterZ = 0.5
            * (rotaryExportContext.sectionBounds.minZ + rotaryExportContext.sectionBounds.maxZ);
    }
    else
    {
        computeRotaryJudgeCenter
        (
            orderedItems,
            rotaryExportContext.tubeCenterY,
            rotaryExportContext.tubeCenterZ
        );
    }

    double maxCollisionRadius = 0.0;
    rotaryExportContext.safeMachineZ = computeSafeMachineZFromTubeCenter
    (
        orderedItems,
        rotaryExportContext.tubeCenterY,
        rotaryExportContext.tubeCenterZ,
        rotaryAxisConfig.safeZ,
        &maxCollisionRadius
    );

    if (m_generationMode == GenerationMode::Mode3D)
    {
        writeCommentLine(stream, QStringLiteral("TUBE CENTER Y: %1").arg(formatDebugValue(rotaryExportContext.tubeCenterY)));
        writeCommentLine(stream, QStringLiteral("TUBE CENTER Z: %1").arg(formatDebugValue(rotaryExportContext.tubeCenterZ)));
        writeCommentLine(stream, QStringLiteral("ROTARY AXIS Y: %1").arg(formatDebugValue(rotaryExportContext.axisY)));
        writeCommentLine(stream, QStringLiteral("ROTARY AXIS Z: %1").arg(formatDebugValue(rotaryExportContext.axisZ)));
        writeCommentLine(stream, QStringLiteral("MAX COLLISION RADIUS: %1").arg(formatDebugValue(maxCollisionRadius)));
        writeCommentLine(stream, QStringLiteral("FINAL SAFE MACHINE Z: %1").arg(formatDebugValue(rotaryExportContext.safeMachineZ)));

        if (rotaryExportContext.sectionBounds.valid)
        {
            writeCommentLine
            (
                stream,
                QStringLiteral("SQUARE TUBE SECTION Y: %1 -> %2")
                    .arg(formatDebugValue(rotaryExportContext.sectionBounds.minY))
                    .arg(formatDebugValue(rotaryExportContext.sectionBounds.maxY))
            );
            writeCommentLine
            (
                stream,
                QStringLiteral("SQUARE TUBE SECTION Z: %1 -> %2")
                    .arg(formatDebugValue(rotaryExportContext.sectionBounds.minZ))
                    .arg(formatDebugValue(rotaryExportContext.sectionBounds.maxZ))
            );

            const auto writeCornerCenterComment =
                [&stream](const QString& name, const RotaryCornerCenter& center)
                {
                    if (!center.valid)
                    {
                        return;
                    }

                    writeCommentLine
                    (
                        stream,
                        QStringLiteral("SQUARE TUBE %1 CORNER CENTER Y/Z: %2, %3")
                            .arg(name)
                            .arg(formatDebugValue(center.y))
                            .arg(formatDebugValue(center.z))
                    );
                };

            writeCornerCenterComment(QStringLiteral("TOP RIGHT"), rotaryExportContext.sectionBounds.topRightCorner);
            writeCornerCenterComment(QStringLiteral("TOP LEFT"), rotaryExportContext.sectionBounds.topLeftCorner);
            writeCornerCenterComment(QStringLiteral("BOTTOM RIGHT"), rotaryExportContext.sectionBounds.bottomRightCorner);
            writeCornerCenterComment(QStringLiteral("BOTTOM LEFT"), rotaryExportContext.sectionBounds.bottomLeftCorner);
        }
    }

    qDebug().noquote()
        << QStringLiteral("[RotarySafeCenter] tubeCenterY=%1 tubeCenterZ=%2 rotaryAxisY=%3 rotaryAxisZ=%4 maxCollisionRadius=%5 finalSafeMachineZ=%6")
            .arg(formatDebugValue(rotaryExportContext.tubeCenterY))
            .arg(formatDebugValue(rotaryExportContext.tubeCenterZ))
            .arg(formatDebugValue(rotaryExportContext.axisY))
            .arg(formatDebugValue(rotaryExportContext.axisZ))
            .arg(formatDebugValue(maxCollisionRadius))
            .arg(formatDebugValue(rotaryExportContext.safeMachineZ));

    bool hasPrevious4AxisEndPoint = false;
    ControlPoint4Axis previous4AxisEndPoint;
    RawPathPoint3D previous4AxisRawEndPoint;
    QHash<int, int> remainingItemsByContinuousGroup;
    QHash<int, RotaryOvercutPath> overcutPathsByGroup;

    if (m_generationMode == GenerationMode::Mode3D)
    {
        for (CadItem* item : orderedItems)
        {
            if (item != nullptr && item->m_processContinuousGroupId >= 0)
            {
                remainingItemsByContinuousGroup[item->m_processContinuousGroupId] += 1;
            }
        }
    }

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

        QString geometryError;
        ControlPoint4Axis currentItemEndPoint;
        RawPathPoint3D currentItemRawEndPoint;
        const bool geometryWritten = m_generationMode == GenerationMode::Mode3D
            ? writeItemGeometry4Axis
            (
                geometryStream,
                item,
                rotaryAxisConfig,
                rotaryExportContext,
                hasPrevious4AxisEndPoint ? &previous4AxisEndPoint : nullptr,
                hasPrevious4AxisEndPoint ? &previous4AxisRawEndPoint : nullptr,
                &currentItemEndPoint,
                &currentItemRawEndPoint,
                &geometryError
            )
            : writeItemGeometry(geometryStream, item);

        if (!geometryWritten)
        {
            if (m_generationMode == GenerationMode::Mode3D
                && !geometryError.trimmed().isEmpty()
                && item->m_type != DRW::ETYPE::POINT)
            {
                if (errorMessage != nullptr)
                {
                    *errorMessage = geometryError;
                }

                return false;
            }

            continue;
        }

        bool completesContinuousGroup = false;
        const int continuousGroupId = item->m_processContinuousGroupId;
        if (m_generationMode == GenerationMode::Mode3D && continuousGroupId >= 0)
        {
            appendItemToOvercutPath(item, overcutPathsByGroup[continuousGroupId]);
            int& remainingCount = remainingItemsByContinuousGroup[continuousGroupId];
            remainingCount = std::max(0, remainingCount - 1);
            completesContinuousGroup = remainingCount == 0;
        }

        if (!geometryText.isEmpty())
        {
            QString rapidPrefix;
            QString cuttingBody;
            splitLeadingRapidMoves(geometryText, rapidPrefix, cuttingBody);

            if (!rapidPrefix.isEmpty())
            {
                stream << normalizeLineEndingsToCrLf(rapidPrefix);
            }

            if (!cuttingBody.trimmed().isEmpty())
            {
                writeTextBlock(stream, layerCode.header);
                writeTextBlock(stream, colorCode.header);
                writeTextBlock(stream, typeCode.header);
                stream << normalizeLineEndingsToCrLf(cuttingBody);

                if (completesContinuousGroup
                    && rotaryAxisConfig.overcutDistance > kControlPointTolerance)
                {
                    ControlPoint4Axis overcutEndPoint;
                    RawPathPoint3D overcutRawEndPoint;
                    if (writeRotaryOvercut
                    (
                        stream,
                        overcutPathsByGroup.value(continuousGroupId),
                        rotaryAxisConfig.overcutDistance,
                        currentItemEndPoint,
                        overcutEndPoint,
                        overcutRawEndPoint
                    ))
                    {
                        currentItemEndPoint = overcutEndPoint;
                        currentItemRawEndPoint = overcutRawEndPoint;
                    }
                }

                writeTextBlock(stream, typeCode.footer);
                writeTextBlock(stream, colorCode.footer);
                writeTextBlock(stream, layerCode.footer);
            }
        }

        if (m_generationMode == GenerationMode::Mode3D)
        {
            previous4AxisEndPoint = currentItemEndPoint;
            previous4AxisRawEndPoint = currentItemRawEndPoint;
            hasPrevious4AxisEndPoint = true;
        }
    }

    writeTextBlock(stream, m_profile->fileCode().footer);
    stream.flush();

    const QByteArray encodedProgram = removeRedundantLaserRestartPairs(programText).toUtf8();
    if (file.write(encodedProgram) != encodedProgram.size())
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("写入 G 代码文件失败: %1").arg(filePath);
        }

        file.close();
        return false;
    }

    file.close();

    if (errorMessage != nullptr)
    {
        errorMessage->clear();
    }

    return true;
}
