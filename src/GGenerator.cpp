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
#include "GCodeGenerator4Axis.h"
#include "Rotary4AxisPlanner.h"

#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QStringList>
#include <QTextStream>
#include <QVector3D>
#include <QWidget>

#include <algorithm>
#include <cmath>
#include <numeric>
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

    RotaryConfig buildRotaryConfig(const GProfileRotaryAxisConfig& profileConfig)
    {
        RotaryConfig rotary;
        rotary.yCenter = profileConfig.centerY;
        rotary.zCenter = profileConfig.centerZ;
        rotary.aOffsetDeg = profileConfig.aAxisOffsetDegrees;
        rotary.aPositiveCCW = !profileConfig.invertAAxisDirection;
        rotary.targetNormalYZ = QVector2D(0.0f, 1.0f);
        return rotary;
    }

    MachineConfig4Axis buildMachineConfig(const GProfileRotaryAxisConfig& profileConfig)
    {
        MachineConfig4Axis machine;
        machine.safeZ = profileConfig.safeZ;
        machine.useSafeZBeforeRapid = profileConfig.useSafeZBeforeRapid;
        machine.emitProgramPreamble = false;
        machine.emitProgramEnd = false;
        return machine;
    }

    bool isClosedItemGeometry(const CadItem* item)
    {
        if (item == nullptr)
        {
            return false;
        }

        switch (item->m_type)
        {
        case DRW::ETYPE::CIRCLE:
            return true;
        case DRW::ETYPE::ELLIPSE:
        {
            const CadEllipseItem* ellipseItem = static_cast<const CadEllipseItem*>(item);
            return ellipseItem != nullptr && isFullEllipsePath(ellipseItem->m_data);
        }
        case DRW::ETYPE::POLYLINE:
        {
            const CadPolylineItem* polylineItem = static_cast<const CadPolylineItem*>(item);
            return polylineItem != nullptr && polylineItem->m_data != nullptr && ((polylineItem->m_data->flags & 1) != 0);
        }
        case DRW::ETYPE::LWPOLYLINE:
        {
            const CadLWPolylineItem* lwpolylineItem = static_cast<const CadLWPolylineItem*>(item);
            return lwpolylineItem != nullptr && lwpolylineItem->m_data != nullptr && ((lwpolylineItem->m_data->flags & 1) != 0);
        }
        default:
            return false;
        }
    }

    bool canReverseEntityForJoin(const CadItem* item, const ProcessTolerance& tolerance)
    {
        return tolerance.allowReverseEntityForJoin
            && item != nullptr
            && !item->m_hasCustomProcessStart
            && !isClosedItemGeometry(item);
    }

    QString buildProcessKey(const CadItem* item)
    {
        return QStringLiteral("%1|%2|%3")
            .arg(entityTypeKey(item))
            .arg(entityLayerKey(item))
            .arg(entityColorKey(item));
    }

    QString processModeToString(ProcessMode mode)
    {
        switch (mode)
        {
        case ProcessMode::XYZ_3Axis:
            return QStringLiteral("XYZ_3Axis");
        case ProcessMode::A_Indexed_XYZ:
            return QStringLiteral("A_Indexed_XYZ");
        case ProcessMode::XYZA_Continuous:
            return QStringLiteral("XYZA_Continuous");
        default:
            return QStringLiteral("Unknown");
        }
    }

    QVector<QVector3D> buildPathPointsForItem(const CadItem* item, bool reverseDirection)
    {
        QVector<QVector3D> points;

        if (item == nullptr)
        {
            return points;
        }

        points.reserve(item->m_geometry.vertices.size());

        for (const QVector3D& vertex : item->m_geometry.vertices)
        {
            if (!points.isEmpty())
            {
                const QVector3D delta = vertex - points.back();

                if (delta.lengthSquared() <= 1.0e-12f)
                {
                    continue;
                }
            }

            points.append(vertex);
        }

        if (reverseDirection && points.size() >= 2)
        {
            std::reverse(points.begin(), points.end());
        }

        return points;
    }

    bool usesRoundedRectSection(const GProfileRotaryAxisConfig& profileConfig)
    {
        const QString sectionType = profileConfig.sectionType.trimmed().toLower();
        return sectionType == QStringLiteral("rounded_rect")
            || sectionType == QStringLiteral("rounded-rect")
            || sectionType == QStringLiteral("roundedrect")
            || sectionType == QStringLiteral("ellipse_corner_rect")
            || sectionType == QStringLiteral("ellipse-corner-rect")
            || sectionType == QStringLiteral("ellipsecornerrect")
            || sectionType == QStringLiteral("rounded_rectangle");
    }

    QVector<SamplePoint> buildRadialSamples
    (
        const QVector<QVector3D>& pathPoints,
        const RotaryConfig& rotaryConfig
    )
    {
        QVector<SamplePoint> samples;
        samples.reserve(pathPoints.size());

        for (const QVector3D& point : pathPoints)
        {
            const double relativeY = static_cast<double>(point.y()) - rotaryConfig.yCenter;
            const double relativeZ = static_cast<double>(point.z()) - rotaryConfig.zCenter;
            const double radius = std::hypot(relativeY, relativeZ);

            SamplePoint sample;
            sample.pos = point;
            sample.hasRawA = false;
            sample.rawA = 0.0;
            sample.hasSnappedA = false;
            sample.snappedA = 0.0;
            sample.regionType = SectionRegionType::Unknown;
            sample.sideIndex = -1;
            sample.hasTargetA = false;
            sample.targetA = 0.0;

            if (radius > 1.0e-12)
            {
                sample.normal = QVector3D
                (
                    0.0f,
                    static_cast<float>(relativeY / radius),
                    static_cast<float>(relativeZ / radius)
                );
                sample.hasNormal = true;
            }

            samples.append(sample);
        }

        return samples;
    }

    QVector<QVector3D> densifyPathPointsLinear(const QVector<QVector3D>& pathPoints, double maxLinearStep)
    {
        QVector<QVector3D> densePoints;

        if (pathPoints.isEmpty())
        {
            return densePoints;
        }

        const double safeMaxStep = std::max(maxLinearStep, 1.0e-6);
        densePoints.reserve(pathPoints.size() * 2);
        densePoints.append(pathPoints.front());

        for (int index = 1; index < pathPoints.size(); ++index)
        {
            const QVector3D startPoint = pathPoints.at(index - 1);
            const QVector3D endPoint = pathPoints.at(index);
            const double segmentLength = static_cast<double>((endPoint - startPoint).length());
            const int stepCount = std::max(1, static_cast<int>(std::ceil(segmentLength / safeMaxStep)));

            for (int step = 1; step <= stepCount; ++step)
            {
                const double t = static_cast<double>(step) / static_cast<double>(stepCount);
                densePoints.append
                (
                    QVector3D
                    (
                        static_cast<float>(startPoint.x() + (endPoint.x() - startPoint.x()) * t),
                        static_cast<float>(startPoint.y() + (endPoint.y() - startPoint.y()) * t),
                        static_cast<float>(startPoint.z() + (endPoint.z() - startPoint.z()) * t)
                    )
                );
            }
        }

        return densePoints;
    }

    QString sampleRegionDebugString(const SamplePoint& sample)
    {
        QString regionLabel = QStringLiteral("Unknown");

        if (sample.regionType == SectionRegionType::FlatSide)
        {
            regionLabel = sample.sideIndex >= 0
                ? QStringLiteral("FlatSide(side=%1)").arg(sample.sideIndex)
                : QStringLiteral("FlatSide");
        }
        else if (sample.regionType == SectionRegionType::CornerTransition)
        {
            regionLabel = QStringLiteral("CornerTransition");
        }

        const QString rawAText = sample.hasRawA
            ? QString::number(sample.rawA, 'f', 3)
            : QStringLiteral("NA");
        const QString snappedAText = sample.hasSnappedA
            ? QString::number(sample.snappedA, 'f', 3)
            : QStringLiteral("NA");

        return QStringLiteral("%1,rawA=%2,snapA=%3")
            .arg(regionLabel)
            .arg(rawAText)
            .arg(snappedAText);
    }

    QString buildLineSampleDebug(const QVector<SamplePoint>& samples)
    {
        QStringList parts;
        parts.reserve(samples.size());

        for (const SamplePoint& sample : samples)
        {
            parts.append(sampleRegionDebugString(sample));
        }

        return QStringLiteral("[%1]").arg(parts.join(QStringLiteral("; ")));
    }

    void resetExportDebugState(CadItem* item)
    {
        if (item == nullptr)
        {
            return;
        }

        item->m_exportPathPoints.clear();
        item->m_hasExportPathPoints = false;
        item->m_exportDirectionReversed = false;
        item->m_exportProcessMode.clear();
        item->m_exportRequiresA = false;
        item->m_exportAStartDeg = 0.0;
        item->m_exportAEndDeg = 0.0;
        item->m_exportARangeDeg = 0.0;
        item->m_exportRegionSummary.clear();
    }

    QVector<QVector3D> buildExportPathFromSegments(const QVector<ToolpathSegment4Axis>& segments)
    {
        QVector<QVector3D> pathPoints;

        for (const ToolpathSegment4Axis& segment : segments)
        {
            for (int pointIndex = 0; pointIndex < segment.points.size(); ++pointIndex)
            {
                const QVector3D localPoint = segment.points.at(pointIndex).localPos;

                if (!pathPoints.isEmpty() && (pathPoints.back() - localPoint).lengthSquared() <= 1.0e-12f)
                {
                    continue;
                }

                pathPoints.append(localPoint);
            }
        }

        return pathPoints;
    }

    QString summarizeProcessModeFromSegments(const QVector<ToolpathSegment4Axis>& segments)
    {
        bool hasAContinuous = false;
        bool hasARequired = false;

        for (const ToolpathSegment4Axis& segment : segments)
        {
            hasAContinuous = hasAContinuous || segment.mode == SegmentMotionMode::AContinuous;
            hasARequired = hasARequired || segment.requiresA;
        }

        if (!hasARequired)
        {
            return QStringLiteral("XYZ_3Axis");
        }

        return hasAContinuous ? QStringLiteral("XYZA_Continuous") : QStringLiteral("A_Indexed_XYZ");
    }

    QString summarizeRegionFromSegments(const QVector<ToolpathSegment4Axis>& segments)
    {
        bool hasFlat = false;
        bool hasCorner = false;
        bool hasUnknown = false;
        bool hasMixed = false;
        int firstSide = -1;
        bool sameSide = true;

        for (const ToolpathSegment4Axis& segment : segments)
        {
            const QString summary = segment.regionSummary.trimmed();

            if (summary.startsWith(QStringLiteral("Mixed")))
            {
                hasMixed = true;
                continue;
            }

            if (summary.startsWith(QStringLiteral("CornerTransition")))
            {
                hasCorner = true;
                continue;
            }

            if (summary.startsWith(QStringLiteral("FlatSide")))
            {
                hasFlat = true;
                const int sidePos = summary.indexOf(QStringLiteral("side="));

                if (sidePos >= 0)
                {
                    const int valueStart = sidePos + 5;
                    int valueEnd = valueStart;

                    while (valueEnd < summary.size() && summary.at(valueEnd).isDigit())
                    {
                        ++valueEnd;
                    }

                    bool ok = false;
                    const int sideValue = summary.mid(valueStart, valueEnd - valueStart).toInt(&ok);

                    if (ok)
                    {
                        if (firstSide < 0)
                        {
                            firstSide = sideValue;
                        }
                        else if (firstSide != sideValue)
                        {
                            sameSide = false;
                        }
                    }
                }

                continue;
            }

            hasUnknown = true;
        }

        if (hasMixed)
        {
            return QStringLiteral("Mixed");
        }

        if (hasFlat && !hasCorner && !hasUnknown && sameSide && firstSide >= 0)
        {
            return QStringLiteral("FlatSide(side=%1)").arg(firstSide);
        }

        if (hasFlat && !hasCorner && !hasUnknown)
        {
            return QStringLiteral("FlatSide");
        }

        if (hasCorner && !hasFlat && !hasUnknown)
        {
            return QStringLiteral("CornerTransition");
        }

        if (hasUnknown && !hasFlat && !hasCorner)
        {
            return QStringLiteral("Unknown");
        }

        return QStringLiteral("Mixed");
    }

    void writeExportDebugStateToItem
    (
        CadItem* item,
        const QVector<ToolpathSegment4Axis>& segments,
        bool reversedByJoin
    )
    {
        if (item == nullptr)
        {
            return;
        }

        resetExportDebugState(item);
        item->m_exportPathPoints = buildExportPathFromSegments(segments);
        item->m_hasExportPathPoints = item->m_exportPathPoints.size() >= 2;
        item->m_exportDirectionReversed = reversedByJoin;
        item->m_exportProcessMode = summarizeProcessModeFromSegments(segments);
        item->m_exportRegionSummary = summarizeRegionFromSegments(segments);

        bool requiresA = false;
        bool firstA = true;
        double minA = 0.0;
        double maxA = 0.0;
        double startA = 0.0;
        double endA = 0.0;

        for (const ToolpathSegment4Axis& segment : segments)
        {
            if (segment.points.isEmpty())
            {
                continue;
            }

            requiresA = requiresA || segment.requiresA;

            for (const ToolpathPoint4Axis& point : segment.points)
            {
                if (firstA)
                {
                    minA = maxA = startA = point.aDeg;
                    firstA = false;
                }

                minA = std::min(minA, point.aDeg);
                maxA = std::max(maxA, point.aDeg);
                endA = point.aDeg;
            }
        }

        item->m_exportRequiresA = requiresA;
        item->m_exportAStartDeg = firstA ? 0.0 : startA;
        item->m_exportAEndDeg = firstA ? 0.0 : endA;
        item->m_exportARangeDeg = firstA ? 0.0 : (maxA - minA);
    }

    bool buildSamplesForPathPoints
    (
        const QVector<QVector3D>& pathPoints,
        const GProfileRotaryAxisConfig& profileConfig,
        const RotaryConfig& rotaryConfig,
        const ProcessTolerance& tolerance,
        QStringList& warnings,
        QVector<SamplePoint>& outSamples
    )
    {
        outSamples.clear();

        if (pathPoints.size() < 2)
        {
            return false;
        }

        if (usesRoundedRectSection(profileConfig)
            && profileConfig.sectionHalfWidthY > tolerance.normalEps
            && profileConfig.sectionHalfHeightZ > tolerance.normalEps)
        {
            RoundedRectSection2D section
            (
                profileConfig.sectionHalfWidthY,
                profileConfig.sectionHalfHeightZ,
                profileConfig.sectionCornerRadiusY,
                profileConfig.sectionCornerRadiusZ
            );

            if (section.isValid()
                && buildSamplesFromSectionByNearestProjection(pathPoints, section, rotaryConfig, tolerance, outSamples, &warnings))
            {
                return true;
            }

            warnings.append(QStringLiteral("截面法向构建失败，已回退为径向法向。"));
        }

        outSamples = buildRadialSamples(pathPoints, rotaryConfig);
        return outSamples.size() >= 2;
    }

    bool refineSamplesAndAAngles
    (
        const QVector<QVector3D>& inputPathPoints,
        const GProfileRotaryAxisConfig& profileConfig,
        const RotaryConfig& rotaryConfig,
        const ProcessTolerance& tolerance,
        QStringList& warnings,
        QVector<SamplePoint>& outSamples,
        QVector<double>& outContinuousA,
        ReachabilityResult& outReachability
    )
    {
        QVector<QVector3D> pathPoints = inputPathPoints;
        constexpr int kMaxRefineIterations = 8;

        for (int iteration = 0; iteration < kMaxRefineIterations; ++iteration)
        {
            QVector<SamplePoint> samples;

            if (!buildSamplesForPathPoints(pathPoints, profileConfig, rotaryConfig, tolerance, warnings, samples))
            {
                return false;
            }

            ReachabilityResult reachability;
            const QVector<double> continuousA = computeContinuousAAngles(samples, rotaryConfig, tolerance, &reachability);

            if (continuousA.size() != samples.size() || continuousA.size() < 2)
            {
                return false;
            }

            bool needSplit = false;
            QVector<QVector3D> refinedPathPoints;
            refinedPathPoints.reserve(pathPoints.size() * 2);
            refinedPathPoints.append(pathPoints.front());
            const double maxDeltaA = std::max(tolerance.maxDeltaADegPerStep, 1.0e-6);

            for (int index = 1; index < pathPoints.size(); ++index)
            {
                const QVector3D startPoint = pathPoints.at(index - 1);
                const QVector3D endPoint = pathPoints.at(index);
                const SamplePoint& leftSample = samples.at(index - 1);
                const SamplePoint& rightSample = samples.at(index);
                const bool preserveFixedA = leftSample.regionType == SectionRegionType::FlatSide
                    && rightSample.regionType == SectionRegionType::FlatSide
                    && leftSample.sideIndex >= 0
                    && leftSample.sideIndex == rightSample.sideIndex
                    && leftSample.hasTargetA
                    && rightSample.hasTargetA
                    && std::abs(leftSample.targetA - rightSample.targetA) <= 1.0e-9;
                const double deltaA = preserveFixedA
                    ? 0.0
                    : std::abs(continuousA.at(index) - continuousA.at(index - 1));
                const int splitCount = preserveFixedA
                    ? 1
                    : std::max(1, static_cast<int>(std::ceil(deltaA / maxDeltaA)));

                if (splitCount > 1)
                {
                    needSplit = true;
                }

                for (int split = 1; split <= splitCount; ++split)
                {
                    const double t = static_cast<double>(split) / static_cast<double>(splitCount);
                    refinedPathPoints.append
                    (
                        QVector3D
                        (
                            static_cast<float>(startPoint.x() + (endPoint.x() - startPoint.x()) * t),
                            static_cast<float>(startPoint.y() + (endPoint.y() - startPoint.y()) * t),
                            static_cast<float>(startPoint.z() + (endPoint.z() - startPoint.z()) * t)
                        )
                    );
                }
            }

            if (!needSplit)
            {
                outSamples = samples;
                outContinuousA = continuousA;
                outReachability = reachability;
                return true;
            }

            pathPoints = refinedPathPoints;
        }

        QVector<SamplePoint> samples;

        if (!buildSamplesForPathPoints(pathPoints, profileConfig, rotaryConfig, tolerance, warnings, samples))
        {
            return false;
        }

        outContinuousA = computeContinuousAAngles(samples, rotaryConfig, tolerance, &outReachability);
        outSamples = samples;
        return outContinuousA.size() == outSamples.size() && outContinuousA.size() >= 2;
    }

    QVector<ToolpathSegment4Axis> build4AxisSegmentsForItem
    (
        const CadItem* item,
        const GProfileRotaryAxisConfig& profileConfig,
        const RotaryConfig& rotaryConfig,
        const ProcessTolerance& tolerance,
        bool reverseDirection,
        const QString& processKey,
        QStringList& warnings,
        QStringList* lineDebugLines = nullptr
    )
    {
        QVector<ToolpathSegment4Axis> segments;
        const bool isLineGeometry = item != nullptr && item->m_type == DRW::ETYPE::LINE;
        const QVector<QVector3D> pathPoints = densifyPathPointsLinear
        (
            buildPathPointsForItem(item, reverseDirection),
            tolerance.maxLinearStep
        );

        if (pathPoints.size() < 2)
        {
            return segments;
        }

        QVector<SamplePoint> samples;
        QVector<double> continuousA;
        ReachabilityResult reachability;

        if (!refineSamplesAndAAngles
        (
            pathPoints,
            profileConfig,
            rotaryConfig,
            tolerance,
            warnings,
            samples,
            continuousA,
            reachability
        ))
        {
            return segments;
        }

        FeatureClassifier classifier;
        const ProcessMode processMode = classifier.classify(samples, rotaryConfig, tolerance, &reachability, nullptr);
        const double neutralA = commandAngleFromMathDeg(0.0, rotaryConfig);
        warnings.append(QStringLiteral("图元 %1 分类为 %2。")
            .arg(entityTypeKey(item))
            .arg(processModeToString(processMode)));

        if (!reachability.aOnlyReachable)
        {
            warnings.append
            (
                QStringLiteral("图元 %1 的 |nx|max=%2，超过阈值 %3，仅靠 A 轴无法理想对正，已降级为固定姿态。")
                    .arg(entityTypeKey(item))
                    .arg(reachability.nxAbsMax, 0, 'f', 4)
                    .arg(tolerance.nxThreshold, 0, 'f', 4)
            );
        }

        if (processMode == ProcessMode::XYZ_3Axis)
        {
            continuousA.fill(neutralA, continuousA.size());
        }
        else if (processMode == ProcessMode::A_Indexed_XYZ)
        {
            const double averageA = std::accumulate(continuousA.cbegin(), continuousA.cend(), 0.0) / static_cast<double>(continuousA.size());
            continuousA.fill(averageA, continuousA.size());
        }

        const QVector<ToolpathPoint4Axis> resampled = resampleForRotaryMotion
        (
            samples,
            continuousA,
            rotaryConfig,
            tolerance,
            1200.0,
            100.0
        );

        segments = segmentByAMode(resampled, samples, rotaryConfig, tolerance);

        if (processMode == ProcessMode::A_Indexed_XYZ)
        {
            for (ToolpathSegment4Axis& segment : segments)
            {
                segment.mode = SegmentMotionMode::AFixed;
            }
        }

        for (ToolpathSegment4Axis& segment : segments)
        {
            double minA = segment.points.isEmpty() ? neutralA : segment.points.front().aDeg;
            double maxA = minA;
            double sumA = 0.0;

            for (const ToolpathPoint4Axis& point : segment.points)
            {
                minA = std::min(minA, point.aDeg);
                maxA = std::max(maxA, point.aDeg);
                sumA += point.aDeg;
            }

            const double rangeA = maxA - minA;
            const double averageA = segment.points.isEmpty() ? neutralA : (sumA / static_cast<double>(segment.points.size()));
            const bool nearNeutral = std::abs(averageA - neutralA) <= tolerance.aNeutralToleranceDeg;
            const bool isCornerSegment = segment.regionSummary.startsWith(QStringLiteral("CornerTransition"));
            const bool isFlatSegment = segment.regionSummary.startsWith(QStringLiteral("FlatSide"));
            const bool requiresA = processMode != ProcessMode::XYZ_3Axis
                && (isCornerSegment
                    || (isFlatSegment && (!nearNeutral || rangeA > tolerance.aEnableThresholdDeg))
                    || (!isCornerSegment && !isFlatSegment
                        && (rangeA > tolerance.aNeutralToleranceDeg
                            || std::abs(averageA - neutralA) > tolerance.aEnableThresholdDeg)));

            if (!requiresA)
            {
                segment.mode = SegmentMotionMode::AFixed;
                segment.hasFixedA = true;
                segment.fixedADeg = neutralA;
                segment.emitAInEachLine = false;
                segment.prePositionAOnly = false;

                for (ToolpathPoint4Axis& point : segment.points)
                {
                    point.aDeg = neutralA;
                    point.machinePos = rotatePointByA(point.localPos, point.aDeg, rotaryConfig);
                }
            }
            else if (segment.mode == SegmentMotionMode::AFixed)
            {
                segment.hasFixedA = true;
                segment.fixedADeg = segment.points.isEmpty() ? neutralA : segment.points.front().aDeg;
                segment.emitAInEachLine = false;
                segment.prePositionAOnly = true;
            }
            else
            {
                segment.hasFixedA = false;
                segment.fixedADeg = 0.0;
                segment.emitAInEachLine = true;
                segment.prePositionAOnly = false;
            }

            segment.processKey = processKey;
            segment.processMode = !requiresA
                ? ProcessMode::XYZ_3Axis
                : (segment.mode == SegmentMotionMode::AFixed ? ProcessMode::A_Indexed_XYZ : ProcessMode::XYZA_Continuous);
            segment.requiresA = requiresA;

            if (!segment.points.isEmpty())
            {
                segment.startADeg = segment.points.front().aDeg;
                segment.endADeg = segment.points.back().aDeg;
                double rangeMin = segment.startADeg;
                double rangeMax = segment.startADeg;

                for (const ToolpathPoint4Axis& point : segment.points)
                {
                    rangeMin = std::min(rangeMin, point.aDeg);
                    rangeMax = std::max(rangeMax, point.aDeg);
                }

                segment.aRangeDeg = rangeMax - rangeMin;

                if (segment.mode == SegmentMotionMode::AFixed && segment.hasFixedA)
                {
                    segment.fixedADeg = segment.startADeg;
                    segment.aRangeDeg = 0.0;
                }
            }
            else
            {
                segment.startADeg = 0.0;
                segment.endADeg = 0.0;
                segment.aRangeDeg = 0.0;
            }
        }

        if (isLineGeometry && lineDebugLines != nullptr)
        {
            lineDebugLines->append
            (
                QStringLiteral("DEBUG_LINE: entity=%1 isLine=Y sampleCount=%2")
                    .arg(entityTypeKey(item))
                    .arg(samples.size())
            );
            lineDebugLines->append(QStringLiteral("DEBUG_LINE: samples=%1").arg(buildLineSampleDebug(samples)));

            for (const ToolpathSegment4Axis& segment : segments)
            {
                lineDebugLines->append
                (
                    QStringLiteral("DEBUG_LINE: final mode=%1 requiresA=%2 fixedA=%3 startA=%4 endA=%5 range=%6 emitAInEachLine=%7 prePositionAOnly=%8")
                        .arg(processModeToString(segment.processMode))
                        .arg(segment.requiresA ? QStringLiteral("Y") : QStringLiteral("N"))
                        .arg(segment.hasFixedA ? QString::number(segment.fixedADeg, 'f', 3) : QStringLiteral("NA"))
                        .arg(segment.startADeg, 0, 'f', 3)
                        .arg(segment.endADeg, 0, 'f', 3)
                        .arg(segment.aRangeDeg, 0, 'f', 3)
                        .arg(segment.emitAInEachLine ? QStringLiteral("Y") : QStringLiteral("N"))
                        .arg(segment.prePositionAOnly ? QStringLiteral("Y") : QStringLiteral("N"))
                );
            }
        }

        return segments;
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

    if (m_generationMode == GenerationMode::Mode3D)
    {
        const GProfileRotaryAxisConfig& profileRotaryConfig = m_profile->rotaryAxisConfig();
        const RotaryConfig rotaryConfig = buildRotaryConfig(profileRotaryConfig);
        ProcessTolerance tolerance;
        MachineConfig4Axis machineConfig = buildMachineConfig(profileRotaryConfig);
        machineConfig.aNeutralDeg = commandAngleFromMathDeg(0.0, rotaryConfig);
        machineConfig.aNeutralToleranceDeg = tolerance.aNeutralToleranceDeg;
        machineConfig.fixedASwitchThresholdDeg = tolerance.fixedASwitchThresholdDeg;
        machineConfig.connectionTolerance = tolerance.connectionTolerance;
        machineConfig.aJoinToleranceDeg = tolerance.aJoinToleranceDeg;
        machineConfig.feedTolerance = tolerance.feedTolerance;
        machineConfig.powerTolerance = tolerance.powerTolerance;
        machineConfig.allowLaserOffNoLiftTransition = tolerance.allowLaserOffNoLiftTransition;
        QStringList warnings;
        QStringList debugLines;
        QStringList lineDebugLines;
        QVector<ToolpathSegment4Axis> allSegments;
        bool hasPreviousEndPoint = false;
        ToolpathPoint4Axis previousEndPoint;

        for (CadItem* item : orderedItems)
        {
            resetExportDebugState(item);
        }

        for (CadItem* item : orderedItems)
        {
            if (item == nullptr || item->m_nativeEntity == nullptr)
            {
                continue;
            }

            const QString processKey = buildProcessKey(item);
            QStringList forwardWarnings;
            QStringList selectedItemLineDebug;
            const bool defaultReverse = item->m_isReverse;
            QVector<ToolpathSegment4Axis> selectedSegments = build4AxisSegmentsForItem
            (
                item,
                profileRotaryConfig,
                rotaryConfig,
                tolerance,
                defaultReverse,
                processKey,
                forwardWarnings,
                &selectedItemLineDebug
            );
            QStringList selectedWarnings = forwardWarnings;
            bool selectedReverse = defaultReverse;

            if (canReverseEntityForJoin(item, tolerance) && hasPreviousEndPoint)
            {
                QStringList reverseWarnings;
                QStringList reverseLineDebug;
                const QVector<ToolpathSegment4Axis> reverseSegments = build4AxisSegmentsForItem
                (
                    item,
                    profileRotaryConfig,
                    rotaryConfig,
                    tolerance,
                    !defaultReverse,
                    processKey,
                    reverseWarnings,
                    &reverseLineDebug
                );

                if (!selectedSegments.isEmpty() && !reverseSegments.isEmpty())
                {
                    const QVector3D forwardStart = selectedSegments.front().points.front().machinePos;
                    const QVector3D reverseStart = reverseSegments.front().points.front().machinePos;
                    const double forwardDistance = static_cast<double>((forwardStart - previousEndPoint.machinePos).length());
                    const double reverseDistance = static_cast<double>((reverseStart - previousEndPoint.machinePos).length());

                    if (reverseDistance + 1.0e-9 < forwardDistance)
                    {
                        selectedSegments = reverseSegments;
                        selectedWarnings = reverseWarnings;
                        selectedReverse = !defaultReverse;
                        selectedItemLineDebug = reverseLineDebug;
                    }
                }
                else if (selectedSegments.isEmpty() && !reverseSegments.isEmpty())
                {
                    selectedSegments = reverseSegments;
                    selectedWarnings = reverseWarnings;
                    selectedReverse = !defaultReverse;
                    selectedItemLineDebug = reverseLineDebug;
                }
            }

            if (selectedSegments.isEmpty())
            {
                continue;
            }

            warnings.append(selectedWarnings);
            lineDebugLines.append(selectedItemLineDebug);
            writeExportDebugStateToItem(item, selectedSegments, selectedReverse != defaultReverse);

            if (selectedReverse != defaultReverse)
            {
                warnings.append(QStringLiteral("图元 %1 为减少空行程已自动反转加工方向。").arg(entityTypeKey(item)));
            }

            debugLines.append
            (
                QStringLiteral("ENTITY[%1|order=%2] mode=%3 reversed=%4 requiresA=%5 region=%6 A(start=%7,end=%8,range=%9) previewUsesExportPath=Y")
                    .arg(entityTypeKey(item))
                    .arg(item->m_processOrder)
                    .arg(item->m_exportProcessMode)
                    .arg(item->m_exportDirectionReversed ? QStringLiteral("Y") : QStringLiteral("N"))
                    .arg(item->m_exportRequiresA ? QStringLiteral("Y") : QStringLiteral("N"))
                    .arg(item->m_exportRegionSummary.isEmpty() ? QStringLiteral("Unknown") : item->m_exportRegionSummary)
                    .arg(item->m_exportAStartDeg, 0, 'f', 3)
                    .arg(item->m_exportAEndDeg, 0, 'f', 3)
                    .arg(item->m_exportARangeDeg, 0, 'f', 3)
            );

            for (const ToolpathSegment4Axis& segment : selectedSegments)
            {
                allSegments.append(segment);
            }

            previousEndPoint = allSegments.back().points.back();
            hasPreviousEndPoint = true;
        }

        if (allSegments.isEmpty())
        {
            file.close();

            if (errorMessage != nullptr)
            {
                *errorMessage = QStringLiteral("未生成有效 4 轴路径：请检查图元几何、回转中心与截面配置。");
            }

            return false;
        }

        GCodeGenerator4Axis rotaryGenerator;
        const QString geometryText = rotaryGenerator.generate(allSegments, rotaryConfig, machineConfig, &warnings);

        if (geometryText.trimmed().isEmpty())
        {
            file.close();

            if (errorMessage != nullptr)
            {
                *errorMessage = QStringLiteral("4 轴 G 代码生成失败：几何输出为空。");
            }

            return false;
        }

        for (const QString& warning : warnings)
        {
            if (!warning.trimmed().isEmpty())
            {
                stream << "; WARN: " << warning << "\r\n";
            }
        }

        for (const QString& debugLine : debugLines)
        {
            if (!debugLine.trimmed().isEmpty())
            {
                stream << "; DEBUG: " << debugLine << "\r\n";
            }
        }

        for (const QString& lineDebug : lineDebugLines)
        {
            if (!lineDebug.trimmed().isEmpty())
            {
                stream << "; " << lineDebug << "\r\n";
            }
        }

        stream << geometryText;
        writeTextBlock(stream, m_profile->fileCode().footer);
        file.close();

        if (errorMessage != nullptr)
        {
            *errorMessage = warnings.join('\n');
        }

        return true;
    }

    for (CadItem* item : orderedItems)
    {
        resetExportDebugState(item);
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

        const bool geometryWritten = writeItemGeometry(geometryStream, item);

        if (!geometryWritten)
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
