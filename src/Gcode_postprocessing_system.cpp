#include "pch.h"

#include "Gcode_postprocessing_system.h"
#include "CadBitmapImportDialog.h"
#include "CadBitmapVectorizer.h"
#include "CadItem.h"
#include "CadProcessVisualUtils.h"
#include "GGenerator.h"
#include "GProfileDialog.h"

#include <QActionGroup>
#include <QApplication>
#include <QCoreApplication>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QInputDialog>
#include <QKeySequence>
#include <QMap>
#include <QMessageBox>
#include <QMenu>
#include <QMenuBar>
#include <QSettings>
#include <QStatusBar>
#include <QStyleFactory>
#include <QSet>
#include <QToolBar>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace
{
    constexpr const char* kBuiltinThreeAxisProfileId = "builtin:3axis";
    constexpr const char* kBuiltinFourAxisProfileId = "builtin:4axis";
    constexpr double kSortEpsilon = 1.0e-9;
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kTwoPi = 6.28318530717958647692;
    constexpr int kColorByLayer = 256;
    constexpr int kClosedEllipseSampleCount = 16;
    constexpr double kNextDistanceWeight = 0.15;
    constexpr double kDirectionPenaltyWeight = 0.35;
    constexpr double kBacktrackPenaltyWeight = 1.2;
    constexpr double kRotaryAngleDistanceWeight = 0.08;
    constexpr double kRotaryNextDistanceWeight = 0.12;
    constexpr double kRotaryBacktrackPenaltyWeight = 1.35;
    constexpr double kRotaryDirectionPenaltyWeight = 0.2;
    constexpr double kSortConnectionEpsilon = 1.0e-6;
    constexpr double kNearGapPriorityDistance2D = 1.0;
    constexpr double kNearGapPriorityDistance3D = 1.0;
    constexpr double kPreferredStartGapDistance2D = 1.0;
    constexpr double kPreferredStartGapDistance3D = 1.0;
    constexpr double kDedupTolerance = 1.0e-6;
    const QVector3D kSortOrigin(0.0f, 0.0f, 0.0f);
    const QVector3D kRotaryInitialSortOrigin(0.0f, 0.0f, 50.0f);

    enum class SortStrategy
    {
        KeepDirection,
        Smart
    };

    struct SortCandidate
    {
        int index = -1;
        bool reverse = false;
        bool hasCustomStart = false;
        double processStartParameter = 0.0;
        double connectionDistance = std::numeric_limits<double>::max();
        double priorityDistance = std::numeric_limits<double>::max();
        double gapDistance = std::numeric_limits<double>::max();
        double score = std::numeric_limits<double>::max();
        QVector3D startPoint;
        QVector3D endPoint;
    };

    struct ProcessPathOption
    {
        bool reverse = false;
        bool hasCustomStart = false;
        double processStartParameter = 0.0;
        QVector3D startPoint;
        QVector3D endPoint;
        QVector3D startTangent;
        QVector3D endTangent;
    };

    struct RotarySortPoint
    {
        double axis = 0.0;
        double angleDegrees = 0.0;
    };

    struct ProcessConnectionSegment
    {
        QVector3D startPoint;
        QVector3D endPoint;
    };

    struct EndpointNode
    {
        size_t itemIndex = 0;
        QVector3D point;
    };

    struct GapStartSelectionContext
    {
        std::vector<int> componentIds;
        std::vector<std::vector<QVector3D>> preferredStartPointsByComponent;
    };

    std::vector<ProcessPathOption> buildPathOptionsForItem(const CadItem* item, SortStrategy strategy);

    qint64 quantizeDedupValue(double value)
    {
        const qint64 quantized = static_cast<qint64>(std::llround(value / kDedupTolerance));
        return quantized == 0 ? 0 : quantized;
    }

    QString dedupNumberToken(double value)
    {
        return QString::number(quantizeDedupValue(value));
    }

    QString dedupCoordToken(const DRW_Coord& coord)
    {
        return QStringLiteral("%1,%2,%3")
            .arg(dedupNumberToken(coord.x))
            .arg(dedupNumberToken(coord.y))
            .arg(dedupNumberToken(coord.z));
    }

    QString dedupCoordToken(double x, double y, double z = 0.0)
    {
        return QStringLiteral("%1,%2,%3")
            .arg(dedupNumberToken(x))
            .arg(dedupNumberToken(y))
            .arg(dedupNumberToken(z));
    }

    double normalizeAngleZeroToTwoPi(double angle)
    {
        double normalized = std::fmod(angle, kTwoPi);

        if (normalized < 0.0)
        {
            normalized += kTwoPi;
        }

        return normalized;
    }

    QString dedupAngleToken(double angle)
    {
        return dedupNumberToken(normalizeAngleZeroToTwoPi(angle));
    }

    QString buildLineDuplicateKey(const DRW_Line* line)
    {
        if (line == nullptr)
        {
            return QString();
        }

        const QString start = dedupCoordToken(line->basePoint);
        const QString end = dedupCoordToken(line->secPoint);
        const auto ordered = start <= end
            ? std::pair<QString, QString>(start, end)
            : std::pair<QString, QString>(end, start);

        return QStringLiteral("LINE|%1|%2").arg(ordered.first, ordered.second);
    }

    QString buildCircleDuplicateKey(const DRW_Circle* circle)
    {
        if (circle == nullptr)
        {
            return QString();
        }

        return QStringLiteral("CIRCLE|%1|%2")
            .arg(dedupCoordToken(circle->basePoint))
            .arg(dedupNumberToken(circle->radious));
    }

    QString buildArcDuplicateKey(const DRW_Arc* arc)
    {
        if (arc == nullptr)
        {
            return QString();
        }

        return QStringLiteral("ARC|%1|%2|%3|%4")
            .arg(dedupCoordToken(arc->basePoint))
            .arg(dedupNumberToken(arc->radious))
            .arg(dedupAngleToken(arc->staangle))
            .arg(dedupAngleToken(arc->endangle));
    }

    QString buildEllipseDuplicateKey(const DRW_Ellipse* ellipse)
    {
        if (ellipse == nullptr)
        {
            return QString();
        }

        DRW_Coord majorAxis = ellipse->secPoint;
        double startParam = ellipse->staparam;
        double endParam = ellipse->endparam;
        const double span = endParam - startParam;
        const bool isFullEllipse = std::abs(span) < kDedupTolerance
            || std::abs(std::abs(span) - kTwoPi) < kDedupTolerance;

        const bool flipMajorAxis =
            quantizeDedupValue(majorAxis.x) < 0
            || (quantizeDedupValue(majorAxis.x) == 0 && quantizeDedupValue(majorAxis.y) < 0)
            || (quantizeDedupValue(majorAxis.x) == 0 && quantizeDedupValue(majorAxis.y) == 0 && quantizeDedupValue(majorAxis.z) < 0);

        if (flipMajorAxis)
        {
            majorAxis.x = -majorAxis.x;
            majorAxis.y = -majorAxis.y;
            majorAxis.z = -majorAxis.z;
            startParam += kPi;
            endParam += kPi;
        }

        if (isFullEllipse)
        {
            startParam = 0.0;
            endParam = kTwoPi;
        }
        else
        {
            startParam = normalizeAngleZeroToTwoPi(startParam);
            double normalizedEnd = normalizeAngleZeroToTwoPi(endParam);

            while (normalizedEnd <= startParam)
            {
                normalizedEnd += kTwoPi;
            }

            endParam = normalizedEnd;
        }

        DRW_Coord normal = ellipse->extPoint;
        const double normalLength = std::sqrt(normal.x * normal.x + normal.y * normal.y + normal.z * normal.z);

        if (normalLength > kDedupTolerance)
        {
            normal.x /= normalLength;
            normal.y /= normalLength;
            normal.z /= normalLength;
        }

        return QStringLiteral("ELLIPSE|%1|%2|%3|%4|%5|%6")
            .arg(dedupCoordToken(ellipse->basePoint))
            .arg(dedupCoordToken(majorAxis))
            .arg(dedupCoordToken(normal))
            .arg(dedupNumberToken(ellipse->ratio))
            .arg(dedupNumberToken(startParam))
            .arg(dedupNumberToken(endParam));
    }

    QString buildPolylineVertexSequenceToken(const std::vector<std::shared_ptr<DRW_Vertex>>& vertlist)
    {
        QStringList vertexTokens;
        vertexTokens.reserve(static_cast<int>(vertlist.size()));

        for (const std::shared_ptr<DRW_Vertex>& vertex : vertlist)
        {
            if (!vertex)
            {
                vertexTokens.push_back(QStringLiteral("null"));
                continue;
            }

            vertexTokens.push_back
            (
                QStringLiteral("%1|%2")
                .arg(dedupCoordToken(vertex->basePoint))
                .arg(dedupNumberToken(vertex->bulge))
            );
        }

        return vertexTokens.join(QStringLiteral(";"));
    }

    QString buildLWPolylineVertexSequenceToken(const std::vector<std::shared_ptr<DRW_Vertex2D>>& vertlist)
    {
        QStringList vertexTokens;
        vertexTokens.reserve(static_cast<int>(vertlist.size()));

        for (const std::shared_ptr<DRW_Vertex2D>& vertex : vertlist)
        {
            if (!vertex)
            {
                vertexTokens.push_back(QStringLiteral("null"));
                continue;
            }

            vertexTokens.push_back
            (
                QStringLiteral("%1|%2")
                .arg(dedupCoordToken(vertex->x, vertex->y, 0.0))
                .arg(dedupNumberToken(vertex->bulge))
            );
        }

        return vertexTokens.join(QStringLiteral(";"));
    }

    QString buildPolylineDuplicateKey(const DRW_Polyline* polyline)
    {
        if (polyline == nullptr)
        {
            return QString();
        }

        return QStringLiteral("POLYLINE|%1|%2|%3")
            .arg((polyline->flags & 0x01) != 0 ? QStringLiteral("C") : QStringLiteral("O"))
            .arg(dedupCoordToken(polyline->extPoint))
            .arg(buildPolylineVertexSequenceToken(polyline->vertlist));
    }

    QString buildLWPolylineDuplicateKey(const DRW_LWPolyline* polyline)
    {
        if (polyline == nullptr)
        {
            return QString();
        }

        return QStringLiteral("LWPOLYLINE|%1|%2|%3|%4")
            .arg((polyline->flags & 0x01) != 0 ? QStringLiteral("C") : QStringLiteral("O"))
            .arg(dedupNumberToken(polyline->elevation))
            .arg(dedupCoordToken(polyline->extPoint))
            .arg(buildLWPolylineVertexSequenceToken(polyline->vertlist));
    }

    QString duplicateGeometryKey(const CadItem* item)
    {
        if (item == nullptr || item->m_nativeEntity == nullptr)
        {
            return QString();
        }

        switch (item->m_type)
        {
        case DRW::ETYPE::LINE:
            return buildLineDuplicateKey(static_cast<const DRW_Line*>(item->m_nativeEntity));
        case DRW::ETYPE::CIRCLE:
            return buildCircleDuplicateKey(static_cast<const DRW_Circle*>(item->m_nativeEntity));
        case DRW::ETYPE::ARC:
            return buildArcDuplicateKey(static_cast<const DRW_Arc*>(item->m_nativeEntity));
        case DRW::ETYPE::ELLIPSE:
            return buildEllipseDuplicateKey(static_cast<const DRW_Ellipse*>(item->m_nativeEntity));
        case DRW::ETYPE::POLYLINE:
            return buildPolylineDuplicateKey(static_cast<const DRW_Polyline*>(item->m_nativeEntity));
        case DRW::ETYPE::LWPOLYLINE:
            return buildLWPolylineDuplicateKey(static_cast<const DRW_LWPolyline*>(item->m_nativeEntity));
        default:
            return QString();
        }
    }

    QColor colorFromAci(int colorIndex)
    {
        static const QRgb aciStandardColors[] =
        {
            qRgb(0, 0, 0),
            qRgb(255, 0, 0),
            qRgb(255, 255, 0),
            qRgb(0, 255, 0),
            qRgb(0, 255, 255),
            qRgb(0, 0, 255),
            qRgb(255, 0, 255),
            qRgb(255, 255, 255),
            qRgb(128, 128, 128),
            qRgb(192, 192, 192)
        };

        if (colorIndex >= 1 && colorIndex <= 9)
        {
            return QColor(aciStandardColors[colorIndex]);
        }

        if (colorIndex == 0)
        {
            return QColor(Qt::white);
        }

        return QColor();
    }

    QColor colorFromTrueColor(int color24)
    {
        if (color24 < 0)
        {
            return QColor();
        }

        return QColor((color24 >> 16) & 0xFF, (color24 >> 8) & 0xFF, color24 & 0xFF);
    }

    QVector3D geometryBoundsCenter(const CadItem* item)
    {
        if (item == nullptr || item->m_geometry.vertices.isEmpty())
        {
            return QVector3D();
        }

        QVector3D minPoint = item->m_geometry.vertices.front();
        QVector3D maxPoint = item->m_geometry.vertices.front();

        for (const QVector3D& point : item->m_geometry.vertices)
        {
            minPoint.setX(std::min(minPoint.x(), point.x()));
            minPoint.setY(std::min(minPoint.y(), point.y()));
            minPoint.setZ(std::min(minPoint.z(), point.z()));
            maxPoint.setX(std::max(maxPoint.x(), point.x()));
            maxPoint.setY(std::max(maxPoint.y(), point.y()));
            maxPoint.setZ(std::max(maxPoint.z(), point.z()));
        }

        return QVector3D
        (
            (minPoint.x() + maxPoint.x()) * 0.5f,
            (minPoint.y() + maxPoint.y()) * 0.5f,
            (minPoint.z() + maxPoint.z()) * 0.5f
        );
    }

    QVector3D geometryBoundsCenter(const QVector<CadItem*>& items)
    {
        if (items.isEmpty())
        {
            return QVector3D();
        }

        QVector3D minPoint;
        QVector3D maxPoint;
        bool initialized = false;

        for (const CadItem* item : items)
        {
            if (item == nullptr || item->m_geometry.vertices.isEmpty())
            {
                continue;
            }

            for (const QVector3D& point : item->m_geometry.vertices)
            {
                if (!initialized)
                {
                    minPoint = point;
                    maxPoint = point;
                    initialized = true;
                    continue;
                }

                minPoint.setX(std::min(minPoint.x(), point.x()));
                minPoint.setY(std::min(minPoint.y(), point.y()));
                minPoint.setZ(std::min(minPoint.z(), point.z()));
                maxPoint.setX(std::max(maxPoint.x(), point.x()));
                maxPoint.setY(std::max(maxPoint.y(), point.y()));
                maxPoint.setZ(std::max(maxPoint.z(), point.z()));
            }
        }

        if (!initialized)
        {
            return QVector3D();
        }

        return QVector3D
        (
            (minPoint.x() + maxPoint.x()) * 0.5f,
            (minPoint.y() + maxPoint.y()) * 0.5f,
            (minPoint.z() + maxPoint.z()) * 0.5f
        );
    }

    QString entityLayerName(const CadItem* item)
    {
        if (item == nullptr || item->m_nativeEntity == nullptr)
        {
            return QStringLiteral("0");
        }

        const QString layerName = QString::fromUtf8(item->m_nativeEntity->layer.c_str()).trimmed();
        return layerName.isEmpty() ? QStringLiteral("0") : layerName;
    }

    int entityColorIndex(const CadItem* item)
    {
        if (item == nullptr || item->m_nativeEntity == nullptr)
        {
            return kColorByLayer;
        }

        return item->m_nativeEntity->color24 != -1
            ? -1
            : item->m_nativeEntity->color;
    }

    QColor entityDisplayColor(const CadDocument& document, const CadItem* item)
    {
        if (item == nullptr || item->m_nativeEntity == nullptr)
        {
            return QColor(Qt::white);
        }

        const QColor trueColor = colorFromTrueColor(item->m_nativeEntity->color24);

        if (trueColor.isValid())
        {
            return trueColor;
        }

        if (item->m_nativeEntity->color == kColorByLayer)
        {
            return document.layerColor(entityLayerName(item), item->m_color);
        }

        const QColor aciColor = colorFromAci(item->m_nativeEntity->color);
        return aciColor.isValid() ? aciColor : item->m_color;
    }

    bool hasSuffix(const QString& filePath, std::initializer_list<const char*> suffixes)
    {
        for (const char* suffix : suffixes)
        {
            if (filePath.endsWith(QString::fromLatin1(suffix), Qt::CaseInsensitive))
            {
                return true;
            }
        }

        return false;
    }

    bool isCadVectorFile(const QString& filePath)
    {
        return hasSuffix(filePath, { ".dxf", ".dwg" });
    }

    bool isBitmapFile(const QString& filePath)
    {
        return hasSuffix(filePath, { ".bmp", ".png", ".jpg", ".jpeg" });
    }

    bool isPointLexicographicallyLess(const QVector3D& left, const QVector3D& right)
    {
        if (left.x() != right.x())
        {
            return left.x() < right.x();
        }

        if (left.y() != right.y())
        {
            return left.y() < right.y();
        }

        return left.z() < right.z();
    }

    double planarDistanceSquared(const QVector3D& left, const QVector3D& right)
    {
        const double dx = static_cast<double>(left.x()) - static_cast<double>(right.x());
        const double dy = static_cast<double>(left.y()) - static_cast<double>(right.y());
        return dx * dx + dy * dy;
    }

    double spatialDistanceSquared(const QVector3D& left, const QVector3D& right)
    {
        const double dx = static_cast<double>(left.x()) - static_cast<double>(right.x());
        const double dy = static_cast<double>(left.y()) - static_cast<double>(right.y());
        const double dz = static_cast<double>(left.z()) - static_cast<double>(right.z());
        return dx * dx + dy * dy + dz * dz;
    }

    bool isPointNearAnyPreferredStart(const QVector3D& point, const std::vector<QVector3D>& preferredPoints, double maxDistance)
    {
        const double maxDistanceSquared = maxDistance * maxDistance;

        for (const QVector3D& preferredPoint : preferredPoints)
        {
            if (spatialDistanceSquared(point, preferredPoint) <= maxDistanceSquared)
            {
                return true;
            }
        }

        return false;
    }

    bool documentContainsThreeDimensionalGeometry(const CadDocument& document)
    {
        constexpr float kThreeDimensionalTolerance = 1.0e-5f;

        for (const std::unique_ptr<CadItem>& entity : document.m_entities)
        {
            if (entity == nullptr)
            {
                continue;
            }

            for (const QVector3D& vertex : entity->m_geometry.vertices)
            {
                if (std::abs(vertex.z()) > kThreeDimensionalTolerance)
                {
                    return true;
                }
            }
        }

        return false;
    }

    double unwrapAngleDegrees(double referenceDegrees, double wrappedDegrees)
    {
        return referenceDegrees + std::remainder(wrappedDegrees - referenceDegrees, 360.0);
    }

    bool tryBuildRotarySortPoint(const QVector3D& point, const GProfileRotaryAxisConfig& config, RotarySortPoint& rotaryPoint)
    {
        const double relativeY = static_cast<double>(point.y()) - config.centerY;
        const double relativeZ = static_cast<double>(point.z()) - config.centerZ;

        if (std::hypot(relativeY, relativeZ) <= kSortEpsilon)
        {
            return false;
        }

        double angleDegrees = std::atan2(relativeZ, relativeY) * 180.0 / kPi;

        if (config.invertAAxisDirection)
        {
            angleDegrees = -angleDegrees;
        }

        rotaryPoint.axis = static_cast<double>(point.x());
        rotaryPoint.angleDegrees = angleDegrees + config.aAxisOffsetDegrees;
        return true;
    }

    double rotarySortTravelDistance
    (
        const QVector3D& fromPoint,
        const QVector3D& toPoint,
        const GProfileRotaryAxisConfig& config,
        double* resolvedToAngleDegrees = nullptr
    )
    {
        const double dx = static_cast<double>(toPoint.x()) - static_cast<double>(fromPoint.x());
        const double spatialDistance = static_cast<double>((toPoint - fromPoint).length());
        RotarySortPoint fromRotaryPoint;
        RotarySortPoint toRotaryPoint;

        if (!tryBuildRotarySortPoint(fromPoint, config, fromRotaryPoint) || !tryBuildRotarySortPoint(toPoint, config, toRotaryPoint))
        {
            if (resolvedToAngleDegrees != nullptr)
            {
                *resolvedToAngleDegrees = 0.0;
            }

            return spatialDistance;
        }

        const double resolvedToAngle = unwrapAngleDegrees(fromRotaryPoint.angleDegrees, toRotaryPoint.angleDegrees);
        const double angleDistance = std::abs(resolvedToAngle - fromRotaryPoint.angleDegrees);

        if (resolvedToAngleDegrees != nullptr)
        {
            *resolvedToAngleDegrees = resolvedToAngle;
        }

        return std::abs(dx) + spatialDistance + angleDistance * kRotaryAngleDistanceWeight;
    }

    QVector3D flattenToSortPlane(const QVector3D& point)
    {
        return QVector3D(point.x(), point.y(), 0.0f);
    }

    double pointToSegmentDistanceSquared(const QVector3D& point, const QVector3D& segmentStart, const QVector3D& segmentEnd)
    {
        const QVector3D segment = segmentEnd - segmentStart;
        const double segmentLengthSquared = static_cast<double>(QVector3D::dotProduct(segment, segment));

        if (segmentLengthSquared <= kSortEpsilon)
        {
            return static_cast<double>((point - segmentStart).lengthSquared());
        }

        const double t = std::clamp
        (
            static_cast<double>(QVector3D::dotProduct(point - segmentStart, segment)) / segmentLengthSquared,
            0.0,
            1.0
        );
        const QVector3D projection = segmentStart + segment * static_cast<float>(t);
        return static_cast<double>((point - projection).lengthSquared());
    }

    double segmentToSegmentDistanceSquared(const QVector3D& firstStart, const QVector3D& firstEnd, const QVector3D& secondStart, const QVector3D& secondEnd)
    {
        const QVector3D u = firstEnd - firstStart;
        const QVector3D v = secondEnd - secondStart;
        const QVector3D w = firstStart - secondStart;
        const double a = static_cast<double>(QVector3D::dotProduct(u, u));
        const double b = static_cast<double>(QVector3D::dotProduct(u, v));
        const double c = static_cast<double>(QVector3D::dotProduct(v, v));

        if (a <= kSortEpsilon && c <= kSortEpsilon)
        {
            return static_cast<double>((firstStart - secondStart).lengthSquared());
        }

        if (a <= kSortEpsilon)
        {
            return pointToSegmentDistanceSquared(firstStart, secondStart, secondEnd);
        }

        if (c <= kSortEpsilon)
        {
            return pointToSegmentDistanceSquared(secondStart, firstStart, firstEnd);
        }

        const double d = static_cast<double>(QVector3D::dotProduct(u, w));
        const double e = static_cast<double>(QVector3D::dotProduct(v, w));
        const double denominator = a * c - b * b;

        double sNumerator = 0.0;
        double sDenominator = denominator;
        double tNumerator = 0.0;
        double tDenominator = denominator;

        if (denominator <= kSortEpsilon)
        {
            sNumerator = 0.0;
            sDenominator = 1.0;
            tNumerator = e;
            tDenominator = c;
        }
        else
        {
            sNumerator = b * e - c * d;
            tNumerator = a * e - b * d;

            if (sNumerator < 0.0)
            {
                sNumerator = 0.0;
                tNumerator = e;
                tDenominator = c;
            }
            else if (sNumerator > sDenominator)
            {
                sNumerator = sDenominator;
                tNumerator = e + b;
                tDenominator = c;
            }
        }

        if (tNumerator < 0.0)
        {
            tNumerator = 0.0;

            if (-d < 0.0)
            {
                sNumerator = 0.0;
            }
            else if (-d > a)
            {
                sNumerator = sDenominator;
            }
            else
            {
                sNumerator = -d;
                sDenominator = a;
            }
        }
        else if (tNumerator > tDenominator)
        {
            tNumerator = tDenominator;

            if ((-d + b) < 0.0)
            {
                sNumerator = 0.0;
            }
            else if ((-d + b) > a)
            {
                sNumerator = sDenominator;
            }
            else
            {
                sNumerator = -d + b;
                sDenominator = a;
            }
        }

        const double s = std::abs(sNumerator) <= kSortEpsilon ? 0.0 : sNumerator / sDenominator;
        const double t = std::abs(tNumerator) <= kSortEpsilon ? 0.0 : tNumerator / tDenominator;
        const QVector3D delta = w + u * static_cast<float>(s) - v * static_cast<float>(t);
        return static_cast<double>(QVector3D::dotProduct(delta, delta));
    }

    double planarSegmentToSegmentDistance(const QVector3D& firstStart, const QVector3D& firstEnd, const QVector3D& secondStart, const QVector3D& secondEnd)
    {
        return std::sqrt
        (
            segmentToSegmentDistanceSquared
            (
                flattenToSortPlane(firstStart),
                flattenToSortPlane(firstEnd),
                flattenToSortPlane(secondStart),
                flattenToSortPlane(secondEnd)
            )
        );
    }

    double spatialSegmentToSegmentDistance(const QVector3D& firstStart, const QVector3D& firstEnd, const QVector3D& secondStart, const QVector3D& secondEnd)
    {
        return std::sqrt(segmentToSegmentDistanceSquared(firstStart, firstEnd, secondStart, secondEnd));
    }

    double computeClosestConnectionDistance2D
    (
        const std::vector<ProcessConnectionSegment>& processedSegments,
        const QVector3D& candidateStartPoint,
        const QVector3D& candidateEndPoint
    )
    {
        if (processedSegments.empty())
        {
            return std::numeric_limits<double>::max();
        }

        double bestDistance = std::numeric_limits<double>::max();

        for (const ProcessConnectionSegment& segment : processedSegments)
        {
            bestDistance = std::min
            (
                bestDistance,
                planarSegmentToSegmentDistance(segment.startPoint, segment.endPoint, candidateStartPoint, candidateEndPoint)
            );

            if (bestDistance <= kSortConnectionEpsilon)
            {
                return 0.0;
            }
        }

        return bestDistance;
    }

    double computeClosestConnectionDistance3D
    (
        const std::vector<ProcessConnectionSegment>& processedSegments,
        const QVector3D& candidateStartPoint,
        const QVector3D& candidateEndPoint
    )
    {
        if (processedSegments.empty())
        {
            return std::numeric_limits<double>::max();
        }

        double bestDistance = std::numeric_limits<double>::max();

        for (const ProcessConnectionSegment& segment : processedSegments)
        {
            bestDistance = std::min
            (
                bestDistance,
                spatialSegmentToSegmentDistance(segment.startPoint, segment.endPoint, candidateStartPoint, candidateEndPoint)
            );

            if (bestDistance <= kSortConnectionEpsilon)
            {
                return 0.0;
            }
        }

        return bestDistance;
    }

    QVector3D normalizeOrZero(QVector3D vector)
    {
        vector.setZ(0.0f);

        if (vector.lengthSquared() <= kSortEpsilon)
        {
            return QVector3D();
        }

        vector.normalize();
        return vector;
    }

    QVector3D leftPerpendicular(const QVector3D& vector)
    {
        return QVector3D(-vector.y(), vector.x(), 0.0f);
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

    QVector3D bulgeArcCenter(const QVector3D& startPoint, const QVector3D& endPoint, double bulge, bool* valid = nullptr)
    {
        const QVector3D chord = endPoint - startPoint;
        const double chordLength = chord.length();

        if (valid != nullptr)
        {
            *valid = false;
        }

        if (chordLength <= kSortEpsilon || std::abs(bulge) < 1.0e-8)
        {
            return QVector3D();
        }

        const QVector3D midpoint = (startPoint + endPoint) * 0.5f;
        const double centerOffset = chordLength * (1.0 / bulge - bulge) * 0.25;
        const QVector3D leftNormal
        (
            static_cast<float>(-chord.y() / chordLength),
            static_cast<float>(chord.x() / chordLength),
            0.0f
        );

        if (valid != nullptr)
        {
            *valid = true;
        }

        return midpoint + leftNormal * static_cast<float>(centerOffset);
    }

    QVector3D bulgeSegmentTangentAtStart(const QVector3D& startPoint, const QVector3D& endPoint, double bulge)
    {
        if (std::abs(bulge) < 1.0e-8)
        {
            return normalizeOrZero(endPoint - startPoint);
        }

        bool valid = false;
        const QVector3D center = bulgeArcCenter(startPoint, endPoint, bulge, &valid);

        if (!valid)
        {
            return normalizeOrZero(endPoint - startPoint);
        }

        const QVector3D radiusVector = startPoint - center;
        const QVector3D tangent = bulge > 0.0
            ? leftPerpendicular(radiusVector)
            : -leftPerpendicular(radiusVector);

        return normalizeOrZero(tangent);
    }

    QVector3D bulgeSegmentTangentAtEnd(const QVector3D& startPoint, const QVector3D& endPoint, double bulge)
    {
        if (std::abs(bulge) < 1.0e-8)
        {
            return normalizeOrZero(endPoint - startPoint);
        }

        bool valid = false;
        const QVector3D center = bulgeArcCenter(startPoint, endPoint, bulge, &valid);

        if (!valid)
        {
            return normalizeOrZero(endPoint - startPoint);
        }

        const QVector3D radiusVector = endPoint - center;
        const QVector3D tangent = bulge > 0.0
            ? leftPerpendicular(radiusVector)
            : -leftPerpendicular(radiusVector);

        return normalizeOrZero(tangent);
    }

    bool tryBuildEllipseAxes(const DRW_Ellipse* ellipse, QVector3D& majorAxis, QVector3D& minorAxis)
    {
        if (ellipse == nullptr)
        {
            return false;
        }

        majorAxis = QVector3D(ellipse->secPoint.x, ellipse->secPoint.y, ellipse->secPoint.z);

        if (majorAxis.lengthSquared() <= kSortEpsilon || ellipse->ratio <= 0.0)
        {
            return false;
        }

        QVector3D normal(ellipse->extPoint.x, ellipse->extPoint.y, ellipse->extPoint.z);

        if (normal.lengthSquared() <= kSortEpsilon)
        {
            normal = QVector3D(0.0f, 0.0f, 1.0f);
        }
        else
        {
            normal.normalize();
        }

        minorAxis = QVector3D::crossProduct(normal, majorAxis);

        if (minorAxis.lengthSquared() <= kSortEpsilon)
        {
            return false;
        }

        minorAxis.normalize();
        minorAxis *= static_cast<float>(majorAxis.length() * ellipse->ratio);
        return true;
    }

    QVector3D resolveNormal(const DRW_Coord& extPoint)
    {
        QVector3D normal(extPoint.x, extPoint.y, extPoint.z);

        if (normal.lengthSquared() <= kSortEpsilon)
        {
            return QVector3D(0.0f, 0.0f, 1.0f);
        }

        normal.normalize();
        return normal;
    }

    void buildPlaneBasis(const QVector3D& normal, QVector3D& axisX, QVector3D& axisY)
    {
        if (std::abs(normal.x()) <= 1.0e-6f && std::abs(normal.y()) <= 1.0e-6f)
        {
            axisX = QVector3D(1.0f, 0.0f, 0.0f);
            axisY = QVector3D::crossProduct(normal, axisX);

            if (axisY.lengthSquared() <= kSortEpsilon)
            {
                axisY = QVector3D(0.0f, 1.0f, 0.0f);
            }
            else
            {
                axisY.normalize();
            }

            return;
        }

        const QVector3D helper = std::abs(normal.z()) < 0.999f
            ? QVector3D(0.0f, 0.0f, 1.0f)
            : QVector3D(0.0f, 1.0f, 0.0f);

        axisX = QVector3D::crossProduct(helper, normal);

        if (axisX.lengthSquared() <= kSortEpsilon)
        {
            axisX = QVector3D(1.0f, 0.0f, 0.0f);
        }
        else
        {
            axisX.normalize();
        }

        axisY = QVector3D::crossProduct(normal, axisX);

        if (axisY.lengthSquared() <= kSortEpsilon)
        {
            axisY = QVector3D(0.0f, 1.0f, 0.0f);
        }
        else
        {
            axisY.normalize();
        }
    }

    QVector3D arcPointAt(const DRW_Arc* arc, double angle)
    {
        if (arc == nullptr || arc->radious <= 0.0)
        {
            return QVector3D();
        }

        const QVector3D center(arc->basePoint.x, arc->basePoint.y, arc->basePoint.z);
        const QVector3D normal = resolveNormal(arc->extPoint);
        QVector3D axisX;
        QVector3D axisY;
        buildPlaneBasis(normal, axisX, axisY);

        return center
            + axisX * static_cast<float>(std::cos(angle) * arc->radious)
            + axisY * static_cast<float>(std::sin(angle) * arc->radious);
    }

    QVector3D arcTangentAt(const DRW_Arc* arc, double angle, bool reverseDirection)
    {
        if (arc == nullptr || arc->radious <= 0.0)
        {
            return QVector3D();
        }

        const QVector3D normal = resolveNormal(arc->extPoint);
        QVector3D axisX;
        QVector3D axisY;
        buildPlaneBasis(normal, axisX, axisY);

        QVector3D tangent
        (
            axisX * static_cast<float>(-std::sin(angle))
            + axisY * static_cast<float>(std::cos(angle))
        );

        if (reverseDirection)
        {
            tangent = -tangent;
        }

        return normalizeOrZero(tangent);
    }

    QVector3D circlePointAt(const DRW_Circle* circle, double parameter)
    {
        if (circle == nullptr || circle->radious <= 0.0)
        {
            return QVector3D();
        }

        const QVector3D center(circle->basePoint.x, circle->basePoint.y, circle->basePoint.z);
        const QVector3D normal = resolveNormal(circle->extPoint);
        QVector3D axisX;
        QVector3D axisY;
        buildPlaneBasis(normal, axisX, axisY);

        return center
            + axisX * static_cast<float>(std::cos(parameter) * circle->radious)
            + axisY * static_cast<float>(std::sin(parameter) * circle->radious);
    }

    QVector3D circleTangentAt(const DRW_Circle* circle, double parameter, bool reverseDirection)
    {
        if (circle == nullptr || circle->radious <= 0.0)
        {
            return QVector3D();
        }

        const QVector3D normal = resolveNormal(circle->extPoint);
        QVector3D axisX;
        QVector3D axisY;
        buildPlaneBasis(normal, axisX, axisY);

        QVector3D tangent
        (
            axisX * static_cast<float>(-std::sin(parameter))
            + axisY * static_cast<float>(std::cos(parameter))
        );

        if (reverseDirection)
        {
            tangent = -tangent;
        }

        return normalizeOrZero(tangent);
    }

    QVector3D ellipsePointAt(const DRW_Ellipse* ellipse, double parameter)
    {
        if (ellipse == nullptr)
        {
            return QVector3D();
        }

        const QVector3D center(ellipse->basePoint.x, ellipse->basePoint.y, ellipse->basePoint.z);
        QVector3D majorAxis;
        QVector3D minorAxis;

        if (!tryBuildEllipseAxes(ellipse, majorAxis, minorAxis))
        {
            return QVector3D();
        }

        return center
            + majorAxis * static_cast<float>(std::cos(parameter))
            + minorAxis * static_cast<float>(std::sin(parameter));
    }

    QVector3D ellipseTangentAt(const DRW_Ellipse* ellipse, double parameter, bool reverseDirection)
    {
        QVector3D majorAxis;
        QVector3D minorAxis;

        if (!tryBuildEllipseAxes(ellipse, majorAxis, minorAxis))
        {
            return QVector3D();
        }

        QVector3D tangent
        (
            static_cast<float>(-std::sin(parameter)) * majorAxis
            + static_cast<float>(std::cos(parameter)) * minorAxis
        );

        if (reverseDirection)
        {
            tangent = -tangent;
        }

        return normalizeOrZero(tangent);
    }

    QVector3D polylineForwardStartTangent(const DRW_Polyline* polyline)
    {
        if (polyline == nullptr || polyline->vertlist.size() < 2)
        {
            return QVector3D();
        }

        for (size_t index = 0; index + 1 < polyline->vertlist.size(); ++index)
        {
            const auto& current = polyline->vertlist.at(index);
            const auto& next = polyline->vertlist.at(index + 1);
            const QVector3D startPoint(current->basePoint.x, current->basePoint.y, current->basePoint.z);
            const QVector3D endPoint(next->basePoint.x, next->basePoint.y, next->basePoint.z);
            const QVector3D tangent = bulgeSegmentTangentAtStart(startPoint, endPoint, current->bulge);

            if (tangent.lengthSquared() > kSortEpsilon)
            {
                return tangent;
            }
        }

        if ((polyline->flags & 1) != 0)
        {
            const auto& current = polyline->vertlist.back();
            const auto& next = polyline->vertlist.front();
            const QVector3D startPoint(current->basePoint.x, current->basePoint.y, current->basePoint.z);
            const QVector3D endPoint(next->basePoint.x, next->basePoint.y, next->basePoint.z);
            return bulgeSegmentTangentAtStart(startPoint, endPoint, current->bulge);
        }

        return QVector3D();
    }

    QVector3D polylineForwardStartTangentAt(const DRW_Polyline* polyline, size_t startIndex)
    {
        if (polyline == nullptr || polyline->vertlist.size() < 2)
        {
            return QVector3D();
        }

        const size_t count = polyline->vertlist.size();
        const size_t nextIndex = (startIndex + 1) % count;

        if (nextIndex == startIndex)
        {
            return QVector3D();
        }

        const auto& current = polyline->vertlist.at(startIndex);
        const auto& next = polyline->vertlist.at(nextIndex);
        const QVector3D startPoint(current->basePoint.x, current->basePoint.y, current->basePoint.z);
        const QVector3D endPoint(next->basePoint.x, next->basePoint.y, next->basePoint.z);
        return bulgeSegmentTangentAtStart(startPoint, endPoint, current->bulge);
    }

    QVector3D polylineForwardEndTangentAt(const DRW_Polyline* polyline, size_t startIndex)
    {
        if (polyline == nullptr || polyline->vertlist.size() < 2)
        {
            return QVector3D();
        }

        const size_t count = polyline->vertlist.size();
        const size_t previousIndex = (startIndex + count - 1) % count;
        const auto& previous = polyline->vertlist.at(previousIndex);
        const auto& current = polyline->vertlist.at(startIndex);
        const QVector3D startPoint(previous->basePoint.x, previous->basePoint.y, previous->basePoint.z);
        const QVector3D endPoint(current->basePoint.x, current->basePoint.y, current->basePoint.z);
        return bulgeSegmentTangentAtEnd(startPoint, endPoint, previous->bulge);
    }

    QVector3D polylineReverseStartTangent(const DRW_Polyline* polyline)
    {
        if (polyline == nullptr || polyline->vertlist.size() < 2)
        {
            return QVector3D();
        }

        for (size_t index = polyline->vertlist.size() - 1; index > 0; --index)
        {
            const auto& current = polyline->vertlist.at(index);
            const auto& next = polyline->vertlist.at(index - 1);
            const QVector3D startPoint(current->basePoint.x, current->basePoint.y, current->basePoint.z);
            const QVector3D endPoint(next->basePoint.x, next->basePoint.y, next->basePoint.z);
            const QVector3D tangent = bulgeSegmentTangentAtStart(startPoint, endPoint, -next->bulge);

            if (tangent.lengthSquared() > kSortEpsilon)
            {
                return tangent;
            }
        }

        if ((polyline->flags & 1) != 0)
        {
            const auto& current = polyline->vertlist.front();
            const auto& next = polyline->vertlist.back();
            const QVector3D startPoint(current->basePoint.x, current->basePoint.y, current->basePoint.z);
            const QVector3D endPoint(next->basePoint.x, next->basePoint.y, next->basePoint.z);
            return bulgeSegmentTangentAtStart(startPoint, endPoint, -next->bulge);
        }

        return QVector3D();
    }

    QVector3D polylineReverseStartTangentAt(const DRW_Polyline* polyline, size_t startIndex)
    {
        return -polylineForwardEndTangentAt(polyline, startIndex);
    }

    QVector3D polylineReverseEndTangentAt(const DRW_Polyline* polyline, size_t startIndex)
    {
        return -polylineForwardStartTangentAt(polyline, startIndex);
    }

    QVector3D lwPolylineForwardStartTangent(const DRW_LWPolyline* polyline)
    {
        if (polyline == nullptr || polyline->vertlist.size() < 2)
        {
            return QVector3D();
        }

        const float z = static_cast<float>(polyline->elevation);

        for (size_t index = 0; index + 1 < polyline->vertlist.size(); ++index)
        {
            const auto& current = polyline->vertlist.at(index);
            const auto& next = polyline->vertlist.at(index + 1);
            const QVector3D startPoint(static_cast<float>(current->x), static_cast<float>(current->y), z);
            const QVector3D endPoint(static_cast<float>(next->x), static_cast<float>(next->y), z);
            const QVector3D tangent = bulgeSegmentTangentAtStart(startPoint, endPoint, current->bulge);

            if (tangent.lengthSquared() > kSortEpsilon)
            {
                return tangent;
            }
        }

        if ((polyline->flags & 1) != 0)
        {
            const auto& current = polyline->vertlist.back();
            const auto& next = polyline->vertlist.front();
            const QVector3D startPoint(static_cast<float>(current->x), static_cast<float>(current->y), z);
            const QVector3D endPoint(static_cast<float>(next->x), static_cast<float>(next->y), z);
            return bulgeSegmentTangentAtStart(startPoint, endPoint, current->bulge);
        }

        return QVector3D();
    }

    QVector3D lwPolylineForwardStartTangentAt(const DRW_LWPolyline* polyline, size_t startIndex)
    {
        if (polyline == nullptr || polyline->vertlist.size() < 2)
        {
            return QVector3D();
        }

        const size_t count = polyline->vertlist.size();
        const size_t nextIndex = (startIndex + 1) % count;
        const float z = static_cast<float>(polyline->elevation);
        const auto& current = polyline->vertlist.at(startIndex);
        const auto& next = polyline->vertlist.at(nextIndex);
        const QVector3D startPoint(static_cast<float>(current->x), static_cast<float>(current->y), z);
        const QVector3D endPoint(static_cast<float>(next->x), static_cast<float>(next->y), z);
        return bulgeSegmentTangentAtStart(startPoint, endPoint, current->bulge);
    }

    QVector3D lwPolylineForwardEndTangentAt(const DRW_LWPolyline* polyline, size_t startIndex)
    {
        if (polyline == nullptr || polyline->vertlist.size() < 2)
        {
            return QVector3D();
        }

        const size_t count = polyline->vertlist.size();
        const size_t previousIndex = (startIndex + count - 1) % count;
        const float z = static_cast<float>(polyline->elevation);
        const auto& previous = polyline->vertlist.at(previousIndex);
        const auto& current = polyline->vertlist.at(startIndex);
        const QVector3D startPoint(static_cast<float>(previous->x), static_cast<float>(previous->y), z);
        const QVector3D endPoint(static_cast<float>(current->x), static_cast<float>(current->y), z);
        return bulgeSegmentTangentAtEnd(startPoint, endPoint, previous->bulge);
    }

    QVector3D lwPolylineReverseStartTangent(const DRW_LWPolyline* polyline)
    {
        if (polyline == nullptr || polyline->vertlist.size() < 2)
        {
            return QVector3D();
        }

        const float z = static_cast<float>(polyline->elevation);

        for (size_t index = polyline->vertlist.size() - 1; index > 0; --index)
        {
            const auto& current = polyline->vertlist.at(index);
            const auto& next = polyline->vertlist.at(index - 1);
            const QVector3D startPoint(static_cast<float>(current->x), static_cast<float>(current->y), z);
            const QVector3D endPoint(static_cast<float>(next->x), static_cast<float>(next->y), z);
            const QVector3D tangent = bulgeSegmentTangentAtStart(startPoint, endPoint, -next->bulge);

            if (tangent.lengthSquared() > kSortEpsilon)
            {
                return tangent;
            }
        }

        if ((polyline->flags & 1) != 0)
        {
            const auto& current = polyline->vertlist.front();
            const auto& next = polyline->vertlist.back();
            const QVector3D startPoint(static_cast<float>(current->x), static_cast<float>(current->y), z);
            const QVector3D endPoint(static_cast<float>(next->x), static_cast<float>(next->y), z);
            return bulgeSegmentTangentAtStart(startPoint, endPoint, -next->bulge);
        }

        return QVector3D();
    }

    QVector3D lwPolylineReverseStartTangentAt(const DRW_LWPolyline* polyline, size_t startIndex)
    {
        return -lwPolylineForwardEndTangentAt(polyline, startIndex);
    }

    QVector3D lwPolylineReverseEndTangentAt(const DRW_LWPolyline* polyline, size_t startIndex)
    {
        return -lwPolylineForwardStartTangentAt(polyline, startIndex);
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

    QVector3D computeSweepDirection(const std::vector<CadItem*>& sortableItems)
    {
        bool hasAnchor = false;
        QVector3D minPoint;
        QVector3D maxPoint;

        for (CadItem* item : sortableItems)
        {
            const CadProcessVisualInfo info = buildProcessVisualInfo(item);

            if (!info.valid)
            {
                continue;
            }

            if (!hasAnchor)
            {
                minPoint = info.labelAnchor;
                maxPoint = info.labelAnchor;
                hasAnchor = true;
                continue;
            }

            minPoint.setX(std::min(minPoint.x(), info.labelAnchor.x()));
            minPoint.setY(std::min(minPoint.y(), info.labelAnchor.y()));
            maxPoint.setX(std::max(maxPoint.x(), info.labelAnchor.x()));
            maxPoint.setY(std::max(maxPoint.y(), info.labelAnchor.y()));
        }

        if (!hasAnchor)
        {
            return normalizeOrZero(QVector3D(1.0f, 1.0f, 0.0f));
        }

        const QVector3D diagonal(maxPoint.x() - minPoint.x(), maxPoint.y() - minPoint.y(), 0.0f);
        const QVector3D normalized = normalizeOrZero(diagonal);
        return normalized.lengthSquared() > kSortEpsilon
            ? normalized
            : normalizeOrZero(QVector3D(1.0f, 1.0f, 0.0f));
    }

    QVector3D computeRotarySweepDirection(const std::vector<CadItem*>& sortableItems, const GProfileRotaryAxisConfig& config)
    {
        bool hasBounds = false;
        double minAxis = 0.0;
        double maxAxis = 0.0;
        double minAngle = 0.0;
        double maxAngle = 0.0;
        double referenceAngle = 0.0;

        for (CadItem* item : sortableItems)
        {
            const std::vector<ProcessPathOption> options = buildPathOptionsForItem(item, SortStrategy::KeepDirection);

            if (options.empty())
            {
                continue;
            }

            RotarySortPoint rotaryPoint;

            if (!tryBuildRotarySortPoint(options.front().startPoint, config, rotaryPoint))
            {
                continue;
            }

            if (!hasBounds)
            {
                minAxis = maxAxis = rotaryPoint.axis;
                minAngle = maxAngle = rotaryPoint.angleDegrees;
                referenceAngle = rotaryPoint.angleDegrees;
                hasBounds = true;
                continue;
            }

            const double resolvedAngle = unwrapAngleDegrees(referenceAngle, rotaryPoint.angleDegrees);
            minAxis = std::min(minAxis, rotaryPoint.axis);
            maxAxis = std::max(maxAxis, rotaryPoint.axis);
            minAngle = std::min(minAngle, resolvedAngle);
            maxAngle = std::max(maxAngle, resolvedAngle);
        }

        if (!hasBounds)
        {
            return normalizeOrZero(QVector3D(1.0f, 1.0f, 0.0f));
        }

        const QVector3D diagonal
        (
            static_cast<float>(maxAxis - minAxis),
            static_cast<float>(maxAngle - minAngle),
            0.0f
        );
        const QVector3D normalized = normalizeOrZero(diagonal);
        return normalized.lengthSquared() > kSortEpsilon
            ? normalized
            : normalizeOrZero(QVector3D(1.0f, 1.0f, 0.0f));
    }

    double movementContinuityPenalty(const QVector3D& moveVector, const QVector3D& tangentVector)
    {
        const QVector3D normalizedMove = normalizeOrZero(moveVector);
        const QVector3D normalizedTangent = normalizeOrZero(tangentVector);

        if (normalizedMove.lengthSquared() <= kSortEpsilon || normalizedTangent.lengthSquared() <= kSortEpsilon)
        {
            return 0.0;
        }

        const double alignment = std::clamp(static_cast<double>(QVector3D::dotProduct(normalizedMove, normalizedTangent)), -1.0, 1.0);
        return 1.0 - alignment;
    }

    double rotaryMovementContinuityPenalty
    (
        const QVector3D& fromPoint,
        const QVector3D& toPoint,
        const QVector3D& tangentVector,
        const GProfileRotaryAxisConfig& config
    )
    {
        RotarySortPoint fromRotaryPoint;
        RotarySortPoint toRotaryPoint;

        if (!tryBuildRotarySortPoint(fromPoint, config, fromRotaryPoint) || !tryBuildRotarySortPoint(toPoint, config, toRotaryPoint))
        {
            return movementContinuityPenalty(toPoint - fromPoint, tangentVector);
        }

        const double resolvedToAngle = unwrapAngleDegrees(fromRotaryPoint.angleDegrees, toRotaryPoint.angleDegrees);
        QVector3D movementVector
        (
            static_cast<float>(toPoint.x() - fromPoint.x()),
            static_cast<float>(resolvedToAngle - fromRotaryPoint.angleDegrees),
            0.0f
        );
        QVector3D tangentRotary
        (
            tangentVector.x(),
            tangentVector.y() * static_cast<float>(kRotaryAngleDistanceWeight),
            0.0f
        );

        return movementContinuityPenalty(movementVector, tangentRotary);
    }

    std::vector<ProcessPathOption> buildPathOptionsForItem(const CadItem* item, SortStrategy strategy)
    {
        std::vector<ProcessPathOption> options;

        if (item == nullptr || item->m_nativeEntity == nullptr)
        {
            return options;
        }

        switch (item->m_type)
        {
        case DRW::ETYPE::LINE:
        {
            const DRW_Line* line = static_cast<const DRW_Line*>(item->m_nativeEntity);
            const QVector3D forwardStart(line->basePoint.x, line->basePoint.y, line->basePoint.z);
            const QVector3D forwardEnd(line->secPoint.x, line->secPoint.y, line->secPoint.z);
            const QVector3D forwardTangent = normalizeOrZero(forwardEnd - forwardStart);
            const std::initializer_list<bool> reverseOptions = strategy == SortStrategy::Smart
                ? std::initializer_list<bool>{ false, true }
                : std::initializer_list<bool>{ item->m_isReverse };

            for (const bool reverse : reverseOptions)
            {
                ProcessPathOption option;
                option.reverse = reverse;
                option.startPoint = reverse ? forwardEnd : forwardStart;
                option.endPoint = reverse ? forwardStart : forwardEnd;
                option.startTangent = reverse ? -forwardTangent : forwardTangent;
                option.endTangent = option.startTangent;
                options.push_back(option);
            }

            break;
        }
        case DRW::ETYPE::ARC:
        {
            const DRW_Arc* arc = static_cast<const DRW_Arc*>(item->m_nativeEntity);
            const std::initializer_list<bool> reverseOptions = strategy == SortStrategy::Smart
                ? std::initializer_list<bool>{ false, true }
                : std::initializer_list<bool>{ item->m_isReverse };

            for (const bool reverse : reverseOptions)
            {
                ProcessPathOption option;
                option.reverse = reverse;
                option.startPoint = reverse ? arcPointAt(arc, arc->endangle) : arcPointAt(arc, arc->staangle);
                option.endPoint = reverse ? arcPointAt(arc, arc->staangle) : arcPointAt(arc, arc->endangle);
                option.startTangent = reverse ? arcTangentAt(arc, arc->endangle, true) : arcTangentAt(arc, arc->staangle, false);
                option.endTangent = reverse ? arcTangentAt(arc, arc->staangle, true) : arcTangentAt(arc, arc->endangle, false);
                options.push_back(option);
            }

            break;
        }
        case DRW::ETYPE::CIRCLE:
        {
            const DRW_Circle* circle = static_cast<const DRW_Circle*>(item->m_nativeEntity);
            const std::initializer_list<bool> reverseOptions = strategy == SortStrategy::Smart
                ? std::initializer_list<bool>{ false, true }
                : std::initializer_list<bool>{ item->m_isReverse };
            const double startParameter = strategy == SortStrategy::Smart
                ? kPi * 0.5
                : (item->m_hasCustomProcessStart ? item->m_processStartParameter : kPi * 0.5);

            for (const bool reverse : reverseOptions)
            {
                ProcessPathOption option;
                option.reverse = reverse;
                option.hasCustomStart = false;
                option.processStartParameter = startParameter;
                option.startPoint = circlePointAt(circle, startParameter);
                option.endPoint = option.startPoint;
                option.startTangent = circleTangentAt(circle, startParameter, reverse);
                option.endTangent = option.startTangent;
                options.push_back(option);
            }

            break;
        }
        case DRW::ETYPE::ELLIPSE:
        {
            const DRW_Ellipse* ellipse = static_cast<const DRW_Ellipse*>(item->m_nativeEntity);
            const bool isClosed = isFullEllipsePath(ellipse);

            if (isClosed && strategy == SortStrategy::Smart)
            {
                for (int sampleIndex = 0; sampleIndex < kClosedEllipseSampleCount; ++sampleIndex)
                {
                    const double parameter = kTwoPi * static_cast<double>(sampleIndex) / static_cast<double>(kClosedEllipseSampleCount);

                    for (const bool reverse : { false, true })
                    {
                        ProcessPathOption option;
                        option.reverse = reverse;
                        option.hasCustomStart = true;
                        option.processStartParameter = parameter;
                        option.startPoint = ellipsePointAt(ellipse, parameter);
                        option.endPoint = option.startPoint;
                        option.startTangent = ellipseTangentAt(ellipse, parameter, reverse);
                        option.endTangent = option.startTangent;
                        options.push_back(option);
                    }
                }
            }
            else
            {
                const std::initializer_list<bool> reverseOptions = strategy == SortStrategy::Smart
                    ? std::initializer_list<bool>{ false, true }
                    : std::initializer_list<bool>{ item->m_isReverse };

                double startParam = ellipse->staparam;
                double endParam = ellipse->endparam;
                bool hasCustomStart = false;

                if (isClosed)
                {
                    hasCustomStart = item->m_hasCustomProcessStart;
                    startParam = item->m_hasCustomProcessStart ? item->m_processStartParameter : ellipse->staparam;
                    endParam = startParam;
                }
                else
                {
                    while (endParam <= startParam)
                    {
                        endParam += kTwoPi;
                    }
                }

                for (const bool reverse : reverseOptions)
                {
                    ProcessPathOption option;
                    option.reverse = reverse;
                    option.hasCustomStart = hasCustomStart;
                    option.processStartParameter = startParam;
                    option.startPoint = reverse ? ellipsePointAt(ellipse, endParam) : ellipsePointAt(ellipse, startParam);
                    option.endPoint = reverse ? ellipsePointAt(ellipse, startParam) : ellipsePointAt(ellipse, endParam);
                    option.startTangent = reverse ? ellipseTangentAt(ellipse, endParam, true) : ellipseTangentAt(ellipse, startParam, false);
                    option.endTangent = reverse ? ellipseTangentAt(ellipse, startParam, true) : ellipseTangentAt(ellipse, endParam, false);
                    options.push_back(option);
                }
            }

            break;
        }
        case DRW::ETYPE::POLYLINE:
        {
            const DRW_Polyline* polyline = static_cast<const DRW_Polyline*>(item->m_nativeEntity);

            if (polyline->vertlist.empty())
            {
                break;
            }

            const bool isClosed = (polyline->flags & 1) != 0;

            if (isClosed && strategy == SortStrategy::Smart)
            {
                const size_t count = polyline->vertlist.size();

                for (size_t startIndex = 0; startIndex < count; ++startIndex)
                {
                    const auto& seamVertex = polyline->vertlist.at(startIndex);
                    const QVector3D seamPoint(seamVertex->basePoint.x, seamVertex->basePoint.y, seamVertex->basePoint.z);

                    for (const bool reverse : { false, true })
                    {
                        ProcessPathOption option;
                        option.reverse = reverse;
                        option.hasCustomStart = true;
                        option.processStartParameter = static_cast<double>(startIndex);
                        option.startPoint = seamPoint;
                        option.endPoint = seamPoint;
                        option.startTangent = reverse
                            ? polylineReverseStartTangentAt(polyline, startIndex)
                            : polylineForwardStartTangentAt(polyline, startIndex);
                        option.endTangent = reverse
                            ? polylineReverseEndTangentAt(polyline, startIndex)
                            : polylineForwardEndTangentAt(polyline, startIndex);
                        options.push_back(option);
                    }
                }
            }
            else
            {
                const auto& firstVertex = polyline->vertlist.front();
                const auto& lastVertex = polyline->vertlist.back();
                const size_t seamIndex = isClosed
                    ? effectiveClosedPolylineStartIndex(item, polyline->vertlist.size())
                    : 0;
                const QVector3D forwardStart = isClosed
                    ? QVector3D(polyline->vertlist.at(seamIndex)->basePoint.x, polyline->vertlist.at(seamIndex)->basePoint.y, polyline->vertlist.at(seamIndex)->basePoint.z)
                    : QVector3D(firstVertex->basePoint.x, firstVertex->basePoint.y, firstVertex->basePoint.z);
                const QVector3D forwardEnd = isClosed
                    ? forwardStart
                    : QVector3D(lastVertex->basePoint.x, lastVertex->basePoint.y, lastVertex->basePoint.z);
                const QVector3D forwardStartTangent = isClosed
                    ? polylineForwardStartTangentAt(polyline, seamIndex)
                    : polylineForwardStartTangent(polyline);
                const QVector3D reverseStartTangent = isClosed
                    ? polylineReverseStartTangentAt(polyline, seamIndex)
                    : polylineReverseStartTangent(polyline);
                const QVector3D forwardEndTangent = isClosed
                    ? polylineForwardEndTangentAt(polyline, seamIndex)
                    : -reverseStartTangent;
                const QVector3D reverseEndTangent = isClosed
                    ? polylineReverseEndTangentAt(polyline, seamIndex)
                    : -forwardStartTangent;
                const std::initializer_list<bool> reverseOptions = strategy == SortStrategy::Smart
                    ? std::initializer_list<bool>{ false, true }
                    : std::initializer_list<bool>{ item->m_isReverse };

                for (const bool reverse : reverseOptions)
                {
                    ProcessPathOption option;
                    option.reverse = reverse;
                    option.hasCustomStart = isClosed && item->m_hasCustomProcessStart;
                    option.processStartParameter = isClosed ? static_cast<double>(seamIndex) : 0.0;
                    option.startPoint = reverse ? forwardEnd : forwardStart;
                    option.endPoint = reverse ? forwardStart : forwardEnd;
                    option.startTangent = reverse ? reverseStartTangent : forwardStartTangent;
                    option.endTangent = reverse ? reverseEndTangent : forwardEndTangent;
                    options.push_back(option);
                }
            }

            break;
        }
        case DRW::ETYPE::LWPOLYLINE:
        {
            const DRW_LWPolyline* polyline = static_cast<const DRW_LWPolyline*>(item->m_nativeEntity);

            if (polyline->vertlist.empty())
            {
                break;
            }

            const bool isClosed = (polyline->flags & 1) != 0;

            if (isClosed && strategy == SortStrategy::Smart)
            {
                const size_t count = polyline->vertlist.size();
                const float z = static_cast<float>(polyline->elevation);

                for (size_t startIndex = 0; startIndex < count; ++startIndex)
                {
                    const auto& seamVertex = polyline->vertlist.at(startIndex);
                    const QVector3D seamPoint(static_cast<float>(seamVertex->x), static_cast<float>(seamVertex->y), z);

                    for (const bool reverse : { false, true })
                    {
                        ProcessPathOption option;
                        option.reverse = reverse;
                        option.hasCustomStart = true;
                        option.processStartParameter = static_cast<double>(startIndex);
                        option.startPoint = seamPoint;
                        option.endPoint = seamPoint;
                        option.startTangent = reverse
                            ? lwPolylineReverseStartTangentAt(polyline, startIndex)
                            : lwPolylineForwardStartTangentAt(polyline, startIndex);
                        option.endTangent = reverse
                            ? lwPolylineReverseEndTangentAt(polyline, startIndex)
                            : lwPolylineForwardEndTangentAt(polyline, startIndex);
                        options.push_back(option);
                    }
                }
            }
            else
            {
                const auto& firstVertex = polyline->vertlist.front();
                const auto& lastVertex = polyline->vertlist.back();
                const float z = static_cast<float>(polyline->elevation);
                const size_t seamIndex = isClosed
                    ? effectiveClosedPolylineStartIndex(item, polyline->vertlist.size())
                    : 0;
                const QVector3D forwardStart = isClosed
                    ? QVector3D(static_cast<float>(polyline->vertlist.at(seamIndex)->x), static_cast<float>(polyline->vertlist.at(seamIndex)->y), z)
                    : QVector3D(static_cast<float>(firstVertex->x), static_cast<float>(firstVertex->y), z);
                const QVector3D forwardEnd = isClosed
                    ? forwardStart
                    : QVector3D(static_cast<float>(lastVertex->x), static_cast<float>(lastVertex->y), z);
                const QVector3D forwardStartTangent = isClosed
                    ? lwPolylineForwardStartTangentAt(polyline, seamIndex)
                    : lwPolylineForwardStartTangent(polyline);
                const QVector3D reverseStartTangent = isClosed
                    ? lwPolylineReverseStartTangentAt(polyline, seamIndex)
                    : lwPolylineReverseStartTangent(polyline);
                const QVector3D forwardEndTangent = isClosed
                    ? lwPolylineForwardEndTangentAt(polyline, seamIndex)
                    : -reverseStartTangent;
                const QVector3D reverseEndTangent = isClosed
                    ? lwPolylineReverseEndTangentAt(polyline, seamIndex)
                    : -forwardStartTangent;
                const std::initializer_list<bool> reverseOptions = strategy == SortStrategy::Smart
                    ? std::initializer_list<bool>{ false, true }
                    : std::initializer_list<bool>{ item->m_isReverse };

                for (const bool reverse : reverseOptions)
                {
                    ProcessPathOption option;
                    option.reverse = reverse;
                    option.hasCustomStart = isClosed && item->m_hasCustomProcessStart;
                    option.processStartParameter = isClosed ? static_cast<double>(seamIndex) : 0.0;
                    option.startPoint = reverse ? forwardEnd : forwardStart;
                    option.endPoint = reverse ? forwardStart : forwardEnd;
                    option.startTangent = reverse ? reverseStartTangent : forwardStartTangent;
                    option.endTangent = reverse ? reverseEndTangent : forwardEndTangent;
                    options.push_back(option);
                }
            }

            break;
        }
        default:
            break;
        }

        return options;
    }

    std::vector<EndpointNode> collectOpenEndpointNodes(const std::vector<CadItem*>& sortableItems)
    {
        std::vector<EndpointNode> endpoints;

        for (size_t index = 0; index < sortableItems.size(); ++index)
        {
            const std::vector<ProcessPathOption> options = buildPathOptionsForItem(sortableItems[index], SortStrategy::KeepDirection);

            if (options.empty())
            {
                continue;
            }

            const ProcessPathOption& option = options.front();

            if (spatialDistanceSquared(option.startPoint, option.endPoint) <= kSortConnectionEpsilon * kSortConnectionEpsilon)
            {
                continue;
            }

            endpoints.push_back({ index, option.startPoint });
            endpoints.push_back({ index, option.endPoint });
        }

        return endpoints;
    }

    std::vector<int> collectLooseEndpointIndices
    (
        const std::vector<EndpointNode>& endpoints,
        double exactConnectionDistance
    )
    {
        std::vector<int> looseIndices;
        const double exactConnectionDistanceSquared = exactConnectionDistance * exactConnectionDistance;

        for (size_t index = 0; index < endpoints.size(); ++index)
        {
            bool hasExactMatch = false;

            for (size_t otherIndex = 0; otherIndex < endpoints.size(); ++otherIndex)
            {
                if (index == otherIndex || endpoints[index].itemIndex == endpoints[otherIndex].itemIndex)
                {
                    continue;
                }

                if (spatialDistanceSquared(endpoints[index].point, endpoints[otherIndex].point) <= exactConnectionDistanceSquared)
                {
                    hasExactMatch = true;
                    break;
                }
            }

            if (!hasExactMatch)
            {
                looseIndices.push_back(static_cast<int>(index));
            }
        }

        return looseIndices;
    }

    std::vector<int> buildItemConnectivityComponents(const std::vector<CadItem*>& sortableItems)
    {
        const std::vector<EndpointNode> endpoints = collectOpenEndpointNodes(sortableItems);
        std::vector<std::vector<size_t>> itemEndpointIndices(sortableItems.size());
        const double exactConnectionDistanceSquared = kSortConnectionEpsilon * kSortConnectionEpsilon;

        for (size_t endpointIndex = 0; endpointIndex < endpoints.size(); ++endpointIndex)
        {
            itemEndpointIndices[endpoints[endpointIndex].itemIndex].push_back(endpointIndex);
        }

        std::vector<std::vector<size_t>> adjacency(sortableItems.size());

        for (size_t leftItem = 0; leftItem < itemEndpointIndices.size(); ++leftItem)
        {
            for (size_t rightItem = leftItem + 1; rightItem < itemEndpointIndices.size(); ++rightItem)
            {
                bool connected = false;

                for (size_t leftEndpointIndex : itemEndpointIndices[leftItem])
                {
                    for (size_t rightEndpointIndex : itemEndpointIndices[rightItem])
                    {
                        if (spatialDistanceSquared(endpoints[leftEndpointIndex].point, endpoints[rightEndpointIndex].point) <= exactConnectionDistanceSquared)
                        {
                            connected = true;
                            break;
                        }
                    }

                    if (connected)
                    {
                        break;
                    }
                }

                if (connected)
                {
                    adjacency[leftItem].push_back(rightItem);
                    adjacency[rightItem].push_back(leftItem);
                }
            }
        }

        std::vector<int> componentIds(sortableItems.size(), -1);
        int nextComponentId = 0;

        for (size_t itemIndex = 0; itemIndex < sortableItems.size(); ++itemIndex)
        {
            if (componentIds[itemIndex] >= 0)
            {
                continue;
            }

            std::vector<size_t> stack = { itemIndex };
            componentIds[itemIndex] = nextComponentId;

            while (!stack.empty())
            {
                const size_t currentItem = stack.back();
                stack.pop_back();

                for (size_t neighborItem : adjacency[currentItem])
                {
                    if (componentIds[neighborItem] >= 0)
                    {
                        continue;
                    }

                    componentIds[neighborItem] = nextComponentId;
                    stack.push_back(neighborItem);
                }
            }

            ++nextComponentId;
        }

        return componentIds;
    }

    std::vector<std::vector<QVector3D>> detectPreferredGapStartPointsByComponent
    (
        const std::vector<CadItem*>& sortableItems,
        const std::vector<int>& componentIds,
        double preferredGapDistance
    )
    {
        const std::vector<EndpointNode> endpoints = collectOpenEndpointNodes(sortableItems);
        const std::vector<int> looseIndices = collectLooseEndpointIndices(endpoints, kSortConnectionEpsilon);
        const int componentCount = componentIds.empty()
            ? 0
            : (*std::max_element(componentIds.begin(), componentIds.end()) + 1);
        std::vector<std::vector<QVector3D>> preferredPointsByComponent(static_cast<size_t>(std::max(0, componentCount)));

        if (looseIndices.size() < 2 || componentCount <= 0)
        {
            return preferredPointsByComponent;
        }

        std::vector<int> nearestLooseIndex(looseIndices.size(), -1);
        std::vector<double> nearestLooseDistance(looseIndices.size(), std::numeric_limits<double>::max());

        for (size_t localIndex = 0; localIndex < looseIndices.size(); ++localIndex)
        {
            const EndpointNode& node = endpoints[static_cast<size_t>(looseIndices[localIndex])];
            const int componentId = componentIds[node.itemIndex];

            if (componentId < 0)
            {
                continue;
            }

            for (size_t otherLocalIndex = 0; otherLocalIndex < looseIndices.size(); ++otherLocalIndex)
            {
                if (localIndex == otherLocalIndex)
                {
                    continue;
                }

                const EndpointNode& otherNode = endpoints[static_cast<size_t>(looseIndices[otherLocalIndex])];

                if (node.itemIndex == otherNode.itemIndex)
                {
                    continue;
                }

                if (componentIds[otherNode.itemIndex] != componentId)
                {
                    continue;
                }

                const double distance = std::sqrt(spatialDistanceSquared(node.point, otherNode.point));

                const bool shouldReplace = nearestLooseIndex[localIndex] < 0
                    || distance < nearestLooseDistance[localIndex] - kSortEpsilon
                    || (std::abs(distance - nearestLooseDistance[localIndex]) <= kSortEpsilon
                        && isPointLexicographicallyLess
                        (
                            otherNode.point,
                            endpoints[static_cast<size_t>(looseIndices[static_cast<size_t>(nearestLooseIndex[localIndex])])].point
                        ));

                if (shouldReplace)
                {
                    nearestLooseDistance[localIndex] = distance;
                    nearestLooseIndex[localIndex] = static_cast<int>(otherLocalIndex);
                }
            }
        }

        std::vector<QVector3D> preferredPoints;

        for (size_t localIndex = 0; localIndex < looseIndices.size(); ++localIndex)
        {
            const int nearestIndex = nearestLooseIndex[localIndex];
            const EndpointNode& firstNode = endpoints[static_cast<size_t>(looseIndices[localIndex])];
            const int componentId = componentIds[firstNode.itemIndex];

            if (nearestIndex < 0
                || componentId < 0
                || nearestLooseDistance[localIndex] > preferredGapDistance + kSortEpsilon
                || nearestLooseIndex[static_cast<size_t>(nearestIndex)] != static_cast<int>(localIndex))
            {
                continue;
            }

            std::vector<QVector3D>& preferredPoints = preferredPointsByComponent[static_cast<size_t>(componentId)];
            const QVector3D firstPoint = firstNode.point;
            const QVector3D secondPoint = endpoints[static_cast<size_t>(looseIndices[static_cast<size_t>(nearestIndex)])].point;

            if (!isPointNearAnyPreferredStart(firstPoint, preferredPoints, kSortConnectionEpsilon))
            {
                preferredPoints.push_back(firstPoint);
            }

            if (!isPointNearAnyPreferredStart(secondPoint, preferredPoints, kSortConnectionEpsilon))
            {
                preferredPoints.push_back(secondPoint);
            }
        }

        return preferredPointsByComponent;
    }

    std::vector<bool> buildVisitedComponentMask(const std::vector<bool>& visited, const std::vector<int>& componentIds)
    {
        const int componentCount = componentIds.empty()
            ? 0
            : (*std::max_element(componentIds.begin(), componentIds.end()) + 1);
        std::vector<bool> visitedComponents(static_cast<size_t>(std::max(0, componentCount)), false);

        for (size_t itemIndex = 0; itemIndex < visited.size() && itemIndex < componentIds.size(); ++itemIndex)
        {
            const int componentId = componentIds[itemIndex];

            if (visited[itemIndex] && componentId >= 0)
            {
                visitedComponents[static_cast<size_t>(componentId)] = true;
            }
        }

        return visitedComponents;
    }

    bool hasRemainingUnvisitedInComponent
    (
        const std::vector<bool>& visited,
        const std::vector<int>& componentIds,
        int componentId
    )
    {
        if (componentId < 0)
        {
            return false;
        }

        for (size_t itemIndex = 0; itemIndex < visited.size() && itemIndex < componentIds.size(); ++itemIndex)
        {
            if (!visited[itemIndex] && componentIds[itemIndex] == componentId)
            {
                return true;
            }
        }

        return false;
    }

    GapStartSelectionContext buildGapStartSelectionContext(const std::vector<CadItem*>& sortableItems, double preferredGapDistance)
    {
        GapStartSelectionContext context;
        context.componentIds = buildItemConnectivityComponents(sortableItems);
        context.preferredStartPointsByComponent = detectPreferredGapStartPointsByComponent
        (
            sortableItems,
            context.componentIds,
            preferredGapDistance
        );
        return context;
    }

    bool tryFindNearestNextStartPoint
    (
        const std::vector<CadItem*>& sortableItems,
        const std::vector<bool>& visited,
        SortStrategy strategy,
        size_t currentIndex,
        const QVector3D& currentEndPoint,
        QVector3D& nextStartPoint
    )
    {
        int bestIndex = -1;
        double bestDistance = std::numeric_limits<double>::max();
        QVector3D bestStartPoint;

        for (size_t index = 0; index < sortableItems.size(); ++index)
        {
            if (index == currentIndex || visited[index])
            {
                continue;
            }

            const std::vector<ProcessPathOption> options = buildPathOptionsForItem(sortableItems[index], strategy);

            for (const ProcessPathOption& option : options)
            {
                const double distance = std::sqrt(planarDistanceSquared(option.startPoint, currentEndPoint));
                const bool shouldReplace = bestIndex < 0
                    || distance < bestDistance - kSortEpsilon
                    || (std::abs(distance - bestDistance) <= kSortEpsilon
                        && isPointLexicographicallyLess(option.startPoint, bestStartPoint));

                if (!shouldReplace)
                {
                    continue;
                }

                bestIndex = static_cast<int>(index);
                bestDistance = distance;
                bestStartPoint = option.startPoint;
            }
        }

        if (bestIndex < 0)
        {
            return false;
        }

        nextStartPoint = bestStartPoint;
        return true;
    }

    bool tryFindNearestNextStartPoint3D
    (
        const std::vector<CadItem*>& sortableItems,
        const std::vector<bool>& visited,
        SortStrategy strategy,
        size_t currentIndex,
        const QVector3D& currentEndPoint,
        const GProfileRotaryAxisConfig& config,
        QVector3D& nextStartPoint
    )
    {
        int bestIndex = -1;
        double bestDistance = std::numeric_limits<double>::max();
        QVector3D bestStartPoint;

        for (size_t index = 0; index < sortableItems.size(); ++index)
        {
            if (index == currentIndex || visited[index])
            {
                continue;
            }

            const std::vector<ProcessPathOption> options = buildPathOptionsForItem(sortableItems[index], strategy);

            for (const ProcessPathOption& option : options)
            {
                const double distance = rotarySortTravelDistance(currentEndPoint, option.startPoint, config);
                const bool shouldReplace = bestIndex < 0
                    || distance < bestDistance - kSortEpsilon
                    || (std::abs(distance - bestDistance) <= kSortEpsilon
                        && isPointLexicographicallyLess(option.startPoint, bestStartPoint));

                if (!shouldReplace)
                {
                    continue;
                }

                bestIndex = static_cast<int>(index);
                bestDistance = distance;
                bestStartPoint = option.startPoint;
            }
        }

        if (bestIndex < 0)
        {
            return false;
        }

        nextStartPoint = bestStartPoint;
        return true;
    }

    SortCandidate chooseNext2DSortCandidate
    (
        const std::vector<CadItem*>& sortableItems,
        const std::vector<bool>& visited,
        const std::vector<ProcessConnectionSegment>& processedSegments,
        const GapStartSelectionContext& gapStartContext,
        SortStrategy strategy,
        int currentComponentId,
        int restrictedComponentId,
        bool preferPreferredGapStart,
        bool hasCurrentEndPoint,
        const QVector3D& currentEndPoint,
        const QVector3D& sweepDirection
    )
    {
        SortCandidate bestCandidate;
        const std::vector<bool> visitedComponents = buildVisitedComponentMask(visited, gapStartContext.componentIds);
        const bool mustStayInCurrentComponent = hasCurrentEndPoint
            && hasRemainingUnvisitedInComponent(visited, gapStartContext.componentIds, currentComponentId);
        const QVector3D referencePoint = hasCurrentEndPoint ? currentEndPoint : kSortOrigin;
        const QVector3D normalizedSweepDirection = normalizeOrZero(sweepDirection);
        const double referenceProgress = static_cast<double>(QVector3D::dotProduct(referencePoint, normalizedSweepDirection));

        for (size_t index = 0; index < sortableItems.size(); ++index)
        {
            if (visited[index])
            {
                continue;
            }

            const std::vector<ProcessPathOption> options = buildPathOptionsForItem(sortableItems[index], strategy);

            for (const ProcessPathOption& option : options)
            {
                const int componentId = index < gapStartContext.componentIds.size()
                    ? gapStartContext.componentIds[index]
                    : -1;

                if (mustStayInCurrentComponent && componentId != currentComponentId)
                {
                    continue;
                }

                if (restrictedComponentId >= 0 && componentId != restrictedComponentId)
                {
                    continue;
                }

                const std::vector<QVector3D> emptyPreferredPoints;
                const std::vector<QVector3D>& componentPreferredPoints =
                    (componentId >= 0 && static_cast<size_t>(componentId) < gapStartContext.preferredStartPointsByComponent.size())
                    ? gapStartContext.preferredStartPointsByComponent[static_cast<size_t>(componentId)]
                    : emptyPreferredPoints;
                const double connectionDistance = computeClosestConnectionDistance2D(processedSegments, option.startPoint, option.endPoint);
                const bool directlyConnected = connectionDistance <= kSortConnectionEpsilon;
                const bool bestDirectlyConnected = bestCandidate.connectionDistance <= kSortConnectionEpsilon;
                const double entryDistance = std::sqrt(planarDistanceSquared(option.startPoint, referencePoint));
                const double currentGapDistance = hasCurrentEndPoint
                    ? std::sqrt(planarDistanceSquared(option.startPoint, currentEndPoint))
                    : entryDistance;
                const bool nearCurrentGap = hasCurrentEndPoint && currentGapDistance <= kNearGapPriorityDistance2D;
                const bool bestNearCurrentGap = hasCurrentEndPoint && bestCandidate.gapDistance <= kNearGapPriorityDistance2D;
                const bool preferredGapStart = preferPreferredGapStart
                    && componentId == restrictedComponentId
                    && isPointNearAnyPreferredStart(option.startPoint, componentPreferredPoints, kPreferredStartGapDistance2D);
                const bool bestPreferredGapStart =
                    preferPreferredGapStart
                    && restrictedComponentId >= 0
                    && bestCandidate.index >= 0
                    && static_cast<size_t>(bestCandidate.index) < gapStartContext.componentIds.size()
                    && gapStartContext.componentIds[static_cast<size_t>(bestCandidate.index)] == restrictedComponentId
                    && static_cast<size_t>(restrictedComponentId) < gapStartContext.preferredStartPointsByComponent.size()
                    && isPointNearAnyPreferredStart
                    (
                        bestCandidate.startPoint,
                        gapStartContext.preferredStartPointsByComponent[static_cast<size_t>(restrictedComponentId)],
                        kPreferredStartGapDistance2D
                    );
                QVector3D nextStartPoint;
                const bool hasNextStartPoint = tryFindNearestNextStartPoint
                (
                    sortableItems,
                    visited,
                    strategy,
                    index,
                    option.endPoint,
                    nextStartPoint
                );
                const double nextDistance = hasNextStartPoint
                    ? std::sqrt(planarDistanceSquared(nextStartPoint, option.endPoint))
                    : 0.0;
                const double candidateProgress = static_cast<double>(QVector3D::dotProduct(option.startPoint, normalizedSweepDirection));
                const double backtrackDistance = hasCurrentEndPoint && normalizedSweepDirection.lengthSquared() > kSortEpsilon
                    ? std::max(0.0, referenceProgress - candidateProgress)
                    : 0.0;
                const double continuityPenalty =
                    movementContinuityPenalty(option.startPoint - referencePoint, option.startTangent)
                    + (hasNextStartPoint ? movementContinuityPenalty(nextStartPoint - option.endPoint, option.endTangent) : 0.0);
                const double continuityScale = std::max(1.0, 0.5 * (entryDistance + nextDistance));
                const double optionScore = entryDistance
                    + nextDistance * kNextDistanceWeight
                    + backtrackDistance * kBacktrackPenaltyWeight
                    + continuityScale * kDirectionPenaltyWeight * continuityPenalty;

                const bool shouldReplace = bestCandidate.index < 0
                    || (preferPreferredGapStart && preferredGapStart && !bestPreferredGapStart)
                    || (preferPreferredGapStart
                        && preferredGapStart == bestPreferredGapStart
                        && (entryDistance < bestCandidate.priorityDistance - kSortEpsilon
                            || (std::abs(entryDistance - bestCandidate.priorityDistance) <= kSortEpsilon
                                && (optionScore < bestCandidate.score - kSortEpsilon
                                    || (std::abs(optionScore - bestCandidate.score) <= kSortEpsilon
                                        && isPointLexicographicallyLess(option.startPoint, bestCandidate.startPoint))))))
                    || (nearCurrentGap && !bestNearCurrentGap)
                    || (directlyConnected && !bestDirectlyConnected)
                    || (nearCurrentGap == bestNearCurrentGap
                        && directlyConnected == bestDirectlyConnected
                        && (!preferPreferredGapStart || preferredGapStart == bestPreferredGapStart)
                        && (connectionDistance < bestCandidate.connectionDistance - kSortEpsilon
                            || (std::abs(connectionDistance - bestCandidate.connectionDistance) <= kSortEpsilon
                                && (optionScore < bestCandidate.score - kSortEpsilon
                                    || (std::abs(optionScore - bestCandidate.score) <= kSortEpsilon
                                        && (entryDistance < bestCandidate.priorityDistance - kSortEpsilon
                                            || (std::abs(entryDistance - bestCandidate.priorityDistance) <= kSortEpsilon
                                                && isPointLexicographicallyLess(option.startPoint, bestCandidate.startPoint))))))));

                if (!shouldReplace)
                {
                    continue;
                }

                bestCandidate.index = static_cast<int>(index);
                bestCandidate.reverse = option.reverse;
                bestCandidate.hasCustomStart = option.hasCustomStart;
                bestCandidate.processStartParameter = option.processStartParameter;
                bestCandidate.connectionDistance = connectionDistance;
                bestCandidate.priorityDistance = entryDistance;
                bestCandidate.gapDistance = currentGapDistance;
                bestCandidate.score = optionScore;
                bestCandidate.startPoint = option.startPoint;
                bestCandidate.endPoint = option.endPoint;
            }
        }

        return bestCandidate;
    }

    SortCandidate chooseNext3DSortCandidate
    (
        const std::vector<CadItem*>& sortableItems,
        const std::vector<bool>& visited,
        const std::vector<ProcessConnectionSegment>& processedSegments,
        const GapStartSelectionContext& gapStartContext,
        SortStrategy strategy,
        int currentComponentId,
        int restrictedComponentId,
        bool preferPreferredGapStart,
        bool hasCurrentEndPoint,
        const QVector3D& currentEndPoint,
        const QVector3D& sweepDirection,
        const GProfileRotaryAxisConfig& config
    )
    {
        SortCandidate bestCandidate;
        const std::vector<bool> visitedComponents = buildVisitedComponentMask(visited, gapStartContext.componentIds);
        const bool mustStayInCurrentComponent = hasCurrentEndPoint
            && hasRemainingUnvisitedInComponent(visited, gapStartContext.componentIds, currentComponentId);
        const QVector3D referencePoint = hasCurrentEndPoint ? currentEndPoint : kRotaryInitialSortOrigin;
        RotarySortPoint referenceRotaryPoint;
        const bool hasReferenceRotaryPoint = tryBuildRotarySortPoint(referencePoint, config, referenceRotaryPoint);
        const QVector3D normalizedSweepDirection = normalizeOrZero(sweepDirection);
        const double referenceProgress = hasReferenceRotaryPoint
            ? static_cast<double>(referencePoint.x()) * static_cast<double>(normalizedSweepDirection.x())
                + referenceRotaryPoint.angleDegrees * static_cast<double>(normalizedSweepDirection.y())
            : static_cast<double>(referencePoint.x()) * static_cast<double>(normalizedSweepDirection.x());

        for (size_t index = 0; index < sortableItems.size(); ++index)
        {
            if (visited[index])
            {
                continue;
            }

            const std::vector<ProcessPathOption> options = buildPathOptionsForItem(sortableItems[index], strategy);

            for (const ProcessPathOption& option : options)
            {
                const int componentId = index < gapStartContext.componentIds.size()
                    ? gapStartContext.componentIds[index]
                    : -1;

                if (mustStayInCurrentComponent && componentId != currentComponentId)
                {
                    continue;
                }

                if (restrictedComponentId >= 0 && componentId != restrictedComponentId)
                {
                    continue;
                }

                const std::vector<QVector3D> emptyPreferredPoints;
                const std::vector<QVector3D>& componentPreferredPoints =
                    (componentId >= 0 && static_cast<size_t>(componentId) < gapStartContext.preferredStartPointsByComponent.size())
                    ? gapStartContext.preferredStartPointsByComponent[static_cast<size_t>(componentId)]
                    : emptyPreferredPoints;
                const double connectionDistance = computeClosestConnectionDistance3D(processedSegments, option.startPoint, option.endPoint);
                const bool directlyConnected = connectionDistance <= kSortConnectionEpsilon;
                const bool bestDirectlyConnected = bestCandidate.connectionDistance <= kSortConnectionEpsilon;
                double resolvedCandidateAngle = 0.0;
                const double entryDistance = rotarySortTravelDistance(referencePoint, option.startPoint, config, &resolvedCandidateAngle);
                const double currentGapDistance = hasCurrentEndPoint
                    ? std::sqrt(spatialDistanceSquared(option.startPoint, currentEndPoint))
                    : std::sqrt(spatialDistanceSquared(option.startPoint, kRotaryInitialSortOrigin));
                const bool nearCurrentGap = hasCurrentEndPoint && currentGapDistance <= kNearGapPriorityDistance3D;
                const bool bestNearCurrentGap = hasCurrentEndPoint && bestCandidate.gapDistance <= kNearGapPriorityDistance3D;
                const bool preferredGapStart = preferPreferredGapStart
                    && componentId == restrictedComponentId
                    && isPointNearAnyPreferredStart(option.startPoint, componentPreferredPoints, kPreferredStartGapDistance3D);
                const bool bestPreferredGapStart =
                    preferPreferredGapStart
                    && restrictedComponentId >= 0
                    && bestCandidate.index >= 0
                    && static_cast<size_t>(bestCandidate.index) < gapStartContext.componentIds.size()
                    && gapStartContext.componentIds[static_cast<size_t>(bestCandidate.index)] == restrictedComponentId
                    && static_cast<size_t>(restrictedComponentId) < gapStartContext.preferredStartPointsByComponent.size()
                    && isPointNearAnyPreferredStart
                    (
                        bestCandidate.startPoint,
                        gapStartContext.preferredStartPointsByComponent[static_cast<size_t>(restrictedComponentId)],
                        kPreferredStartGapDistance3D
                    );
                QVector3D nextStartPoint;
                const bool hasNextStartPoint = tryFindNearestNextStartPoint3D
                (
                    sortableItems,
                    visited,
                    strategy,
                    index,
                    option.endPoint,
                    config,
                    nextStartPoint
                );
                const double nextDistance = hasNextStartPoint
                    ? rotarySortTravelDistance(option.endPoint, nextStartPoint, config)
                    : 0.0;
                const double candidateProgress = static_cast<double>(option.startPoint.x()) * static_cast<double>(normalizedSweepDirection.x())
                    + resolvedCandidateAngle * static_cast<double>(normalizedSweepDirection.y());
                const double backtrackDistance = hasCurrentEndPoint && normalizedSweepDirection.lengthSquared() > kSortEpsilon
                    ? std::max(0.0, referenceProgress - candidateProgress)
                    : 0.0;
                const double continuityPenalty =
                    rotaryMovementContinuityPenalty(referencePoint, option.startPoint, option.startTangent, config)
                    + (hasNextStartPoint ? rotaryMovementContinuityPenalty(option.endPoint, nextStartPoint, option.endTangent, config) : 0.0);
                const double continuityScale = std::max(1.0, 0.5 * (entryDistance + nextDistance));
                const double optionScore = entryDistance
                    + nextDistance * kRotaryNextDistanceWeight
                    + backtrackDistance * kRotaryBacktrackPenaltyWeight
                    + continuityScale * kRotaryDirectionPenaltyWeight * continuityPenalty;

                const bool shouldReplace = bestCandidate.index < 0
                    || (preferPreferredGapStart && preferredGapStart && !bestPreferredGapStart)
                    || (preferPreferredGapStart
                        && preferredGapStart == bestPreferredGapStart
                        && (entryDistance < bestCandidate.priorityDistance - kSortEpsilon
                            || (std::abs(entryDistance - bestCandidate.priorityDistance) <= kSortEpsilon
                                && (optionScore < bestCandidate.score - kSortEpsilon
                                    || (std::abs(optionScore - bestCandidate.score) <= kSortEpsilon
                                        && isPointLexicographicallyLess(option.startPoint, bestCandidate.startPoint))))))
                    || (nearCurrentGap && !bestNearCurrentGap)
                    || (directlyConnected && !bestDirectlyConnected)
                    || (nearCurrentGap == bestNearCurrentGap
                        && directlyConnected == bestDirectlyConnected
                        && (!preferPreferredGapStart || preferredGapStart == bestPreferredGapStart)
                        && (connectionDistance < bestCandidate.connectionDistance - kSortEpsilon
                            || (std::abs(connectionDistance - bestCandidate.connectionDistance) <= kSortEpsilon
                                && (optionScore < bestCandidate.score - kSortEpsilon
                                    || (std::abs(optionScore - bestCandidate.score) <= kSortEpsilon
                                        && (entryDistance < bestCandidate.priorityDistance - kSortEpsilon
                                            || (std::abs(entryDistance - bestCandidate.priorityDistance) <= kSortEpsilon
                                                && isPointLexicographicallyLess(option.startPoint, bestCandidate.startPoint))))))));

                if (!shouldReplace)
                {
                    continue;
                }

                bestCandidate.index = static_cast<int>(index);
                bestCandidate.reverse = option.reverse;
                bestCandidate.hasCustomStart = option.hasCustomStart;
                bestCandidate.processStartParameter = option.processStartParameter;
                bestCandidate.connectionDistance = connectionDistance;
                bestCandidate.priorityDistance = entryDistance;
                bestCandidate.gapDistance = currentGapDistance;
                bestCandidate.score = optionScore;
                bestCandidate.startPoint = option.startPoint;
                bestCandidate.endPoint = option.endPoint;
            }
        }

        return bestCandidate;
    }

    int nextProcessOrder(const CadDocument& document)
    {
        int maxOrder = -1;

        for (const std::unique_ptr<CadItem>& entity : document.m_entities)
        {
            if (entity != nullptr)
            {
                maxOrder = std::max(maxOrder, entity->m_processOrder);
            }
        }

        return maxOrder + 1;
    }
}

Gcode_postprocessing_system::Gcode_postprocessing_system(QWidget* parent)
    : QMainWindow(parent)
    , ui(new Ui::Gcode_postprocessing_systemClass())
{
    ui->setupUi(this);
    loadAvailableProfiles();

    m_commandLineWidget = new CadCommandLineWidget(this);
    m_statusPaneWidget = new CadStatusPaneWidget(this);

    if (QVBoxLayout* centralLayout = qobject_cast<QVBoxLayout*>(ui->centralWidget->layout()))
    {
        centralLayout->addWidget(m_commandLineWidget);
        centralLayout->addWidget(m_statusPaneWidget);
    }

    m_editer.setDocument(&m_document);
    ui->openGLWidget->setEditer(&m_editer);
    ui->openGLWidget->setDocument(&m_document);
    ui->openGLWidget->refreshCommandPrompt();

    connect(ui->openGLWidget, &CadViewer::hoveredWorldPositionChanged, m_statusPaneWidget, &CadStatusPaneWidget::setWorldPosition);
    connect(m_statusPaneWidget, &CadStatusPaneWidget::basePointSnapToggled, ui->openGLWidget, &CadViewer::setBasePointSnapEnabled);
    connect(m_statusPaneWidget, &CadStatusPaneWidget::controlPointSnapToggled, ui->openGLWidget, &CadViewer::setControlPointSnapEnabled);
    connect(m_statusPaneWidget, &CadStatusPaneWidget::gridSnapToggled, ui->openGLWidget, &CadViewer::setGridSnapEnabled);
    connect(m_statusPaneWidget, &CadStatusPaneWidget::endpointSnapToggled, ui->openGLWidget, &CadViewer::setEndpointSnapEnabled);
    connect(m_statusPaneWidget, &CadStatusPaneWidget::midpointSnapToggled, ui->openGLWidget, &CadViewer::setMidpointSnapEnabled);
    connect(m_statusPaneWidget, &CadStatusPaneWidget::centerSnapToggled, ui->openGLWidget, &CadViewer::setCenterSnapEnabled);
    connect(m_statusPaneWidget, &CadStatusPaneWidget::intersectionSnapToggled, ui->openGLWidget, &CadViewer::setIntersectionSnapEnabled);
    connect
    (
        m_statusPaneWidget,
        &CadStatusPaneWidget::snapOptionMaskChanged,
        this,
        [this](quint32 mask)
        {
            saveSnapOptionMask(mask);
        }
    );

    m_statusPaneWidget->setSnapOptionMask(loadSnapOptionMask());
    connect(ui->openGLWidget, &CadViewer::commandPromptChanged, m_commandLineWidget, &CadCommandLineWidget::setPrompt);
    connect(ui->openGLWidget, &CadViewer::commandMessageAppended, m_commandLineWidget, &CadCommandLineWidget::appendMessage);
    connect
    (
        ui->openGLWidget,
        &CadViewer::fileDropRequested,
        this,
        [this](const QString& filePath)
        {
            importCadFile(filePath);
        }
    );

    connect
    (
        ui->action_File_Import_Dxf,
        &QAction::triggered,
        this,
        [this]()
        {
            const QString filePath = QFileDialog::getOpenFileName
            (
                this,
                QStringLiteral("导入文件"),
                QString(),
                QStringLiteral("支持文件 (*.dxf *.dwg *.bmp *.png *.jpg *.jpeg);;CAD 文件 (*.dxf *.dwg);;位图文件 (*.bmp *.png *.jpg *.jpeg)")
            );

            if (filePath.isEmpty())
            {
                return;
            }

            importCadFile(filePath);
        }
    );

    QAction* importDxfOnlyAction = new QAction(QStringLiteral("导入DXF..."), this);
    QAction* importDwgOnlyAction = new QAction(QStringLiteral("导入DWG..."), this);
    ui->menuFile->insertAction(ui->action_File_Import_Image, importDxfOnlyAction);
    ui->menuFile->insertAction(ui->action_File_Import_Image, importDwgOnlyAction);

    connect
    (
        importDxfOnlyAction,
        &QAction::triggered,
        this,
        [this]()
        {
            const QString filePath = QFileDialog::getOpenFileName
            (
                this,
                QStringLiteral("导入DXF"),
                QString(),
                QStringLiteral("DXF 文件 (*.dxf)")
            );

            if (!filePath.isEmpty())
            {
                importDxfFile(filePath);
            }
        }
    );

    connect
    (
        importDwgOnlyAction,
        &QAction::triggered,
        this,
        [this]()
        {
            const QString filePath = QFileDialog::getOpenFileName
            (
                this,
                QStringLiteral("导入DWG"),
                QString(),
                QStringLiteral("DWG 文件 (*.dwg)")
            );

            if (!filePath.isEmpty())
            {
                importDxfFile(filePath);
            }
        }
    );

    connect
    (
        ui->action_File_Import_Image,
        &QAction::triggered,
        this,
        [this]()
        {
            const QString filePath = QFileDialog::getOpenFileName
            (
                this,
                QStringLiteral("导入图片"),
                QString(),
                QStringLiteral("位图文件 (*.bmp *.png *.jpg *.jpeg)")
            );

            if (filePath.isEmpty())
            {
                return;
            }

            importBitmapFile(filePath);
        }
    );

    ui->action_FileExport->setText(QStringLiteral("保存文件"));
    ui->action_FileExport->setShortcut(QKeySequence::Save);
    ui->action_FileExport->setShortcutContext(Qt::ApplicationShortcut);
    ui->menuFile->insertAction(ui->action_File_Export_G, ui->action_FileExport);

    QAction* exportDxfAction = new QAction(QStringLiteral("导出为DXF..."), this);
    QAction* exportSafeDxfAction = new QAction(QStringLiteral("导出为DXF（安全模式）..."), this);

    ui->menuFile->insertAction(ui->action_File_Export_G, exportDxfAction);
    ui->menuFile->insertAction(ui->action_File_Export_G, exportSafeDxfAction);
    ui->menuFile->insertSeparator(ui->action_File_Export_G);

    ui->action_File_Export_G->setText(QStringLiteral("导出G代码"));

    connect(ui->action_FileExport, &QAction::triggered, this, [this]() { saveCurrentDocument(); });
    connect(exportDxfAction, &QAction::triggered, this, [this]() { exportDxfDocument(); });
    connect(exportSafeDxfAction, &QAction::triggered, this, [this]() { exportDxfDocument(true); });
    connect(ui->action_File_Export_G, &QAction::triggered, this, [this]() { exportGCode(); });
    connect(ui->action_Edit_ReversePeocess, &QAction::triggered, this, [this]() { toggleSelectedEntityReverse(); });
    connect(ui->action_Sort_2D_Assign, &QAction::triggered, this, [this]() { sortEntitiesByCurrentMode(false); });
    connect(ui->action_Sort_2D_Smart, &QAction::triggered, this, [this]() { sortEntitiesByCurrentMode(true); });

    ui->action_Sort_3D_Assign->setVisible(false);
    ui->action_Sort_3D_Smart->setVisible(false);

    m_generationPreference = loadGenerationPreference();
    initializeThemeMenu();
    initializeToolPanel();
    applyDefaultDrawingProperties();
    applyTheme(loadThemeMode());
    syncToolPanelState();
}

Gcode_postprocessing_system::~Gcode_postprocessing_system()
{
    delete ui;
}

bool Gcode_postprocessing_system::importCadFile(const QString& filePath)
{
    if (filePath.isEmpty())
    {
        return false;
    }

    if (isCadVectorFile(filePath))
    {
        return importDxfFile(filePath);
    }

    if (isBitmapFile(filePath))
    {
        return importBitmapFile(filePath);
    }

    QMessageBox::warning(this, QStringLiteral("导入失败"), QStringLiteral("当前不支持该文件类型: %1").arg(QFileInfo(filePath).suffix()));
    return false;
}

bool Gcode_postprocessing_system::importDxfFile(const QString& filePath)
{
    m_editer.clearHistory();
    m_document.readDxfDocument(filePath);
    m_currentDocumentPath = ensureDxfSuffix(filePath);
    ui->openGLWidget->setDocument(&m_document);
    ui->openGLWidget->appendCommandMessage(QStringLiteral("已导入文件: %1").arg(QFileInfo(filePath).fileName()));
    ui->openGLWidget->refreshCommandPrompt();
    statusBar()->showMessage(QStringLiteral("已导入: %1").arg(QFileInfo(filePath).fileName()), 5000);

    if (m_document.m_entities.empty())
    {
        QMessageBox::warning(this, QStringLiteral("导入结果"), QStringLiteral("文件已读取，但未生成可显示的 CAD 图元。"));
    }

    syncToolPanelState();
    return true;
}

bool Gcode_postprocessing_system::importBitmapFile(const QString& filePath)
{
    CadBitmapImportDialog dialog(filePath, this);

    if (!dialog.isReady())
    {
        QMessageBox::warning(this, QStringLiteral("位图导入失败"), dialog.errorMessage());
        return false;
    }

    if (dialog.exec() != QDialog::Accepted)
    {
        return false;
    }

    const CadBitmapImportOptions importOptions = dialog.options();
    CadBitmapImportResult importResult;
    QString errorMessage;

    if (!CadBitmapVectorizer::vectorize(dialog.sourceImage(), importOptions, importResult, &errorMessage))
    {
        QMessageBox::warning(this, QStringLiteral("位图导入失败"), errorMessage);
        return false;
    }

    const bool replaceExisting = importOptions.importMode == CadBitmapImportMode::ReplaceDocument;
    m_editer.clearHistory();

    const int appendedCount = m_document.appendEntities(std::move(importResult.entities), replaceExisting);

    if (importOptions.autoFitScene)
    {
        ui->openGLWidget->fitScene();
    }

    ui->openGLWidget->appendCommandMessage
    (
        QStringLiteral("位图导入完成: %1，图层 %2，%3")
            .arg(QFileInfo(filePath).fileName())
            .arg(importOptions.layerName)
            .arg(importResult.summaryText)
    );
    ui->openGLWidget->refreshCommandPrompt();

    if (appendedCount <= 0)
    {
        QMessageBox::warning(this, QStringLiteral("位图导入结果"), QStringLiteral("位图处理完成，但没有生成可显示的 CAD 图元。"));
        return false;
    }

    statusBar()->showMessage
    (
        QStringLiteral("位图已导入: %1，新增实体 %2").arg(QFileInfo(filePath).fileName()).arg(appendedCount),
        5000
    );

    syncToolPanelState();
    return true;
}

bool Gcode_postprocessing_system::saveCurrentDocument()
{
    QString filePath = m_currentDocumentPath.trimmed();

    if (filePath.isEmpty())
    {
        filePath = QFileDialog::getSaveFileName
        (
            this,
            QStringLiteral("保存DXF文件"),
            defaultDxfPathForCurrentDocument(),
            QStringLiteral("DXF 文件 (*.dxf)")
        );

        if (filePath.isEmpty())
        {
            return false;
        }
    }

    filePath = ensureDxfSuffix(filePath);

    if (!writeDocumentToDxf(filePath, true, false))
    {
        return false;
    }

    ui->openGLWidget->appendCommandMessage(QStringLiteral("文件已保存: %1").arg(QFileInfo(filePath).fileName()));
    ui->openGLWidget->refreshCommandPrompt();
    statusBar()->showMessage(QStringLiteral("保存完成: %1").arg(QFileInfo(filePath).fileName()), 5000);
    return true;
}

bool Gcode_postprocessing_system::exportDxfDocument(bool safeMode)
{
    QString filePath = QFileDialog::getSaveFileName
    (
        this,
        safeMode ? QStringLiteral("导出为DXF（安全模式）") : QStringLiteral("导出为DXF"),
        defaultDxfPathForCurrentDocument(),
        QStringLiteral("DXF 文件 (*.dxf)")
    );

    if (filePath.isEmpty())
    {
        return false;
    }

    filePath = ensureDxfSuffix(filePath);

    if (!writeDocumentToDxf(filePath, false, safeMode))
    {
        return false;
    }

    ui->openGLWidget->appendCommandMessage
    (
        safeMode
            ? QStringLiteral("文件已安全导出: %1").arg(QFileInfo(filePath).fileName())
            : QStringLiteral("文件已导出: %1").arg(QFileInfo(filePath).fileName())
    );
    ui->openGLWidget->refreshCommandPrompt();
    statusBar()->showMessage
    (
        safeMode
            ? QStringLiteral("安全导出完成: %1").arg(QFileInfo(filePath).fileName())
            : QStringLiteral("导出完成: %1").arg(QFileInfo(filePath).fileName()),
        5000
    );
    return true;
}

bool Gcode_postprocessing_system::exportGCode()
{
    const GGenerator::GenerationMode generationMode = resolveGenerationMode();
    const QString modeDisplayName = QStringLiteral("当前模式 %1").arg(generationModeDisplayName(generationMode));

    return exportGCode(generationMode, modeDisplayName);
}

bool Gcode_postprocessing_system::sortEntitiesByCurrentMode(bool smartSort)
{
    const GGenerator::GenerationMode generationMode = resolveGenerationMode();
    return smartSort
        ? (generationMode == GGenerator::GenerationMode::Mode3D
            ? smartSortEntities3D()
            : smartSortEntities())
        : (generationMode == GGenerator::GenerationMode::Mode3D
            ? sortEntitiesByCurrentDirection3D()
            : sortEntitiesByCurrentDirection());
}

bool Gcode_postprocessing_system::exportGCode
(
    GGenerator::GenerationMode generationMode,
    const QString& modeDisplayName
)
{
    GGenerator generator;
    generator.setDocument(&m_document);
    generator.setProfile(&m_activeProfile);
    generator.setGenerationMode(generationMode);

    QString errorMessage;

    if (!generator.generate(this, &errorMessage))
    {
        if (!errorMessage.trimmed().isEmpty())
        {
            QMessageBox::warning(this, QStringLiteral("导出G代码失败"), errorMessage);
        }

        return false;
    }

    const QString resolvedModeDisplayName = modeDisplayName.trimmed().isEmpty()
        ? generationModeDisplayName(generationMode)
        : modeDisplayName;

    ui->openGLWidget->appendCommandMessage
    (
        QStringLiteral("G代码已导出（%1）。").arg(resolvedModeDisplayName)
    );
    ui->openGLWidget->refreshCommandPrompt();
    statusBar()->showMessage(QStringLiteral("G代码导出完成（%1）").arg(resolvedModeDisplayName), 5000);
    return true;
}

bool Gcode_postprocessing_system::writeDocumentToDxf(const QString& filePath, bool updateCurrentPath, bool safeMode)
{
    if (filePath.trimmed().isEmpty())
    {
        return false;
    }

    const QString normalizedPath = ensureDxfSuffix(filePath);
    const bool writeSuccess = updateCurrentPath
        ? m_document.saveDxfDocument(normalizedPath, safeMode)
        : m_document.eportDxfDocument(normalizedPath, safeMode);

    if (!writeSuccess)
    {
        QMessageBox::warning(this, QStringLiteral("文件操作失败"), QStringLiteral("写入 DXF 文件失败: %1").arg(normalizedPath));
        return false;
    }

    if (updateCurrentPath)
    {
        m_currentDocumentPath = normalizedPath;
    }

    return true;
}

QString Gcode_postprocessing_system::ensureDxfSuffix(const QString& filePath) const
{
    const QString trimmedPath = filePath.trimmed();

    if (trimmedPath.isEmpty())
    {
        return QString();
    }

    if (trimmedPath.endsWith(QStringLiteral(".dxf"), Qt::CaseInsensitive))
    {
        return trimmedPath;
    }

    const QFileInfo fileInfo(trimmedPath);

    if (!fileInfo.suffix().isEmpty())
    {
        return fileInfo.absolutePath()
            + QLatin1Char('/')
            + fileInfo.completeBaseName()
            + QStringLiteral(".dxf");
    }

    return trimmedPath + QStringLiteral(".dxf");
}

QString Gcode_postprocessing_system::defaultDxfPathForCurrentDocument() const
{
    if (!m_currentDocumentPath.trimmed().isEmpty())
    {
        return ensureDxfSuffix(m_currentDocumentPath);
    }

    return QStringLiteral("untitled.dxf");
}

bool Gcode_postprocessing_system::toggleSelectedEntityReverse()
{
    const QVector<CadItem*> selectedItems = ui->openGLWidget->selectedEntities();

    if (selectedItems.isEmpty())
    {
        QMessageBox::warning(this, QStringLiteral("反向加工"), QStringLiteral("请先选择图元。"));
        return false;
    }

    int updatedCount = 0;

    for (CadItem* item : selectedItems)
    {
        if (item != nullptr && m_editer.toggleEntityReverse(item))
        {
            ++updatedCount;
        }
    }

    if (updatedCount <= 0)
    {
        QMessageBox::warning(this, QStringLiteral("反向加工"), QStringLiteral("选中图元的反向加工状态切换失败。"));
        return false;
    }

    ui->openGLWidget->appendCommandMessage
    (
        updatedCount > 1
            ? QStringLiteral("已切换 %1 个图元的加工方向。").arg(updatedCount)
            : QStringLiteral("当前选中图元加工方向已切换。")
    );
    ui->openGLWidget->refreshCommandPrompt();
    statusBar()->showMessage(QStringLiteral("加工方向切换完成（%1）").arg(updatedCount), 5000);
    return true;
}

bool Gcode_postprocessing_system::deleteSelectedEntity()
{
    const QVector<CadItem*> selectedItems = ui->openGLWidget->selectedEntities();

    if (selectedItems.isEmpty())
    {
        QMessageBox::warning(this, QStringLiteral("删除图元"), QStringLiteral("请先选择图元。"));
        return false;
    }

    int deletedCount = 0;

    for (CadItem* item : selectedItems)
    {
        if (item != nullptr && m_editer.deleteEntity(item))
        {
            ++deletedCount;
        }
    }

    if (deletedCount <= 0)
    {
        QMessageBox::warning(this, QStringLiteral("删除图元"), QStringLiteral("选中图元删除失败。"));
        return false;
    }

    ui->openGLWidget->appendCommandMessage
    (
        deletedCount > 1
            ? QStringLiteral("已删除 %1 个图元。").arg(deletedCount)
            : QStringLiteral("已删除选中图元。")
    );
    statusBar()->showMessage(QStringLiteral("图元删除完成（%1）").arg(deletedCount), 4000);
    return true;
}

bool Gcode_postprocessing_system::copySelectedEntity()
{
    const QVector<CadItem*> selectedItems = ui->openGLWidget->selectedEntities();

    if (selectedItems.isEmpty())
    {
        QMessageBox::warning(this, QStringLiteral("复制图元"), QStringLiteral("请先选择图元。"));
        return false;
    }

    bool ok = false;
    const double deltaX = QInputDialog::getDouble
    (
        this,
        QStringLiteral("复制图元"),
        QStringLiteral("请输入 X 偏移量:"),
        10.0,
        -1000000.0,
        1000000.0,
        3,
        &ok
    );

    if (!ok)
    {
        return false;
    }

    const double deltaY = QInputDialog::getDouble
    (
        this,
        QStringLiteral("复制图元"),
        QStringLiteral("请输入 Y 偏移量:"),
        10.0,
        -1000000.0,
        1000000.0,
        3,
        &ok
    );

    if (!ok)
    {
        return false;
    }

    int copiedCount = 0;
    const QVector3D delta(static_cast<float>(deltaX), static_cast<float>(deltaY), 0.0f);

    for (CadItem* item : selectedItems)
    {
        if (item != nullptr && m_editer.copyEntity(item, delta))
        {
            ++copiedCount;
        }
    }

    if (copiedCount <= 0)
    {
        QMessageBox::warning(this, QStringLiteral("复制图元"), QStringLiteral("选中图元复制失败。"));
        return false;
    }

    ui->openGLWidget->appendCommandMessage
    (
        QStringLiteral("已复制 %1 个图元，偏移量为 (%2, %3)。").arg(copiedCount).arg(deltaX).arg(deltaY)
    );
    statusBar()->showMessage(QStringLiteral("图元复制完成（%1）").arg(copiedCount), 4000);
    return true;
}

bool Gcode_postprocessing_system::rotateSelectedEntity()
{
    const QVector<CadItem*> selectedItems = ui->openGLWidget->selectedEntities();

    if (selectedItems.isEmpty())
    {
        QMessageBox::warning(this, QStringLiteral("旋转图元"), QStringLiteral("请先选择图元。"));
        return false;
    }

    bool ok = false;
    const double angleDegrees = QInputDialog::getDouble
    (
        this,
        QStringLiteral("旋转图元"),
        QStringLiteral("请输入旋转角度（度）:"),
        90.0,
        -3600.0,
        3600.0,
        3,
        &ok
    );

    if (!ok)
    {
        return false;
    }

    const QVector3D basePoint = geometryBoundsCenter(selectedItems);

    int rotatedCount = 0;

    for (CadItem* item : selectedItems)
    {
        if (item != nullptr && m_editer.rotateEntity(item, basePoint, angleDegrees))
        {
            ++rotatedCount;
        }
    }

    if (rotatedCount <= 0)
    {
        QMessageBox::warning(this, QStringLiteral("旋转图元"), QStringLiteral("选中图元旋转失败。"));
        return false;
    }

    ui->openGLWidget->appendCommandMessage
    (
        QStringLiteral("已将 %1 个图元绕中心旋转 %2 度。").arg(rotatedCount).arg(angleDegrees)
    );
    statusBar()->showMessage(QStringLiteral("图元旋转完成（%1）").arg(rotatedCount), 4000);
    return true;
}

bool Gcode_postprocessing_system::scaleSelectedEntity()
{
    const QVector<CadItem*> selectedItems = ui->openGLWidget->selectedEntities();

    if (selectedItems.isEmpty())
    {
        QMessageBox::warning(this, QStringLiteral("缩放图元"), QStringLiteral("请先选择图元。"));
        return false;
    }

    bool ok = false;
    const double scaleFactor = QInputDialog::getDouble
    (
        this,
        QStringLiteral("缩放图元"),
        QStringLiteral("请输入缩放倍率:"),
        2.0,
        0.001,
        1000.0,
        3,
        &ok
    );

    if (!ok)
    {
        return false;
    }

    const QVector3D basePoint = geometryBoundsCenter(selectedItems);

    int scaledCount = 0;

    for (CadItem* item : selectedItems)
    {
        if (item != nullptr && m_editer.scaleEntity(item, basePoint, scaleFactor))
        {
            ++scaledCount;
        }
    }

    if (scaledCount <= 0)
    {
        QMessageBox::warning(this, QStringLiteral("缩放图元"), QStringLiteral("选中图元缩放失败。"));
        return false;
    }

    ui->openGLWidget->appendCommandMessage
    (
        QStringLiteral("已将 %1 个图元绕中心缩放为 %2 倍。").arg(scaledCount).arg(scaleFactor)
    );
    statusBar()->showMessage(QStringLiteral("图元缩放完成（%1）").arg(scaledCount), 4000);
    return true;
}

bool Gcode_postprocessing_system::arraySelectedEntity()
{
    const QVector<CadItem*> selectedItems = ui->openGLWidget->selectedEntities();

    if (selectedItems.isEmpty())
    {
        QMessageBox::warning(this, QStringLiteral("阵列图元"), QStringLiteral("请先选择图元。"));
        return false;
    }

    bool ok = false;
    const int rowCount = QInputDialog::getInt
    (
        this,
        QStringLiteral("矩形阵列"),
        QStringLiteral("请输入行数:"),
        2,
        1,
        999,
        1,
        &ok
    );

    if (!ok)
    {
        return false;
    }

    const int columnCount = QInputDialog::getInt
    (
        this,
        QStringLiteral("矩形阵列"),
        QStringLiteral("请输入列数:"),
        2,
        1,
        999,
        1,
        &ok
    );

    if (!ok)
    {
        return false;
    }

    if (rowCount == 1 && columnCount == 1)
    {
        QMessageBox::warning(this, QStringLiteral("矩形阵列"), QStringLiteral("行数和列数不能同时为 1。"));
        return false;
    }

    const double rowSpacing = QInputDialog::getDouble
    (
        this,
        QStringLiteral("矩形阵列"),
        QStringLiteral("请输入行间距（Y 方向）:"),
        10.0,
        -1000000.0,
        1000000.0,
        3,
        &ok
    );

    if (!ok)
    {
        return false;
    }

    const double columnSpacing = QInputDialog::getDouble
    (
        this,
        QStringLiteral("矩形阵列"),
        QStringLiteral("请输入列间距（X 方向）:"),
        10.0,
        -1000000.0,
        1000000.0,
        3,
        &ok
    );

    if (!ok)
    {
        return false;
    }

    int arrayedCount = 0;
    const QVector3D rowOffset(0.0f, static_cast<float>(rowSpacing), 0.0f);
    const QVector3D columnOffset(static_cast<float>(columnSpacing), 0.0f, 0.0f);

    for (CadItem* item : selectedItems)
    {
        if (item != nullptr && m_editer.arrayEntity(item, rowCount, columnCount, rowOffset, columnOffset))
        {
            ++arrayedCount;
        }
    }

    if (arrayedCount <= 0)
    {
        QMessageBox::warning(this, QStringLiteral("矩形阵列"), QStringLiteral("选中图元阵列失败。"));
        return false;
    }

    ui->openGLWidget->appendCommandMessage
    (
        QStringLiteral("已对 %1 个图元执行 %2 x %3 矩形阵列。").arg(arrayedCount).arg(rowCount).arg(columnCount)
    );
    statusBar()->showMessage(QStringLiteral("矩形阵列完成（%1）").arg(arrayedCount), 4000);
    return true;
}

bool Gcode_postprocessing_system::sortEntitiesByCurrentDirection()
{
    if (m_document.m_entities.empty())
    {
        QMessageBox::warning(this, QStringLiteral("3轴排序"), QStringLiteral("当前文档为空，无法执行排序。"));
        return false;
    }

    std::vector<CadItem*> sortableItems;

    for (const std::unique_ptr<CadItem>& entity : m_document.m_entities)
    {
        if (entity == nullptr)
        {
            continue;
        }

        const CadProcessVisualInfo info = buildProcessVisualInfo(entity.get());

        if (!info.valid)
        {
            continue;
        }

        sortableItems.push_back(entity.get());
    }

    if (sortableItems.empty())
    {
        QMessageBox::warning(this, QStringLiteral("3轴排序"), QStringLiteral("当前文档中没有可参与 G 代码排序的图元。"));
        return false;
    }

    const QVector3D sweepDirection = computeSweepDirection(sortableItems);
    const GapStartSelectionContext gapStartContext = buildGapStartSelectionContext(sortableItems, kPreferredStartGapDistance2D);
    std::vector<CadEditer::ProcessStateUpdate> processUpdates;
    std::vector<ProcessConnectionSegment> processedSegments;
    std::vector<bool> visited(sortableItems.size(), false);

    processUpdates.reserve(sortableItems.size());
    processedSegments.reserve(sortableItems.size());

    bool hasCurrentEndPoint = false;
    int currentComponentId = -1;
    QVector3D currentEndPoint;

    for (size_t order = 0; order < sortableItems.size(); ++order)
    {
        SortCandidate bestCandidate = chooseNext2DSortCandidate
        (
            sortableItems,
            visited,
            processedSegments,
            gapStartContext,
            SortStrategy::KeepDirection,
            currentComponentId,
            -1,
            false,
            hasCurrentEndPoint,
            currentEndPoint,
            sweepDirection
        );

        const std::vector<bool> visitedComponents = buildVisitedComponentMask(visited, gapStartContext.componentIds);
        const int selectedComponentId =
            bestCandidate.index >= 0 && static_cast<size_t>(bestCandidate.index) < gapStartContext.componentIds.size()
            ? gapStartContext.componentIds[static_cast<size_t>(bestCandidate.index)]
            : -1;
        const bool enteringFreshPreferredComponent =
            selectedComponentId >= 0
            && static_cast<size_t>(selectedComponentId) < visitedComponents.size()
            && !visitedComponents[static_cast<size_t>(selectedComponentId)]
            && static_cast<size_t>(selectedComponentId) < gapStartContext.preferredStartPointsByComponent.size()
            && !gapStartContext.preferredStartPointsByComponent[static_cast<size_t>(selectedComponentId)].empty();

        if (enteringFreshPreferredComponent)
        {
            bestCandidate = chooseNext2DSortCandidate
            (
                sortableItems,
                visited,
                processedSegments,
                gapStartContext,
                SortStrategy::KeepDirection,
                currentComponentId,
                selectedComponentId,
                true,
                hasCurrentEndPoint,
                currentEndPoint,
                sweepDirection
            );
        }

        if (bestCandidate.index < 0)
        {
            QMessageBox::warning(this, QStringLiteral("3轴排序"), QStringLiteral("排序过程中出现无效图元，排序已中止。"));
            return false;
        }

        visited[static_cast<size_t>(bestCandidate.index)] = true;
        processUpdates.push_back
        ({
            sortableItems[static_cast<size_t>(bestCandidate.index)],
            static_cast<int>(order),
            sortableItems[static_cast<size_t>(bestCandidate.index)]->m_isReverse,
            sortableItems[static_cast<size_t>(bestCandidate.index)]->m_hasCustomProcessStart,
            sortableItems[static_cast<size_t>(bestCandidate.index)]->m_processStartParameter
        });
        hasCurrentEndPoint = true;
        currentComponentId =
            static_cast<size_t>(bestCandidate.index) < gapStartContext.componentIds.size()
            ? gapStartContext.componentIds[static_cast<size_t>(bestCandidate.index)]
            : -1;
        currentEndPoint = bestCandidate.endPoint;
        processedSegments.push_back({ bestCandidate.startPoint, bestCandidate.endPoint });
    }

    if (!m_editer.applyEntityProcessStates(processUpdates))
    {
        QMessageBox::warning(this, QStringLiteral("3轴排序"), QStringLiteral("排序结果写入失败。"));
        return false;
    }

    ui->openGLWidget->appendCommandMessage
    (
        QStringLiteral("3轴排序完成，共更新 %1 个图元的加工顺序，首件已按最接近原点的当前起点选取，并保留当前加工方向设置。").arg(processUpdates.size())
    );
    ui->openGLWidget->refreshCommandPrompt();
    statusBar()->showMessage(QStringLiteral("3轴排序完成，共更新 %1 个图元").arg(processUpdates.size()), 5000);
    return true;
}

bool Gcode_postprocessing_system::assignSelectedEntityProcessOrder()
{
    CadItem* selectedItem = ui->openGLWidget->selectedEntity();

    if (selectedItem == nullptr)
    {
        QMessageBox::warning(this, QStringLiteral("排序"), QStringLiteral("请先选择一个图元。"));
        return false;
    }

    if (!isProcessVisualizable(selectedItem))
    {
        QMessageBox::warning(this, QStringLiteral("排序"), QStringLiteral("当前图元类型暂不支持加工排序。"));
        return false;
    }

    const int processOrder = nextProcessOrder(m_document);

    if (!m_editer.setEntityProcessOrder(selectedItem, processOrder))
    {
        QMessageBox::warning(this, QStringLiteral("排序"), QStringLiteral("当前图元加工顺序设置失败。"));
        return false;
    }

    ui->openGLWidget->appendCommandMessage(QStringLiteral("当前选中图元已设置为第 %1 个加工对象。").arg(processOrder + 1));
    ui->openGLWidget->refreshCommandPrompt();
    statusBar()->showMessage(QStringLiteral("已设置加工顺序 #%1").arg(processOrder + 1), 5000);
    return true;
}

bool Gcode_postprocessing_system::smartSortEntities()
{
    if (m_document.m_entities.empty())
    {
        QMessageBox::warning(this, QStringLiteral("3轴智能排序"), QStringLiteral("当前文档为空，无法执行智能排序。"));
        return false;
    }

    std::vector<CadItem*> sortableItems;

    for (const std::unique_ptr<CadItem>& entity : m_document.m_entities)
    {
        if (entity == nullptr)
        {
            continue;
        }

        const CadProcessVisualInfo info = buildProcessVisualInfo(entity.get());

        if (!info.valid)
        {
            continue;
        }

        sortableItems.push_back(entity.get());
    }

    if (sortableItems.empty())
    {
        QMessageBox::warning(this, QStringLiteral("3轴智能排序"), QStringLiteral("当前文档中没有可参与 G 代码排序的图元。"));
        return false;
    }

    const QVector3D sweepDirection = computeSweepDirection(sortableItems);
    const GapStartSelectionContext gapStartContext = buildGapStartSelectionContext(sortableItems, kPreferredStartGapDistance2D);
    std::vector<CadEditer::ProcessStateUpdate> processUpdates;
    std::vector<ProcessConnectionSegment> processedSegments;
    std::vector<bool> visited(sortableItems.size(), false);

    processUpdates.reserve(sortableItems.size());
    processedSegments.reserve(sortableItems.size());

    bool hasCurrentEndPoint = false;
    int currentComponentId = -1;
    QVector3D currentEndPoint;

    for (size_t order = 0; order < sortableItems.size(); ++order)
    {
        SortCandidate bestCandidate = chooseNext2DSortCandidate
        (
            sortableItems,
            visited,
            processedSegments,
            gapStartContext,
            SortStrategy::Smart,
            currentComponentId,
            -1,
            false,
            hasCurrentEndPoint,
            currentEndPoint,
            sweepDirection
        );

        const std::vector<bool> visitedComponents = buildVisitedComponentMask(visited, gapStartContext.componentIds);
        const int selectedComponentId =
            bestCandidate.index >= 0 && static_cast<size_t>(bestCandidate.index) < gapStartContext.componentIds.size()
            ? gapStartContext.componentIds[static_cast<size_t>(bestCandidate.index)]
            : -1;
        const bool enteringFreshPreferredComponent =
            selectedComponentId >= 0
            && static_cast<size_t>(selectedComponentId) < visitedComponents.size()
            && !visitedComponents[static_cast<size_t>(selectedComponentId)]
            && static_cast<size_t>(selectedComponentId) < gapStartContext.preferredStartPointsByComponent.size()
            && !gapStartContext.preferredStartPointsByComponent[static_cast<size_t>(selectedComponentId)].empty();

        if (enteringFreshPreferredComponent)
        {
            bestCandidate = chooseNext2DSortCandidate
            (
                sortableItems,
                visited,
                processedSegments,
                gapStartContext,
                SortStrategy::Smart,
                currentComponentId,
                selectedComponentId,
                true,
                hasCurrentEndPoint,
                currentEndPoint,
                sweepDirection
            );
        }

        if (bestCandidate.index < 0)
        {
            QMessageBox::warning(this, QStringLiteral("3轴智能排序"), QStringLiteral("智能排序过程中出现无效图元，排序已中止。"));
            return false;
        }

        visited[static_cast<size_t>(bestCandidate.index)] = true;
        processUpdates.push_back
        ({
            sortableItems[static_cast<size_t>(bestCandidate.index)],
            static_cast<int>(order),
            bestCandidate.reverse,
            bestCandidate.hasCustomStart,
            bestCandidate.processStartParameter
        });
        hasCurrentEndPoint = true;
        currentComponentId =
            static_cast<size_t>(bestCandidate.index) < gapStartContext.componentIds.size()
            ? gapStartContext.componentIds[static_cast<size_t>(bestCandidate.index)]
            : -1;
        currentEndPoint = bestCandidate.endPoint;
        processedSegments.push_back({ bestCandidate.startPoint, bestCandidate.endPoint });
    }

    if (!m_editer.applyEntityProcessStates(processUpdates))
    {
        QMessageBox::warning(this, QStringLiteral("3轴智能排序"), QStringLiteral("智能排序结果写入失败。"));
        return false;
    }

    ui->openGLWidget->appendCommandMessage
    (
        QStringLiteral("3轴智能排序完成，共更新 %1 个图元的加工顺序，并已对闭合图元的方向/起刀缝点做连续性优化。").arg(processUpdates.size())
    );
    ui->openGLWidget->refreshCommandPrompt();
    statusBar()->showMessage(QStringLiteral("3轴智能排序完成，共更新 %1 个图元").arg(processUpdates.size()), 5000);
    return true;
}

bool Gcode_postprocessing_system::sortEntitiesByCurrentDirection3D()
{
    if (m_document.m_entities.empty())
    {
        QMessageBox::warning(this, QStringLiteral("4轴(绕A)排序"), QStringLiteral("当前文档为空，无法执行排序。"));
        return false;
    }

    if (!documentContainsThreeDimensionalGeometry(m_document))
    {
        QMessageBox::warning(this, QStringLiteral("4轴(绕A)排序"), QStringLiteral("当前文档未检测到可用于4轴排序的有效路径。"));
        return false;
    }

    std::vector<CadItem*> sortableItems;

    for (const std::unique_ptr<CadItem>& entity : m_document.m_entities)
    {
        if (entity == nullptr)
        {
            continue;
        }

        const CadProcessVisualInfo info = buildProcessVisualInfo(entity.get());

        if (!info.valid)
        {
            continue;
        }

        sortableItems.push_back(entity.get());
    }

    if (sortableItems.empty())
    {
        QMessageBox::warning(this, QStringLiteral("4轴(绕A)排序"), QStringLiteral("当前文档中没有可参与 4 轴 G 代码排序的图元。"));
        return false;
    }

    const GProfileRotaryAxisConfig& rotaryAxisConfig = m_activeProfile.rotaryAxisConfig();
    const QVector3D sweepDirection = computeRotarySweepDirection(sortableItems, rotaryAxisConfig);
    const GapStartSelectionContext gapStartContext = buildGapStartSelectionContext(sortableItems, kPreferredStartGapDistance3D);
    std::vector<CadEditer::ProcessStateUpdate> processUpdates;
    std::vector<ProcessConnectionSegment> processedSegments;
    std::vector<bool> visited(sortableItems.size(), false);

    processUpdates.reserve(sortableItems.size());
    processedSegments.reserve(sortableItems.size());

    bool hasCurrentEndPoint = false;
    int currentComponentId = -1;
    QVector3D currentEndPoint;

    for (size_t order = 0; order < sortableItems.size(); ++order)
    {
        SortCandidate bestCandidate = chooseNext3DSortCandidate
        (
            sortableItems,
            visited,
            processedSegments,
            gapStartContext,
            SortStrategy::KeepDirection,
            currentComponentId,
            -1,
            false,
            hasCurrentEndPoint,
            currentEndPoint,
            sweepDirection,
            rotaryAxisConfig
        );

        const std::vector<bool> visitedComponents = buildVisitedComponentMask(visited, gapStartContext.componentIds);
        const int selectedComponentId =
            bestCandidate.index >= 0 && static_cast<size_t>(bestCandidate.index) < gapStartContext.componentIds.size()
            ? gapStartContext.componentIds[static_cast<size_t>(bestCandidate.index)]
            : -1;
        const bool enteringFreshPreferredComponent =
            selectedComponentId >= 0
            && static_cast<size_t>(selectedComponentId) < visitedComponents.size()
            && !visitedComponents[static_cast<size_t>(selectedComponentId)]
            && static_cast<size_t>(selectedComponentId) < gapStartContext.preferredStartPointsByComponent.size()
            && !gapStartContext.preferredStartPointsByComponent[static_cast<size_t>(selectedComponentId)].empty();

        if (enteringFreshPreferredComponent)
        {
            bestCandidate = chooseNext3DSortCandidate
            (
                sortableItems,
                visited,
                processedSegments,
                gapStartContext,
                SortStrategy::KeepDirection,
                currentComponentId,
                selectedComponentId,
                true,
                hasCurrentEndPoint,
                currentEndPoint,
                sweepDirection,
                rotaryAxisConfig
            );
        }

        if (bestCandidate.index < 0)
        {
            QMessageBox::warning(this, QStringLiteral("4轴(绕A)排序"), QStringLiteral("4轴排序过程中出现无效图元，排序已中止。"));
            return false;
        }

        visited[static_cast<size_t>(bestCandidate.index)] = true;
        processUpdates.push_back
        ({
            sortableItems[static_cast<size_t>(bestCandidate.index)],
            static_cast<int>(order),
            sortableItems[static_cast<size_t>(bestCandidate.index)]->m_isReverse,
            sortableItems[static_cast<size_t>(bestCandidate.index)]->m_hasCustomProcessStart,
            sortableItems[static_cast<size_t>(bestCandidate.index)]->m_processStartParameter
        });
        hasCurrentEndPoint = true;
        currentComponentId =
            static_cast<size_t>(bestCandidate.index) < gapStartContext.componentIds.size()
            ? gapStartContext.componentIds[static_cast<size_t>(bestCandidate.index)]
            : -1;
        currentEndPoint = bestCandidate.endPoint;
        processedSegments.push_back({ bestCandidate.startPoint, bestCandidate.endPoint });
    }

    if (!m_editer.applyEntityProcessStates(processUpdates))
    {
        QMessageBox::warning(this, QStringLiteral("4轴(绕A)排序"), QStringLiteral("4轴排序结果写入失败。"));
        return false;
    }

    ui->openGLWidget->appendCommandMessage
    (
        QStringLiteral("4轴(绕A)排序完成，共更新 %1 个图元的加工顺序，排序已按 X 与 A 轴联动连续性重新整理。").arg(processUpdates.size())
    );
    ui->openGLWidget->refreshCommandPrompt();
    statusBar()->showMessage(QStringLiteral("4轴(绕A)排序完成，共更新 %1 个图元").arg(processUpdates.size()), 5000);
    return true;
}

bool Gcode_postprocessing_system::smartSortEntities3D()
{
    if (m_document.m_entities.empty())
    {
        QMessageBox::warning(this, QStringLiteral("4轴(绕A)智能排序"), QStringLiteral("当前文档为空，无法执行智能排序。"));
        return false;
    }

    if (!documentContainsThreeDimensionalGeometry(m_document))
    {
        QMessageBox::warning(this, QStringLiteral("4轴(绕A)智能排序"), QStringLiteral("当前文档未检测到可用于4轴智能排序的有效路径。"));
        return false;
    }

    std::vector<CadItem*> sortableItems;

    for (const std::unique_ptr<CadItem>& entity : m_document.m_entities)
    {
        if (entity == nullptr)
        {
            continue;
        }

        const CadProcessVisualInfo info = buildProcessVisualInfo(entity.get());

        if (!info.valid)
        {
            continue;
        }

        sortableItems.push_back(entity.get());
    }

    if (sortableItems.empty())
    {
        QMessageBox::warning(this, QStringLiteral("4轴(绕A)智能排序"), QStringLiteral("当前文档中没有可参与 4 轴 G 代码排序的图元。"));
        return false;
    }

    const GProfileRotaryAxisConfig& rotaryAxisConfig = m_activeProfile.rotaryAxisConfig();
    const QVector3D sweepDirection = computeRotarySweepDirection(sortableItems, rotaryAxisConfig);
    const GapStartSelectionContext gapStartContext = buildGapStartSelectionContext(sortableItems, kPreferredStartGapDistance3D);
    std::vector<CadEditer::ProcessStateUpdate> processUpdates;
    std::vector<ProcessConnectionSegment> processedSegments;
    std::vector<bool> visited(sortableItems.size(), false);

    processUpdates.reserve(sortableItems.size());
    processedSegments.reserve(sortableItems.size());

    bool hasCurrentEndPoint = false;
    int currentComponentId = -1;
    QVector3D currentEndPoint;

    for (size_t order = 0; order < sortableItems.size(); ++order)
    {
        SortCandidate bestCandidate = chooseNext3DSortCandidate
        (
            sortableItems,
            visited,
            processedSegments,
            gapStartContext,
            SortStrategy::Smart,
            currentComponentId,
            -1,
            false,
            hasCurrentEndPoint,
            currentEndPoint,
            sweepDirection,
            rotaryAxisConfig
        );

        const std::vector<bool> visitedComponents = buildVisitedComponentMask(visited, gapStartContext.componentIds);
        const int selectedComponentId =
            bestCandidate.index >= 0 && static_cast<size_t>(bestCandidate.index) < gapStartContext.componentIds.size()
            ? gapStartContext.componentIds[static_cast<size_t>(bestCandidate.index)]
            : -1;
        const bool enteringFreshPreferredComponent =
            selectedComponentId >= 0
            && static_cast<size_t>(selectedComponentId) < visitedComponents.size()
            && !visitedComponents[static_cast<size_t>(selectedComponentId)]
            && static_cast<size_t>(selectedComponentId) < gapStartContext.preferredStartPointsByComponent.size()
            && !gapStartContext.preferredStartPointsByComponent[static_cast<size_t>(selectedComponentId)].empty();

        if (enteringFreshPreferredComponent)
        {
            bestCandidate = chooseNext3DSortCandidate
            (
                sortableItems,
                visited,
                processedSegments,
                gapStartContext,
                SortStrategy::Smart,
                currentComponentId,
                selectedComponentId,
                true,
                hasCurrentEndPoint,
                currentEndPoint,
                sweepDirection,
                rotaryAxisConfig
            );
        }

        if (bestCandidate.index < 0)
        {
            QMessageBox::warning(this, QStringLiteral("4轴(绕A)智能排序"), QStringLiteral("4轴智能排序过程中出现无效图元，排序已中止。"));
            return false;
        }

        visited[static_cast<size_t>(bestCandidate.index)] = true;
        processUpdates.push_back
        ({
            sortableItems[static_cast<size_t>(bestCandidate.index)],
            static_cast<int>(order),
            bestCandidate.reverse,
            bestCandidate.hasCustomStart,
            bestCandidate.processStartParameter
        });
        hasCurrentEndPoint = true;
        currentComponentId =
            static_cast<size_t>(bestCandidate.index) < gapStartContext.componentIds.size()
            ? gapStartContext.componentIds[static_cast<size_t>(bestCandidate.index)]
            : -1;
        currentEndPoint = bestCandidate.endPoint;
        processedSegments.push_back({ bestCandidate.startPoint, bestCandidate.endPoint });
    }

    if (!m_editer.applyEntityProcessStates(processUpdates))
    {
        QMessageBox::warning(this, QStringLiteral("4轴(绕A)智能排序"), QStringLiteral("4轴智能排序结果写入失败。"));
        return false;
    }

    ui->openGLWidget->appendCommandMessage
    (
        QStringLiteral("4轴(绕A)智能排序完成，共更新 %1 个图元的加工顺序，并已按 A 轴连续性优化方向与闭合图元缝点。").arg(processUpdates.size())
    );
    ui->openGLWidget->refreshCommandPrompt();
    statusBar()->showMessage(QStringLiteral("4轴(绕A)智能排序完成，共更新 %1 个图元").arg(processUpdates.size()), 5000);
    return true;
}

void Gcode_postprocessing_system::initializeThemeMenu()
{
    ui->menuSet->setTitle(QStringLiteral("用户设置"));
    ui->menuSort->setTitle(QStringLiteral("排序"));
    ui->action_Sort_2D_Assign->setText(QStringLiteral("排序（保留方向）"));
    ui->action_Sort_2D_Smart->setText(QStringLiteral("智能排序"));

    QMenu* generationMenu = ui->menuSet->addMenu(QStringLiteral("G代码模式"));
    QActionGroup* generationModeActionGroup = new QActionGroup(this);
    generationModeActionGroup->setExclusive(true);

    m_generationModeAutoAction = generationMenu->addAction(QStringLiteral("自动"));
    m_generationModeAutoAction->setCheckable(true);
    generationModeActionGroup->addAction(m_generationModeAutoAction);

    m_generationMode2DAction = generationMenu->addAction(QStringLiteral("3轴"));
    m_generationMode2DAction->setCheckable(true);
    generationModeActionGroup->addAction(m_generationMode2DAction);

    m_generationMode3DAction = generationMenu->addAction(QStringLiteral("4轴(绕A)"));
    m_generationMode3DAction->setCheckable(true);
    generationModeActionGroup->addAction(m_generationMode3DAction);

    m_generationModeAutoAction->setChecked(m_generationPreference == GCodeGenerationPreference::Auto);
    m_generationMode2DAction->setChecked(m_generationPreference == GCodeGenerationPreference::Force2D);
    m_generationMode3DAction->setChecked(m_generationPreference == GCodeGenerationPreference::Force3D);

    connect
    (
        m_generationModeAutoAction,
        &QAction::triggered,
        this,
        [this]()
        {
            applyGenerationPreference(GCodeGenerationPreference::Auto);
        }
    );
    connect
    (
        m_generationMode2DAction,
        &QAction::triggered,
        this,
        [this]()
        {
            applyGenerationPreference(GCodeGenerationPreference::Force2D);
        }
    );
    connect
    (
        m_generationMode3DAction,
        &QAction::triggered,
        this,
        [this]()
        {
            applyGenerationPreference(GCodeGenerationPreference::Force3D);
        }
    );

    ui->menuSet->addSeparator();
    QMenu* themeMenu = ui->menuSet->addMenu(QStringLiteral("主题"));
    QActionGroup* themeActionGroup = new QActionGroup(this);
    themeActionGroup->setExclusive(true);

    m_lightThemeAction = themeMenu->addAction(QStringLiteral("浅色模式"));
    m_lightThemeAction->setCheckable(true);
    themeActionGroup->addAction(m_lightThemeAction);

    m_darkThemeAction = themeMenu->addAction(QStringLiteral("深色模式"));
    m_darkThemeAction->setCheckable(true);
    themeActionGroup->addAction(m_darkThemeAction);

    connect(m_lightThemeAction, &QAction::triggered, this, [this]() { applyTheme(AppThemeMode::Light); });
    connect(m_darkThemeAction, &QAction::triggered, this, [this]() { applyTheme(AppThemeMode::Dark); });

    ui->menuSet->addSeparator();
    m_profileSettingsAction = ui->menuSet->addAction(QStringLiteral("G代码配置..."));
    connect(m_profileSettingsAction, &QAction::triggered, this, [this]() { openProfileSettingsDialog(); });
}

void Gcode_postprocessing_system::openProfileSettingsDialog()
{
    QMap<QString, QColor> layerColors;

    for (const QString& layerName : m_document.layerNames())
    {
        layerColors.insert(layerName, m_document.layerColor(layerName, QColor(Qt::white)));
    }

    GProfileDialog dialog
    (
        m_activeProfile,
        m_document.layerNames(),
        layerColors,
        buildAppThemeColors(m_themeMode),
        this
    );

    if (dialog.exec() != QDialog::Accepted)
    {
        return;
    }

    const GProfile updatedProfile = dialog.profile();
    const QString importedProfilePath = dialog.importedProfilePath().trimmed();

    if (!importedProfilePath.isEmpty())
    {
        const QString runtimeDirectory = runtimeProfileDirectoryPath();
        const QFileInfo importedFileInfo(importedProfilePath);
        const QString absoluteImportedPath = importedFileInfo.absoluteFilePath();
        const QString displayName = updatedProfile.profileName().trimmed().isEmpty()
            ? importedFileInfo.completeBaseName()
            : updatedProfile.profileName().trimmed();

        if (QFileInfo(absoluteImportedPath).dir().absolutePath().compare(runtimeDirectory, Qt::CaseInsensitive) == 0)
        {
            const QString profileId = QStringLiteral("file:%1").arg(QDir::toNativeSeparators(absoluteImportedPath));
            m_loadedProfiles.insert(profileId, updatedProfile);
            m_loadedProfileNames.insert(profileId, displayName);

            if (!m_loadedProfileOrder.contains(profileId))
            {
                m_loadedProfileOrder.append(profileId);
            }

            m_activeProfile = updatedProfile;
            m_activeProfileId = profileId;
        }
        else
        {
            const QString profileId = QStringLiteral("session:%1").arg(++m_sessionImportedProfileSerial);
            m_loadedProfiles.insert(profileId, updatedProfile);
            m_loadedProfileNames.insert(profileId, displayName);
            m_loadedProfileOrder.append(profileId);
            m_activeProfile = updatedProfile;
            m_activeProfileId = profileId;
        }
    }
    else
    {
        m_activeProfile = updatedProfile;

        if (!m_activeProfileId.isEmpty() && m_loadedProfiles.contains(m_activeProfileId))
        {
            m_loadedProfiles[m_activeProfileId] = updatedProfile;
            const QString displayName = updatedProfile.profileName().trimmed().isEmpty()
                ? m_loadedProfileNames.value(m_activeProfileId, QStringLiteral("未命名配置"))
                : updatedProfile.profileName().trimmed();
            m_loadedProfileNames[m_activeProfileId] = displayName;
        }
    }

    refreshAvailableProfilesUi();
    saveSelectedProfileId(m_activeProfileId);

    const QString profileName = m_activeProfile.profileName().trimmed().isEmpty()
        ? QStringLiteral("未命名配置")
        : m_activeProfile.profileName().trimmed();

    statusBar()->showMessage(QStringLiteral("当前 G 代码配置已更新为: %1").arg(profileName), 4000);
}

void Gcode_postprocessing_system::loadAvailableProfiles()
{
    m_loadedProfiles.clear();
    m_loadedProfileNames.clear();
    m_loadedProfileOrder.clear();
    m_sessionImportedProfileSerial = 0;

    const QString builtinThreeAxisProfileId = QString::fromLatin1(kBuiltinThreeAxisProfileId);
    const QString builtinFourAxisProfileId = QString::fromLatin1(kBuiltinFourAxisProfileId);

    m_loadedProfiles.insert(builtinThreeAxisProfileId, GProfile::createDefaultLaserProfile());
    m_loadedProfileNames.insert(builtinThreeAxisProfileId, QStringLiteral("内置3轴默认"));
    m_loadedProfileOrder.append(builtinThreeAxisProfileId);

    m_loadedProfiles.insert(builtinFourAxisProfileId, GProfile::createDefaultRotaryProfile());
    m_loadedProfileNames.insert(builtinFourAxisProfileId, QStringLiteral("内置4轴默认"));
    m_loadedProfileOrder.append(builtinFourAxisProfileId);

    const QDir runtimeDirectory(runtimeProfileDirectoryPath());
    const QFileInfoList profileFiles = runtimeDirectory.entryInfoList(QStringList() << QStringLiteral("*.json"), QDir::Files | QDir::Readable, QDir::Name);

    for (const QFileInfo& profileFileInfo : profileFiles)
    {
        QString errorMessage;
        const GProfile profile = GProfile::loadFromFile(profileFileInfo.absoluteFilePath(), &errorMessage);

        if (!errorMessage.trimmed().isEmpty())
        {
            continue;
        }

        const QString profileId = QStringLiteral("file:%1").arg(QDir::toNativeSeparators(profileFileInfo.absoluteFilePath()));
        const QString displayName = profile.profileName().trimmed().isEmpty()
            ? profileFileInfo.completeBaseName()
            : profile.profileName().trimmed();

        m_loadedProfiles.insert(profileId, profile);
        m_loadedProfileNames.insert(profileId, displayName);
        m_loadedProfileOrder.append(profileId);
    }

    const QString preferredProfileId = loadSelectedProfileId();

    if (!preferredProfileId.isEmpty() && m_loadedProfiles.contains(preferredProfileId))
    {
        m_activeProfileId = preferredProfileId;
        m_activeProfile = m_loadedProfiles.value(preferredProfileId, GProfile::createDefaultLaserProfile());
        return;
    }

    m_activeProfileId = builtinThreeAxisProfileId;
    m_activeProfile = m_loadedProfiles.value(m_activeProfileId, GProfile::createDefaultLaserProfile());
}

void Gcode_postprocessing_system::refreshAvailableProfilesUi()
{
    if (m_toolPanelWidget == nullptr)
    {
        return;
    }

    QList<QPair<QString, QString>> profiles;
    profiles.reserve(m_loadedProfileOrder.size());

    for (const QString& profileId : m_loadedProfileOrder)
    {
        profiles.append(qMakePair(profileId, m_loadedProfileNames.value(profileId, QStringLiteral("未命名配置"))));
    }

    m_toolPanelWidget->setAvailableProfiles(profiles);
    m_toolPanelWidget->setCurrentProfileSelection(m_activeProfileId);
}

bool Gcode_postprocessing_system::applyLoadedProfileById(const QString& profileId, bool announceChange)
{
    if (profileId.trimmed().isEmpty() || !m_loadedProfiles.contains(profileId))
    {
        return false;
    }

    m_activeProfileId = profileId;
    m_activeProfile = m_loadedProfiles.value(profileId, GProfile::createDefaultLaserProfile());
    saveSelectedProfileId(m_activeProfileId);
    refreshAvailableProfilesUi();

    if (announceChange)
    {
        statusBar()->showMessage
        (
            QStringLiteral("当前 G 代码配置已切换为: %1").arg(m_loadedProfileNames.value(profileId, QStringLiteral("未命名配置"))),
            4000
        );
    }

    return true;
}

QString Gcode_postprocessing_system::runtimeProfileDirectoryPath() const
{
    return QCoreApplication::applicationDirPath();
}

QString Gcode_postprocessing_system::loadSelectedProfileId() const
{
    QSettings settings(QStringLiteral("GCodePostProcessingSystem"), QStringLiteral("GCodePostProcessingSystem"));
    return settings.value(QStringLiteral("gcode/selectedProfileId"), QString::fromLatin1(kBuiltinThreeAxisProfileId)).toString().trimmed();
}

void Gcode_postprocessing_system::saveSelectedProfileId(const QString& profileId) const
{
    QSettings settings(QStringLiteral("GCodePostProcessingSystem"), QStringLiteral("GCodePostProcessingSystem"));
    settings.setValue(QStringLiteral("gcode/selectedProfileId"), profileId.trimmed());
}

void Gcode_postprocessing_system::applyTheme(AppThemeMode mode)
{
    m_themeMode = mode;
    const AppThemeColors theme = buildAppThemeColors(mode);

    qApp->setStyle(QStyleFactory::create(QStringLiteral("Fusion")));
    qApp->setPalette(theme.palette);

    setStyleSheet
    (
        QStringLiteral
        (
            "QMainWindow { background-color: %1; color: %2; }"
            "QWidget#centralWidget { background-color: %1; }"
            "QMenuBar { background-color: %3; color: %2; border-bottom: 1px solid %4; }"
            "QMenuBar::item { background: transparent; padding: 4px 10px; }"
            "QMenuBar::item:selected { background: %5; }"
            "QToolBar { background-color: %3; border: none; border-bottom: 1px solid %4; spacing: 0px; }"
            "QStatusBar { background-color: %3; color: %2; border-top: 1px solid %4; }"
            "QStatusBar::item { border: none; }"
        )
        .arg(theme.windowBackground.name())
        .arg(theme.textPrimaryColor.name())
        .arg(theme.panelBackground.name())
        .arg(theme.borderColor.name())
        .arg(theme.hoverBackgroundColor.name())
    );

    if (m_commandLineWidget != nullptr)
    {
        m_commandLineWidget->setTheme(theme);
    }

    if (m_statusPaneWidget != nullptr)
    {
        m_statusPaneWidget->setTheme(theme);
    }

    if (m_toolPanelWidget != nullptr)
    {
        m_toolPanelWidget->setTheme(theme);
    }

    if (ui->openGLWidget != nullptr)
    {
        ui->openGLWidget->setTheme(theme);
    }

    if (m_lightThemeAction != nullptr)
    {
        m_lightThemeAction->setChecked(mode == AppThemeMode::Light);
    }

    if (m_darkThemeAction != nullptr)
    {
        m_darkThemeAction->setChecked(mode == AppThemeMode::Dark);
    }

    saveThemeMode(mode);
}

AppThemeMode Gcode_postprocessing_system::loadThemeMode() const
{
    QSettings settings(QStringLiteral("GCodePostProcessingSystem"), QStringLiteral("GCodePostProcessingSystem"));
    const QString themeValue = settings.value(QStringLiteral("ui/themeMode"), QStringLiteral("light")).toString().trimmed().toLower();
    return themeValue == QStringLiteral("dark") ? AppThemeMode::Dark : AppThemeMode::Light;
}

void Gcode_postprocessing_system::saveThemeMode(AppThemeMode mode) const
{
    QSettings settings(QStringLiteral("GCodePostProcessingSystem"), QStringLiteral("GCodePostProcessingSystem"));
    settings.setValue(QStringLiteral("ui/themeMode"), mode == AppThemeMode::Dark ? QStringLiteral("dark") : QStringLiteral("light"));
}

quint32 Gcode_postprocessing_system::loadSnapOptionMask() const
{
    QSettings settings(QStringLiteral("GCodePostProcessingSystem"), QStringLiteral("GCodePostProcessingSystem"));
    const quint32 defaultMask = CadStatusPaneWidget::defaultSnapOptionMask();
    bool converted = false;
    const quint32 storedMask = settings.value(QStringLiteral("ui/snapModeMask"), defaultMask).toUInt(&converted);

    if (!converted)
    {
        return defaultMask;
    }

    return storedMask & CadStatusPaneWidget::allSnapOptionMask();
}

void Gcode_postprocessing_system::saveSnapOptionMask(quint32 mask) const
{
    QSettings settings(QStringLiteral("GCodePostProcessingSystem"), QStringLiteral("GCodePostProcessingSystem"));
    settings.setValue
    (
        QStringLiteral("ui/snapModeMask"),
        static_cast<uint>(mask & CadStatusPaneWidget::allSnapOptionMask())
    );
}

bool Gcode_postprocessing_system::removeDuplicateEntities()
{
    if (m_document.m_entities.empty())
    {
        statusBar()->showMessage(QStringLiteral("当前文档没有可去重的图元"), 3000);
        return false;
    }

    QSet<QString> seenKeys;
    QVector<CadItem*> duplicates;

    for (auto it = m_document.m_entities.rbegin(); it != m_document.m_entities.rend(); ++it)
    {
        CadItem* item = it->get();

        if (item == nullptr || item->m_nativeEntity == nullptr)
        {
            continue;
        }

        const QString geometryKey = duplicateGeometryKey(item);

        if (geometryKey.isEmpty())
        {
            continue;
        }

        if (seenKeys.contains(geometryKey))
        {
            duplicates.push_back(item);
            continue;
        }

        seenKeys.insert(geometryKey);
    }

    if (duplicates.isEmpty())
    {
        statusBar()->showMessage(QStringLiteral("未发现完全重叠的同类型图元"), 3000);
        return false;
    }

    std::reverse(duplicates.begin(), duplicates.end());

    if (!m_editer.deleteEntities(duplicates))
    {
        QMessageBox::warning(this, QStringLiteral("去重失败"), QStringLiteral("删除重复图元时发生错误。"));
        return false;
    }

    ui->openGLWidget->clearSelection();
    ui->openGLWidget->appendCommandMessage(QStringLiteral("去重完成，删除重复图元 %1 个").arg(duplicates.size()));
    statusBar()->showMessage(QStringLiteral("去重完成，删除重复图元 %1 个").arg(duplicates.size()), 4000);
    return true;
}

QString Gcode_postprocessing_system::generationModeDisplayName(GGenerator::GenerationMode generationMode) const
{
    return generationMode == GGenerator::GenerationMode::Mode3D
        ? QStringLiteral("4轴(绕A)")
        : QStringLiteral("3轴");
}

Gcode_postprocessing_system::GCodeGenerationPreference Gcode_postprocessing_system::loadGenerationPreference() const
{
    QSettings settings(QStringLiteral("GCodePostProcessingSystem"), QStringLiteral("GCodePostProcessingSystem"));
    const QString modeValue = settings.value(QStringLiteral("gcode/outputMode"), QStringLiteral("auto")).toString().trimmed().toLower();

    if (modeValue == QStringLiteral("2d"))
    {
        return GCodeGenerationPreference::Force2D;
    }

    if (modeValue == QStringLiteral("3d"))
    {
        return GCodeGenerationPreference::Force3D;
    }

    return GCodeGenerationPreference::Auto;
}

void Gcode_postprocessing_system::saveGenerationPreference(GCodeGenerationPreference preference) const
{
    QSettings settings(QStringLiteral("GCodePostProcessingSystem"), QStringLiteral("GCodePostProcessingSystem"));
    QString modeValue = QStringLiteral("auto");

    if (preference == GCodeGenerationPreference::Force2D)
    {
        modeValue = QStringLiteral("2d");
    }
    else if (preference == GCodeGenerationPreference::Force3D)
    {
        modeValue = QStringLiteral("3d");
    }

    settings.setValue(QStringLiteral("gcode/outputMode"), modeValue);
}

void Gcode_postprocessing_system::applyGenerationPreference(GCodeGenerationPreference preference)
{
    m_generationPreference = preference;
    saveGenerationPreference(m_generationPreference);

    if (m_generationModeAutoAction != nullptr)
    {
        m_generationModeAutoAction->setChecked(m_generationPreference == GCodeGenerationPreference::Auto);
    }

    if (m_generationMode2DAction != nullptr)
    {
        m_generationMode2DAction->setChecked(m_generationPreference == GCodeGenerationPreference::Force2D);
    }

    if (m_generationMode3DAction != nullptr)
    {
        m_generationMode3DAction->setChecked(m_generationPreference == GCodeGenerationPreference::Force3D);
    }

    if (m_toolPanelWidget != nullptr)
    {
        switch (m_generationPreference)
        {
        case GCodeGenerationPreference::Auto:
            m_toolPanelWidget->setGCodeModeSelection(CadToolPanelWidget::GCodeModeSelection::Auto);
            break;
        case GCodeGenerationPreference::Force2D:
            m_toolPanelWidget->setGCodeModeSelection(CadToolPanelWidget::GCodeModeSelection::ThreeAxis);
            break;
        case GCodeGenerationPreference::Force3D:
            m_toolPanelWidget->setGCodeModeSelection(CadToolPanelWidget::GCodeModeSelection::FourAxisAroundA);
            break;
        default:
            break;
        }
    }

    const QString modeText = m_generationPreference == GCodeGenerationPreference::Force3D
        ? QStringLiteral("4轴(绕A)")
        : (m_generationPreference == GCodeGenerationPreference::Force2D
            ? QStringLiteral("3轴")
            : QStringLiteral("自动"));
    statusBar()->showMessage(QStringLiteral("G 代码输出模式已切换为%1").arg(modeText), 3000);
}

GGenerator::GenerationMode Gcode_postprocessing_system::resolveGenerationMode() const
{
    switch (m_generationPreference)
    {
    case GCodeGenerationPreference::Force2D:
        return GGenerator::GenerationMode::Mode2D;
    case GCodeGenerationPreference::Force3D:
        return GGenerator::GenerationMode::Mode3D;
    case GCodeGenerationPreference::Auto:
    default:
        return documentContainsThreeDimensionalGeometry(m_document)
            ? GGenerator::GenerationMode::Mode3D
            : GGenerator::GenerationMode::Mode2D;
    }
}

void Gcode_postprocessing_system::initializeToolPanel()
{
    m_toolPanelWidget = new CadToolPanelWidget(this);
    ui->mainToolBar->setMovable(false);
    ui->mainToolBar->setFloatable(false);
    ui->mainToolBar->addWidget(m_toolPanelWidget);

    connect(&m_document, &CadDocument::sceneChanged, this, [this]() { syncToolPanelState(); });
    connect(ui->openGLWidget, &CadViewer::selectedEntityChanged, this, [this](CadItem*) { syncToolPanelState(); });

    connect
    (
        m_toolPanelWidget,
        &CadToolPanelWidget::drawRequested,
        this,
        [this](DrawType drawType)
        {
            applyDefaultDrawingProperties();
            ui->openGLWidget->startDrawing(drawType);
        }
    );

    connect
    (
        m_toolPanelWidget,
        &CadToolPanelWidget::moveRequested,
        this,
        [this]()
        {
            if (!ui->openGLWidget->startMoveSelected())
            {
                statusBar()->showMessage(QStringLiteral("请先选择一个图元再执行移动"), 3000);
            }
        }
    );

    connect(m_toolPanelWidget, &CadToolPanelWidget::deleteRequested, this, [this]() { deleteSelectedEntity(); });
    connect(m_toolPanelWidget, &CadToolPanelWidget::copyRequested, this, [this]() { copySelectedEntity(); });
    connect(m_toolPanelWidget, &CadToolPanelWidget::rotateRequested, this, [this]() { rotateSelectedEntity(); });
    connect(m_toolPanelWidget, &CadToolPanelWidget::scaleRequested, this, [this]() { scaleSelectedEntity(); });
    connect(m_toolPanelWidget, &CadToolPanelWidget::arrayRequested, this, [this]() { arraySelectedEntity(); });
    connect(m_toolPanelWidget, &CadToolPanelWidget::importFileRequested, this, [this]() { ui->action_File_Import_Dxf->trigger(); });
    connect(m_toolPanelWidget, &CadToolPanelWidget::exportGCodeRequested, this, [this]() { ui->action_File_Export_G->trigger(); });
    connect(m_toolPanelWidget, &CadToolPanelWidget::deduplicateRequested, this, [this]() { removeDuplicateEntities(); });
    connect(m_toolPanelWidget, &CadToolPanelWidget::sortKeepDirectionRequested, this, [this]() { ui->action_Sort_2D_Assign->trigger(); });
    connect(m_toolPanelWidget, &CadToolPanelWidget::smartSortRequested, this, [this]() { ui->action_Sort_2D_Smart->trigger(); });
    connect
    (
        m_toolPanelWidget,
        &CadToolPanelWidget::profileSelectionChanged,
        this,
        [this](const QString& profileId)
        {
            applyLoadedProfileById(profileId);
        }
    );
    connect(m_toolPanelWidget, &CadToolPanelWidget::profileSettingsRequested, this, [this]() { openProfileSettingsDialog(); });
    connect
    (
        m_toolPanelWidget,
        &CadToolPanelWidget::gcodeModeSelectionChanged,
        this,
        [this](CadToolPanelWidget::GCodeModeSelection selection)
        {
            switch (selection)
            {
            case CadToolPanelWidget::GCodeModeSelection::Auto:
                applyGenerationPreference(GCodeGenerationPreference::Auto);
                break;
            case CadToolPanelWidget::GCodeModeSelection::ThreeAxis:
                applyGenerationPreference(GCodeGenerationPreference::Force2D);
                break;
            case CadToolPanelWidget::GCodeModeSelection::FourAxisAroundA:
                applyGenerationPreference(GCodeGenerationPreference::Force3D);
                break;
            default:
                break;
            }
        }
    );

    connect
    (
        m_toolPanelWidget,
        &CadToolPanelWidget::layerChangeRequested,
        this,
        [this](const QString& layerName)
        {
            const QString normalizedLayerName = layerName.trimmed().isEmpty() ? QStringLiteral("0") : layerName.trimmed();
            const QVector<CadItem*> selectedItems = ui->openGLWidget->selectedEntities();

            if (!selectedItems.isEmpty())
            {
                int changedCount = 0;

                for (CadItem* item : selectedItems)
                {
                    if (item != nullptr && m_editer.changeEntityLayer(item, normalizedLayerName))
                    {
                        ++changedCount;
                    }
                }

                if (changedCount > 0)
                {
                    statusBar()->showMessage
                    (
                        QStringLiteral("已将 %1 个图元图层更新为 %2").arg(changedCount).arg(normalizedLayerName),
                        3000
                    );
                }

                return;
            }

            m_currentLayerName = normalizedLayerName;

            if (m_document.ensureLayerExists(m_currentLayerName))
            {
                m_document.notifySceneChanged();
            }

            applyDefaultDrawingProperties();
            syncToolPanelState();
        }
    );

    connect
    (
        m_toolPanelWidget,
        &CadToolPanelWidget::colorChangeRequested,
        this,
        [this](int colorIndex)
        {
            const QVector<CadItem*> selectedItems = ui->openGLWidget->selectedEntities();

            if (!selectedItems.isEmpty())
            {
                int changedCount = 0;

                for (CadItem* item : selectedItems)
                {
                    if (item == nullptr)
                    {
                        continue;
                    }

                    const QColor targetColor = colorIndex == kColorByLayer
                        ? m_document.layerColor(entityLayerName(item), entityDisplayColor(m_document, item))
                        : (colorIndex < 0 ? entityDisplayColor(m_document, item) : colorFromAci(colorIndex));

                    if (m_editer.changeEntityColor(item, targetColor, colorIndex))
                    {
                        ++changedCount;
                    }
                }

                if (changedCount > 0)
                {
                    statusBar()->showMessage(QStringLiteral("已更新 %1 个图元颜色").arg(changedCount), 3000);
                }

                return;
            }

            m_currentColorIndex = colorIndex;

            if (colorIndex == kColorByLayer)
            {
                m_currentColor = m_document.layerColor(m_currentLayerName, QColor(Qt::white));
            }
            else if (colorIndex >= 0)
            {
                m_currentColor = colorFromAci(colorIndex);
            }

            applyDefaultDrawingProperties();
            syncToolPanelState();
        }
    );

    switch (m_generationPreference)
    {
    case GCodeGenerationPreference::Auto:
        m_toolPanelWidget->setGCodeModeSelection(CadToolPanelWidget::GCodeModeSelection::Auto);
        break;
    case GCodeGenerationPreference::Force2D:
        m_toolPanelWidget->setGCodeModeSelection(CadToolPanelWidget::GCodeModeSelection::ThreeAxis);
        break;
    case GCodeGenerationPreference::Force3D:
        m_toolPanelWidget->setGCodeModeSelection(CadToolPanelWidget::GCodeModeSelection::FourAxisAroundA);
        break;
    default:
        break;
    }

    refreshAvailableProfilesUi();
}

void Gcode_postprocessing_system::syncToolPanelState()
{
    if (m_toolPanelWidget == nullptr)
    {
        return;
    }

    const QStringList layerNames = m_document.layerNames();
    QMap<QString, QColor> layerColors;

    for (const QString& layerName : layerNames)
    {
        layerColors.insert(layerName, m_document.layerColor(layerName, QColor(Qt::white)));
    }

    m_toolPanelWidget->setLayerNames(layerNames, layerColors);

    CadItem* selectedItem = ui->openGLWidget->selectedEntity();

    if (selectedItem != nullptr)
    {
        m_toolPanelWidget->setModifyActionsEnabled(true);
        m_toolPanelWidget->setLayerStatusText(QStringLiteral("当前选中图元图层"));
        m_toolPanelWidget->setPropertyStatusText(QStringLiteral("当前选中图元特性"));
        m_toolPanelWidget->setActiveLayerName(entityLayerName(selectedItem));
        m_toolPanelWidget->setActiveColorState
        (
            entityDisplayColor(m_document, selectedItem),
            entityColorIndex(selectedItem),
            m_document.layerColor(entityLayerName(selectedItem), QColor(Qt::white))
        );
        return;
    }

    if (m_currentLayerName.trimmed().isEmpty())
    {
        m_currentLayerName = layerNames.isEmpty() ? QStringLiteral("0") : layerNames.front();
    }

    if (m_currentColorIndex == kColorByLayer)
    {
        m_currentColor = m_document.layerColor(m_currentLayerName, QColor(Qt::white));
    }
    else if (m_currentColorIndex >= 0)
    {
        m_currentColor = colorFromAci(m_currentColorIndex);
    }

    m_toolPanelWidget->setModifyActionsEnabled(false);
    m_toolPanelWidget->setLayerStatusText(QStringLiteral("当前默认绘图图层"));
    m_toolPanelWidget->setPropertyStatusText(QStringLiteral("当前默认绘图特性"));
    m_toolPanelWidget->setActiveLayerName(m_currentLayerName);
    m_toolPanelWidget->setActiveColorState
    (
        m_currentColor,
        m_currentColorIndex,
        m_document.layerColor(m_currentLayerName, QColor(Qt::white))
    );
}

void Gcode_postprocessing_system::applyDefaultDrawingProperties()
{
    if (m_currentLayerName.trimmed().isEmpty())
    {
        m_currentLayerName = QStringLiteral("0");
    }

    if (m_currentColorIndex == kColorByLayer)
    {
        m_currentColor = m_document.layerColor(m_currentLayerName, QColor(Qt::white));
    }
    else if (m_currentColorIndex >= 0)
    {
        m_currentColor = colorFromAci(m_currentColorIndex);
    }

    ui->openGLWidget->setDefaultDrawingProperties(m_currentLayerName, m_currentColor, m_currentColorIndex);
}

QString Gcode_postprocessing_system::activeLayerName() const
{
    CadItem* selectedItem = ui->openGLWidget->selectedEntity();
    return selectedItem != nullptr ? entityLayerName(selectedItem) : m_currentLayerName;
}

QColor Gcode_postprocessing_system::activeColor() const
{
    CadItem* selectedItem = ui->openGLWidget->selectedEntity();
    return selectedItem != nullptr ? entityDisplayColor(m_document, selectedItem) : m_currentColor;
}

int Gcode_postprocessing_system::activeColorIndex() const
{
    CadItem* selectedItem = ui->openGLWidget->selectedEntity();
    return selectedItem != nullptr ? entityColorIndex(selectedItem) : m_currentColorIndex;
}
