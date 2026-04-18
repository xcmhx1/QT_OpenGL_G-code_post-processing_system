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
#include <QToolBar>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace
{
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
    const QVector3D kSortOrigin(0.0f, 0.0f, 0.0f);

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
        double priorityDistance = std::numeric_limits<double>::max();
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

    std::vector<ProcessPathOption> buildPathOptionsForItem(const CadItem* item, SortStrategy strategy);

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
        SortStrategy strategy,
        bool hasCurrentEndPoint,
        const QVector3D& currentEndPoint,
        const QVector3D& sweepDirection
    )
    {
        SortCandidate bestCandidate;
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
                const double entryDistance = std::sqrt(planarDistanceSquared(option.startPoint, referencePoint));
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
                    || optionScore < bestCandidate.score - kSortEpsilon
                    || (std::abs(optionScore - bestCandidate.score) <= kSortEpsilon
                        && (entryDistance < bestCandidate.priorityDistance - kSortEpsilon
                            || (std::abs(entryDistance - bestCandidate.priorityDistance) <= kSortEpsilon
                                && isPointLexicographicallyLess(option.startPoint, bestCandidate.startPoint))));

                if (!shouldReplace)
                {
                    continue;
                }

                bestCandidate.index = static_cast<int>(index);
                bestCandidate.reverse = option.reverse;
                bestCandidate.hasCustomStart = option.hasCustomStart;
                bestCandidate.processStartParameter = option.processStartParameter;
                bestCandidate.priorityDistance = entryDistance;
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
        SortStrategy strategy,
        bool hasCurrentEndPoint,
        const QVector3D& currentEndPoint,
        const QVector3D& sweepDirection,
        const GProfileRotaryAxisConfig& config
    )
    {
        SortCandidate bestCandidate;
        const QVector3D referencePoint = hasCurrentEndPoint ? currentEndPoint : kSortOrigin;
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
                double resolvedCandidateAngle = 0.0;
                const double entryDistance = rotarySortTravelDistance(referencePoint, option.startPoint, config, &resolvedCandidateAngle);
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
                    || optionScore < bestCandidate.score - kSortEpsilon
                    || (std::abs(optionScore - bestCandidate.score) <= kSortEpsilon
                        && (entryDistance < bestCandidate.priorityDistance - kSortEpsilon
                            || (std::abs(entryDistance - bestCandidate.priorityDistance) <= kSortEpsilon
                                && isPointLexicographicallyLess(option.startPoint, bestCandidate.startPoint))));

                if (!shouldReplace)
                {
                    continue;
                }

                bestCandidate.index = static_cast<int>(index);
                bestCandidate.reverse = option.reverse;
                bestCandidate.hasCustomStart = option.hasCustomStart;
                bestCandidate.processStartParameter = option.processStartParameter;
                bestCandidate.priorityDistance = entryDistance;
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

    ui->action_File_Export_G->setText(QStringLiteral("导出为DXF..."));
    QAction* exportSafeDxfAction = new QAction(QStringLiteral("导出为DXF（安全模式）..."), this);
    QAction* exportGCodeAction = new QAction(QStringLiteral("导出G代码..."), this);
    ui->menuFile->insertAction(ui->action_File_Export_G, exportSafeDxfAction);
    ui->menuFile->insertAction(ui->action_File_Export_G, exportGCodeAction);
    connect(ui->action_FileExport, &QAction::triggered, this, [this]() { saveCurrentDocument(); });
    connect(ui->action_File_Export_G, &QAction::triggered, this, [this]() { exportDxfDocument(); });
    connect(exportSafeDxfAction, &QAction::triggered, this, [this]() { exportDxfDocument(true); });
    connect(exportGCodeAction, &QAction::triggered, this, [this]() { exportGCodeDocument(); });
    connect(ui->action_Edit_ReversePeocess, &QAction::triggered, this, [this]() { toggleSelectedEntityReverse(); });
    connect(ui->action_Sort_2D_Assign, &QAction::triggered, this, [this]() { sortEntitiesByCurrentDirection(); });
    connect(ui->action_Sort_2D_Smart, &QAction::triggered, this, [this]() { smartSortEntities(); });
    connect(ui->action_Sort_3D_Assign, &QAction::triggered, this, [this]() { sortEntitiesByCurrentDirection3D(); });
    connect(ui->action_Sort_3D_Smart, &QAction::triggered, this, [this]() { smartSortEntities3D(); });

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

bool Gcode_postprocessing_system::exportGCodeDocument()
{
    if (m_document.m_entities.empty())
    {
        QMessageBox::warning(this, QStringLiteral("导出G代码"), QStringLiteral("当前文档为空，无法导出 G 代码。"));
        return false;
    }

    GGenerator generator;
    generator.setDocument(&m_document);
    generator.setProfile(&m_activeProfile);
    generator.setGenerationMode(resolveGenerationMode());

    QString message;

    if (!generator.generate(this, &message))
    {
        if (!message.trimmed().isEmpty())
        {
            QMessageBox::warning(this, QStringLiteral("导出G代码"), message);
        }

        return false;
    }

    if (!message.trimmed().isEmpty())
    {
        statusBar()->showMessage(QStringLiteral("G代码导出完成（含提示）"), 5000);
        ui->openGLWidget->appendCommandMessage(QStringLiteral("G代码导出完成：%1").arg(message));
    }
    else
    {
        statusBar()->showMessage(QStringLiteral("G代码导出完成"), 5000);
        ui->openGLWidget->appendCommandMessage(QStringLiteral("G代码导出完成。"));
    }

    ui->openGLWidget->refreshCommandPrompt();
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
        QMessageBox::warning(this, QStringLiteral("2D排序"), QStringLiteral("当前文档为空，无法执行排序。"));
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
        QMessageBox::warning(this, QStringLiteral("2D排序"), QStringLiteral("当前文档中没有可参与 G 代码排序的图元。"));
        return false;
    }

    const QVector3D sweepDirection = computeSweepDirection(sortableItems);
    std::vector<CadEditer::ProcessStateUpdate> processUpdates;
    std::vector<bool> visited(sortableItems.size(), false);

    processUpdates.reserve(sortableItems.size());

    bool hasCurrentEndPoint = false;
    QVector3D currentEndPoint;

    for (size_t order = 0; order < sortableItems.size(); ++order)
    {
        const SortCandidate bestCandidate = chooseNext2DSortCandidate
        (
            sortableItems,
            visited,
            SortStrategy::KeepDirection,
            hasCurrentEndPoint,
            currentEndPoint,
            sweepDirection
        );

        if (bestCandidate.index < 0)
        {
            QMessageBox::warning(this, QStringLiteral("2D排序"), QStringLiteral("排序过程中出现无效图元，排序已中止。"));
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
        currentEndPoint = bestCandidate.endPoint;
    }

    if (!m_editer.applyEntityProcessStates(processUpdates))
    {
        QMessageBox::warning(this, QStringLiteral("2D排序"), QStringLiteral("排序结果写入失败。"));
        return false;
    }

    ui->openGLWidget->appendCommandMessage
    (
        QStringLiteral("2D排序完成，共更新 %1 个图元的加工顺序，首件已按最接近原点的当前起点选取，并保留当前加工方向设置。").arg(processUpdates.size())
    );
    ui->openGLWidget->refreshCommandPrompt();
    statusBar()->showMessage(QStringLiteral("2D排序完成，共更新 %1 个图元").arg(processUpdates.size()), 5000);
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
        QMessageBox::warning(this, QStringLiteral("2D智能排序"), QStringLiteral("当前文档为空，无法执行智能排序。"));
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
        QMessageBox::warning(this, QStringLiteral("2D智能排序"), QStringLiteral("当前文档中没有可参与 G 代码排序的图元。"));
        return false;
    }

    const QVector3D sweepDirection = computeSweepDirection(sortableItems);
    std::vector<CadEditer::ProcessStateUpdate> processUpdates;
    std::vector<bool> visited(sortableItems.size(), false);

    processUpdates.reserve(sortableItems.size());

    bool hasCurrentEndPoint = false;
    QVector3D currentEndPoint;

    for (size_t order = 0; order < sortableItems.size(); ++order)
    {
        const SortCandidate bestCandidate = chooseNext2DSortCandidate
        (
            sortableItems,
            visited,
            SortStrategy::Smart,
            hasCurrentEndPoint,
            currentEndPoint,
            sweepDirection
        );

        if (bestCandidate.index < 0)
        {
            QMessageBox::warning(this, QStringLiteral("2D智能排序"), QStringLiteral("智能排序过程中出现无效图元，排序已中止。"));
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
        currentEndPoint = bestCandidate.endPoint;
    }

    if (!m_editer.applyEntityProcessStates(processUpdates))
    {
        QMessageBox::warning(this, QStringLiteral("2D智能排序"), QStringLiteral("智能排序结果写入失败。"));
        return false;
    }

    ui->openGLWidget->appendCommandMessage
    (
        QStringLiteral("2D智能排序完成，共更新 %1 个图元的加工顺序，并已对闭合图元的方向/起刀缝点做连续性优化。").arg(processUpdates.size())
    );
    ui->openGLWidget->refreshCommandPrompt();
    statusBar()->showMessage(QStringLiteral("2D智能排序完成，共更新 %1 个图元").arg(processUpdates.size()), 5000);
    return true;
}

bool Gcode_postprocessing_system::sortEntitiesByCurrentDirection3D()
{
    if (m_document.m_entities.empty())
    {
        QMessageBox::warning(this, QStringLiteral("3D排序"), QStringLiteral("当前文档为空，无法执行排序。"));
        return false;
    }

    if (!documentContainsThreeDimensionalGeometry(m_document))
    {
        QMessageBox::warning(this, QStringLiteral("3D排序"), QStringLiteral("当前文档未检测到真实 3D 路径，3D 排序仅适用于导入后的三维图元。"));
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
        QMessageBox::warning(this, QStringLiteral("3D排序"), QStringLiteral("当前文档中没有可参与 3D G 代码排序的图元。"));
        return false;
    }

    const GProfileRotaryAxisConfig& rotaryAxisConfig = m_activeProfile.rotaryAxisConfig();
    const QVector3D sweepDirection = computeRotarySweepDirection(sortableItems, rotaryAxisConfig);
    std::vector<CadEditer::ProcessStateUpdate> processUpdates;
    std::vector<bool> visited(sortableItems.size(), false);

    processUpdates.reserve(sortableItems.size());

    bool hasCurrentEndPoint = false;
    QVector3D currentEndPoint;

    for (size_t order = 0; order < sortableItems.size(); ++order)
    {
        const SortCandidate bestCandidate = chooseNext3DSortCandidate
        (
            sortableItems,
            visited,
            SortStrategy::KeepDirection,
            hasCurrentEndPoint,
            currentEndPoint,
            sweepDirection,
            rotaryAxisConfig
        );

        if (bestCandidate.index < 0)
        {
            QMessageBox::warning(this, QStringLiteral("3D排序"), QStringLiteral("3D 排序过程中出现无效图元，排序已中止。"));
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
        currentEndPoint = bestCandidate.endPoint;
    }

    if (!m_editer.applyEntityProcessStates(processUpdates))
    {
        QMessageBox::warning(this, QStringLiteral("3D排序"), QStringLiteral("3D 排序结果写入失败。"));
        return false;
    }

    ui->openGLWidget->appendCommandMessage
    (
        QStringLiteral("3D排序完成，共更新 %1 个图元的加工顺序，排序已按 X 与 A 轴联动连续性重新整理。").arg(processUpdates.size())
    );
    ui->openGLWidget->refreshCommandPrompt();
    statusBar()->showMessage(QStringLiteral("3D排序完成，共更新 %1 个图元").arg(processUpdates.size()), 5000);
    return true;
}

bool Gcode_postprocessing_system::smartSortEntities3D()
{
    if (m_document.m_entities.empty())
    {
        QMessageBox::warning(this, QStringLiteral("3D智能排序"), QStringLiteral("当前文档为空，无法执行智能排序。"));
        return false;
    }

    if (!documentContainsThreeDimensionalGeometry(m_document))
    {
        QMessageBox::warning(this, QStringLiteral("3D智能排序"), QStringLiteral("当前文档未检测到真实 3D 路径，3D 智能排序仅适用于导入后的三维图元。"));
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
        QMessageBox::warning(this, QStringLiteral("3D智能排序"), QStringLiteral("当前文档中没有可参与 3D G 代码排序的图元。"));
        return false;
    }

    const GProfileRotaryAxisConfig& rotaryAxisConfig = m_activeProfile.rotaryAxisConfig();
    const QVector3D sweepDirection = computeRotarySweepDirection(sortableItems, rotaryAxisConfig);
    std::vector<CadEditer::ProcessStateUpdate> processUpdates;
    std::vector<bool> visited(sortableItems.size(), false);

    processUpdates.reserve(sortableItems.size());

    bool hasCurrentEndPoint = false;
    QVector3D currentEndPoint;

    for (size_t order = 0; order < sortableItems.size(); ++order)
    {
        const SortCandidate bestCandidate = chooseNext3DSortCandidate
        (
            sortableItems,
            visited,
            SortStrategy::Smart,
            hasCurrentEndPoint,
            currentEndPoint,
            sweepDirection,
            rotaryAxisConfig
        );

        if (bestCandidate.index < 0)
        {
            QMessageBox::warning(this, QStringLiteral("3D智能排序"), QStringLiteral("3D 智能排序过程中出现无效图元，排序已中止。"));
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
        currentEndPoint = bestCandidate.endPoint;
    }

    if (!m_editer.applyEntityProcessStates(processUpdates))
    {
        QMessageBox::warning(this, QStringLiteral("3D智能排序"), QStringLiteral("3D 智能排序结果写入失败。"));
        return false;
    }

    ui->openGLWidget->appendCommandMessage
    (
        QStringLiteral("3D智能排序完成，共更新 %1 个图元的加工顺序，并已按 A 轴连续性优化方向与闭合图元缝点。").arg(processUpdates.size())
    );
    ui->openGLWidget->refreshCommandPrompt();
    statusBar()->showMessage(QStringLiteral("3D智能排序完成，共更新 %1 个图元").arg(processUpdates.size()), 5000);
    return true;
}

void Gcode_postprocessing_system::initializeThemeMenu()
{
    ui->menuSet->setTitle(QStringLiteral("用户设置"));

    QMenu* generationMenu = ui->menuSet->addMenu(QStringLiteral("G代码模式"));
    QActionGroup* generationModeActionGroup = new QActionGroup(this);
    generationModeActionGroup->setExclusive(true);

    m_generationModeAutoAction = generationMenu->addAction(QStringLiteral("自动"));
    m_generationModeAutoAction->setCheckable(true);
    generationModeActionGroup->addAction(m_generationModeAutoAction);

    m_generationMode2DAction = generationMenu->addAction(QStringLiteral("强制2D"));
    m_generationMode2DAction->setCheckable(true);
    generationModeActionGroup->addAction(m_generationMode2DAction);

    m_generationMode3DAction = generationMenu->addAction(QStringLiteral("强制3D(A轴)"));
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
            m_generationPreference = GCodeGenerationPreference::Auto;
            saveGenerationPreference(m_generationPreference);
            statusBar()->showMessage(QStringLiteral("G 代码输出模式已切换为自动"), 3000);
        }
    );
    connect
    (
        m_generationMode2DAction,
        &QAction::triggered,
        this,
        [this]()
        {
            m_generationPreference = GCodeGenerationPreference::Force2D;
            saveGenerationPreference(m_generationPreference);
            statusBar()->showMessage(QStringLiteral("G 代码输出模式已切换为强制2D"), 3000);
        }
    );
    connect
    (
        m_generationMode3DAction,
        &QAction::triggered,
        this,
        [this]()
        {
            m_generationPreference = GCodeGenerationPreference::Force3D;
            saveGenerationPreference(m_generationPreference);
            statusBar()->showMessage(QStringLiteral("G 代码输出模式已切换为强制3D(A轴)"), 3000);
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

    m_activeProfile = dialog.profile();

    const QString profileName = m_activeProfile.profileName().trimmed().isEmpty()
        ? QStringLiteral("未命名配置")
        : m_activeProfile.profileName().trimmed();

    statusBar()->showMessage(QStringLiteral("当前 G 代码配置已更新为: %1").arg(profileName), 4000);
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
