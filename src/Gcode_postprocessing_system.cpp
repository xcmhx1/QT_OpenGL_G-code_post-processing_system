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

    QVector3D arcTangentAt(double angle, bool reverseDirection)
    {
        QVector3D tangent
        (
            static_cast<float>(-std::sin(angle)),
            static_cast<float>(std::cos(angle)),
            0.0f
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
            const QVector3D center(arc->basePoint.x, arc->basePoint.y, arc->basePoint.z);
            const auto pointAtAngle =
                [&center, arc](double angle)
                {
                    return QVector3D
                    (
                        static_cast<float>(center.x() + std::cos(angle) * arc->radious),
                        static_cast<float>(center.y() + std::sin(angle) * arc->radious),
                        center.z()
                    );
                };
            const std::initializer_list<bool> reverseOptions = strategy == SortStrategy::Smart
                ? std::initializer_list<bool>{ false, true }
                : std::initializer_list<bool>{ item->m_isReverse };

            for (const bool reverse : reverseOptions)
            {
                ProcessPathOption option;
                option.reverse = reverse;
                option.startPoint = reverse ? pointAtAngle(arc->endangle) : pointAtAngle(arc->staangle);
                option.endPoint = reverse ? pointAtAngle(arc->staangle) : pointAtAngle(arc->endangle);
                option.startTangent = reverse ? arcTangentAt(arc->endangle, true) : arcTangentAt(arc->staangle, false);
                option.endTangent = reverse ? arcTangentAt(arc->staangle, true) : arcTangentAt(arc->endangle, false);
                options.push_back(option);
            }

            break;
        }
        case DRW::ETYPE::CIRCLE:
        {
            const DRW_Circle* circle = static_cast<const DRW_Circle*>(item->m_nativeEntity);
            const QVector3D center(circle->basePoint.x, circle->basePoint.y, circle->basePoint.z);
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
                option.startPoint = QVector3D
                (
                    static_cast<float>(center.x() + std::cos(startParameter) * circle->radious),
                    static_cast<float>(center.y() + std::sin(startParameter) * circle->radious),
                    center.z()
                );
                option.endPoint = option.startPoint;
                option.startTangent = arcTangentAt(startParameter, reverse);
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

    connect(ui->action_File_Export_G, &QAction::triggered, this, [this]() { exportGCode(); });
    connect(ui->action_Edit_ReversePeocess, &QAction::triggered, this, [this]() { toggleSelectedEntityReverse(); });
    connect(ui->action_Sort_2D_Assign, &QAction::triggered, this, [this]() { sortEntitiesByCurrentDirection(); });
    connect(ui->action_Sort_2D_Smart, &QAction::triggered, this, [this]() { smartSortEntities(); });
    connect(ui->action_Sort_3D_Assign, &QAction::triggered, this, [this]() { sortEntitiesByCurrentDirection3D(); });
    connect(ui->action_Sort_3D_Smart, &QAction::triggered, this, [this]() { smartSortEntities3D(); });

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

bool Gcode_postprocessing_system::exportGCode()
{
    if (m_document.m_entities.empty())
    {
        QMessageBox::warning(this, QStringLiteral("导出失败"), QStringLiteral("当前文档为空，无法导出 G 代码。"));
        return false;
    }

    GGenerator generator;
    generator.setDocument(&m_document);
    generator.setProfile(&m_activeProfile);

    QString errorMessage;

    if (!generator.generate(this, &errorMessage))
    {
        if (!errorMessage.isEmpty())
        {
            QMessageBox::warning(this, QStringLiteral("导出失败"), errorMessage);
        }

        return false;
    }

    ui->openGLWidget->appendCommandMessage(QStringLiteral("G 代码导出完成。"));
    ui->openGLWidget->refreshCommandPrompt();
    statusBar()->showMessage(QStringLiteral("G 代码导出完成"), 5000);
    return true;
}

bool Gcode_postprocessing_system::toggleSelectedEntityReverse()
{
    CadItem* selectedItem = ui->openGLWidget->selectedEntity();

    if (selectedItem == nullptr)
    {
        QMessageBox::warning(this, QStringLiteral("反向加工"), QStringLiteral("请先选择一个图元。"));
        return false;
    }

    if (!m_editer.toggleEntityReverse(selectedItem))
    {
        QMessageBox::warning(this, QStringLiteral("反向加工"), QStringLiteral("当前图元的反向加工状态切换失败。"));
        return false;
    }

    const QString reverseStateText = selectedItem->m_isReverse
        ? QStringLiteral("反向")
        : QStringLiteral("正向");

    ui->openGLWidget->appendCommandMessage(QStringLiteral("当前选中图元加工方向已切换为%1。").arg(reverseStateText));
    ui->openGLWidget->refreshCommandPrompt();
    statusBar()->showMessage(QStringLiteral("加工方向已切换为%1").arg(reverseStateText), 5000);
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
    QMessageBox::information
    (
        this,
        QStringLiteral("3D排序"),
        QStringLiteral("3D 排序入口已分类预留，当前版本仅完善了纯 2D 排序逻辑，3D 排序（保留方向）暂未实现。")
    );
    return false;
}

bool Gcode_postprocessing_system::smartSortEntities3D()
{
    QMessageBox::information
    (
        this,
        QStringLiteral("3D智能排序"),
        QStringLiteral("3D 排序入口已分类预留，当前版本仅完善了纯 2D 智能排序逻辑，3D 智能排序暂未实现。")
    );
    return false;
}

void Gcode_postprocessing_system::initializeThemeMenu()
{
    ui->menuSet->setTitle(QStringLiteral("用户设置"));

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

    connect
    (
        m_toolPanelWidget,
        &CadToolPanelWidget::layerChangeRequested,
        this,
        [this](const QString& layerName)
        {
            const QString normalizedLayerName = layerName.trimmed().isEmpty() ? QStringLiteral("0") : layerName.trimmed();
            CadItem* selectedItem = ui->openGLWidget->selectedEntity();

            if (selectedItem != nullptr)
            {
                if (m_editer.changeEntityLayer(selectedItem, normalizedLayerName))
                {
                    statusBar()->showMessage(QStringLiteral("图层已更新为 %1").arg(normalizedLayerName), 3000);
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
            CadItem* selectedItem = ui->openGLWidget->selectedEntity();

            if (selectedItem != nullptr)
            {
                const QColor targetColor = colorIndex == kColorByLayer
                    ? m_document.layerColor(entityLayerName(selectedItem), entityDisplayColor(m_document, selectedItem))
                    : (colorIndex < 0 ? entityDisplayColor(m_document, selectedItem) : colorFromAci(colorIndex));

                if (m_editer.changeEntityColor(selectedItem, targetColor, colorIndex))
                {
                    statusBar()->showMessage(QStringLiteral("图元颜色已更新"), 3000);
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
        m_toolPanelWidget->setMoveEnabled(true);
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

    m_toolPanelWidget->setMoveEnabled(false);
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
