#include "pch.h"

#include "Gcode_postprocessing_system.h"

#include "CadItem.h"
#include "CadOcsGeometry.h"
#include "CadProcessVisualUtils.h"
#include "RotaryCutBoundaryAnalyzer.h"

#include <QMessageBox>
#include <QSet>
#include <QStatusBar>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <vector>

namespace
{
    constexpr double kSortEpsilon = 1.0e-9;
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kTwoPi = 6.28318530717958647692;
    constexpr int kClosedCircleSampleCount = 16;
    constexpr int kClosedEllipseSampleCount = 16;
    constexpr double kNextDistanceWeight = 0.15;
    constexpr double kDirectionPenaltyWeight = 0.35;
    constexpr double kBacktrackPenaltyWeight = 1.2;
    constexpr double kRotaryAngleDistanceWeight = 0.08;
    constexpr double kRotaryNextDistanceWeight = 0.12;
    constexpr double kRotaryBacktrackPenaltyWeight = 1.35;
    constexpr double kRotaryDirectionPenaltyWeight = 0.2;
    constexpr double kSortConnectionEpsilon = 1.0e-6;
    constexpr double kEndCutConnectionTolerance = 1.0;
    constexpr double kNearGapPriorityDistance2D = 1.0;
    constexpr double kNearGapPriorityDistance3D = 1.0;
    constexpr double kPreferredStartGapDistance2D = 1.0;
    constexpr double kPreferredStartGapDistance3D = 1.0;
    constexpr double kRotaryPlaneMatchToleranceDegrees = 3.0;
    constexpr double kSurfaceSweepBoundaryTolerance = 1.0e-4;
    constexpr double kSquareTubeSectionToleranceRatio = 0.015;
    constexpr double kSquareTubeEndCutCoverageThreshold = 0.72;
    constexpr double kSortDedupCoordinateTolerance = 1.0e-4;
    const QVector3D kSortOrigin(0.0f, 0.0f, 0.0f);
    // The machine starts above the workpiece at this pose; use it to select the
    // left end-cut seam that requires the least initial A-axis rotation.
    const QVector3D kRotaryInitialSortOrigin(0.0f, 0.0f, 500.0f);

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
        bool startsOnInitialTopPlane = false;
        bool startsOnSweepBoundary = false;
        bool matchesCurrentRotaryPlane = false;
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

    enum class RotarySurfaceGroup
    {
        Top,
        Right,
        Bottom,
        Left,
        Unknown
    };

    struct RotaryPathBounds
    {
        bool valid = false;
        double minX = 0.0;
        double maxX = 0.0;
        double minY = 0.0;
        double maxY = 0.0;
        double minZ = 0.0;
        double maxZ = 0.0;
        double sumX = 0.0;
        double sumY = 0.0;
        double sumZ = 0.0;
        size_t pointCount = 0;
    };

    struct RotaryItemAnalysis
    {
        size_t itemIndex = 0;
        RotaryPathBounds bounds;
        RotarySurfaceGroup surface = RotarySurfaceGroup::Unknown;
    };

    struct RotaryFeatureComponent
    {
        std::vector<size_t> itemIndices;
        RotaryPathBounds bounds;
        bool isEndCut = false;
    };

    struct RotaryCutCluster
    {
        std::vector<size_t> itemIndices;
        double centerX = 0.0;
    };

    struct ManualRotaryBreakBoundary
    {
        std::vector<size_t> itemIndices;
        int clusterIndex = -1;
        RotaryCutBoundaryAnalysis analysis;
    };

    struct RotaryLazySegment
    {
        int leftClusterIndex = -1;
        int rightClusterIndex = -1;
        double centerX = 0.0;
        std::vector<size_t> continuousItems;
        std::vector<size_t> topItems;
        std::vector<size_t> rightItems;
        std::vector<size_t> bottomItems;
        std::vector<size_t> leftItems;
        std::vector<size_t> unknownItems;
    };

    struct SortableDedupResult
    {
        size_t removedCount = 0;
        QVector<CadItem*> duplicateItems;
    };

    bool hasManualRotaryEndCutAssignments(const std::vector<CadItem*>& items)
    {
        return std::any_of
        (
            items.begin(),
            items.end(),
            [](const CadItem* item)
            {
                return item != nullptr
                    && item->m_rotaryEndCutRole == RotaryEndCutRole::Break;
            }
        );
    }

    std::vector<ProcessPathOption> buildPathOptionsForItem(const CadItem* item, SortStrategy strategy);
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

        const QVector3D center = CadOcsGeometry::center(arc);
        QVector3D normal = CadOcsGeometry::normal(arc->extPoint);
        QVector3D axisX;
        QVector3D axisY;
        CadOcsGeometry::basis(arc->extPoint, axisX, axisY, normal);

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

        QVector3D normal = CadOcsGeometry::normal(arc->extPoint);
        QVector3D axisX;
        QVector3D axisY;
        CadOcsGeometry::basis(arc->extPoint, axisX, axisY, normal);

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

        return CadOcsGeometry::pointAt(circle, parameter);
    }

    QVector3D circleTangentAt(const DRW_Circle* circle, double parameter, bool reverseDirection)
    {
        if (circle == nullptr || circle->radious <= 0.0)
        {
            return QVector3D();
        }

        return CadOcsGeometry::tangentAt(circle, parameter, reverseDirection);
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

            if (strategy == SortStrategy::Smart)
            {
                for (int sampleIndex = 0; sampleIndex < kClosedCircleSampleCount; ++sampleIndex)
                {
                    const double startParameter = kTwoPi * static_cast<double>(sampleIndex)
                        / static_cast<double>(kClosedCircleSampleCount);

                    for (const bool reverse : reverseOptions)
                    {
                        ProcessPathOption option;
                        option.reverse = reverse;
                        option.hasCustomStart = true;
                        option.processStartParameter = startParameter;
                        option.startPoint = circlePointAt(circle, startParameter);
                        option.endPoint = option.startPoint;
                        option.startTangent = circleTangentAt(circle, startParameter, reverse);
                        option.endTangent = option.startTangent;
                        options.push_back(option);
                    }
                }
            }
            else
            {
                const double startParameter = item->m_hasCustomProcessStart
                    ? item->m_processStartParameter
                    : kPi * 0.5;

                for (const bool reverse : reverseOptions)
                {
                    ProcessPathOption option;
                    option.reverse = reverse;
                    option.hasCustomStart = item->m_hasCustomProcessStart;
                    option.processStartParameter = startParameter;
                    option.startPoint = circlePointAt(circle, startParameter);
                    option.endPoint = option.startPoint;
                    option.startTangent = circleTangentAt(circle, startParameter, reverse);
                    option.endTangent = option.startTangent;
                    options.push_back(option);
                }
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

    std::vector<int> buildItemConnectivityComponents
    (
        const std::vector<CadItem*>& sortableItems,
        double connectionDistance = kSortConnectionEpsilon
    )
    {
        const std::vector<EndpointNode> endpoints = collectOpenEndpointNodes(sortableItems);
        std::vector<std::vector<size_t>> itemEndpointIndices(sortableItems.size());
        const double exactConnectionDistanceSquared = connectionDistance * connectionDistance;

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

    QString sortDedupPointToken(const RawPathPoint3D& point)
    {
        const qint64 x = static_cast<qint64>(std::llround(point.x / kSortDedupCoordinateTolerance));
        const qint64 y = static_cast<qint64>(std::llround(point.y / kSortDedupCoordinateTolerance));
        const qint64 z = static_cast<qint64>(std::llround(point.z / kSortDedupCoordinateTolerance));
        return QStringLiteral("%1,%2,%3").arg(x).arg(y).arg(z);
    }

    QString sortDedupJoinTokens(const std::vector<QString>& tokens, size_t startIndex, bool reverse)
    {
        QString key;
        key.reserve(static_cast<int>(tokens.size() * 24));

        for (size_t offset = 0; offset < tokens.size(); ++offset)
        {
            const size_t index = reverse
                ? (startIndex + tokens.size() - offset) % tokens.size()
                : (startIndex + offset) % tokens.size();

            if (!key.isEmpty())
            {
                key.append(QLatin1Char(';'));
            }

            key.append(tokens[index]);
        }

        return key;
    }

    QString sortDedupCanonicalPathKey(CadItem* item)
    {
        if (item == nullptr)
        {
            return QString();
        }

        item->rebuildRawPathPoints3D();

        std::vector<QString> tokens;
        tokens.reserve(item->rawPathPoints3D().size());

        for (const RawPathPoint3D& point : item->rawPathPoints3D())
        {
            const QString token = sortDedupPointToken(point);

            if (!tokens.empty() && tokens.back() == token)
            {
                continue;
            }

            tokens.push_back(token);
        }

        if (tokens.empty())
        {
            return QString();
        }

        bool closed = false;

        if (tokens.size() > 1 && tokens.front() == tokens.back())
        {
            tokens.pop_back();
            closed = true;
        }

        if (tokens.empty())
        {
            return QString();
        }

        QString bestKey;

        if (closed)
        {
            for (size_t index = 0; index < tokens.size(); ++index)
            {
                const QString forwardKey = sortDedupJoinTokens(tokens, index, false);
                const QString reverseKey = sortDedupJoinTokens(tokens, index, true);

                if (bestKey.isEmpty() || forwardKey < bestKey)
                {
                    bestKey = forwardKey;
                }

                if (reverseKey < bestKey)
                {
                    bestKey = reverseKey;
                }
            }
        }
        else
        {
            const QString forwardKey = sortDedupJoinTokens(tokens, 0, false);
            const QString reverseKey = sortDedupJoinTokens(tokens, tokens.size() - 1, true);
            bestKey = forwardKey < reverseKey ? forwardKey : reverseKey;
        }

        return QStringLiteral("%1|%2|%3")
            .arg(static_cast<int>(item->m_type))
            .arg(static_cast<qulonglong>(item->buildColor().rgba()))
            .arg(bestKey);
    }

    SortableDedupResult deduplicateSortableItems(std::vector<CadItem*>& sortableItems)
    {
        SortableDedupResult result;
        std::vector<CadItem*> uniqueItems;
        std::vector<QString> uniqueKeys;
        uniqueItems.reserve(sortableItems.size());
        uniqueKeys.reserve(sortableItems.size());

        for (CadItem* item : sortableItems)
        {
            const QString key = sortDedupCanonicalPathKey(item);

            if (key.isEmpty())
            {
                uniqueItems.push_back(item);
                continue;
            }

            const bool duplicate = std::find(uniqueKeys.begin(), uniqueKeys.end(), key) != uniqueKeys.end();

            if (duplicate)
            {
                result.duplicateItems.append(item);
                ++result.removedCount;
                continue;
            }

            uniqueKeys.push_back(key);
            uniqueItems.push_back(item);
        }

        sortableItems = std::move(uniqueItems);
        return result;
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
        const GProfileRotaryAxisConfig& config,
        bool preferSweepBoundary = false,
        double sweepBoundaryX = 0.0
    )
    {
        SortCandidate bestCandidate;
        const std::vector<bool> visitedComponents = buildVisitedComponentMask(visited, gapStartContext.componentIds);
        const bool mustStayInCurrentComponent = hasCurrentEndPoint
            && hasRemainingUnvisitedInComponent(visited, gapStartContext.componentIds, currentComponentId);
        const QVector3D referencePoint = hasCurrentEndPoint ? currentEndPoint : kRotaryInitialSortOrigin;
        RotarySortPoint referenceRotaryPoint;
        const bool hasReferenceRotaryPoint = tryBuildRotarySortPoint(referencePoint, config, referenceRotaryPoint);
        const bool preferMatchingRotaryPlane = strategy == SortStrategy::Smart
            && hasCurrentEndPoint
            && !mustStayInCurrentComponent
            && hasReferenceRotaryPoint;
        bool hasInitialTopPlane = false;
        double initialTopZ = 0.0;
        double initialTopTolerance = 0.0;

        if (!hasCurrentEndPoint)
        {
            double minimumZ = 0.0;

            for (const CadItem* item : sortableItems)
            {
                if (item == nullptr)
                {
                    continue;
                }

                for (const QVector3D& point : item->m_geometry.vertices)
                {
                    if (!hasInitialTopPlane)
                    {
                        minimumZ = point.z();
                        initialTopZ = point.z();
                        hasInitialTopPlane = true;
                        continue;
                    }

                    minimumZ = std::min(minimumZ, static_cast<double>(point.z()));
                    initialTopZ = std::max(initialTopZ, static_cast<double>(point.z()));
                }
            }

            if (hasInitialTopPlane)
            {
                initialTopTolerance = std::max(0.25, (initialTopZ - minimumZ) * kSquareTubeSectionToleranceRatio);
            }
        }
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
                const bool startsOnInitialTopPlane = hasInitialTopPlane
                    && std::abs(static_cast<double>(option.startPoint.z()) - initialTopZ) <= initialTopTolerance;
                const bool startsOnSweepBoundary = preferSweepBoundary
                    && std::abs(static_cast<double>(option.startPoint.x()) - sweepBoundaryX) <= kSurfaceSweepBoundaryTolerance;
                RotarySortPoint candidateRotaryPoint;
                const bool matchesCurrentRotaryPlane = preferMatchingRotaryPlane
                    && tryBuildRotarySortPoint(option.startPoint, config, candidateRotaryPoint)
                    && std::abs
                    (
                        unwrapAngleDegrees(referenceRotaryPoint.angleDegrees, candidateRotaryPoint.angleDegrees)
                            - referenceRotaryPoint.angleDegrees
                    ) <= kRotaryPlaneMatchToleranceDegrees;
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

                const bool rotaryPlanePreferenceDecides = preferMatchingRotaryPlane
                    && matchesCurrentRotaryPlane != bestCandidate.matchesCurrentRotaryPlane;
                const bool initialTopPlanePreferenceDecides = hasInitialTopPlane
                    && startsOnInitialTopPlane != bestCandidate.startsOnInitialTopPlane;
                const bool sweepBoundaryPreferenceDecides = preferSweepBoundary
                    && startsOnSweepBoundary != bestCandidate.startsOnSweepBoundary;
                const bool shouldReplace = bestCandidate.index < 0
                    || (initialTopPlanePreferenceDecides && startsOnInitialTopPlane)
                    || (!initialTopPlanePreferenceDecides
                        && sweepBoundaryPreferenceDecides
                        && startsOnSweepBoundary)
                    || (!initialTopPlanePreferenceDecides
                        && !sweepBoundaryPreferenceDecides
                        && rotaryPlanePreferenceDecides
                        && matchesCurrentRotaryPlane)
                    || (!initialTopPlanePreferenceDecides
                        && !sweepBoundaryPreferenceDecides
                        && !rotaryPlanePreferenceDecides
                        && ((preferPreferredGapStart && preferredGapStart && !bestPreferredGapStart)
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
                                                        && isPointLexicographicallyLess(option.startPoint, bestCandidate.startPoint))))))))));

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
                bestCandidate.startsOnInitialTopPlane = startsOnInitialTopPlane;
                bestCandidate.startsOnSweepBoundary = startsOnSweepBoundary;
                bestCandidate.matchesCurrentRotaryPlane = matchesCurrentRotaryPlane;
                bestCandidate.startPoint = option.startPoint;
                bestCandidate.endPoint = option.endPoint;
            }
        }

        return bestCandidate;
    }

    void expandRotaryBounds(RotaryPathBounds& bounds, const RawPathPoint3D& point)
    {
        if (!bounds.valid)
        {
            bounds.valid = true;
            bounds.minX = bounds.maxX = point.x;
            bounds.minY = bounds.maxY = point.y;
            bounds.minZ = bounds.maxZ = point.z;
        }
        else
        {
            bounds.minX = std::min(bounds.minX, point.x);
            bounds.maxX = std::max(bounds.maxX, point.x);
            bounds.minY = std::min(bounds.minY, point.y);
            bounds.maxY = std::max(bounds.maxY, point.y);
            bounds.minZ = std::min(bounds.minZ, point.z);
            bounds.maxZ = std::max(bounds.maxZ, point.z);
        }

        bounds.sumX += point.x;
        bounds.sumY += point.y;
        bounds.sumZ += point.z;
        ++bounds.pointCount;
    }

    void mergeRotaryBounds(RotaryPathBounds& target, const RotaryPathBounds& source)
    {
        if (!source.valid)
        {
            return;
        }

        if (!target.valid)
        {
            target = source;
            return;
        }

        target.minX = std::min(target.minX, source.minX);
        target.maxX = std::max(target.maxX, source.maxX);
        target.minY = std::min(target.minY, source.minY);
        target.maxY = std::max(target.maxY, source.maxY);
        target.minZ = std::min(target.minZ, source.minZ);
        target.maxZ = std::max(target.maxZ, source.maxZ);
        target.sumX += source.sumX;
        target.sumY += source.sumY;
        target.sumZ += source.sumZ;
        target.pointCount += source.pointCount;
    }

    double rotaryBoundsCenterX(const RotaryPathBounds& bounds)
    {
        return bounds.pointCount > 0
            ? bounds.sumX / static_cast<double>(bounds.pointCount)
            : (bounds.minX + bounds.maxX) * 0.5;
    }

    double rotaryBoundsCenterY(const RotaryPathBounds& bounds)
    {
        return bounds.pointCount > 0
            ? bounds.sumY / static_cast<double>(bounds.pointCount)
            : (bounds.minY + bounds.maxY) * 0.5;
    }

    double rotaryBoundsCenterZ(const RotaryPathBounds& bounds)
    {
        return bounds.pointCount > 0
            ? bounds.sumZ / static_cast<double>(bounds.pointCount)
            : (bounds.minZ + bounds.maxZ) * 0.5;
    }

    RotarySurfaceGroup classifySquareTubeSurface
    (
        const CadItem* item,
        const RotaryPathBounds& sectionBounds,
        double sectionTolerance
    )
    {
        if (item == nullptr || !sectionBounds.valid)
        {
            return RotarySurfaceGroup::Unknown;
        }

        bool onlyTop = true;
        bool onlyRight = true;
        bool onlyBottom = true;
        bool onlyLeft = true;
        bool hasPoint = false;

        for (const RawPathPoint3D& point : item->rawPathPoints3D())
        {
            hasPoint = true;
            onlyTop = onlyTop && std::abs(point.z - sectionBounds.maxZ) <= sectionTolerance;
            onlyRight = onlyRight && std::abs(point.y - sectionBounds.maxY) <= sectionTolerance;
            onlyBottom = onlyBottom && std::abs(point.z - sectionBounds.minZ) <= sectionTolerance;
            onlyLeft = onlyLeft && std::abs(point.y - sectionBounds.minY) <= sectionTolerance;
        }

        if (!hasPoint)
        {
            return RotarySurfaceGroup::Unknown;
        }

        if (onlyTop)
        {
            return RotarySurfaceGroup::Top;
        }

        if (onlyRight)
        {
            return RotarySurfaceGroup::Right;
        }

        if (onlyBottom)
        {
            return RotarySurfaceGroup::Bottom;
        }

        if (onlyLeft)
        {
            return RotarySurfaceGroup::Left;
        }

        return RotarySurfaceGroup::Unknown;
    }

    bool componentLiesOnEndCutPlane
    (
        const RotaryFeatureComponent& component,
        const std::vector<CadItem*>& sortableItems,
        double sectionTolerance
    )
    {
        // 方管端部切面可表示为 x = a*y + b*z + c；该形式同时覆盖垂直与倾斜切面。
        double normal[3][4] = {};
        size_t pointCount = 0;

        size_t coplanarPointCount = 0;

        for (const size_t itemIndex : component.itemIndices)
        {
            if (itemIndex >= sortableItems.size() || sortableItems[itemIndex] == nullptr)
            {
                continue;
            }

            for (const RawPathPoint3D& point : sortableItems[itemIndex]->rawPathPoints3D())
            {
                const double values[3] = { point.y, point.z, 1.0 };

                for (int row = 0; row < 3; ++row)
                {
                    for (int column = 0; column < 3; ++column)
                    {
                        normal[row][column] += values[row] * values[column];
                    }

                    normal[row][3] += values[row] * point.x;
                }

                ++pointCount;
            }
        }

        if (pointCount < 3)
        {
            return false;
        }

        for (int pivotColumn = 0; pivotColumn < 3; ++pivotColumn)
        {
            int pivotRow = pivotColumn;

            for (int row = pivotColumn + 1; row < 3; ++row)
            {
                if (std::abs(normal[row][pivotColumn]) > std::abs(normal[pivotRow][pivotColumn]))
                {
                    pivotRow = row;
                }
            }

            if (std::abs(normal[pivotRow][pivotColumn]) <= kSortEpsilon)
            {
                return false;
            }

            if (pivotRow != pivotColumn)
            {
                for (int column = pivotColumn; column < 4; ++column)
                {
                    std::swap(normal[pivotRow][column], normal[pivotColumn][column]);
                }
            }

            const double pivot = normal[pivotColumn][pivotColumn];

            for (int column = pivotColumn; column < 4; ++column)
            {
                normal[pivotColumn][column] /= pivot;
            }

            for (int row = 0; row < 3; ++row)
            {
                if (row == pivotColumn)
                {
                    continue;
                }

                const double factor = normal[row][pivotColumn];

                for (int column = pivotColumn; column < 4; ++column)
                {
                    normal[row][column] -= factor * normal[pivotColumn][column];
                }
            }
        }

        const double planeTolerance = std::max(sectionTolerance * 2.0, 0.5);

        for (const size_t itemIndex : component.itemIndices)
        {
            if (itemIndex >= sortableItems.size() || sortableItems[itemIndex] == nullptr)
            {
                continue;
            }

            for (const RawPathPoint3D& point : sortableItems[itemIndex]->rawPathPoints3D())
            {
                const double expectedX = normal[0][3] * point.y + normal[1][3] * point.z + normal[2][3];

                if (std::abs(point.x - expectedX) > planeTolerance)
                {
                    continue;
                }

                ++coplanarPointCount;
            }
        }

        // 允许少量引线、断点等辅助路径偏离切面，但主体必须仍是同一切面。
        return coplanarPointCount * 4 >= pointCount * 3;
    }

    bool componentMatchesSquareTubeEndCut
    (
        const RotaryFeatureComponent& component,
        const std::vector<CadItem*>& sortableItems,
        const RotaryPathBounds& sectionBounds,
        double sectionTolerance
    )
    {
        if (!component.bounds.valid || !sectionBounds.valid)
        {
            return false;
        }

        const double spanY = sectionBounds.maxY - sectionBounds.minY;
        const double spanZ = sectionBounds.maxZ - sectionBounds.minZ;

        if (spanY <= kSortEpsilon || spanZ <= kSortEpsilon)
        {
            return false;
        }

        const double coverageY = (component.bounds.maxY - component.bounds.minY) / spanY;
        const double coverageZ = (component.bounds.maxZ - component.bounds.minZ) / spanZ;

        if (coverageY < kSquareTubeEndCutCoverageThreshold || coverageZ < kSquareTubeEndCutCoverageThreshold)
        {
            return false;
        }

        if (!componentLiesOnEndCutPlane(component, sortableItems, sectionTolerance))
        {
            return false;
        }

        bool touchesTop = false;
        bool touchesRight = false;
        bool touchesBottom = false;
        bool touchesLeft = false;

        for (const size_t itemIndex : component.itemIndices)
        {
            if (itemIndex >= sortableItems.size() || sortableItems[itemIndex] == nullptr)
            {
                continue;
            }

            for (const RawPathPoint3D& point : sortableItems[itemIndex]->rawPathPoints3D())
            {
                touchesTop = touchesTop || std::abs(point.z - sectionBounds.maxZ) <= sectionTolerance;
                touchesRight = touchesRight || std::abs(point.y - sectionBounds.maxY) <= sectionTolerance;
                touchesBottom = touchesBottom || std::abs(point.z - sectionBounds.minZ) <= sectionTolerance;
                touchesLeft = touchesLeft || std::abs(point.y - sectionBounds.minY) <= sectionTolerance;
            }
        }

        return touchesTop && touchesRight && touchesBottom && touchesLeft;
    }

    std::vector<size_t>& segmentItemsForSurface(RotaryLazySegment& segment, RotarySurfaceGroup surface)
    {
        switch (surface)
        {
        case RotarySurfaceGroup::Top:
            return segment.topItems;
        case RotarySurfaceGroup::Right:
            return segment.rightItems;
        case RotarySurfaceGroup::Bottom:
            return segment.bottomItems;
        case RotarySurfaceGroup::Left:
            return segment.leftItems;
        case RotarySurfaceGroup::Unknown:
        default:
            return segment.unknownItems;
        }
    }

    bool appendSorted3DGroup
    (
        const std::vector<CadItem*>& sortableItems,
        const std::vector<size_t>& groupIndices,
        const GProfileRotaryAxisConfig& rotaryAxisConfig,
        std::vector<CadEditer::ProcessStateUpdate>& processUpdates,
        std::vector<ProcessConnectionSegment>& processedSegments,
        bool& hasCurrentEndPoint,
        QVector3D& currentEndPoint,
        bool preferSweepBoundary = false,
        double sweepBoundaryX = 0.0
    )
    {
        if (groupIndices.empty())
        {
            return true;
        }

        std::vector<CadItem*> localItems;
        std::vector<size_t> localToGlobal;
        localItems.reserve(groupIndices.size());
        localToGlobal.reserve(groupIndices.size());

        for (const size_t index : groupIndices)
        {
            if (index >= sortableItems.size() || sortableItems[index] == nullptr)
            {
                continue;
            }

            localItems.push_back(sortableItems[index]);
            localToGlobal.push_back(index);
        }

        if (localItems.empty())
        {
            return true;
        }

        const QVector3D sweepDirection = computeRotarySweepDirection(localItems, rotaryAxisConfig);
        const GapStartSelectionContext gapStartContext = buildGapStartSelectionContext(localItems, kPreferredStartGapDistance3D);
        std::vector<bool> visited(localItems.size(), false);
        int currentComponentId = -1;

        for (size_t order = 0; order < localItems.size(); ++order)
        {
            SortCandidate bestCandidate = chooseNext3DSortCandidate
            (
                localItems,
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
                rotaryAxisConfig,
                preferSweepBoundary && order == 0,
                sweepBoundaryX
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
                    localItems,
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
                    rotaryAxisConfig,
                    preferSweepBoundary && order == 0,
                    sweepBoundaryX
                );
            }

            if (bestCandidate.index < 0 || static_cast<size_t>(bestCandidate.index) >= localToGlobal.size())
            {
                return false;
            }

            const size_t localIndex = static_cast<size_t>(bestCandidate.index);
            const size_t globalIndex = localToGlobal[localIndex];
            visited[localIndex] = true;
            processUpdates.push_back
            ({
                sortableItems[globalIndex],
                static_cast<int>(processUpdates.size()),
                bestCandidate.reverse,
                bestCandidate.hasCustomStart,
                bestCandidate.processStartParameter
            });
            hasCurrentEndPoint = true;
            currentComponentId =
                localIndex < gapStartContext.componentIds.size()
                ? gapStartContext.componentIds[localIndex]
                : -1;
            currentEndPoint = bestCandidate.endPoint;
            processedSegments.push_back({ bestCandidate.startPoint, bestCandidate.endPoint });
        }

        return true;
    }

    bool tryBuildSquareTubeLazyRotaryProcessUpdates
    (
        const std::vector<CadItem*>& sortableItems,
        const GProfileRotaryAxisConfig& rotaryAxisConfig,
        std::vector<CadEditer::ProcessStateUpdate>& processUpdates,
        const QVector<QVector2D>& configuredSectionHull
    )
    {
        if (sortableItems.size() < 3)
        {
            return false;
        }

        std::vector<RotaryItemAnalysis> itemAnalyses(sortableItems.size());
        RotaryPathBounds sectionBounds;

        for (size_t index = 0; index < sortableItems.size(); ++index)
        {
            CadItem* item = sortableItems[index];

            if (item == nullptr)
            {
                return false;
            }

            item->rebuildRawPathPoints3D();

            RotaryItemAnalysis& analysis = itemAnalyses[index];
            analysis.itemIndex = index;

            for (const RawPathPoint3D& point : item->rawPathPoints3D())
            {
                expandRotaryBounds(analysis.bounds, point);
                expandRotaryBounds(sectionBounds, point);
            }

            if (!analysis.bounds.valid)
            {
                return false;
            }
        }

        const double spanX = sectionBounds.maxX - sectionBounds.minX;
        const double spanY = sectionBounds.maxY - sectionBounds.minY;
        const double spanZ = sectionBounds.maxZ - sectionBounds.minZ;
        const double sectionSize = std::max(spanY, spanZ);

        if (spanX <= kSortEpsilon || sectionSize <= kSortEpsilon)
        {
            return false;
        }

        const double sectionTolerance = std::max(0.25, sectionSize * kSquareTubeSectionToleranceRatio);
        const std::vector<int> componentIds = buildItemConnectivityComponents(sortableItems);

        if (componentIds.size() != sortableItems.size())
        {
            return false;
        }

        const int componentCount = componentIds.empty()
            ? 0
            : (*std::max_element(componentIds.begin(), componentIds.end()) + 1);

        if (componentCount <= 0)
        {
            return false;
        }

        std::vector<RotaryFeatureComponent> components(static_cast<size_t>(componentCount));

        for (size_t itemIndex = 0; itemIndex < sortableItems.size(); ++itemIndex)
        {
            const int componentId = componentIds[itemIndex];

            if (componentId < 0 || componentId >= componentCount)
            {
                return false;
            }

            RotaryFeatureComponent& component = components[static_cast<size_t>(componentId)];
            component.itemIndices.push_back(itemIndex);
            mergeRotaryBounds(component.bounds, itemAnalyses[itemIndex].bounds);
        }

        std::vector<bool> componentIsEndCut(components.size(), false);

        std::map<int, ManualRotaryBreakBoundary> manualBoundariesById;

        for (size_t itemIndex = 0; itemIndex < sortableItems.size(); ++itemIndex)
        {
            const CadItem* item = sortableItems[itemIndex];

            if (item == nullptr
                || item->m_rotaryEndCutRole == RotaryEndCutRole::None
                || item->m_rotaryEndCutRole == RotaryEndCutRole::Waste)
            {
                continue;
            }

            if (item->m_rotaryEndCutPairId < 0)
            {
                return false;
            }

            ManualRotaryBreakBoundary& boundary = manualBoundariesById[item->m_rotaryEndCutPairId];
            boundary.itemIndices.push_back(itemIndex);
        }

        std::vector<ManualRotaryBreakBoundary> manualBreakBoundaries;
        manualBreakBoundaries.reserve(manualBoundariesById.size());
        QVector<CadItem*> analysisSceneItems;
        analysisSceneItems.reserve(static_cast<qsizetype>(sortableItems.size()));

        for (CadItem* item : sortableItems)
        {
            analysisSceneItems.push_back(item);
        }

        for (const auto& [boundaryId, boundary] : manualBoundariesById)
        {
            Q_UNUSED(boundaryId);

            if (boundary.itemIndices.empty())
            {
                return false;
            }

            ManualRotaryBreakBoundary analyzedBoundary = boundary;
            QVector<CadItem*> candidateItems;
            candidateItems.reserve(static_cast<qsizetype>(boundary.itemIndices.size()));

            for (const size_t itemIndex : boundary.itemIndices)
            {
                if (itemIndex >= sortableItems.size() || sortableItems[itemIndex] == nullptr)
                {
                    return false;
                }

                candidateItems.push_back(sortableItems[itemIndex]);
            }

            analyzedBoundary.analysis = RotaryCutBoundaryAnalyzer::analyze
            (
                candidateItems,
                analysisSceneItems,
                kEndCutConnectionTolerance,
                configuredSectionHull
            );

            if (!analyzedBoundary.analysis.valid)
            {
                return false;
            }

            manualBreakBoundaries.push_back(std::move(analyzedBoundary));
        }

        const bool useManualBreakBoundaries = !manualBreakBoundaries.empty();

        if (useManualBreakBoundaries)
        {
            // 人工指定以完整连通切面为单位，每个切面都是独立的加工中断边界。
            for (size_t componentIndex = 0; componentIndex < components.size(); ++componentIndex)
            {
                RotaryEndCutRole componentRole = RotaryEndCutRole::None;
                int componentPairId = -1;

                for (const size_t itemIndex : components[componentIndex].itemIndices)
                {
                    if (itemIndex >= sortableItems.size() || sortableItems[itemIndex] == nullptr)
                    {
                        return false;
                    }

                    const CadItem* item = sortableItems[itemIndex];

                    if (item->m_rotaryEndCutRole == RotaryEndCutRole::None)
                    {
                        if (componentRole != RotaryEndCutRole::None)
                        {
                            return false;
                        }

                        continue;
                    }

                    if (componentRole == RotaryEndCutRole::None)
                    {
                        componentRole = item->m_rotaryEndCutRole;
                        componentPairId = item->m_rotaryEndCutPairId;
                        continue;
                    }

                    if (componentRole != item->m_rotaryEndCutRole
                        || componentPairId != item->m_rotaryEndCutPairId)
                    {
                        return false;
                    }
                }

                if (componentRole != RotaryEndCutRole::None)
                {
                    componentIsEndCut[componentIndex] = true;
                    components[componentIndex].isEndCut = true;
                }
            }
        }
        else
        {

        for (size_t componentIndex = 0; componentIndex < components.size(); ++componentIndex)
        {
            RotaryFeatureComponent& component = components[componentIndex];
            component.isEndCut = componentMatchesSquareTubeEndCut(component, sortableItems, sectionBounds, sectionTolerance);
            componentIsEndCut[componentIndex] = component.isEndCut;
        }

        std::vector<size_t> componentOrder(components.size());

        for (size_t componentIndex = 0; componentIndex < components.size(); ++componentIndex)
        {
            componentOrder[componentIndex] = componentIndex;
        }

        std::sort
        (
            componentOrder.begin(),
            componentOrder.end(),
            [&components](size_t left, size_t right)
            {
                return rotaryBoundsCenterX(components[left].bounds) < rotaryBoundsCenterX(components[right].bounds);
            }
        );

        // 倾斜切面中，不同侧面的 X 中心可能相差一个甚至多个截面尺寸。
        const double disconnectedCutMaxSpanX = std::max(sectionSize * 3.0, spanX * 0.10);
        const double disconnectedCutClusterTolerance = disconnectedCutMaxSpanX;
        std::vector<size_t> pendingComponentCluster;
        double pendingClusterCenterX = 0.0;

        auto flushPendingComponentCluster = [&]()
        {
            if (pendingComponentCluster.empty())
            {
                return;
            }

            RotaryFeatureComponent aggregateComponent;

            for (const size_t componentIndex : pendingComponentCluster)
            {
                if (componentIndex >= components.size())
                {
                    continue;
                }

                aggregateComponent.itemIndices.insert
                (
                    aggregateComponent.itemIndices.end(),
                    components[componentIndex].itemIndices.begin(),
                    components[componentIndex].itemIndices.end()
                );
                mergeRotaryBounds(aggregateComponent.bounds, components[componentIndex].bounds);
            }

            const double aggregateSpanX = aggregateComponent.bounds.valid
                ? aggregateComponent.bounds.maxX - aggregateComponent.bounds.minX
                : 0.0;

            if (aggregateSpanX <= disconnectedCutMaxSpanX
                && componentMatchesSquareTubeEndCut(aggregateComponent, sortableItems, sectionBounds, sectionTolerance))
            {
                for (const size_t componentIndex : pendingComponentCluster)
                {
                    if (componentIndex < componentIsEndCut.size())
                    {
                        componentIsEndCut[componentIndex] = true;
                        components[componentIndex].isEndCut = true;
                    }
                }
            }
        };

        for (const size_t componentIndex : componentOrder)
        {
            if (!components[componentIndex].bounds.valid)
            {
                continue;
            }

            const double componentCenterX = rotaryBoundsCenterX(components[componentIndex].bounds);

            if (pendingComponentCluster.empty())
            {
                pendingComponentCluster.push_back(componentIndex);
                pendingClusterCenterX = componentCenterX;
                continue;
            }

            if (std::abs(componentCenterX - pendingClusterCenterX) > disconnectedCutClusterTolerance)
            {
                flushPendingComponentCluster();
                pendingComponentCluster.clear();
                pendingComponentCluster.push_back(componentIndex);
                pendingClusterCenterX = componentCenterX;
                continue;
            }

            const size_t oldCount = pendingComponentCluster.size();
            pendingComponentCluster.push_back(componentIndex);
            pendingClusterCenterX =
                (pendingClusterCenterX * static_cast<double>(oldCount) + componentCenterX)
                / static_cast<double>(pendingComponentCluster.size());
        }

        flushPendingComponentCluster();
        }

        for (RotaryItemAnalysis& analysis : itemAnalyses)
        {
            analysis.surface = classifySquareTubeSurface
            (
                sortableItems[analysis.itemIndex],
                sectionBounds,
                sectionTolerance
            );
        }

        std::vector<RotaryCutCluster> cutClusters;
        std::vector<int> manualBoundaryClusters;

        if (useManualBreakBoundaries)
        {
            const auto buildManualCutCluster = [&itemAnalyses](const std::vector<size_t>& itemIndices)
            {
                RotaryCutCluster cluster;
                RotaryPathBounds bounds;

                for (const size_t itemIndex : itemIndices)
                {
                    if (itemIndex < itemAnalyses.size())
                    {
                        cluster.itemIndices.push_back(itemIndex);
                        mergeRotaryBounds(bounds, itemAnalyses[itemIndex].bounds);
                    }
                }

                cluster.centerX = rotaryBoundsCenterX(bounds);
                return cluster;
            };

            for (ManualRotaryBreakBoundary& boundary : manualBreakBoundaries)
            {
                boundary.clusterIndex = static_cast<int>(cutClusters.size());
                cutClusters.push_back(buildManualCutCluster(boundary.itemIndices));
                manualBoundaryClusters.push_back(boundary.clusterIndex);
            }

            std::sort
            (
                manualBoundaryClusters.begin(),
                manualBoundaryClusters.end(),
                [&cutClusters](int left, int right)
                {
                    return cutClusters[static_cast<size_t>(left)].centerX
                        < cutClusters[static_cast<size_t>(right)].centerX;
                }
            );

            const auto analysisForCluster = [&manualBreakBoundaries](int clusterIndex) -> const RotaryCutBoundaryAnalysis*
                {
                    const auto boundary = std::find_if
                    (
                        manualBreakBoundaries.begin(),
                        manualBreakBoundaries.end(),
                        [clusterIndex](const ManualRotaryBreakBoundary& candidate)
                        {
                            return candidate.clusterIndex == clusterIndex;
                        }
                    );
                    return boundary != manualBreakBoundaries.end() ? &boundary->analysis : nullptr;
                };

            for (size_t boundaryIndex = 1; boundaryIndex < manualBoundaryClusters.size(); ++boundaryIndex)
            {
                const RotaryCutBoundaryAnalysis* leftAnalysis = analysisForCluster(manualBoundaryClusters[boundaryIndex - 1]);
                const RotaryCutBoundaryAnalysis* rightAnalysis = analysisForCluster(manualBoundaryClusters[boundaryIndex]);

                if (leftAnalysis == nullptr || rightAnalysis == nullptr)
                {
                    return false;
                }

                QVector<QVector3D> comparisonPoints = leftAnalysis->orderedPath;
                comparisonPoints += rightAnalysis->orderedPath;

                for (const QVector3D& point : comparisonPoints)
                {
                    double leftX = 0.0;
                    double rightX = 0.0;

                    if (!RotaryCutBoundaryAnalyzer::boundaryXAtPoint(*leftAnalysis, point, leftX)
                        || !RotaryCutBoundaryAnalyzer::boundaryXAtPoint(*rightAnalysis, point, rightX)
                        || rightX < leftX - kEndCutConnectionTolerance)
                    {
                        return false;
                    }
                }
            }
        }
        else
        {
            std::vector<size_t> endCutComponentIndices;
            double maxEndCutSpanX = 0.0;

            for (size_t componentIndex = 0; componentIndex < components.size(); ++componentIndex)
            {
                if (!componentIsEndCut[componentIndex])
                {
                    continue;
                }

                endCutComponentIndices.push_back(componentIndex);
                maxEndCutSpanX = std::max(maxEndCutSpanX, components[componentIndex].bounds.maxX - components[componentIndex].bounds.minX);
            }

            if (endCutComponentIndices.size() < 2)
            {
                return false;
            }

            std::sort
            (
                endCutComponentIndices.begin(),
                endCutComponentIndices.end(),
                [&components](size_t left, size_t right)
                {
                    return rotaryBoundsCenterX(components[left].bounds) < rotaryBoundsCenterX(components[right].bounds);
                }
            );

            const double clusterTolerance = std::max(1.0, std::min(std::max(2.0, spanX * 0.01), std::max(2.0, maxEndCutSpanX * 0.75 + 1.0)));

            for (const size_t componentIndex : endCutComponentIndices)
            {
                const RotaryFeatureComponent& component = components[componentIndex];
                const double centerX = rotaryBoundsCenterX(component.bounds);

                if (cutClusters.empty() || std::abs(centerX - cutClusters.back().centerX) > clusterTolerance)
                {
                    RotaryCutCluster cluster;
                    cluster.centerX = centerX;
                    cluster.itemIndices = component.itemIndices;
                    cutClusters.push_back(cluster);
                    continue;
                }

                RotaryCutCluster& cluster = cutClusters.back();
                const size_t oldCount = cluster.itemIndices.size();
                cluster.itemIndices.insert(cluster.itemIndices.end(), component.itemIndices.begin(), component.itemIndices.end());
                cluster.centerX =
                    (cluster.centerX * static_cast<double>(oldCount) + centerX * static_cast<double>(component.itemIndices.size()))
                    / static_cast<double>(cluster.itemIndices.size());
            }
        }

        if ((!useManualBreakBoundaries && cutClusters.size() < 2)
            || (useManualBreakBoundaries && cutClusters.empty()))
        {
            return false;
        }

        std::vector<RotaryLazySegment> segments;
        const double boundaryTolerance = std::max(1.0, spanX * 0.001);

        if (useManualBreakBoundaries)
        {
            // 相邻边界之间各形成一段，首段和尾段只含一侧边界。
            for (size_t boundaryIndex = 0; boundaryIndex <= manualBoundaryClusters.size(); ++boundaryIndex)
            {
                RotaryLazySegment segment;
                segment.leftClusterIndex = boundaryIndex > 0
                    ? manualBoundaryClusters[boundaryIndex - 1]
                    : -1;
                segment.rightClusterIndex = boundaryIndex < manualBoundaryClusters.size()
                    ? manualBoundaryClusters[boundaryIndex]
                    : -1;
                segment.centerX = segment.rightClusterIndex >= 0
                    ? cutClusters[static_cast<size_t>(segment.rightClusterIndex)].centerX
                    : cutClusters[static_cast<size_t>(segment.leftClusterIndex)].centerX + boundaryTolerance;
                segments.push_back(segment);
            }
        }

        std::vector<bool> componentRequiresContinuousGroup(components.size(), false);

        for (size_t componentIndex = 0; componentIndex < components.size(); ++componentIndex)
        {
            if (componentIsEndCut[componentIndex])
            {
                continue;
            }

            RotarySurfaceGroup sharedSurface = RotarySurfaceGroup::Unknown;

            for (const size_t itemIndex : components[componentIndex].itemIndices)
            {
                if (itemIndex >= itemAnalyses.size())
                {
                    return false;
                }

                const RotarySurfaceGroup surface = itemAnalyses[itemIndex].surface;

                if (surface == RotarySurfaceGroup::Unknown
                    || (sharedSurface != RotarySurfaceGroup::Unknown && surface != sharedSurface))
                {
                    componentRequiresContinuousGroup[componentIndex] = true;
                    break;
                }

                sharedSurface = surface;
            }
        }

        const auto manualBoundaryAnalysis = [&manualBreakBoundaries](int clusterIndex) -> const RotaryCutBoundaryAnalysis*
            {
                const auto boundary = std::find_if
                (
                    manualBreakBoundaries.begin(),
                    manualBreakBoundaries.end(),
                    [clusterIndex](const ManualRotaryBreakBoundary& candidate)
                    {
                        return candidate.clusterIndex == clusterIndex;
                    }
                );
                return boundary != manualBreakBoundaries.end() ? &boundary->analysis : nullptr;
            };

        for (size_t componentIndex = 0; componentIndex < components.size(); ++componentIndex)
        {
            if (componentIsEndCut[componentIndex])
            {
                continue;
            }

            const RotaryFeatureComponent& component = components[componentIndex];
            const double componentCenterX = rotaryBoundsCenterX(component.bounds);
            int leftClusterIndex = -1;
            int rightClusterIndex = -1;

            if (useManualBreakBoundaries)
            {
                for (size_t boundaryIndex = 0; boundaryIndex < manualBoundaryClusters.size(); ++boundaryIndex)
                {
                    const int boundaryClusterIndex = manualBoundaryClusters[boundaryIndex];
                    const RotaryCutBoundaryAnalysis* boundaryAnalysis = manualBoundaryAnalysis(boundaryClusterIndex);

                    if (boundaryAnalysis == nullptr)
                    {
                        return false;
                    }

                    bool hasPointBeforeBoundary = false;
                    bool hasPointAfterBoundary = false;

                    for (const size_t itemIndex : component.itemIndices)
                    {
                        if (itemIndex >= sortableItems.size() || sortableItems[itemIndex] == nullptr)
                        {
                            return false;
                        }

                        CadItem* item = sortableItems[itemIndex];
                        item->rebuildRawPathPoints3D();

                        for (const RawPathPoint3D& rawPoint : item->rawPathPoints3D())
                        {
                            const QVector3D point
                            (
                                static_cast<float>(rawPoint.x),
                                static_cast<float>(rawPoint.y),
                                static_cast<float>(rawPoint.z)
                            );
                            double boundaryX = 0.0;

                            if (!RotaryCutBoundaryAnalyzer::boundaryXAtPoint(*boundaryAnalysis, point, boundaryX))
                            {
                                return false;
                            }

                            hasPointBeforeBoundary = hasPointBeforeBoundary
                                || rawPoint.x < boundaryX - boundaryTolerance;
                            hasPointAfterBoundary = hasPointAfterBoundary
                                || rawPoint.x > boundaryX + boundaryTolerance;
                        }
                    }

                    if (hasPointBeforeBoundary && hasPointAfterBoundary)
                    {
                        return false;
                    }

                    if (!hasPointAfterBoundary)
                    {
                        leftClusterIndex = boundaryIndex > 0
                            ? manualBoundaryClusters[boundaryIndex - 1]
                            : -1;
                        rightClusterIndex = boundaryClusterIndex;
                        break;
                    }
                }

                if (rightClusterIndex < 0)
                {
                    leftClusterIndex = manualBoundaryClusters.back();
                }
            }
            else
            {
                for (size_t clusterIndex = 0; clusterIndex < cutClusters.size(); ++clusterIndex)
                {
                    const double clusterCenterX = cutClusters[clusterIndex].centerX;

                    if (clusterCenterX < componentCenterX - boundaryTolerance)
                    {
                        leftClusterIndex = static_cast<int>(clusterIndex);
                        continue;
                    }

                    if (clusterCenterX > componentCenterX + boundaryTolerance)
                    {
                        rightClusterIndex = static_cast<int>(clusterIndex);
                        break;
                    }
                }
            }

            if ((!useManualBreakBoundaries
                    && (leftClusterIndex < 0 || rightClusterIndex < 0 || leftClusterIndex == rightClusterIndex))
                || (useManualBreakBoundaries && leftClusterIndex < 0 && rightClusterIndex < 0))
            {
                return false;
            }

            auto existingSegment = std::find_if
            (
                segments.begin(),
                segments.end(),
                [leftClusterIndex, rightClusterIndex](const RotaryLazySegment& segment)
                {
                    return segment.leftClusterIndex == leftClusterIndex && segment.rightClusterIndex == rightClusterIndex;
                }
            );

            if (existingSegment == segments.end())
            {
                RotaryLazySegment segment;
                segment.leftClusterIndex = leftClusterIndex;
                segment.rightClusterIndex = rightClusterIndex;
                segment.centerX = (cutClusters[static_cast<size_t>(leftClusterIndex)].centerX + cutClusters[static_cast<size_t>(rightClusterIndex)].centerX) * 0.5;
                segments.push_back(segment);
                existingSegment = std::prev(segments.end());
            }

            if (componentRequiresContinuousGroup[componentIndex])
            {
                existingSegment->continuousItems.insert
                (
                    existingSegment->continuousItems.end(),
                    component.itemIndices.begin(),
                    component.itemIndices.end()
                );
                continue;
            }

            for (const size_t itemIndex : component.itemIndices)
            {
                if (itemIndex >= itemAnalyses.size())
                {
                    return false;
                }

                segmentItemsForSurface(*existingSegment, itemAnalyses[itemIndex].surface).push_back(itemIndex);
            }
        }

        if (segments.empty())
        {
            return false;
        }

        std::sort
        (
            segments.begin(),
            segments.end(),
            [](const RotaryLazySegment& left, const RotaryLazySegment& right)
            {
                return left.centerX < right.centerX;
            }
        );

        processUpdates.clear();
        processUpdates.reserve(sortableItems.size());
        std::vector<ProcessConnectionSegment> processedSegments;
        processedSegments.reserve(sortableItems.size());
        std::vector<bool> scheduled(sortableItems.size(), false);
        bool hasCurrentEndPoint = false;
        QVector3D currentEndPoint;

        auto appendGroup = [&](const std::vector<size_t>& rawIndices) -> bool
        {
            std::vector<size_t> filteredIndices;
            filteredIndices.reserve(rawIndices.size());

            for (const size_t index : rawIndices)
            {
                if (index < sortableItems.size() && !scheduled[index])
                {
                    filteredIndices.push_back(index);
                }
            }

            if (filteredIndices.empty())
            {
                return true;
            }

            if (!appendSorted3DGroup(sortableItems, filteredIndices, rotaryAxisConfig, processUpdates, processedSegments, hasCurrentEndPoint, currentEndPoint))
            {
                return false;
            }

            for (const size_t index : filteredIndices)
            {
                scheduled[index] = true;
            }

            return true;
        };

        auto appendSurfaceSweepGroup =
            [&](const std::vector<size_t>& rawIndices, bool leftToRight) -> bool
            {
                std::vector<CadItem*> localItems;
                std::vector<size_t> localToGlobal;
                localItems.reserve(rawIndices.size());
                localToGlobal.reserve(rawIndices.size());

                for (const size_t index : rawIndices)
                {
                    if (index < sortableItems.size()
                        && !scheduled[index]
                        && sortableItems[index] != nullptr)
                    {
                        localItems.push_back(sortableItems[index]);
                        localToGlobal.push_back(index);
                    }
                }

                if (localItems.empty())
                {
                    return true;
                }

                const std::vector<int> componentIds = buildItemConnectivityComponents(localItems);
                if (componentIds.size() != localItems.size())
                {
                    return false;
                }

                const int componentCount = componentIds.empty()
                    ? 0
                    : *std::max_element(componentIds.begin(), componentIds.end()) + 1;
                if (componentCount <= 0)
                {
                    return false;
                }

                std::vector<std::vector<size_t>> componentIndices(static_cast<size_t>(componentCount));

                for (size_t localIndex = 0; localIndex < localToGlobal.size(); ++localIndex)
                {
                    const int componentId = componentIds[localIndex];
                    if (componentId < 0 || componentId >= componentCount)
                    {
                        return false;
                    }

                    componentIndices[static_cast<size_t>(componentId)].push_back(localToGlobal[localIndex]);
                }

                const auto componentSweepBoundaryX = [&sortableItems, leftToRight](const std::vector<size_t>& indices)
                    {
                        bool hasPoint = false;
                        double boundaryX = 0.0;

                        for (const size_t index : indices)
                        {
                            if (index >= sortableItems.size() || sortableItems[index] == nullptr)
                            {
                                continue;
                            }

                            for (const QVector3D& point : sortableItems[index]->m_geometry.vertices)
                            {
                                if (!hasPoint)
                                {
                                    boundaryX = point.x();
                                    hasPoint = true;
                                    continue;
                                }

                                boundaryX = leftToRight
                                    ? std::min(boundaryX, static_cast<double>(point.x()))
                                    : std::max(boundaryX, static_cast<double>(point.x()));
                            }
                        }

                        return boundaryX;
                    };

                std::sort
                (
                    componentIndices.begin(),
                    componentIndices.end(),
                    [&componentSweepBoundaryX, leftToRight](const std::vector<size_t>& left, const std::vector<size_t>& right)
                    {
                        const double leftBoundaryX = componentSweepBoundaryX(left);
                        const double rightBoundaryX = componentSweepBoundaryX(right);

                        if (std::abs(leftBoundaryX - rightBoundaryX) <= kSortEpsilon)
                        {
                            return left < right;
                        }

                        return leftToRight ? leftBoundaryX < rightBoundaryX : leftBoundaryX > rightBoundaryX;
                    }
                );

                for (const std::vector<size_t>& component : componentIndices)
                {
                    const double sweepBoundaryX = componentSweepBoundaryX(component);

                    if (component.size() == 1)
                    {
                        const size_t itemIndex = component.front();
                        const std::vector<ProcessPathOption> options = buildPathOptionsForItem
                        (
                            sortableItems[itemIndex],
                            SortStrategy::Smart
                        );

                        if (options.empty())
                        {
                            return false;
                        }

                        const QVector3D referencePoint = hasCurrentEndPoint
                            ? currentEndPoint
                            : kRotaryInitialSortOrigin;
                        const ProcessPathOption* selectedOption = nullptr;
                        double bestBoundaryDistance = std::numeric_limits<double>::max();
                        double bestTravelDistance = std::numeric_limits<double>::max();

                        for (const ProcessPathOption& option : options)
                        {
                            const double boundaryDistance = std::abs
                            (
                                static_cast<double>(option.startPoint.x()) - sweepBoundaryX
                            );
                            const double travelDistance = rotarySortTravelDistance
                            (
                                referencePoint,
                                option.startPoint,
                                rotaryAxisConfig
                            );

                            if (selectedOption == nullptr
                                || boundaryDistance < bestBoundaryDistance - kSurfaceSweepBoundaryTolerance
                                || (std::abs(boundaryDistance - bestBoundaryDistance) <= kSurfaceSweepBoundaryTolerance
                                    && (travelDistance < bestTravelDistance - kSortEpsilon
                                        || (std::abs(travelDistance - bestTravelDistance) <= kSortEpsilon
                                            && isPointLexicographicallyLess(option.startPoint, selectedOption->startPoint)))))
                            {
                                selectedOption = &option;
                                bestBoundaryDistance = boundaryDistance;
                                bestTravelDistance = travelDistance;
                            }
                        }

                        if (selectedOption == nullptr)
                        {
                            return false;
                        }

                        processUpdates.push_back
                        ({
                            sortableItems[itemIndex],
                            static_cast<int>(processUpdates.size()),
                            selectedOption->reverse,
                            selectedOption->hasCustomStart,
                            selectedOption->processStartParameter
                        });
                        hasCurrentEndPoint = true;
                        currentEndPoint = selectedOption->endPoint;
                        processedSegments.push_back({ selectedOption->startPoint, selectedOption->endPoint });
                        scheduled[itemIndex] = true;
                        continue;
                    }

                    if (!appendSorted3DGroup
                    (
                        sortableItems,
                        component,
                        rotaryAxisConfig,
                        processUpdates,
                        processedSegments,
                        hasCurrentEndPoint,
                        currentEndPoint,
                        true,
                        sweepBoundaryX
                    ))
                    {
                        return false;
                    }

                    for (const size_t index : component)
                    {
                        scheduled[index] = true;
                    }
                }

                return true;
            };

        for (const RotaryLazySegment& segment : segments)
        {
            if ((!useManualBreakBoundaries
                    && (segment.leftClusterIndex < 0 || segment.rightClusterIndex < 0))
                || (segment.leftClusterIndex >= 0
                    && static_cast<size_t>(segment.leftClusterIndex) >= cutClusters.size())
                || (segment.rightClusterIndex >= 0
                    && static_cast<size_t>(segment.rightClusterIndex) >= cutClusters.size()))
            {
                return false;
            }

            if ((segment.leftClusterIndex >= 0
                    && !appendGroup(cutClusters[static_cast<size_t>(segment.leftClusterIndex)].itemIndices))
                || !appendGroup(segment.continuousItems)
                // S-shaped machining sweep: top L->R, right R->L, bottom L->R, left R->L.
                || !appendSurfaceSweepGroup(segment.topItems, true)
                || !appendSurfaceSweepGroup(segment.rightItems, false)
                || !appendSurfaceSweepGroup(segment.bottomItems, true)
                || !appendSurfaceSweepGroup(segment.leftItems, false)
                || !appendGroup(segment.unknownItems)
                || (segment.rightClusterIndex >= 0
                    && !appendGroup(cutClusters[static_cast<size_t>(segment.rightClusterIndex)].itemIndices)))
            {
                processUpdates.clear();
                return false;
            }
        }

        if (processUpdates.size() != sortableItems.size())
        {
            processUpdates.clear();
            return false;
        }

        return true;
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

bool Gcode_postprocessing_system::sortEntitiesByCurrentMode(bool smartSort)
{
    const int excludedCount = refreshWasteProcessingExclusions();

    if (excludedCount > 0)
    {
        ui->openGLWidget->appendCommandMessage
        (
            QStringLiteral("废面规则已排除 %1 个图元，本次排序不会处理这些图元。").arg(excludedCount)
        );
    }

    const GGenerator::GenerationMode generationMode = resolveGenerationMode();
    return smartSort
        ? (generationMode == GGenerator::GenerationMode::Mode3D
            ? smartSortEntities3D()
            : smartSortEntities())
        : (generationMode == GGenerator::GenerationMode::Mode3D
            ? sortEntitiesByCurrentDirection3D()
            : sortEntitiesByCurrentDirection());
}

QVector<CadItem*> Gcode_postprocessing_system::expandedSelectedRotaryEndCut(QString* errorMessage) const
{
    const QVector<CadItem*> selectedItems = ui->openGLWidget->selectedEntities();

    if (selectedItems.isEmpty())
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("请先选中切面中的一个或部分图元。系统会自动扩展相连图元。");
        }

        return {};
    }

    QVector<CadItem*> sceneItems;
    sceneItems.reserve(static_cast<qsizetype>(m_document.m_entities.size()));

    for (const std::unique_ptr<CadItem>& entity : m_document.m_entities)
    {
        if (entity != nullptr)
        {
            sceneItems.push_back(entity.get());
        }
    }

    const RotaryTubeSectionModel candidateSection = RotaryTubeGeometryAnalyzer::buildSectionModel
    (
        selectedItems,
        sceneItems,
        kEndCutConnectionTolerance
    );

    if (!candidateSection.valid || candidateSection.outerBoundaryItems.isEmpty())
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = candidateSection.errorMessage;
        }

        return {};
    }

    const QVector<QVector2D>& validationHull = m_rotaryTubeSectionModel.valid
        ? m_rotaryTubeSectionModel.sectionHull
        : candidateSection.sectionHull;

    const RotaryCutBoundaryAnalysis analysis = RotaryCutBoundaryAnalyzer::analyze
    (
        candidateSection.outerBoundaryItems,
        sceneItems,
        kEndCutConnectionTolerance,
        validationHull
    );

    if (!analysis.valid)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = analysis.errorMessage;
        }

        return {};
    }

    if (errorMessage != nullptr)
    {
        errorMessage->clear();
    }

    return candidateSection.outerBoundaryItems;
}

bool Gcode_postprocessing_system::assignSelectedRotaryEndCut()
{
    QString selectionError;
    const QVector<CadItem*> expandedItems = expandedSelectedRotaryEndCut(&selectionError);

    if (expandedItems.isEmpty())
    {
        QMessageBox::warning(this, QStringLiteral("指定切面"), selectionError);
        return false;
    }

    for (const CadItem* item : expandedItems)
    {
        if (item != nullptr && item->m_rotaryEndCutRole != RotaryEndCutRole::None)
        {
            QMessageBox::warning
            (
                this,
                QStringLiteral("指定切面"),
                QStringLiteral("选中的图元已属于一个切面边界。如需重新指定，请先清除切面指定。")
            );
            return false;
        }
    }

    int highestBoundaryId = -1;

    for (const std::unique_ptr<CadItem>& entity : m_document.m_entities)
    {
        if (entity == nullptr || entity->m_rotaryEndCutRole == RotaryEndCutRole::None)
        {
            continue;
        }

        highestBoundaryId = std::max(highestBoundaryId, entity->m_rotaryEndCutPairId);
    }

    const int boundaryId = highestBoundaryId + 1;

    for (CadItem* item : expandedItems)
    {
        if (item == nullptr)
        {
            continue;
        }

        item->m_rotaryEndCutPairId = boundaryId;
        item->m_rotaryEndCutRole = RotaryEndCutRole::Break;
    }

    const QString message = QStringLiteral("已指定中断切面 断%1，共识别 %2 个相连图元，已通过方管周向分离验证。")
        .arg(boundaryId + 1)
        .arg(expandedItems.size());

    invalidateProcessOrdersAfterEndCutChange();
    refreshWasteProcessingExclusions();
    ui->openGLWidget->appendCommandMessage(message);
    ui->openGLWidget->update();
    statusBar()->showMessage(message, 5000);
    return true;
}

bool Gcode_postprocessing_system::assignSelectedWasteEndCut()
{
    QString selectionError;
    const QVector<CadItem*> expandedItems = expandedSelectedRotaryEndCut(&selectionError);

    if (expandedItems.isEmpty())
    {
        QMessageBox::warning(this, QStringLiteral("指定废面"), selectionError);
        return false;
    }

    for (const CadItem* item : expandedItems)
    {
        if (item != nullptr && item->m_rotaryEndCutRole != RotaryEndCutRole::None)
        {
            QMessageBox::warning(this, QStringLiteral("指定废面"), QStringLiteral("选中的图元已属于一个切面边界，请先清除原切面指定。"));
            return false;
        }
    }

    int highestBoundaryId = -1;

    for (const std::unique_ptr<CadItem>& entity : m_document.m_entities)
    {
        if (entity != nullptr && entity->m_rotaryEndCutRole != RotaryEndCutRole::None)
        {
            highestBoundaryId = std::max(highestBoundaryId, entity->m_rotaryEndCutPairId);
        }
    }

    const int wasteId = highestBoundaryId + 1;

    for (CadItem* item : expandedItems)
    {
        if (item == nullptr)
        {
            continue;
        }

        item->m_rotaryEndCutPairId = wasteId;
        item->m_rotaryEndCutRole = RotaryEndCutRole::Waste;
    }

    invalidateProcessOrdersAfterEndCutChange();
    const int excludedCount = refreshWasteProcessingExclusions();
    const QString message = QStringLiteral("已指定废弃面 W%1，共识别 %2 个相连图元，已通过方管周向分离验证；当前废弃区共排除 %3 个图元。")
        .arg(wasteId + 1)
        .arg(expandedItems.size())
        .arg(excludedCount);
    ui->openGLWidget->appendCommandMessage(message);
    ui->openGLWidget->update();
    statusBar()->showMessage(message, 6000);
    return true;
}

bool Gcode_postprocessing_system::smartAssignSelectedRotaryEndCut()
{
    return assignSelectedRotaryEndCut();
}

bool Gcode_postprocessing_system::recognizeRotaryTubeSection()
{
    QVector<CadItem*> sceneItems;
    sceneItems.reserve(static_cast<qsizetype>(m_document.m_entities.size()));

    for (const std::unique_ptr<CadItem>& entity : m_document.m_entities)
    {
        if (entity != nullptr)
        {
            sceneItems.push_back(entity.get());
        }
    }

    const RotaryTubeSectionModel recognized = RotaryTubeGeometryAnalyzer::buildSectionModel
    (
        ui->openGLWidget->selectedEntities(),
        sceneItems,
        kEndCutConnectionTolerance
    );

    if (!recognized.valid)
    {
        QMessageBox::warning(this, QStringLiteral("识别方管垂直截面"), recognized.errorMessage);
        return false;
    }

    m_rotaryTubeSectionModel = recognized;
    syncToolPanelState();
    const QString message = QStringLiteral("方管垂直截面识别完成，共提取 %1 个外轮廓点；内部线和无用支线已忽略。")
        .arg(recognized.sectionHull.size());
    ui->openGLWidget->appendCommandMessage(message);
    statusBar()->showMessage(message, 5000);
    return true;
}

bool Gcode_postprocessing_system::removeInternalMachiningPaths()
{
    QVector<CadItem*> sceneItems;
    sceneItems.reserve(static_cast<qsizetype>(m_document.m_entities.size()));

    for (const std::unique_ptr<CadItem>& entity : m_document.m_entities)
    {
        if (entity != nullptr)
        {
            entity->m_excludedAsInternalGeometry = false;
            sceneItems.push_back(entity.get());
        }
    }

    const RotaryInternalPathResult result = RotaryTubeGeometryAnalyzer::findInternalPaths
    (
        m_rotaryTubeSectionModel,
        sceneItems,
        kEndCutConnectionTolerance
    );
    QSet<CadItem*> internalItems;

    for (CadItem* item : result.physicalInteriorItems)
    {
        internalItems.insert(item);
    }

    for (CadItem* item : result.topologicalInteriorItems)
    {
        internalItems.insert(item);
    }

    for (CadItem* item : internalItems)
    {
        item->m_excludedAsInternalGeometry = true;
        item->m_excludedFromProcessing = true;
        item->m_processOrder = -1;
    }

    invalidateProcessOrdersAfterEndCutChange();
    refreshWasteProcessingExclusions();
    const QString message = m_rotaryTubeSectionModel.valid
        ? QStringLiteral("内部线条识别完成：进入方管内部 %1 个，最大外轮廓之外 %2 个，共排除 %3 个图元。")
            .arg(result.physicalInteriorItems.size())
            .arg(result.topologicalInteriorItems.size())
            .arg(internalItems.size())
        : QStringLiteral("方管垂直截面尚未识别，本次仅执行拓扑过滤：最大外轮廓之外 %1 个，共排除 %2 个图元。")
            .arg(result.topologicalInteriorItems.size())
            .arg(internalItems.size());
    ui->openGLWidget->appendCommandMessage(message);
    ui->openGLWidget->update();
    statusBar()->showMessage(message, 6000);
    return true;
}

bool Gcode_postprocessing_system::restoreInternalMachiningPaths()
{
    int restoredCount = 0;

    for (const std::unique_ptr<CadItem>& entity : m_document.m_entities)
    {
        if (entity != nullptr && entity->m_excludedAsInternalGeometry)
        {
            entity->m_excludedAsInternalGeometry = false;
            ++restoredCount;
        }
    }

    refreshWasteProcessingExclusions();
    ui->openGLWidget->update();

    const QString message = restoredCount > 0
        ? QStringLiteral("已恢复 %1 个由内部线识别排除的图元。").arg(restoredCount)
        : QStringLiteral("当前没有由内部线识别排除的图元。");
    ui->openGLWidget->appendCommandMessage(message);
    statusBar()->showMessage(message, 4000);
    return restoredCount > 0;
}

bool Gcode_postprocessing_system::clearSelectedRotaryEndCutAssignments()
{
    const QVector<CadItem*> selectedItems = ui->openGLWidget->selectedEntities();
    QSet<int> pairIds;

    for (const CadItem* item : selectedItems)
    {
        if (item != nullptr
            && item->m_rotaryEndCutRole != RotaryEndCutRole::None
            && item->m_rotaryEndCutPairId >= 0)
        {
            pairIds.insert(item->m_rotaryEndCutPairId);
        }
    }

    if (pairIds.isEmpty())
    {
        QMessageBox::information(this, QStringLiteral("清除切面指定"), QStringLiteral("请先选中一个已指定的切面。"));
        return false;
    }

    int clearedCount = 0;

    for (const std::unique_ptr<CadItem>& entity : m_document.m_entities)
    {
        if (entity == nullptr || !pairIds.contains(entity->m_rotaryEndCutPairId))
        {
            continue;
        }

        entity->m_rotaryEndCutPairId = -1;
        entity->m_rotaryEndCutRole = RotaryEndCutRole::None;
        ++clearedCount;
    }

    const QString message = QStringLiteral("已清除 %1 个切面边界，共 %2 个图元。")
        .arg(pairIds.size())
        .arg(clearedCount);
    invalidateProcessOrdersAfterEndCutChange();
    refreshWasteProcessingExclusions();
    ui->openGLWidget->appendCommandMessage(message);
    ui->openGLWidget->update();
    statusBar()->showMessage(message, 5000);
    return true;
}

bool Gcode_postprocessing_system::clearRotaryEndCutAssignments()
{
    int clearedCount = 0;

    for (const std::unique_ptr<CadItem>& entity : m_document.m_entities)
    {
        if (entity == nullptr || entity->m_rotaryEndCutRole == RotaryEndCutRole::None)
        {
            continue;
        }

        entity->m_rotaryEndCutPairId = -1;
        entity->m_rotaryEndCutRole = RotaryEndCutRole::None;
        ++clearedCount;
    }

    if (clearedCount == 0)
    {
        statusBar()->showMessage(QStringLiteral("当前没有已指定的切面。"), 3000);
        return false;
    }

    const QString message = QStringLiteral("已清除 %1 个图元的切面指定。").arg(clearedCount);
    invalidateProcessOrdersAfterEndCutChange();
    refreshWasteProcessingExclusions();
    ui->openGLWidget->appendCommandMessage(message);
    ui->openGLWidget->update();
    statusBar()->showMessage(message, 5000);
    return true;
}

void Gcode_postprocessing_system::invalidateProcessOrdersAfterEndCutChange()
{
    for (const std::unique_ptr<CadItem>& entity : m_document.m_entities)
    {
        if (entity != nullptr)
        {
            entity->m_processOrder = -1;
        }
    }
}

int Gcode_postprocessing_system::refreshWasteProcessingExclusions()
{
    struct BoundaryGroup
    {
        RotaryEndCutRole role = RotaryEndCutRole::None;
        QVector<CadItem*> items;
    };

    std::map<int, BoundaryGroup> boundaryGroups;
    QVector<CadItem*> sceneItems;
    sceneItems.reserve(static_cast<qsizetype>(m_document.m_entities.size()));

    for (const std::unique_ptr<CadItem>& entity : m_document.m_entities)
    {
        if (entity == nullptr)
        {
            continue;
        }

        entity->m_excludedFromProcessing = entity->m_excludedAsInternalGeometry;
        sceneItems.push_back(entity.get());

        if (entity->m_rotaryEndCutRole == RotaryEndCutRole::None || entity->m_rotaryEndCutPairId < 0)
        {
            continue;
        }

        const int roleIndex = static_cast<int>(entity->m_rotaryEndCutRole);
        const int key = entity->m_rotaryEndCutPairId * 4 + roleIndex;
        BoundaryGroup& group = boundaryGroups[key];
        group.role = entity->m_rotaryEndCutRole;
        group.items.push_back(entity.get());

        if (entity->m_rotaryEndCutRole == RotaryEndCutRole::Waste)
        {
            entity->m_excludedFromProcessing = true;
            entity->m_processOrder = -1;
        }
    }

    struct BoundaryPosition
    {
        RotaryEndCutRole role = RotaryEndCutRole::None;
        double centerX = 0.0;
        RotaryCutBoundaryAnalysis analysis;
    };

    std::vector<BoundaryPosition> boundaries;
    boundaries.reserve(boundaryGroups.size());

    for (const auto& [key, group] : boundaryGroups)
    {
        Q_UNUSED(key);
        BoundaryPosition boundary;
        boundary.role = group.role;
        boundary.analysis = RotaryCutBoundaryAnalyzer::analyze
        (
            group.items,
            sceneItems,
            kEndCutConnectionTolerance,
            m_rotaryTubeSectionModel.sectionHull
        );

        if (!boundary.analysis.valid || boundary.analysis.orderedPath.isEmpty())
        {
            continue;
        }

        for (const QVector3D& point : boundary.analysis.orderedPath)
        {
            boundary.centerX += point.x();
        }

        boundary.centerX /= static_cast<double>(boundary.analysis.orderedPath.size());
        boundaries.push_back(std::move(boundary));
    }

    std::sort
    (
        boundaries.begin(),
        boundaries.end(),
        [](const BoundaryPosition& left, const BoundaryPosition& right)
        {
            return left.centerX < right.centerX;
        }
    );

    bool boundariesHaveStableOrder = true;

    for (size_t boundaryIndex = 1; boundaryIndex < boundaries.size() && boundariesHaveStableOrder; ++boundaryIndex)
    {
        const BoundaryPosition& leftBoundary = boundaries[boundaryIndex - 1];
        const BoundaryPosition& rightBoundary = boundaries[boundaryIndex];
        QVector<QVector3D> comparisonPoints = leftBoundary.analysis.orderedPath;
        comparisonPoints += rightBoundary.analysis.orderedPath;

        for (const QVector3D& point : comparisonPoints)
        {
            double leftX = 0.0;
            double rightX = 0.0;

            if (!RotaryCutBoundaryAnalyzer::boundaryXAtPoint(leftBoundary.analysis, point, leftX)
                || !RotaryCutBoundaryAnalyzer::boundaryXAtPoint(rightBoundary.analysis, point, rightX)
                || rightX < leftX - kEndCutConnectionTolerance)
            {
                boundariesHaveStableOrder = false;
                break;
            }
        }
    }

    if (!boundariesHaveStableOrder)
    {
        boundaries.clear();
    }

    int excludedCount = 0;

    for (const std::unique_ptr<CadItem>& entity : m_document.m_entities)
    {
        if (entity == nullptr)
        {
            continue;
        }

        if (entity->m_rotaryEndCutRole == RotaryEndCutRole::Waste)
        {
            ++excludedCount;
            continue;
        }

        if (entity->m_rotaryEndCutRole != RotaryEndCutRole::None || boundaries.size() < 2)
        {
            continue;
        }

        entity->rebuildRawPathPoints3D();

        if (entity->rawPathPoints3D().empty())
        {
            continue;
        }

        size_t intervalIndex = boundaries.size();
        bool crossesBoundary = false;

        for (size_t boundaryIndex = 0; boundaryIndex < boundaries.size(); ++boundaryIndex)
        {
            bool hasPointBefore = false;
            bool hasPointAfter = false;

            for (const RawPathPoint3D& rawPoint : entity->rawPathPoints3D())
            {
                const QVector3D point
                (
                    static_cast<float>(rawPoint.x),
                    static_cast<float>(rawPoint.y),
                    static_cast<float>(rawPoint.z)
                );
                double boundaryX = 0.0;

                if (!RotaryCutBoundaryAnalyzer::boundaryXAtPoint(boundaries[boundaryIndex].analysis, point, boundaryX))
                {
                    crossesBoundary = true;
                    break;
                }

                hasPointBefore = hasPointBefore || rawPoint.x < boundaryX - kEndCutConnectionTolerance;
                hasPointAfter = hasPointAfter || rawPoint.x > boundaryX + kEndCutConnectionTolerance;
            }

            if (crossesBoundary || (hasPointBefore && hasPointAfter))
            {
                crossesBoundary = true;
                break;
            }

            if (!hasPointAfter)
            {
                intervalIndex = boundaryIndex;
                break;
            }
        }

        if (crossesBoundary || intervalIndex == 0 || intervalIndex >= boundaries.size())
        {
            continue;
        }

        if (boundaries[intervalIndex - 1].role == RotaryEndCutRole::Waste
            || boundaries[intervalIndex].role == RotaryEndCutRole::Waste)
        {
            entity->m_excludedFromProcessing = true;
            entity->m_processOrder = -1;
            ++excludedCount;
        }
    }

    return excludedCount;
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

    const SortableDedupResult dedupResult = deduplicateSortableItems(sortableItems);

    if (!dedupResult.duplicateItems.isEmpty() && !m_editer.deleteEntities(dedupResult.duplicateItems))
    {
        QMessageBox::warning(this, QStringLiteral("3轴排序"), QStringLiteral("重复图元自动去重失败，排序已中止。"));
        return false;
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
        QStringLiteral("3轴排序完成，共更新 %1 个图元的加工顺序，首件已按最接近原点的当前起点选取，并保留当前加工方向设置。%2")
            .arg(processUpdates.size())
            .arg(dedupResult.removedCount > 0 ? QStringLiteral("已自动删除 %1 个重复图元。").arg(dedupResult.removedCount) : QString())
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

    const SortableDedupResult dedupResult = deduplicateSortableItems(sortableItems);

    if (!dedupResult.duplicateItems.isEmpty() && !m_editer.deleteEntities(dedupResult.duplicateItems))
    {
        QMessageBox::warning(this, QStringLiteral("3轴智能排序"), QStringLiteral("重复图元自动去重失败，排序已中止。"));
        return false;
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
        QStringLiteral("3轴智能排序完成，共更新 %1 个图元的加工顺序，并已对闭合图元的方向/起刀缝点做连续性优化。%2")
            .arg(processUpdates.size())
            .arg(dedupResult.removedCount > 0 ? QStringLiteral("已自动删除 %1 个重复图元。").arg(dedupResult.removedCount) : QString())
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

    const SortableDedupResult dedupResult = deduplicateSortableItems(sortableItems);

    if (!dedupResult.duplicateItems.isEmpty() && !m_editer.deleteEntities(dedupResult.duplicateItems))
    {
        QMessageBox::warning(this, QStringLiteral("4轴(绕A)排序"), QStringLiteral("重复图元自动去重失败，排序已中止。"));
        return false;
    }

    if (sortableItems.empty())
    {
        QMessageBox::warning(this, QStringLiteral("4轴(绕A)排序"), QStringLiteral("当前文档中没有可参与 4 轴 G 代码排序的图元。"));
        return false;
    }

    const GProfileRotaryAxisConfig& rotaryAxisConfig = m_activeProfile.rotaryAxisConfig();
    std::vector<CadEditer::ProcessStateUpdate> lazyRotaryUpdates;

    if (tryBuildSquareTubeLazyRotaryProcessUpdates
    (
        sortableItems,
        rotaryAxisConfig,
        lazyRotaryUpdates,
        m_rotaryTubeSectionModel.sectionHull
    ))
    {
        if (!m_editer.applyEntityProcessStates(lazyRotaryUpdates))
        {
            QMessageBox::warning(this, QStringLiteral("4轴(绕A)排序"), QStringLiteral("4轴排序结果写入失败。"));
            return false;
        }

        ui->openGLWidget->appendCommandMessage
        (
            QStringLiteral("4轴(绕A)排序完成，共更新 %1 个图元；已按中断切面分段，并在每个切面前完成其左侧图元的懒旋转加工。%2")
                .arg(lazyRotaryUpdates.size())
                .arg(dedupResult.removedCount > 0 ? QStringLiteral("已自动删除 %1 个重复图元。").arg(dedupResult.removedCount) : QString())
        );
        ui->openGLWidget->refreshCommandPrompt();
        statusBar()->showMessage(QStringLiteral("4轴(绕A)懒旋转排序完成，共更新 %1 个图元").arg(lazyRotaryUpdates.size()), 5000);
        return true;
    }

    if (hasManualRotaryEndCutAssignments(sortableItems))
    {
        QMessageBox::warning
        (
            this,
            QStringLiteral("4轴(绕A)排序"),
            QStringLiteral("已指定中断切面，但无法形成稳定加工分段。请检查是否有图元横跨切面，或多个三维切面在方管展开面上交叉。")
        );
        return false;
    }

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
        QStringLiteral("4轴(绕A)排序完成，共更新 %1 个图元的加工顺序，排序已按 X 与 A 轴联动连续性重新整理。%2")
            .arg(processUpdates.size())
            .arg(dedupResult.removedCount > 0 ? QStringLiteral("已自动删除 %1 个重复图元。").arg(dedupResult.removedCount) : QString())
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

    const SortableDedupResult dedupResult = deduplicateSortableItems(sortableItems);

    if (!dedupResult.duplicateItems.isEmpty() && !m_editer.deleteEntities(dedupResult.duplicateItems))
    {
        QMessageBox::warning(this, QStringLiteral("4轴(绕A)智能排序"), QStringLiteral("重复图元自动去重失败，排序已中止。"));
        return false;
    }

    if (sortableItems.empty())
    {
        QMessageBox::warning(this, QStringLiteral("4轴(绕A)智能排序"), QStringLiteral("当前文档中没有可参与 4 轴 G 代码排序的图元。"));
        return false;
    }

    const GProfileRotaryAxisConfig& rotaryAxisConfig = m_activeProfile.rotaryAxisConfig();
    std::vector<CadEditer::ProcessStateUpdate> lazyRotaryUpdates;

    if (tryBuildSquareTubeLazyRotaryProcessUpdates
    (
        sortableItems,
        rotaryAxisConfig,
        lazyRotaryUpdates,
        m_rotaryTubeSectionModel.sectionHull
    ))
    {
        if (!m_editer.applyEntityProcessStates(lazyRotaryUpdates))
        {
            QMessageBox::warning(this, QStringLiteral("4轴(绕A)智能排序"), QStringLiteral("4轴智能排序结果写入失败。"));
            return false;
        }

        ui->openGLWidget->appendCommandMessage
        (
            QStringLiteral("4轴(绕A)智能排序完成，共更新 %1 个图元；已按中断切面分段，并在每个切面前完成其左侧图元的懒旋转加工。%2")
                .arg(lazyRotaryUpdates.size())
                .arg(dedupResult.removedCount > 0 ? QStringLiteral("已自动删除 %1 个重复图元。").arg(dedupResult.removedCount) : QString())
        );
        ui->openGLWidget->refreshCommandPrompt();
        statusBar()->showMessage(QStringLiteral("4轴(绕A)懒旋转智能排序完成，共更新 %1 个图元").arg(lazyRotaryUpdates.size()), 5000);
        return true;
    }

    if (hasManualRotaryEndCutAssignments(sortableItems))
    {
        QMessageBox::warning
        (
            this,
            QStringLiteral("4轴(绕A)智能排序"),
            QStringLiteral("已指定中断切面，但无法形成稳定加工分段。请检查是否有图元横跨切面，或多个三维切面在方管展开面上交叉。")
        );
        return false;
    }

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
