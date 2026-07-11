#include "pch.h"

#include "CadProcessVisualUtils.h"

#include "CadItem.h"
#include "CadOcsGeometry.h"

#include <cmath>

namespace
{
    constexpr double kVisualEpsilon = 1.0e-9;
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kTwoPi = 6.28318530717958647692;

    QVector3D normalizeOrZero(QVector3D vector)
    {
        if (vector.lengthSquared() <= kVisualEpsilon)
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

    QVector3D mapPlaneVectorToWorld(const QVector3D& axisU, const QVector3D& axisV, double u, double v)
    {
        return normalizeOrZero
        (
            axisU * static_cast<float>(u)
            + axisV * static_cast<float>(v)
        );
    }

    QVector3D bulgeArcCenter(const QVector3D& startPoint, const QVector3D& endPoint, double bulge, bool* valid = nullptr)
    {
        const QVector3D chord = endPoint - startPoint;
        const double chordLength = chord.length();

        if (valid != nullptr)
        {
            *valid = false;
        }

        if (chordLength <= kVisualEpsilon || std::abs(bulge) < 1.0e-8)
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

        if (majorAxis.lengthSquared() <= kVisualEpsilon || ellipse->ratio <= 0.0)
        {
            return false;
        }

        QVector3D normal(ellipse->extPoint.x, ellipse->extPoint.y, ellipse->extPoint.z);

        if (normal.lengthSquared() <= kVisualEpsilon)
        {
            normal = QVector3D(0.0f, 0.0f, 1.0f);
        }
        else
        {
            normal.normalize();
        }

        minorAxis = QVector3D::crossProduct(normal, majorAxis);

        if (minorAxis.lengthSquared() <= kVisualEpsilon)
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

        if (normal.lengthSquared() <= kVisualEpsilon)
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

            if (axisY.lengthSquared() <= kVisualEpsilon)
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

        if (axisX.lengthSquared() <= kVisualEpsilon)
        {
            axisX = QVector3D(1.0f, 0.0f, 0.0f);
        }
        else
        {
            axisX.normalize();
        }

        axisY = QVector3D::crossProduct(normal, axisX);

        if (axisY.lengthSquared() <= kVisualEpsilon)
        {
            axisY = QVector3D(0.0f, 1.0f, 0.0f);
        }
        else
        {
            axisY.normalize();
        }
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

    double effectiveCircleStartParameter(const CadItem* item)
    {
        if (item != nullptr && item->m_hasCustomProcessStart)
        {
            return normalizeAnglePositive(item->m_processStartParameter);
        }

        return kPi * 0.5;
    }

    double effectiveClosedEllipseStartParameter(const CadItem* item, const DRW_Ellipse* ellipse)
    {
        if (item != nullptr && item->m_hasCustomProcessStart)
        {
            return item->m_processStartParameter;
        }

        return ellipse != nullptr ? ellipse->staparam : 0.0;
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

    QVector3D resolvePolylineNormal(const DRW_Coord& extPoint)
    {
        QVector3D normal(extPoint.x, extPoint.y, extPoint.z);

        if (normal.lengthSquared() <= kVisualEpsilon)
        {
            return QVector3D();
        }

        normal.normalize();
        return normal;
    }

    bool is3DPolyline(const DRW_Polyline* polyline)
    {
        return polyline != nullptr && ((polyline->flags & 8) != 0);
    }

    QVector3D inferPolylinePlaneNormal(const std::vector<std::shared_ptr<DRW_Vertex>>& vertices)
    {
        if (vertices.size() < 3)
        {
            return QVector3D();
        }

        const QVector3D p0
        (
            static_cast<float>(vertices.front()->basePoint.x),
            static_cast<float>(vertices.front()->basePoint.y),
            static_cast<float>(vertices.front()->basePoint.z)
        );

        for (size_t index = 1; index + 1 < vertices.size(); ++index)
        {
            const QVector3D p1
            (
                static_cast<float>(vertices[index]->basePoint.x),
                static_cast<float>(vertices[index]->basePoint.y),
                static_cast<float>(vertices[index]->basePoint.z)
            );
            const QVector3D p2
            (
                static_cast<float>(vertices[index + 1]->basePoint.x),
                static_cast<float>(vertices[index + 1]->basePoint.y),
                static_cast<float>(vertices[index + 1]->basePoint.z)
            );

            QVector3D normal = QVector3D::crossProduct(p1 - p0, p2 - p0);

            if (normal.lengthSquared() > kVisualEpsilon)
            {
                normal.normalize();
                return normal;
            }
        }

        return QVector3D();
    }

    bool buildPolylinePlaneBasis
    (
        const DRW_Polyline* polyline,
        QVector3D& origin,
        QVector3D& axisU,
        QVector3D& axisV,
        QVector3D& normal
    )
    {
        if (polyline == nullptr || polyline->vertlist.empty())
        {
            return false;
        }

        if (is3DPolyline(polyline))
        {
            normal = inferPolylinePlaneNormal(polyline->vertlist);

            if (normal.lengthSquared() <= kVisualEpsilon)
            {
                return false;
            }

            origin = QVector3D
            (
                static_cast<float>(polyline->vertlist.front()->basePoint.x),
                static_cast<float>(polyline->vertlist.front()->basePoint.y),
                static_cast<float>(polyline->vertlist.front()->basePoint.z)
            );
            buildPlaneBasis(normal, axisU, axisV);
            return true;
        }

        normal = resolvePolylineNormal(polyline->extPoint);

        if (normal.lengthSquared() <= kVisualEpsilon)
        {
            normal = QVector3D(0.0f, 0.0f, 1.0f);
        }

        buildPlaneBasis(normal, axisU, axisV);
        origin = normal * static_cast<float>(polyline->basePoint.z);
        return true;
    }

    bool buildLWPolylinePlaneBasis
    (
        const DRW_LWPolyline* polyline,
        QVector3D& origin,
        QVector3D& axisU,
        QVector3D& axisV,
        QVector3D& normal
    )
    {
        if (polyline == nullptr)
        {
            return false;
        }

        normal = resolveNormal(polyline->extPoint);
        buildPlaneBasis(normal, axisU, axisV);
        origin = normal * static_cast<float>(polyline->elevation);
        return true;
    }

    QVector3D polylineVertexToWcs(const DRW_Polyline* polyline, const std::shared_ptr<DRW_Vertex>& vertex)
    {
        if (polyline == nullptr || vertex == nullptr)
        {
            return QVector3D();
        }

        if (is3DPolyline(polyline))
        {
            return QVector3D
            (
                static_cast<float>(vertex->basePoint.x),
                static_cast<float>(vertex->basePoint.y),
                static_cast<float>(vertex->basePoint.z)
            );
        }

        QVector3D origin;
        QVector3D axisU;
        QVector3D axisV;
        QVector3D normal;
        buildPolylinePlaneBasis(polyline, origin, axisU, axisV, normal);

        return origin
            + axisU * static_cast<float>(vertex->basePoint.x)
            + axisV * static_cast<float>(vertex->basePoint.y)
            + normal * static_cast<float>(vertex->basePoint.z);
    }

    QVector3D lwPolylineVertexToWcs(const DRW_LWPolyline* polyline, const std::shared_ptr<DRW_Vertex2D>& vertex)
    {
        if (polyline == nullptr || vertex == nullptr)
        {
            return QVector3D();
        }

        QVector3D origin;
        QVector3D axisU;
        QVector3D axisV;
        QVector3D normal;
        buildLWPolylinePlaneBasis(polyline, origin, axisU, axisV, normal);

        return origin
            + axisU * static_cast<float>(vertex->x)
            + axisV * static_cast<float>(vertex->y);
    }

    void projectPointToPlaneUV
    (
        const QVector3D& origin,
        const QVector3D& axisU,
        const QVector3D& axisV,
        const QVector3D& point,
        double& u,
        double& v
    )
    {
        const QVector3D delta = point - origin;
        u = QVector3D::dotProduct(delta, axisU);
        v = QVector3D::dotProduct(delta, axisV);
    }

    QVector3D bulgeSegmentTangentAtStartOnPlane
    (
        const QVector3D& origin,
        const QVector3D& axisU,
        const QVector3D& axisV,
        const QVector3D& startPoint,
        const QVector3D& endPoint,
        double bulge
    )
    {
        if (std::abs(bulge) < 1.0e-8)
        {
            return normalizeOrZero(endPoint - startPoint);
        }

        double su = 0.0;
        double sv = 0.0;
        double eu = 0.0;
        double ev = 0.0;
        projectPointToPlaneUV(origin, axisU, axisV, startPoint, su, sv);
        projectPointToPlaneUV(origin, axisU, axisV, endPoint, eu, ev);

        const double dx = eu - su;
        const double dy = ev - sv;
        const double chordLength = std::sqrt(dx * dx + dy * dy);

        if (chordLength <= kVisualEpsilon)
        {
            return QVector3D();
        }

        const double midpointU = (su + eu) * 0.5;
        const double midpointV = (sv + ev) * 0.5;
        const double centerOffset = chordLength * (1.0 / bulge - bulge) * 0.25;
        const double centerU = midpointU - centerOffset * (dy / chordLength);
        const double centerV = midpointV + centerOffset * (dx / chordLength);
        const double radiusU = su - centerU;
        const double radiusV = sv - centerV;
        const double tangentU = bulge > 0.0 ? -radiusV : radiusV;
        const double tangentV = bulge > 0.0 ? radiusU : -radiusU;

        return mapPlaneVectorToWorld(axisU, axisV, tangentU, tangentV);
    }

    QVector3D bulgeSegmentTangentAtEndOnPlane
    (
        const QVector3D& origin,
        const QVector3D& axisU,
        const QVector3D& axisV,
        const QVector3D& startPoint,
        const QVector3D& endPoint,
        double bulge
    )
    {
        if (std::abs(bulge) < 1.0e-8)
        {
            return normalizeOrZero(endPoint - startPoint);
        }

        double su = 0.0;
        double sv = 0.0;
        double eu = 0.0;
        double ev = 0.0;
        projectPointToPlaneUV(origin, axisU, axisV, startPoint, su, sv);
        projectPointToPlaneUV(origin, axisU, axisV, endPoint, eu, ev);

        const double dx = eu - su;
        const double dy = ev - sv;
        const double chordLength = std::sqrt(dx * dx + dy * dy);

        if (chordLength <= kVisualEpsilon)
        {
            return QVector3D();
        }

        const double midpointU = (su + eu) * 0.5;
        const double midpointV = (sv + ev) * 0.5;
        const double centerOffset = chordLength * (1.0 / bulge - bulge) * 0.25;
        const double centerU = midpointU - centerOffset * (dy / chordLength);
        const double centerV = midpointV + centerOffset * (dx / chordLength);
        const double radiusU = eu - centerU;
        const double radiusV = ev - centerV;
        const double tangentU = bulge > 0.0 ? -radiusV : radiusV;
        const double tangentV = bulge > 0.0 ? radiusU : -radiusU;

        return mapPlaneVectorToWorld(axisU, axisV, tangentU, tangentV);
    }

    QVector3D polylineForwardStartTangent(const DRW_Polyline* polyline)
    {
        if (polyline == nullptr || polyline->vertlist.size() < 2)
        {
            return QVector3D();
        }

        QVector3D origin;
        QVector3D axisU;
        QVector3D axisV;
        QVector3D normal;
        const bool hasPlaneBasis = buildPolylinePlaneBasis(polyline, origin, axisU, axisV, normal);

        for (size_t index = 0; index + 1 < polyline->vertlist.size(); ++index)
        {
            const auto& current = polyline->vertlist.at(index);
            const auto& next = polyline->vertlist.at(index + 1);
            const QVector3D startPoint = polylineVertexToWcs(polyline, current);
            const QVector3D endPoint = polylineVertexToWcs(polyline, next);
            const QVector3D tangent = hasPlaneBasis && !is3DPolyline(polyline)
                ? bulgeSegmentTangentAtStartOnPlane(origin, axisU, axisV, startPoint, endPoint, current->bulge)
                : normalizeOrZero(endPoint - startPoint);

            if (tangent.lengthSquared() > kVisualEpsilon)
            {
                return tangent;
            }
        }

        if ((polyline->flags & 1) != 0)
        {
            const auto& current = polyline->vertlist.back();
            const auto& next = polyline->vertlist.front();
            const QVector3D startPoint = polylineVertexToWcs(polyline, current);
            const QVector3D endPoint = polylineVertexToWcs(polyline, next);
            return hasPlaneBasis && !is3DPolyline(polyline)
                ? bulgeSegmentTangentAtStartOnPlane(origin, axisU, axisV, startPoint, endPoint, current->bulge)
                : normalizeOrZero(endPoint - startPoint);
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
        const auto& current = polyline->vertlist.at(startIndex);
        const auto& next = polyline->vertlist.at(nextIndex);
        const QVector3D startPoint = polylineVertexToWcs(polyline, current);
        const QVector3D endPoint = polylineVertexToWcs(polyline, next);
        QVector3D origin;
        QVector3D axisU;
        QVector3D axisV;
        QVector3D normal;
        const bool hasPlaneBasis = buildPolylinePlaneBasis(polyline, origin, axisU, axisV, normal);
        return hasPlaneBasis && !is3DPolyline(polyline)
            ? bulgeSegmentTangentAtStartOnPlane(origin, axisU, axisV, startPoint, endPoint, current->bulge)
            : normalizeOrZero(endPoint - startPoint);
    }

    QVector3D polylineReverseStartTangent(const DRW_Polyline* polyline)
    {
        if (polyline == nullptr || polyline->vertlist.size() < 2)
        {
            return QVector3D();
        }

        QVector3D origin;
        QVector3D axisU;
        QVector3D axisV;
        QVector3D normal;
        const bool hasPlaneBasis = buildPolylinePlaneBasis(polyline, origin, axisU, axisV, normal);

        for (size_t index = polyline->vertlist.size() - 1; index > 0; --index)
        {
            const auto& current = polyline->vertlist.at(index);
            const auto& next = polyline->vertlist.at(index - 1);
            const QVector3D startPoint = polylineVertexToWcs(polyline, current);
            const QVector3D endPoint = polylineVertexToWcs(polyline, next);
            const QVector3D tangent = hasPlaneBasis && !is3DPolyline(polyline)
                ? bulgeSegmentTangentAtStartOnPlane(origin, axisU, axisV, startPoint, endPoint, -next->bulge)
                : normalizeOrZero(endPoint - startPoint);

            if (tangent.lengthSquared() > kVisualEpsilon)
            {
                return tangent;
            }
        }

        if ((polyline->flags & 1) != 0)
        {
            const auto& current = polyline->vertlist.front();
            const auto& next = polyline->vertlist.back();
            const QVector3D startPoint = polylineVertexToWcs(polyline, current);
            const QVector3D endPoint = polylineVertexToWcs(polyline, next);
            return hasPlaneBasis && !is3DPolyline(polyline)
                ? bulgeSegmentTangentAtStartOnPlane(origin, axisU, axisV, startPoint, endPoint, -next->bulge)
                : normalizeOrZero(endPoint - startPoint);
        }

        return QVector3D();
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
        const QVector3D startPoint = polylineVertexToWcs(polyline, previous);
        const QVector3D endPoint = polylineVertexToWcs(polyline, current);
        QVector3D origin;
        QVector3D axisU;
        QVector3D axisV;
        QVector3D normal;
        const bool hasPlaneBasis = buildPolylinePlaneBasis(polyline, origin, axisU, axisV, normal);
        return hasPlaneBasis && !is3DPolyline(polyline)
            ? bulgeSegmentTangentAtEndOnPlane(origin, axisU, axisV, startPoint, endPoint, previous->bulge)
            : normalizeOrZero(endPoint - startPoint);
    }

    QVector3D polylineReverseStartTangentAt(const DRW_Polyline* polyline, size_t startIndex)
    {
        return -polylineForwardEndTangentAt(polyline, startIndex);
    }

    QVector3D lwPolylineForwardStartTangent(const DRW_LWPolyline* polyline)
    {
        if (polyline == nullptr || polyline->vertlist.size() < 2)
        {
            return QVector3D();
        }

        QVector3D origin;
        QVector3D axisU;
        QVector3D axisV;
        QVector3D normal;
        const bool hasPlaneBasis = buildLWPolylinePlaneBasis(polyline, origin, axisU, axisV, normal);

        for (size_t index = 0; index + 1 < polyline->vertlist.size(); ++index)
        {
            const auto& current = polyline->vertlist.at(index);
            const auto& next = polyline->vertlist.at(index + 1);
            const QVector3D startPoint = lwPolylineVertexToWcs(polyline, current);
            const QVector3D endPoint = lwPolylineVertexToWcs(polyline, next);
            const QVector3D tangent = hasPlaneBasis
                ? bulgeSegmentTangentAtStartOnPlane(origin, axisU, axisV, startPoint, endPoint, current->bulge)
                : normalizeOrZero(endPoint - startPoint);

            if (tangent.lengthSquared() > kVisualEpsilon)
            {
                return tangent;
            }
        }

        if ((polyline->flags & 1) != 0)
        {
            const auto& current = polyline->vertlist.back();
            const auto& next = polyline->vertlist.front();
            const QVector3D startPoint = lwPolylineVertexToWcs(polyline, current);
            const QVector3D endPoint = lwPolylineVertexToWcs(polyline, next);
            return hasPlaneBasis
                ? bulgeSegmentTangentAtStartOnPlane(origin, axisU, axisV, startPoint, endPoint, current->bulge)
                : normalizeOrZero(endPoint - startPoint);
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
        const auto& current = polyline->vertlist.at(startIndex);
        const auto& next = polyline->vertlist.at(nextIndex);
        const QVector3D startPoint = lwPolylineVertexToWcs(polyline, current);
        const QVector3D endPoint = lwPolylineVertexToWcs(polyline, next);
        QVector3D origin;
        QVector3D axisU;
        QVector3D axisV;
        QVector3D normal;
        const bool hasPlaneBasis = buildLWPolylinePlaneBasis(polyline, origin, axisU, axisV, normal);
        return hasPlaneBasis
            ? bulgeSegmentTangentAtStartOnPlane(origin, axisU, axisV, startPoint, endPoint, current->bulge)
            : normalizeOrZero(endPoint - startPoint);
    }

    QVector3D lwPolylineReverseStartTangent(const DRW_LWPolyline* polyline)
    {
        if (polyline == nullptr || polyline->vertlist.size() < 2)
        {
            return QVector3D();
        }

        QVector3D origin;
        QVector3D axisU;
        QVector3D axisV;
        QVector3D normal;
        const bool hasPlaneBasis = buildLWPolylinePlaneBasis(polyline, origin, axisU, axisV, normal);

        for (size_t index = polyline->vertlist.size() - 1; index > 0; --index)
        {
            const auto& current = polyline->vertlist.at(index);
            const auto& next = polyline->vertlist.at(index - 1);
            const QVector3D startPoint = lwPolylineVertexToWcs(polyline, current);
            const QVector3D endPoint = lwPolylineVertexToWcs(polyline, next);
            const QVector3D tangent = hasPlaneBasis
                ? bulgeSegmentTangentAtStartOnPlane(origin, axisU, axisV, startPoint, endPoint, -next->bulge)
                : normalizeOrZero(endPoint - startPoint);

            if (tangent.lengthSquared() > kVisualEpsilon)
            {
                return tangent;
            }
        }

        if ((polyline->flags & 1) != 0)
        {
            const auto& current = polyline->vertlist.front();
            const auto& next = polyline->vertlist.back();
            const QVector3D startPoint = lwPolylineVertexToWcs(polyline, current);
            const QVector3D endPoint = lwPolylineVertexToWcs(polyline, next);
            return hasPlaneBasis
                ? bulgeSegmentTangentAtStartOnPlane(origin, axisU, axisV, startPoint, endPoint, -next->bulge)
                : normalizeOrZero(endPoint - startPoint);
        }

        return QVector3D();
    }

    QVector3D lwPolylineForwardEndTangentAt(const DRW_LWPolyline* polyline, size_t startIndex)
    {
        if (polyline == nullptr || polyline->vertlist.size() < 2)
        {
            return QVector3D();
        }

        const size_t count = polyline->vertlist.size();
        const size_t previousIndex = (startIndex + count - 1) % count;
        const auto& previous = polyline->vertlist.at(previousIndex);
        const auto& current = polyline->vertlist.at(startIndex);
        const QVector3D startPoint = lwPolylineVertexToWcs(polyline, previous);
        const QVector3D endPoint = lwPolylineVertexToWcs(polyline, current);
        QVector3D origin;
        QVector3D axisU;
        QVector3D axisV;
        QVector3D normal;
        const bool hasPlaneBasis = buildLWPolylinePlaneBasis(polyline, origin, axisU, axisV, normal);
        return hasPlaneBasis
            ? bulgeSegmentTangentAtEndOnPlane(origin, axisU, axisV, startPoint, endPoint, previous->bulge)
            : normalizeOrZero(endPoint - startPoint);
    }

    QVector3D lwPolylineReverseStartTangentAt(const DRW_LWPolyline* polyline, size_t startIndex)
    {
        return -lwPolylineForwardEndTangentAt(polyline, startIndex);
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

    bool tryBuildGeometryAnchor(const CadItem* item, QVector3D& anchor)
    {
        if (item == nullptr || item->m_geometry.vertices.isEmpty())
        {
            return false;
        }

        QVector3D minPoint = item->m_geometry.vertices.front();
        QVector3D maxPoint = minPoint;

        for (const QVector3D& vertex : item->m_geometry.vertices)
        {
            minPoint.setX(std::min(minPoint.x(), vertex.x()));
            minPoint.setY(std::min(minPoint.y(), vertex.y()));
            minPoint.setZ(std::min(minPoint.z(), vertex.z()));
            maxPoint.setX(std::max(maxPoint.x(), vertex.x()));
            maxPoint.setY(std::max(maxPoint.y(), vertex.y()));
            maxPoint.setZ(std::max(maxPoint.z(), vertex.z()));
        }

        anchor = (minPoint + maxPoint) * 0.5f;
        return true;
    }

    void appendSelectionHandle
    (
        QVector<CadSelectionHandleInfo>& handles,
        const QVector3D& position,
        bool isBasePoint,
        bool editable,
        int pointIndex,
        CadSelectionHandleShape shape = CadSelectionHandleShape::RoundPoint,
        const QVector3D& direction = QVector3D()
    )
    {
        for (const CadSelectionHandleInfo& handle : handles)
        {
            if ((handle.position - position).lengthSquared() <= kVisualEpsilon)
            {
                return;
            }
        }

        CadSelectionHandleInfo handle;
        handle.position = position;
        handle.isBasePoint = isBasePoint;
        handle.editable = editable;
        handle.pointIndex = pointIndex;
        handle.shape = shape;
        handle.direction = normalizeOrZero(direction);
        handles.push_back(std::move(handle));
    }
}

bool isProcessVisualizable(const CadItem* item)
{
    if (item == nullptr)
    {
        return false;
    }

    switch (item->m_type)
    {
    case DRW::ETYPE::LINE:
    case DRW::ETYPE::ARC:
    case DRW::ETYPE::CIRCLE:
    case DRW::ETYPE::ELLIPSE:
    case DRW::ETYPE::POLYLINE:
    case DRW::ETYPE::LWPOLYLINE:
        return true;
    default:
        return false;
    }
}

CadProcessVisualInfo buildProcessVisualInfo(const CadItem* item)
{
    CadProcessVisualInfo info;

    if (item == nullptr || item->m_excludedFromProcessing)
    {
        return info;
    }

    info.processOrder = item->m_processOrder;
    info.isReverse = item->m_isReverse;

    if (!isProcessVisualizable(item) || item->m_nativeEntity == nullptr)
    {
        return info;
    }

    QVector3D preferredAnchor;
    const bool hasGeometryAnchor = tryBuildGeometryAnchor(item, preferredAnchor);

    switch (item->m_type)
    {
    case DRW::ETYPE::LINE:
    {
        const DRW_Line* line = static_cast<const DRW_Line*>(item->m_nativeEntity);
        info.forwardStartPoint = QVector3D(line->basePoint.x, line->basePoint.y, line->basePoint.z);
        info.forwardEndPoint = QVector3D(line->secPoint.x, line->secPoint.y, line->secPoint.z);
        info.labelAnchor = (info.forwardStartPoint + info.forwardEndPoint) * 0.5f;
        info.direction = normalizeOrZero(info.forwardEndPoint - info.forwardStartPoint);
        break;
    }
    case DRW::ETYPE::ARC:
    {
        const DRW_Arc* arc = static_cast<const DRW_Arc*>(item->m_nativeEntity);
        info.forwardStartPoint = arcPointAt(arc, arc->staangle);
        info.forwardEndPoint = arcPointAt(arc, arc->endangle);
        info.labelAnchor = hasGeometryAnchor ? preferredAnchor : (info.forwardStartPoint + info.forwardEndPoint) * 0.5f;
        info.direction = arcTangentAt(arc, arc->staangle, false);
        break;
    }
    case DRW::ETYPE::CIRCLE:
    {
        const DRW_Circle* circle = static_cast<const DRW_Circle*>(item->m_nativeEntity);
        const double startParameter = effectiveCircleStartParameter(item);
        info.closedPath = true;
        info.forwardStartPoint = circlePointAt(circle, startParameter);
        info.forwardEndPoint = info.forwardStartPoint;
        info.labelAnchor = CadOcsGeometry::center(circle);
        info.direction = circleTangentAt(circle, startParameter, false);
        break;
    }
    case DRW::ETYPE::ELLIPSE:
    {
        const DRW_Ellipse* ellipse = static_cast<const DRW_Ellipse*>(item->m_nativeEntity);
        double startParam = ellipse->staparam;
        double endParam = ellipse->endparam;
        info.closedPath = isFullEllipsePath(ellipse);

        if (info.closedPath)
        {
            startParam = effectiveClosedEllipseStartParameter(item, ellipse);
            endParam = startParam;
        }
        else
        {
            while (endParam <= startParam)
            {
                endParam += kTwoPi;
            }
        }

        info.forwardStartPoint = ellipsePointAt(ellipse, startParam);
        info.forwardEndPoint = ellipsePointAt(ellipse, endParam);
        info.labelAnchor = info.closedPath
            ? QVector3D(ellipse->basePoint.x, ellipse->basePoint.y, ellipse->basePoint.z)
            : (hasGeometryAnchor ? preferredAnchor : (info.forwardStartPoint + info.forwardEndPoint) * 0.5f);
        info.direction = ellipseTangentAt(ellipse, startParam, false);
        break;
    }
    case DRW::ETYPE::POLYLINE:
    {
        const DRW_Polyline* polyline = static_cast<const DRW_Polyline*>(item->m_nativeEntity);

        if (polyline->vertlist.empty())
        {
            return info;
        }

        info.closedPath = (polyline->flags & 1) != 0;
        const size_t seamIndex = info.closedPath
            ? effectiveClosedPolylineStartIndex(item, polyline->vertlist.size())
            : 0;
        const auto& firstVertex = polyline->vertlist.at(seamIndex);
        const auto& lastVertex = polyline->vertlist.back();
        info.forwardStartPoint = polylineVertexToWcs(polyline, firstVertex);
        info.forwardEndPoint = info.closedPath
            ? info.forwardStartPoint
            : polylineVertexToWcs(polyline, lastVertex);
        info.labelAnchor = hasGeometryAnchor ? preferredAnchor : info.forwardStartPoint;
        info.direction = info.closedPath
            ? polylineForwardStartTangentAt(polyline, seamIndex)
            : polylineForwardStartTangent(polyline);
        break;
    }
    case DRW::ETYPE::LWPOLYLINE:
    {
        const DRW_LWPolyline* polyline = static_cast<const DRW_LWPolyline*>(item->m_nativeEntity);

        if (polyline->vertlist.empty())
        {
            return info;
        }

        info.closedPath = (polyline->flags & 1) != 0;
        const size_t seamIndex = info.closedPath
            ? effectiveClosedPolylineStartIndex(item, polyline->vertlist.size())
            : 0;
        const auto& firstVertex = polyline->vertlist.at(seamIndex);
        const auto& lastVertex = polyline->vertlist.back();
        info.forwardStartPoint = lwPolylineVertexToWcs(polyline, firstVertex);
        info.forwardEndPoint = info.closedPath
            ? info.forwardStartPoint
            : lwPolylineVertexToWcs(polyline, lastVertex);
        info.labelAnchor = hasGeometryAnchor ? preferredAnchor : info.forwardStartPoint;
        info.direction = info.closedPath
            ? lwPolylineForwardStartTangentAt(polyline, seamIndex)
            : lwPolylineForwardStartTangent(polyline);
        break;
    }
    default:
        return info;
    }

    info.startPoint = info.isReverse ? info.forwardEndPoint : info.forwardStartPoint;
    info.endPoint = info.isReverse ? info.forwardStartPoint : info.forwardEndPoint;

    if (info.isReverse)
    {
        switch (item->m_type)
        {
        case DRW::ETYPE::LINE:
            info.direction = normalizeOrZero(info.endPoint - info.startPoint);
            break;
        case DRW::ETYPE::ARC:
        {
            const DRW_Arc* arc = static_cast<const DRW_Arc*>(item->m_nativeEntity);
            info.direction = arcTangentAt(arc, arc->endangle, true);
            break;
        }
        case DRW::ETYPE::CIRCLE:
            info.direction = circleTangentAt(static_cast<const DRW_Circle*>(item->m_nativeEntity), effectiveCircleStartParameter(item), true);
            break;
        case DRW::ETYPE::ELLIPSE:
        {
            const DRW_Ellipse* ellipse = static_cast<const DRW_Ellipse*>(item->m_nativeEntity);
            const double parameter = info.closedPath
                ? effectiveClosedEllipseStartParameter(item, ellipse)
                : ellipse->endparam;
            info.direction = ellipseTangentAt(ellipse, parameter, true);
            break;
        }
        case DRW::ETYPE::POLYLINE:
        {
            const DRW_Polyline* polyline = static_cast<const DRW_Polyline*>(item->m_nativeEntity);
            info.direction = info.closedPath
                ? polylineReverseStartTangentAt(polyline, effectiveClosedPolylineStartIndex(item, polyline->vertlist.size()))
                : polylineReverseStartTangent(polyline);
            break;
        }
        case DRW::ETYPE::LWPOLYLINE:
        {
            const DRW_LWPolyline* polyline = static_cast<const DRW_LWPolyline*>(item->m_nativeEntity);
            info.direction = info.closedPath
                ? lwPolylineReverseStartTangentAt(polyline, effectiveClosedPolylineStartIndex(item, polyline->vertlist.size()))
                : lwPolylineReverseStartTangent(polyline);
            break;
        }
        default:
            break;
        }
    }

    if (info.direction.lengthSquared() <= kVisualEpsilon)
    {
        info.direction = normalizeOrZero(info.endPoint - info.startPoint);
    }

    if (info.direction.lengthSquared() <= kVisualEpsilon)
    {
        info.direction = normalizeOrZero(item->m_processDirection);
    }

    info.valid = true;
    return info;
}

QVector<CadSelectionHandleInfo> buildSelectionHandleInfo(const CadItem* item, float xlineHandleLength)
{
    QVector<CadSelectionHandleInfo> handles;

    if (item == nullptr || item->m_nativeEntity == nullptr)
    {
        return handles;
    }

    switch (item->m_type)
    {
    case DRW::ETYPE::POINT:
    {
        const DRW_Point* point = static_cast<const DRW_Point*>(item->m_nativeEntity);
        appendSelectionHandle(handles, QVector3D(point->basePoint.x, point->basePoint.y, point->basePoint.z), true, true, 0);
        break;
    }
    case DRW::ETYPE::LINE:
    {
        const DRW_Line* line = static_cast<const DRW_Line*>(item->m_nativeEntity);
        const QVector3D startPoint(line->basePoint.x, line->basePoint.y, line->basePoint.z);
        const QVector3D endPoint(line->secPoint.x, line->secPoint.y, line->secPoint.z);
        appendSelectionHandle(handles, startPoint, true, true, 0);
        appendSelectionHandle(handles, endPoint, false, true, 1);
        appendSelectionHandle(handles, (startPoint + endPoint) * 0.5f, false, true, 2);
        break;
    }
    case DRW::ETYPE::XLINE:
    {
        const DRW_Xline* xline = static_cast<const DRW_Xline*>(item->m_nativeEntity);
        QVector3D direction(xline->secPoint.x, xline->secPoint.y, xline->secPoint.z);

        if (direction.lengthSquared() <= kVisualEpsilon)
        {
            direction = QVector3D(1.0f, 0.0f, 0.0f);
        }
        else
        {
            direction.normalize();
        }

        const QVector3D basePoint(xline->basePoint.x, xline->basePoint.y, xline->basePoint.z);
        const float handleLength = std::isfinite(xlineHandleLength) && xlineHandleLength > kVisualEpsilon
            ? xlineHandleLength
            : 50.0f;
        appendSelectionHandle(handles, basePoint, true, true, 0);
        appendSelectionHandle(handles, basePoint + direction * handleLength, false, true, 1);
        appendSelectionHandle(handles, basePoint - direction * handleLength, false, true, 2);
        break;
    }
    case DRW::ETYPE::CIRCLE:
    {
        const DRW_Circle* circle = static_cast<const DRW_Circle*>(item->m_nativeEntity);
        appendSelectionHandle(handles, CadOcsGeometry::center(circle), true, true, 0);
        appendSelectionHandle(handles, circlePointAt(circle, 0.0), false, true, 1);
        appendSelectionHandle(handles, circlePointAt(circle, kPi * 0.5), false, true, 2);
        appendSelectionHandle(handles, circlePointAt(circle, kPi), false, true, 3);
        appendSelectionHandle(handles, circlePointAt(circle, kPi * 1.5), false, true, 4);
        break;
    }
    case DRW::ETYPE::ARC:
    {
        const DRW_Arc* arc = static_cast<const DRW_Arc*>(item->m_nativeEntity);
        appendSelectionHandle(handles, CadOcsGeometry::center(arc), true, true, 0);
        appendSelectionHandle
        (
            handles,
            arcPointAt(arc, arc->staangle),
            false,
            true,
            1,
            CadSelectionHandleShape::Triangle,
            arcTangentAt(arc, arc->staangle, false)
        );

        double endAngle = arc->endangle;

        while (endAngle <= arc->staangle)
        {
            endAngle += kTwoPi;
        }

        appendSelectionHandle(handles, arcPointAt(arc, (arc->staangle + endAngle) * 0.5), false, true, 2);
        appendSelectionHandle
        (
            handles,
            arcPointAt(arc, endAngle),
            false,
            true,
            3,
            CadSelectionHandleShape::Triangle,
            arcTangentAt(arc, endAngle, false)
        );
        break;
    }
    case DRW::ETYPE::ELLIPSE:
    {
        const DRW_Ellipse* ellipse = static_cast<const DRW_Ellipse*>(item->m_nativeEntity);
        QVector3D majorAxis;
        QVector3D minorAxis;

        if (!tryBuildEllipseAxes(ellipse, majorAxis, minorAxis))
        {
            return handles;
        }

        const QVector3D center(ellipse->basePoint.x, ellipse->basePoint.y, ellipse->basePoint.z);
        appendSelectionHandle(handles, center, true, true, 0);
        appendSelectionHandle(handles, center + majorAxis, false, true, 1);
        appendSelectionHandle(handles, center - majorAxis, false, true, 2);
        appendSelectionHandle(handles, center + minorAxis, false, true, 3);
        appendSelectionHandle(handles, center - minorAxis, false, true, 4);

        if (!isFullEllipsePath(ellipse))
        {
            appendSelectionHandle
            (
                handles,
                ellipsePointAt(ellipse, ellipse->staparam),
                false,
                true,
                5,
                CadSelectionHandleShape::Triangle,
                ellipseTangentAt(ellipse, ellipse->staparam, false)
            );
            appendSelectionHandle
            (
                handles,
                ellipsePointAt(ellipse, ellipse->endparam),
                false,
                true,
                6,
                CadSelectionHandleShape::Triangle,
                ellipseTangentAt(ellipse, ellipse->endparam, false)
            );
        }
        break;
    }
    case DRW::ETYPE::POLYLINE:
    {
        const DRW_Polyline* polyline = static_cast<const DRW_Polyline*>(item->m_nativeEntity);

        if (polyline->vertlist.empty())
        {
            return handles;
        }

        const auto& firstVertex = polyline->vertlist.front();
        appendSelectionHandle
        (
            handles,
            polylineVertexToWcs(polyline, firstVertex),
            true,
            true,
            0
        );

        for (size_t index = 1; index < polyline->vertlist.size(); ++index)
        {
            const auto& vertex = polyline->vertlist.at(index);
            appendSelectionHandle
            (
                handles,
                polylineVertexToWcs(polyline, vertex),
                false,
                true,
                static_cast<int>(index)
            );
        }
        break;
    }
    case DRW::ETYPE::LWPOLYLINE:
    {
        const DRW_LWPolyline* polyline = static_cast<const DRW_LWPolyline*>(item->m_nativeEntity);

        if (polyline->vertlist.empty())
        {
            return handles;
        }

        const auto& firstVertex = polyline->vertlist.front();
        appendSelectionHandle
        (
            handles,
            lwPolylineVertexToWcs(polyline, firstVertex),
            true,
            true,
            0
        );

        for (size_t index = 1; index < polyline->vertlist.size(); ++index)
        {
            const auto& vertex = polyline->vertlist.at(index);
            appendSelectionHandle
            (
                handles,
                lwPolylineVertexToWcs(polyline, vertex),
                false,
                true,
                static_cast<int>(index)
            );
        }
        break;
    }
    default:
        break;
    }

    return handles;
}
