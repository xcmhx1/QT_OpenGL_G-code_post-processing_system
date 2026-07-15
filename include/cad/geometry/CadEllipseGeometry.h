#pragma once

#include "libdxfrw/drw_entities.h"

#include <QVector3D>

#include <cmath>

struct CadEllipseGeometry
{
    QVector3D center;
    QVector3D majorAxis;
    QVector3D minorAxis;
    QVector3D normal;
    double startParameter = 0.0;
    double endParameter = 0.0;
    bool full = false;
};

namespace CadEllipseGeometryUtils
{
    constexpr double kTwoPi = 6.28318530717958647692;

    inline bool isFullEllipseParameterRange
    (
        double startParameter,
        double endParameter,
        double tolerance = 1.0e-6
    )
    {
        const double span = endParameter - startParameter;
        return std::abs(span) <= tolerance
            || std::abs(std::abs(span) - kTwoPi) <= tolerance;
    }

    inline bool buildEllipseGeometry(const DRW_Ellipse* ellipse, CadEllipseGeometry& geometry)
    {
        if (ellipse == nullptr || ellipse->ratio <= 0.0)
        {
            return false;
        }

        geometry.center = QVector3D(ellipse->basePoint.x, ellipse->basePoint.y, ellipse->basePoint.z);
        geometry.majorAxis = QVector3D(ellipse->secPoint.x, ellipse->secPoint.y, ellipse->secPoint.z);
        geometry.normal = QVector3D(ellipse->extPoint.x, ellipse->extPoint.y, ellipse->extPoint.z);

        if (geometry.majorAxis.lengthSquared() <= 1.0e-12f)
        {
            return false;
        }

        if (geometry.normal.lengthSquared() <= 1.0e-12f)
        {
            geometry.normal = QVector3D(0.0f, 0.0f, 1.0f);
        }
        else
        {
            geometry.normal.normalize();
        }

        QVector3D minorDirection = QVector3D::crossProduct(geometry.normal, geometry.majorAxis);

        if (minorDirection.lengthSquared() <= 1.0e-12f)
        {
            return false;
        }

        minorDirection.normalize();
        geometry.minorAxis = minorDirection
            * static_cast<float>(geometry.majorAxis.length() * ellipse->ratio);
        geometry.startParameter = ellipse->staparam;
        geometry.endParameter = ellipse->endparam;
        geometry.full = isFullEllipseParameterRange
        (
            geometry.startParameter,
            geometry.endParameter
        );
        return true;
    }

    inline QVector3D ellipsePointAt(const CadEllipseGeometry& geometry, double parameter)
    {
        return geometry.center
            + geometry.majorAxis * static_cast<float>(std::cos(parameter))
            + geometry.minorAxis * static_cast<float>(std::sin(parameter));
    }

    inline QVector3D ellipseTangentAt
    (
        const CadEllipseGeometry& geometry,
        double parameter,
        bool reverse
    )
    {
        QVector3D tangent = geometry.majorAxis * static_cast<float>(-std::sin(parameter))
            + geometry.minorAxis * static_cast<float>(std::cos(parameter));

        if (reverse)
        {
            tangent = -tangent;
        }

        if (tangent.lengthSquared() > 1.0e-12f)
        {
            tangent.normalize();
        }

        return tangent;
    }
}
