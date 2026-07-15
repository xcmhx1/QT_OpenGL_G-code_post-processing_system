#pragma once

#include "libdxfrw/drw_entities.h"

#include <QVector3D>

#include <cmath>

namespace CadOcsGeometry
{
    constexpr float kEpsilon = 1.0e-12f;
    constexpr double kArbitraryAxisThreshold = 1.0 / 64.0;

    inline QVector3D normal(const DRW_Coord& extrusion)
    {
        QVector3D result(extrusion.x, extrusion.y, extrusion.z);
        if (result.lengthSquared() <= kEpsilon)
        {
            return QVector3D(0.0f, 0.0f, 1.0f);
        }
        result.normalize();
        return result;
    }

    inline void basis(const DRW_Coord& extrusion, QVector3D& axisX, QVector3D& axisY, QVector3D& axisZ)
    {
        axisZ = normal(extrusion);

        // DXF arbitrary-axis algorithm. The 1/64 threshold is defined by DXF.
        if (std::abs(axisZ.x()) < kArbitraryAxisThreshold
            && std::abs(axisZ.y()) < kArbitraryAxisThreshold)
        {
            axisX = QVector3D(axisZ.z(), 0.0f, -axisZ.x());
        }
        else
        {
            axisX = QVector3D(-axisZ.y(), axisZ.x(), 0.0f);
        }

        if (axisX.lengthSquared() <= kEpsilon)
        {
            axisX = QVector3D(1.0f, 0.0f, 0.0f);
        }
        else
        {
            axisX.normalize();
        }

        axisY = QVector3D::crossProduct(axisZ, axisX);
        if (axisY.lengthSquared() <= kEpsilon)
        {
            axisY = QVector3D(0.0f, 1.0f, 0.0f);
        }
        else
        {
            axisY.normalize();
        }
    }

    inline QVector3D pointToWorld(const DRW_Coord& point, const DRW_Coord& extrusion)
    {
        QVector3D axisX;
        QVector3D axisY;
        QVector3D axisZ;
        basis(extrusion, axisX, axisY, axisZ);
        return axisX * static_cast<float>(point.x)
            + axisY * static_cast<float>(point.y)
            + axisZ * static_cast<float>(point.z);
    }

    inline QVector3D center(const DRW_Circle* circle)
    {
        return circle != nullptr ? pointToWorld(circle->basePoint, circle->extPoint) : QVector3D();
    }

    inline QVector3D pointAt(const DRW_Circle* circle, double parameter)
    {
        if (circle == nullptr || circle->radious <= 0.0)
        {
            return center(circle);
        }

        QVector3D axisX;
        QVector3D axisY;
        QVector3D axisZ;
        basis(circle->extPoint, axisX, axisY, axisZ);
        return center(circle)
            + axisX * static_cast<float>(std::cos(parameter) * circle->radious)
            + axisY * static_cast<float>(std::sin(parameter) * circle->radious);
    }

    inline QVector3D tangentAt(const DRW_Circle* circle, double parameter, bool reverse)
    {
        if (circle == nullptr || circle->radious <= 0.0)
        {
            return QVector3D();
        }

        QVector3D axisX;
        QVector3D axisY;
        QVector3D axisZ;
        basis(circle->extPoint, axisX, axisY, axisZ);
        QVector3D tangent = axisX * static_cast<float>(-std::sin(parameter))
            + axisY * static_cast<float>(std::cos(parameter));
        if (reverse)
        {
            tangent = -tangent;
        }
        if (tangent.lengthSquared() > kEpsilon)
        {
            tangent.normalize();
        }
        return tangent;
    }
}
