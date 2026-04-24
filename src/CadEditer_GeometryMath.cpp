// CadEditer 实现文件
// 实现 CadEditer 模块，对应头文件中声明的主要行为和协作流程。
// 编辑器模块，负责绘图创建、实体修改以及 Undo/Redo 命令栈管理。
#include "pch.h"

#include "CadEditerWorkflowInternal.h"

#include <cmath>

namespace CadEditerWorkflowInternal
{
    // 将任意点压回二维绘图平面
    // @param point 输入三维点
    // @return 压平后的点
    QVector3D flattenToDrawingPlane(const QVector3D& point)
    {
        return QVector3D(point.x(), point.y(), 0.0f);
    }

    QVector3D normalizedXlineDirection(const DRW_Xline* xline)
    {
        if (xline == nullptr)
        {
            return QVector3D(1.0f, 0.0f, 0.0f);
        }

        QVector3D direction(xline->secPoint.x, xline->secPoint.y, xline->secPoint.z);

        if (direction.lengthSquared() <= kGeometryEpsilon)
        {
            return QVector3D(1.0f, 0.0f, 0.0f);
        }

        direction.normalize();
        return direction;
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

    QVector3D resolveEntityNormal(const DRW_Coord& extPoint)
    {
        QVector3D normal(extPoint.x, extPoint.y, extPoint.z);

        if (normal.lengthSquared() <= kGeometryEpsilon)
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

            if (axisY.lengthSquared() <= kGeometryEpsilon)
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

        if (axisX.lengthSquared() <= kGeometryEpsilon)
        {
            axisX = QVector3D(1.0f, 0.0f, 0.0f);
        }
        else
        {
            axisX.normalize();
        }

        axisY = QVector3D::crossProduct(normal, axisX);

        if (axisY.lengthSquared() <= kGeometryEpsilon)
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

        const QVector3D center(circle->basePoint.x, circle->basePoint.y, circle->basePoint.z);
        const QVector3D normal = resolveEntityNormal(circle->extPoint);
        QVector3D axisX;
        QVector3D axisY;
        buildPlaneBasis(normal, axisX, axisY);

        return center
            + axisX * static_cast<float>(std::cos(parameter) * circle->radious)
            + axisY * static_cast<float>(std::sin(parameter) * circle->radious);
    }

    QVector3D arcPointAt(const DRW_Arc* arc, double angle)
    {
        if (arc == nullptr || arc->radious <= 0.0)
        {
            return QVector3D();
        }

        const QVector3D center(arc->basePoint.x, arc->basePoint.y, arc->basePoint.z);
        const QVector3D normal = resolveEntityNormal(arc->extPoint);
        QVector3D axisX;
        QVector3D axisY;
        buildPlaneBasis(normal, axisX, axisY);

        return center
            + axisX * static_cast<float>(std::cos(angle) * arc->radious)
            + axisY * static_cast<float>(std::sin(angle) * arc->radious);
    }

    double arcMidAngle(const DRW_Arc* arc)
    {
        if (arc == nullptr)
        {
            return 0.0;
        }

        double endAngle = arc->endangle;

        while (endAngle <= arc->staangle)
        {
            endAngle += kTwoPi;
        }

        return (arc->staangle + endAngle) * 0.5;
    }

    double angleFromPointOnCircle(const DRW_Circle* circle, const QVector3D& point, bool* valid)
    {
        if (valid != nullptr)
        {
            *valid = false;
        }

        if (circle == nullptr)
        {
            return 0.0;
        }

        const QVector3D center(circle->basePoint.x, circle->basePoint.y, circle->basePoint.z);
        const QVector3D local = flattenToDrawingPlane(point) - center;

        if (local.lengthSquared() <= kGeometryEpsilon)
        {
            return 0.0;
        }

        const QVector3D normal = resolveEntityNormal(circle->extPoint);
        QVector3D axisX;
        QVector3D axisY;
        buildPlaneBasis(normal, axisX, axisY);

        const double x = QVector3D::dotProduct(local, axisX);
        const double y = QVector3D::dotProduct(local, axisY);

        if (std::abs(x) <= kGeometryEpsilon && std::abs(y) <= kGeometryEpsilon)
        {
            return 0.0;
        }

        if (valid != nullptr)
        {
            *valid = true;
        }

        return std::atan2(y, x);
    }

    bool tryBuildEllipseAxes(const DRW_Ellipse* ellipse, QVector3D& majorAxis, QVector3D& minorAxis)
    {
        if (ellipse == nullptr)
        {
            return false;
        }

        majorAxis = QVector3D(ellipse->secPoint.x, ellipse->secPoint.y, ellipse->secPoint.z);
        const double majorLength = majorAxis.length();

        if (majorLength <= kGeometryEpsilon || ellipse->ratio <= kMinEllipseRatio)
        {
            return false;
        }

        const QVector3D normal = resolveEntityNormal(ellipse->extPoint);
        minorAxis = QVector3D::crossProduct(normal, majorAxis);

        if (minorAxis.lengthSquared() <= kGeometryEpsilon)
        {
            return false;
        }

        minorAxis.normalize();
        minorAxis *= static_cast<float>(majorLength * ellipse->ratio);
        return true;
    }

    QVector3D ellipsePointAt(const DRW_Ellipse* ellipse, double parameter)
    {
        if (ellipse == nullptr)
        {
            return QVector3D();
        }

        QVector3D majorAxis;
        QVector3D minorAxis;

        if (!tryBuildEllipseAxes(ellipse, majorAxis, minorAxis))
        {
            return QVector3D();
        }

        const QVector3D center(ellipse->basePoint.x, ellipse->basePoint.y, ellipse->basePoint.z);
        return center
            + majorAxis * static_cast<float>(std::cos(parameter))
            + minorAxis * static_cast<float>(std::sin(parameter));
    }

    bool ellipseParameterFromPoint(const DRW_Ellipse* ellipse, const QVector3D& worldPoint, double& parameter)
    {
        QVector3D majorAxis;
        QVector3D minorAxis;

        if (ellipse == nullptr || !tryBuildEllipseAxes(ellipse, majorAxis, minorAxis))
        {
            return false;
        }

        const double majorLength = majorAxis.length();
        const double minorLength = minorAxis.length();

        if (majorLength <= kGeometryEpsilon || minorLength <= kGeometryEpsilon)
        {
            return false;
        }

        QVector3D majorUnit = majorAxis;
        QVector3D minorUnit = minorAxis;
        majorUnit.normalize();
        minorUnit.normalize();

        const QVector3D center(ellipse->basePoint.x, ellipse->basePoint.y, ellipse->basePoint.z);
        const QVector3D local = flattenToDrawingPlane(worldPoint) - center;
        const double x = QVector3D::dotProduct(local, majorUnit);
        const double y = QVector3D::dotProduct(local, minorUnit);
        const double cosValue = x / majorLength;
        const double sinValue = y / minorLength;
        parameter = std::atan2(sinValue, cosValue);
        return true;
    }
}
