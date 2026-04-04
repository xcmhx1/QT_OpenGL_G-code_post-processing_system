#include "pch.h"

#include "CadArcItem.h"

#include <cmath>

namespace
{
constexpr double kTwoPi = 6.28318530717958647692;
constexpr int kFullCircleSegments = 128;

QVector3D resolveNormal(const DRW_Coord& extPoint)
{
    QVector3D normal(extPoint.x, extPoint.y, extPoint.z);

    if (normal.lengthSquared() <= 1.0e-12f)
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

        if (axisY.lengthSquared() <= 1.0e-12f)
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

    if (axisX.lengthSquared() <= 1.0e-12f)
    {
        axisX = QVector3D(1.0f, 0.0f, 0.0f);
    }
    else
    {
        axisX.normalize();
    }

    axisY = QVector3D::crossProduct(normal, axisX);

    if (axisY.lengthSquared() <= 1.0e-12f)
    {
        axisY = QVector3D(0.0f, 1.0f, 0.0f);
    }
    else
    {
        axisY.normalize();
    }
}
}

CadArcItem::CadArcItem(DRW_Entity* entity, QObject* parent)
    : CadItem(entity, parent)
{
    m_data = static_cast<DRW_Arc*>(m_nativeEntity);
    buildGeometryDatay();
    buildProcessDirection();
}

void CadArcItem::buildGeometryDatay()
{
    m_geometry.vertices.clear();

    if (m_data == nullptr || m_data->radious <= 0.0)
    {
        return;
    }

    const QVector3D center(m_data->basePoint.x, m_data->basePoint.y, m_data->basePoint.z);
    const QVector3D normal = resolveNormal(m_data->extPoint);

    QVector3D axisX;
    QVector3D axisY;
    buildPlaneBasis(normal, axisX, axisY);

    double startAngle = m_data->staangle;
    double endAngle = m_data->endangle;

    while (endAngle <= startAngle)
    {
        endAngle += kTwoPi;
    }

    const double span = endAngle - startAngle;
    const int segments = std::max(16, static_cast<int>(std::ceil(span / kTwoPi * kFullCircleSegments)));

    m_geometry.vertices.reserve(segments + 1);

    for (int i = 0; i <= segments; ++i)
    {
        const double t = startAngle + span * static_cast<double>(i) / static_cast<double>(segments);
        const QVector3D offset =
            axisX * static_cast<float>(std::cos(t) * m_data->radious) +
            axisY * static_cast<float>(std::sin(t) * m_data->radious);

        m_geometry.vertices.append(center + offset);
    }
}
