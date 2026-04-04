#include "pch.h"

#include "CadEllipseItem.h"

#include <cmath>

namespace
{
constexpr double kTwoPi = 6.28318530717958647692;
constexpr int kFullEllipseSegments = 128;

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

QVector3D buildMinorAxis(const QVector3D& normal, const QVector3D& majorAxis, double ratio)
{
    QVector3D minorDirection = QVector3D::crossProduct(normal, majorAxis);

    if (minorDirection.lengthSquared() <= 1.0e-12f)
    {
        const QVector3D helper = std::abs(majorAxis.z()) < 0.999f
            ? QVector3D(0.0f, 0.0f, 1.0f)
            : QVector3D(0.0f, 1.0f, 0.0f);

        minorDirection = QVector3D::crossProduct(helper, majorAxis);
    }

    if (minorDirection.lengthSquared() <= 1.0e-12f)
    {
        return QVector3D();
    }

    minorDirection.normalize();
    return minorDirection * static_cast<float>(majorAxis.length() * ratio);
}
}

CadEllipseItem::CadEllipseItem(DRW_Entity* entity, QObject* parent)
    : CadItem(entity, parent)
{
    m_data = static_cast<DRW_Ellipse*>(m_nativeEntity);
    buildGeometryDatay();
    buildProcessDirection();
}

void CadEllipseItem::buildGeometryDatay()
{
    m_geometry.vertices.clear();

    if (m_data == nullptr)
    {
        return;
    }

    const QVector3D center(m_data->basePoint.x, m_data->basePoint.y, m_data->basePoint.z);
    const QVector3D majorAxis(m_data->secPoint.x, m_data->secPoint.y, m_data->secPoint.z);

    if (majorAxis.lengthSquared() <= 1.0e-12f || m_data->ratio <= 0.0)
    {
        return;
    }

    const QVector3D normal = resolveNormal(m_data->extPoint);
    const QVector3D minorAxis = buildMinorAxis(normal, majorAxis, m_data->ratio);

    if (minorAxis.lengthSquared() <= 1.0e-12f)
    {
        return;
    }

    double startParam = m_data->staparam;
    double endParam = m_data->endparam;

    if (std::abs(endParam - startParam) < 1.0e-10 || std::abs(std::abs(endParam - startParam) - kTwoPi) < 1.0e-10)
    {
        endParam = startParam + kTwoPi;
    }

    while (endParam <= startParam)
    {
        endParam += kTwoPi;
    }

    const double span = endParam - startParam;
    const int segments = std::max(16, static_cast<int>(std::ceil(span / kTwoPi * kFullEllipseSegments)));

    m_geometry.vertices.reserve(segments + 1);

    for (int i = 0; i <= segments; ++i)
    {
        const double t = startParam + span * static_cast<double>(i) / static_cast<double>(segments);
        const QVector3D point =
            center +
            majorAxis * static_cast<float>(std::cos(t)) +
            minorAxis * static_cast<float>(std::sin(t));

        m_geometry.vertices.append(point);
    }
}
