#include "pch.h"

#include "CadXlineItem.h"

#include <algorithm>
#include <cmath>

namespace
{
constexpr double kGeometryEpsilon = 1.0e-9;
constexpr double kMinDisplayHalfLength = 65536.0;

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

double displayHalfLengthForXline(const DRW_Xline* xline)
{
    if (xline == nullptr)
    {
        return kMinDisplayHalfLength;
    }

    const double maxAbsCoordinate = std::max
    (
        {
            std::abs(xline->basePoint.x),
            std::abs(xline->basePoint.y),
            std::abs(xline->basePoint.z)
        }
    );

    return std::max(kMinDisplayHalfLength, maxAbsCoordinate * 8.0 + 4096.0);
}
}

CadXlineItem::CadXlineItem(DRW_Entity* entity, QObject* parent)
    : CadItem(entity, parent)
{
    m_data = static_cast<DRW_Xline*>(m_nativeEntity);
    buildGeometryDatay();
}

void CadXlineItem::buildGeometryDatay()
{
    m_geometry.vertices.clear();
    clearPathCaches();

    if (m_data == nullptr)
    {
        return;
    }

    const QVector3D origin(m_data->basePoint.x, m_data->basePoint.y, m_data->basePoint.z);
    const QVector3D direction = normalizedXlineDirection(m_data);
    const float halfLength = static_cast<float>(displayHalfLengthForXline(m_data));

    m_geometry.vertices.reserve(2);
    m_geometry.vertices.append(origin - direction * halfLength);
    m_geometry.vertices.append(origin + direction * halfLength);
}

void CadXlineItem::rebuildRawPathPoints3D()
{
    clearPathCaches();
}
