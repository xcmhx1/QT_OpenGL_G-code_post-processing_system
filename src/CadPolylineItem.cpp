// 实现 CadPolylineItem 模块，对应头文件中声明的主要行为和协作流程。
// 多段线图元模块，负责多段线的离散显示和 bulge 圆弧段解释。
#include "pch.h"

#include "CadPolylineItem.h"

#include <cmath>

namespace
{
constexpr double kTwoPi = 6.28318530717958647692;
constexpr int kFullCircleSegments = 128;

void appendBulgeVertices(QVector<QVector3D>& vertices, const QVector3D& start, const QVector3D& end, double bulge)
{
    const double dx = end.x() - start.x();
    const double dy = end.y() - start.y();
    const double chordLength = std::sqrt(dx * dx + dy * dy);

    if (chordLength <= 1.0e-10)
    {
        vertices.append(end);
        return;
    }

    if (std::abs(bulge) < 1.0e-8)
    {
        vertices.append(end);
        return;
    }

    const double midpointX = (start.x() + end.x()) * 0.5;
    const double midpointY = (start.y() + end.y()) * 0.5;
    const double centerOffset = chordLength * (1.0 / bulge - bulge) * 0.25;
    const double centerX = midpointX - centerOffset * (dy / chordLength);
    const double centerY = midpointY + centerOffset * (dx / chordLength);
    const double radius = std::hypot(start.x() - centerX, start.y() - centerY);
    const double startAngle = std::atan2(start.y() - centerY, start.x() - centerX);
    const double sweepAngle = 4.0 * std::atan(bulge);
    const int segments = std::max(4, static_cast<int>(std::ceil(std::abs(sweepAngle) / kTwoPi * kFullCircleSegments)));

    for (int i = 1; i <= segments; ++i)
    {
        const double factor = static_cast<double>(i) / static_cast<double>(segments);
        const double angle = startAngle + sweepAngle * factor;
        const float z = start.z() + static_cast<float>((end.z() - start.z()) * factor);

        vertices.append
        (
            QVector3D
            (
                static_cast<float>(centerX + radius * std::cos(angle)),
                static_cast<float>(centerY + radius * std::sin(angle)),
                z
            )
        );
    }
}
}

CadPolylineItem::CadPolylineItem(DRW_Entity* entity, QObject* parent)
    : CadItem(entity, parent)
{
    m_data = static_cast<DRW_Polyline*>(m_nativeEntity);
    buildGeometryDatay();
    buildProcessDirection();
}

void CadPolylineItem::buildGeometryDatay()
{
    m_geometry.vertices.clear();

    if (m_data == nullptr || m_data->vertlist.empty())
    {
        return;
    }

    const bool isClosed = (m_data->flags & 1) != 0;
    const auto toVertex = [](const std::shared_ptr<DRW_Vertex>& vertex)
    {
        return QVector3D
        (
            static_cast<float>(vertex->basePoint.x),
            static_cast<float>(vertex->basePoint.y),
            static_cast<float>(vertex->basePoint.z)
        );
    };

    m_geometry.vertices.reserve(static_cast<int>(m_data->vertlist.size()) + (isClosed ? 1 : 0));
    m_geometry.vertices.append(toVertex(m_data->vertlist.front()));

    for (size_t i = 0; i < m_data->vertlist.size(); ++i)
    {
        size_t nextIndex = i + 1;

        if (nextIndex >= m_data->vertlist.size())
        {
            if (!isClosed)
            {
                break;
            }

            nextIndex = 0;
        }

        const auto& current = m_data->vertlist.at(i);
        const auto& next = m_data->vertlist.at(nextIndex);

        appendBulgeVertices(m_geometry.vertices, toVertex(current), toVertex(next), current->bulge);
    }
}
