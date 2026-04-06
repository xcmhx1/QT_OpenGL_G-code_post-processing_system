// 实现 CadLWPolylineItem 模块，对应头文件中声明的主要行为和协作流程。
// 轻量多段线图元模块，负责轻量多段线的离散显示和 bulge 解释。
#include "pch.h"

#include "CadLWPolylineItem.h"

#include <cmath>

namespace
{
constexpr double kTwoPi = 6.28318530717958647692;
// 轻量多段线的 bulge 圆弧离散策略与普通多段线保持一致。
constexpr int kFullCircleSegments = 128;

void appendBulgeVertices(QVector<QVector3D>& vertices, const QVector3D& start, const QVector3D& end, double bulge)
{
    // 轻量多段线顶点本质仍是二维弦段，bulge 解释方式与普通多段线相同。
    const double dx = end.x() - start.x();
    const double dy = end.y() - start.y();
    const double chordLength = std::sqrt(dx * dx + dy * dy);

    if (chordLength <= 1.0e-10)
    {
        vertices.append(end);
        return;
    }

    // bulge 为 0 时直接视作线段终点。
    if (std::abs(bulge) < 1.0e-8)
    {
        vertices.append(end);
        return;
    }

    // 由 bulge 和弦长恢复圆心、半径以及扫角。
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
        // elevation 相同的情况下 z 通常不变，这里仍保留统一的线性插值写法。
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

CadLWPolylineItem::CadLWPolylineItem(DRW_Entity* entity, QObject* parent)
    : CadItem(entity, parent)
{
    // 绑定原生轻量多段线实体并立即准备好离散缓存。
    m_data = static_cast<DRW_LWPolyline*>(m_nativeEntity);
    buildGeometryDatay();
    buildProcessDirection();
}

void CadLWPolylineItem::buildGeometryDatay()
{
    m_geometry.vertices.clear();

    // 轻量多段线至少需要一个顶点才能输出显示结果。
    if (m_data == nullptr || m_data->vertlist.empty())
    {
        return;
    }

    const bool isClosed = (m_data->flags & 1) != 0;
    // 轻量多段线把 Z 统一存放在 elevation 字段中。
    const float z = static_cast<float>(m_data->elevation);
    // 把二维顶点提升为三维点，便于复用统一渲染通道。
    const auto toVertex = [z](const std::shared_ptr<DRW_Vertex2D>& vertex)
    {
        return QVector3D(static_cast<float>(vertex->x), static_cast<float>(vertex->y), z);
    };

    // 与普通多段线一致：先压入首点，后续逐段只追加终点方向的离散结果。
    m_geometry.vertices.reserve(static_cast<int>(m_data->vertlist.size()) + (isClosed ? 1 : 0));
    m_geometry.vertices.append(toVertex(m_data->vertlist.front()));

    for (size_t i = 0; i < m_data->vertlist.size(); ++i)
    {
        size_t nextIndex = i + 1;

        // 末段是否回接首点取决于闭合标记。
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

        // 当前 2D 顶点记录的是 current -> next 这一段的 bulge。
        appendBulgeVertices(m_geometry.vertices, toVertex(current), toVertex(next), current->bulge);
    }
}
