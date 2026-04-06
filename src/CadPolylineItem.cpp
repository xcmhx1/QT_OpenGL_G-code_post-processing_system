// 实现 CadPolylineItem 模块，对应头文件中声明的主要行为和协作流程。
// 多段线图元模块，负责多段线的离散显示和 bulge 圆弧段解释。
#include "pch.h"

#include "CadPolylineItem.h"

#include <cmath>

namespace
{
constexpr double kTwoPi = 6.28318530717958647692;
// bulge 圆弧展开时参考整圆采样密度。
constexpr int kFullCircleSegments = 128;

void appendBulgeVertices(QVector<QVector3D>& vertices, const QVector3D& start, const QVector3D& end, double bulge)
{
    // bulge 只在 XY 平面定义，Z 则在线性插值时单独处理。
    const double dx = end.x() - start.x();
    const double dy = end.y() - start.y();
    const double chordLength = std::sqrt(dx * dx + dy * dy);

    // 退化为零长度弦时，只保留终点即可。
    if (chordLength <= 1.0e-10)
    {
        vertices.append(end);
        return;
    }

    // bulge 接近 0 时等价于直线段。
    if (std::abs(bulge) < 1.0e-8)
    {
        vertices.append(end);
        return;
    }

    // bulge = tan(theta / 4)，这里先由弦求圆心，再恢复扫角。
    const double midpointX = (start.x() + end.x()) * 0.5;
    const double midpointY = (start.y() + end.y()) * 0.5;
    const double centerOffset = chordLength * (1.0 / bulge - bulge) * 0.25;
    const double centerX = midpointX - centerOffset * (dy / chordLength);
    const double centerY = midpointY + centerOffset * (dx / chordLength);
    const double radius = std::hypot(start.x() - centerX, start.y() - centerY);
    const double startAngle = std::atan2(start.y() - centerY, start.x() - centerX);
    const double sweepAngle = 4.0 * std::atan(bulge);
    // 按扫角大小决定采样段数，弧越长采样越密。
    const int segments = std::max(4, static_cast<int>(std::ceil(std::abs(sweepAngle) / kTwoPi * kFullCircleSegments)));

    for (int i = 1; i <= segments; ++i)
    {
        const double factor = static_cast<double>(i) / static_cast<double>(segments);
        const double angle = startAngle + sweepAngle * factor;
        // 2D 圆弧展开时仍保留起终点间的 Z 线性变化。
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
    // 绑定原生多段线实体，并立即完成离散缓存构建。
    m_data = static_cast<DRW_Polyline*>(m_nativeEntity);
    buildGeometryDatay();
    buildProcessDirection();
}

void CadPolylineItem::buildGeometryDatay()
{
    m_geometry.vertices.clear();

    // 没有顶点时无法形成可绘制折线。
    if (m_data == nullptr || m_data->vertlist.empty())
    {
        return;
    }

    // flags 的最低位表示闭合多段线。
    const bool isClosed = (m_data->flags & 1) != 0;
    // 把 libdxfrw 顶点对象转换成项目内部统一的 QVector3D。
    const auto toVertex = [](const std::shared_ptr<DRW_Vertex>& vertex)
    {
        return QVector3D
        (
            static_cast<float>(vertex->basePoint.x),
            static_cast<float>(vertex->basePoint.y),
            static_cast<float>(vertex->basePoint.z)
        );
    };

    // 首点先入列，后续每段只追加“终点方向”的离散结果，避免重复插入段起点。
    m_geometry.vertices.reserve(static_cast<int>(m_data->vertlist.size()) + (isClosed ? 1 : 0));
    m_geometry.vertices.append(toVertex(m_data->vertlist.front()));

    for (size_t i = 0; i < m_data->vertlist.size(); ++i)
    {
        size_t nextIndex = i + 1;

        // 非闭合多段线走到最后一个顶点时结束；闭合多段线则回接到首点。
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

        // 每一段的 bulge 存在当前顶点上，表示 current -> next 的圆弧性质。
        appendBulgeVertices(m_geometry.vertices, toVertex(current), toVertex(next), current->bulge);
    }
}
