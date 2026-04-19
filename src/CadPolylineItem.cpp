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

void appendRawPathPoint(std::vector<RawPathPoint3D>& points, const QVector3D& point)
{
    if (!points.empty())
    {
        const RawPathPoint3D& lastPoint = points.back();
        const double dx = point.x() - lastPoint.x;
        const double dy = point.y() - lastPoint.y;
        const double dz = point.z() - lastPoint.z;

        if (dx * dx + dy * dy + dz * dz <= 1.0e-16)
        {
            return;
        }
    }

    points.push_back({ point.x(), point.y(), point.z() });
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

void appendBulgeRawPathPoints(std::vector<RawPathPoint3D>& points, const QVector3D& start, const QVector3D& end, double bulge)
{
    const double dx = end.x() - start.x();
    const double dy = end.y() - start.y();
    const double chordLength = std::sqrt(dx * dx + dy * dy);

    if (chordLength <= 1.0e-10 || std::abs(bulge) < 1.0e-8)
    {
        appendRawPathPoint(points, end);
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

    for (int index = 1; index <= segments; ++index)
    {
        const double factor = static_cast<double>(index) / static_cast<double>(segments);
        const double angle = startAngle + sweepAngle * factor;
        const QVector3D point
        (
            static_cast<float>(centerX + radius * std::cos(angle)),
            static_cast<float>(centerY + radius * std::sin(angle)),
            start.z() + static_cast<float>((end.z() - start.z()) * factor)
        );
        appendRawPathPoint(points, point);
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
    clearPathCaches();

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

void CadPolylineItem::rebuildRawPathPoints3D()
{
    m_rawPathPoints3D.clear();

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
    const size_t vertexCount = m_data->vertlist.size();
    m_rawPathPoints3D.reserve(vertexCount + 1);

    if (isClosed)
    {
        const size_t startIndex = effectiveClosedPolylineStartIndex(this, vertexCount);
        appendRawPathPoint(m_rawPathPoints3D, toVertex(m_data->vertlist.at(startIndex)));

        if (m_isReverse)
        {
            for (size_t step = 0; step < vertexCount; ++step)
            {
                const size_t currentIndex = (startIndex + vertexCount - (step % vertexCount)) % vertexCount;
                const size_t previousIndex = (currentIndex + vertexCount - 1) % vertexCount;
                appendBulgeRawPathPoints
                (
                    m_rawPathPoints3D,
                    toVertex(m_data->vertlist.at(currentIndex)),
                    toVertex(m_data->vertlist.at(previousIndex)),
                    -m_data->vertlist.at(previousIndex)->bulge
                );
            }
        }
        else
        {
            for (size_t step = 0; step < vertexCount; ++step)
            {
                const size_t currentIndex = (startIndex + step) % vertexCount;
                const size_t nextIndex = (currentIndex + 1) % vertexCount;
                appendBulgeRawPathPoints
                (
                    m_rawPathPoints3D,
                    toVertex(m_data->vertlist.at(currentIndex)),
                    toVertex(m_data->vertlist.at(nextIndex)),
                    m_data->vertlist.at(currentIndex)->bulge
                );
            }
        }

        return;
    }

    if (m_isReverse)
    {
        appendRawPathPoint(m_rawPathPoints3D, toVertex(m_data->vertlist.back()));

        for (size_t index = vertexCount - 1; index > 0; --index)
        {
            appendBulgeRawPathPoints
            (
                m_rawPathPoints3D,
                toVertex(m_data->vertlist.at(index)),
                toVertex(m_data->vertlist.at(index - 1)),
                -m_data->vertlist.at(index - 1)->bulge
            );
        }
    }
    else
    {
        appendRawPathPoint(m_rawPathPoints3D, toVertex(m_data->vertlist.front()));

        for (size_t index = 0; index + 1 < vertexCount; ++index)
        {
            appendBulgeRawPathPoints
            (
                m_rawPathPoints3D,
                toVertex(m_data->vertlist.at(index)),
                toVertex(m_data->vertlist.at(index + 1)),
                m_data->vertlist.at(index)->bulge
            );
        }
    }
}
