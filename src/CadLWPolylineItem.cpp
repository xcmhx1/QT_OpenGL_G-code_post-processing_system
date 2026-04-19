// 实现 CadLWPolylineItem 模块，对应头文件中声明的主要行为和协作流程。
// 轻量多段线图元模块，负责轻量多段线的离散显示和 bulge 解释。
#include "pch.h"

#include "CadLWPolylineItem.h"

#include <cmath>

namespace
{
constexpr double kAxisEps = 1.0e-8;
constexpr double kRadToDeg = 57.2957795130823208768;
constexpr double kTwoPi = 6.28318530717958647692;
// 轻量多段线的 bulge 圆弧离散策略与普通多段线保持一致。
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
    clearPathCaches();

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

void CadLWPolylineItem::rebuildRawPathPoints3D()
{
    m_rawPathPoints3D.clear();

    if (m_data == nullptr || m_data->vertlist.empty())
    {
        return;
    }

    const bool isClosed = (m_data->flags & 1) != 0;
    const float z = static_cast<float>(m_data->elevation);
    const auto toVertex = [z](const std::shared_ptr<DRW_Vertex2D>& vertex)
    {
        return QVector3D(static_cast<float>(vertex->x), static_cast<float>(vertex->y), z);
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

bool CadLWPolylineItem::rebuildControlPoints4Axis
(
    double axisY,
    double axisZ,
    bool invertAAxisDirection,
    double aAxisOffsetDegrees,
    bool keepContinuousAngle,
    QString* errorMessage
)
{
    clearPathCaches();
    rebuildRawPathPoints3D();

    if (m_rawPathPoints3D.empty())
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("轻量多段线原始路径点集为空。");
        }

        return false;
    }

    m_controlPoints4Axis.reserve(m_rawPathPoints3D.size());

    bool hasPrevious = false;
    double previousA = 0.0;

    for (const RawPathPoint3D& point : m_rawPathPoints3D)
    {
        const double dy = point.y - axisY;
        const double dz = point.z - axisZ;

        double aDeg = 0.0;

        if (dy * dy + dz * dz < kAxisEps * kAxisEps)
        {
            aDeg = hasPrevious ? previousA : 0.0;
        }
        else
        {
            double rawA = std::atan2(dy, dz) * kRadToDeg;

            if (invertAAxisDirection)
            {
                rawA = -rawA;
            }

            rawA += aAxisOffsetDegrees;
            rawA = normalizeAngle180(rawA);
            aDeg = (hasPrevious && keepContinuousAngle) ? unwrapAngleNear(previousA, rawA) : rawA;
        }

        m_controlPoints4Axis.push_back({ point.x, point.y, point.z, aDeg });
        previousA = aDeg;
        hasPrevious = true;
    }

    if (errorMessage != nullptr)
    {
        errorMessage->clear();
    }

    return true;
}
