// 实现 CadPolylineItem 模块，对应头文件中声明的主要行为和协作流程。
// 多段线图元模块，负责多段线的离散显示和 bulge 圆弧段解释。
#include "pch.h"

#include "CadPolylineItem.h"

#include <cmath>

namespace
{

    constexpr double kAxisEps = 1.0e-8;
    constexpr double kRadToDeg = 57.2957795130823208768;
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
            const int normalized =
                ((rawIndex % static_cast<int>(vertexCount)) + static_cast<int>(vertexCount))
                % static_cast<int>(vertexCount);
            return static_cast<size_t>(normalized);
        }

        return 0;
    }

    QVector3D resolvePolylineNormal(const DRW_Coord& extPoint)
    {
        QVector3D normal(extPoint.x, extPoint.y, extPoint.z);

        if (normal.lengthSquared() <= 1.0e-12f)
        {
            return QVector3D();
        }

        normal.normalize();
        return normal;
    }

    QVector3D inferPlaneNormalFromVertices(const std::vector<std::shared_ptr<DRW_Vertex>>& vertices)
    {
        if (vertices.size() < 3)
        {
            return QVector3D();
        }

        const QVector3D p0
        (
            static_cast<float>(vertices.front()->basePoint.x),
            static_cast<float>(vertices.front()->basePoint.y),
            static_cast<float>(vertices.front()->basePoint.z)
        );

        for (size_t i = 1; i + 1 < vertices.size(); ++i)
        {
            const QVector3D p1
            (
                static_cast<float>(vertices[i]->basePoint.x),
                static_cast<float>(vertices[i]->basePoint.y),
                static_cast<float>(vertices[i]->basePoint.z)
            );
            const QVector3D p2
            (
                static_cast<float>(vertices[i + 1]->basePoint.x),
                static_cast<float>(vertices[i + 1]->basePoint.y),
                static_cast<float>(vertices[i + 1]->basePoint.z)
            );

            QVector3D n = QVector3D::crossProduct(p1 - p0, p2 - p0);
            if (n.lengthSquared() > 1.0e-12f)
            {
                n.normalize();
                return n;
            }
        }

        return QVector3D();
    }

    void buildPlaneBasis(const QVector3D& normal, QVector3D& axisU, QVector3D& axisV)
    {
        const QVector3D helper = std::abs(normal.z()) < 0.999f
            ? QVector3D(0.0f, 0.0f, 1.0f)
            : QVector3D(0.0f, 1.0f, 0.0f);

        axisU = QVector3D::crossProduct(helper, normal);

        if (axisU.lengthSquared() <= 1.0e-12f)
        {
            axisU = QVector3D(1.0f, 0.0f, 0.0f);
        }
        else
        {
            axisU.normalize();
        }

        axisV = QVector3D::crossProduct(normal, axisU);

        if (axisV.lengthSquared() <= 1.0e-12f)
        {
            axisV = QVector3D(0.0f, 1.0f, 0.0f);
        }
        else
        {
            axisV.normalize();
        }
    }

    bool is3DPolyline(const DRW_Polyline* polyline)
    {
        return polyline != nullptr && ((polyline->flags & 8) != 0);
    }

    bool buildPolylinePlaneBasis(const DRW_Polyline* polyline,
        QVector3D& origin,
        QVector3D& axisU,
        QVector3D& axisV,
        QVector3D& normal)
    {
        if (polyline == nullptr || polyline->vertlist.empty())
        {
            return false;
        }

        if (is3DPolyline(polyline))
        {
            normal = inferPlaneNormalFromVertices(polyline->vertlist);

            if (normal.lengthSquared() <= 1.0e-12f)
            {
                return false;
            }

            origin = QVector3D
            (
                static_cast<float>(polyline->vertlist.front()->basePoint.x),
                static_cast<float>(polyline->vertlist.front()->basePoint.y),
                static_cast<float>(polyline->vertlist.front()->basePoint.z)
            );

            buildPlaneBasis(normal, axisU, axisV);
            return true;
        }

        // 2D polyline：直接按 OCS 平面构造
        normal = resolvePolylineNormal(polyline->extPoint);
        if (normal.lengthSquared() <= 1.0e-12f)
        {
            normal = QVector3D(0.0f, 0.0f, 1.0f);
        }

        buildPlaneBasis(normal, axisU, axisV);
        origin = normal * static_cast<float>(polyline->basePoint.z);
        return true;
    }

    void projectPointToPlaneUV(const QVector3D& origin,
        const QVector3D& axisU,
        const QVector3D& axisV,
        const QVector3D& point,
        double& u,
        double& v)
    {
        const QVector3D d = point - origin;
        u = QVector3D::dotProduct(d, axisU);
        v = QVector3D::dotProduct(d, axisV);
    }

    QVector3D unprojectPointFromPlaneUV(const QVector3D& origin,
        const QVector3D& axisU,
        const QVector3D& axisV,
        const QVector3D& normal,
        double u,
        double v,
        double h)
    {
        return origin
            + axisU * static_cast<float>(u)
            + axisV * static_cast<float>(v)
            + normal * static_cast<float>(h);
    }

    void appendBulgeVertices(QVector<QVector3D>& vertices,
        const QVector3D& origin,
        const QVector3D& axisU,
        const QVector3D& axisV,
        const QVector3D& normal,
        const QVector3D& start,
        const QVector3D& end,
        double bulge)
    {
        double su = 0.0;
        double sv = 0.0;
        double eu = 0.0;
        double ev = 0.0;

        projectPointToPlaneUV(origin, axisU, axisV, start, su, sv);
        projectPointToPlaneUV(origin, axisU, axisV, end, eu, ev);

        const double dx = eu - su;
        const double dy = ev - sv;
        const double chordLength = std::sqrt(dx * dx + dy * dy);

        if (chordLength <= 1.0e-10 || std::abs(bulge) < 1.0e-8)
        {
            vertices.append(end);
            return;
        }

        const double midpointU = (su + eu) * 0.5;
        const double midpointV = (sv + ev) * 0.5;
        const double centerOffset = chordLength * (1.0 / bulge - bulge) * 0.25;
        const double centerU = midpointU - centerOffset * (dy / chordLength);
        const double centerV = midpointV + centerOffset * (dx / chordLength);
        const double radius = std::hypot(su - centerU, sv - centerV);
        const double startAngle = std::atan2(sv - centerV, su - centerU);
        const double sweepAngle = 4.0 * std::atan(bulge);
        const int segments = std::max
        (
            4,
            static_cast<int>(std::ceil(std::abs(sweepAngle) / kTwoPi * kFullCircleSegments))
        );

        const double startHeight = QVector3D::dotProduct(start - origin, normal);
        const double endHeight = QVector3D::dotProduct(end - origin, normal);

        for (int i = 1; i <= segments; ++i)
        {
            const double factor = static_cast<double>(i) / static_cast<double>(segments);
            const double angle = startAngle + sweepAngle * factor;
            const double u = centerU + radius * std::cos(angle);
            const double v = centerV + radius * std::sin(angle);
            const double h = startHeight + (endHeight - startHeight) * factor;

            vertices.append(unprojectPointFromPlaneUV(origin, axisU, axisV, normal, u, v, h));
        }
    }

    void appendBulgeRawPathPoints(std::vector<RawPathPoint3D>& points,
        const QVector3D& origin,
        const QVector3D& axisU,
        const QVector3D& axisV,
        const QVector3D& normal,
        const QVector3D& start,
        const QVector3D& end,
        double bulge)
    {
        double su = 0.0;
        double sv = 0.0;
        double eu = 0.0;
        double ev = 0.0;

        projectPointToPlaneUV(origin, axisU, axisV, start, su, sv);
        projectPointToPlaneUV(origin, axisU, axisV, end, eu, ev);

        const double dx = eu - su;
        const double dy = ev - sv;
        const double chordLength = std::sqrt(dx * dx + dy * dy);

        if (chordLength <= 1.0e-10 || std::abs(bulge) < 1.0e-8)
        {
            appendRawPathPoint(points, end);
            return;
        }

        const double midpointU = (su + eu) * 0.5;
        const double midpointV = (sv + ev) * 0.5;
        const double centerOffset = chordLength * (1.0 / bulge - bulge) * 0.25;
        const double centerU = midpointU - centerOffset * (dy / chordLength);
        const double centerV = midpointV + centerOffset * (dx / chordLength);
        const double radius = std::hypot(su - centerU, sv - centerV);
        const double startAngle = std::atan2(sv - centerV, su - centerU);
        const double sweepAngle = 4.0 * std::atan(bulge);
        const int segments = std::max
        (
            4,
            static_cast<int>(std::ceil(std::abs(sweepAngle) / kTwoPi * kFullCircleSegments))
        );

        const double startHeight = QVector3D::dotProduct(start - origin, normal);
        const double endHeight = QVector3D::dotProduct(end - origin, normal);

        for (int index = 1; index <= segments; ++index)
        {
            const double factor = static_cast<double>(index) / static_cast<double>(segments);
            const double angle = startAngle + sweepAngle * factor;
            const double u = centerU + radius * std::cos(angle);
            const double v = centerV + radius * std::sin(angle);
            const double h = startHeight + (endHeight - startHeight) * factor;

            appendRawPathPoint
            (
                points,
                unprojectPointFromPlaneUV(origin, axisU, axisV, normal, u, v, h)
            );
        }
    }


    QVector3D polylineVertexToWcs(const DRW_Polyline* polyline,
        const std::shared_ptr<DRW_Vertex>& vertex)
    {
        if (polyline == nullptr || vertex == nullptr)
        {
            return QVector3D();
        }

        // 3D polyline：顶点本身就按 WCS 使用
        if (is3DPolyline(polyline))
        {
            return QVector3D
            (
                static_cast<float>(vertex->basePoint.x),
                static_cast<float>(vertex->basePoint.y),
                static_cast<float>(vertex->basePoint.z)
            );
        }

        // 2D polyline：按 OCS -> WCS 转换
        QVector3D normal = resolvePolylineNormal(polyline->extPoint);
        if (normal.lengthSquared() <= 1.0e-12f)
        {
            normal = QVector3D(0.0f, 0.0f, 1.0f);
        }

        QVector3D axisU;
        QVector3D axisV;
        buildPlaneBasis(normal, axisU, axisV);

        // 2D POLYLINE 的 elevation 取实体基点的 z
        const QVector3D origin = normal * static_cast<float>(polyline->basePoint.z);

        return origin
            + axisU * static_cast<float>(vertex->basePoint.x)
            + axisV * static_cast<float>(vertex->basePoint.y)
            + normal * static_cast<float>(vertex->basePoint.z);
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

    if (m_data == nullptr || m_data->vertlist.empty())
    {
        return;
    }

    const bool isClosed = (m_data->flags & 1) != 0;

    const auto toVertex = [this](const std::shared_ptr<DRW_Vertex>& vertex)
        {
            return polylineVertexToWcs(m_data, vertex);
        };

    QVector3D origin;
    QVector3D axisU;
    QVector3D axisV;
    QVector3D normal;
    const bool hasPlaneBasis = buildPolylinePlaneBasis(m_data, origin, axisU, axisV, normal);

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

        const QVector3D start = toVertex(current);
        const QVector3D end = toVertex(next);

        if (is3DPolyline(m_data))
        {
            // 3D polyline 不做 bulge 展开，按直线段处理
            m_geometry.vertices.append(end);
        }
        else if (hasPlaneBasis)
        {
            appendBulgeVertices
            (
                m_geometry.vertices,
                origin,
                axisU,
                axisV,
                normal,
                start,
                end,
                current->bulge
            );
        }
        else
        {
            m_geometry.vertices.append(end);
        }
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

    const auto toVertex = [this](const std::shared_ptr<DRW_Vertex>& vertex)
        {
            return polylineVertexToWcs(m_data, vertex);
        };

    QVector3D origin;
    QVector3D axisU;
    QVector3D axisV;
    QVector3D normal;
    const bool hasPlaneBasis = buildPolylinePlaneBasis(m_data, origin, axisU, axisV, normal);

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
                const size_t currentIndex =
                    (startIndex + vertexCount - (step % vertexCount)) % vertexCount;
                const size_t previousIndex =
                    (currentIndex + vertexCount - 1) % vertexCount;

                const QVector3D start = toVertex(m_data->vertlist.at(currentIndex));
                const QVector3D end = toVertex(m_data->vertlist.at(previousIndex));
                const double bulge = -m_data->vertlist.at(previousIndex)->bulge;

                if (is3DPolyline(m_data))
                {
                    appendRawPathPoint(m_rawPathPoints3D, end);
                }
                else if (hasPlaneBasis)
                {
                    appendBulgeRawPathPoints
                    (
                        m_rawPathPoints3D,
                        origin,
                        axisU,
                        axisV,
                        normal,
                        start,
                        end,
                        bulge
                    );
                }
                else
                {
                    appendRawPathPoint(m_rawPathPoints3D, end);
                }
            }
        }
        else
        {
            for (size_t step = 0; step < vertexCount; ++step)
            {
                const size_t currentIndex = (startIndex + step) % vertexCount;
                const size_t nextIndex = (currentIndex + 1) % vertexCount;

                const QVector3D start = toVertex(m_data->vertlist.at(currentIndex));
                const QVector3D end = toVertex(m_data->vertlist.at(nextIndex));
                const double bulge = m_data->vertlist.at(currentIndex)->bulge;

                if (hasPlaneBasis)
                {
                    appendBulgeRawPathPoints
                    (
                        m_rawPathPoints3D,
                        origin,
                        axisU,
                        axisV,
                        normal,
                        start,
                        end,
                        bulge
                    );
                }
                else
                {
                    appendRawPathPoint(m_rawPathPoints3D, end);
                }
            }
        }

        return;
    }

    if (m_isReverse)
    {
        appendRawPathPoint(m_rawPathPoints3D, toVertex(m_data->vertlist.back()));

        for (size_t index = vertexCount - 1; index > 0; --index)
        {
            const QVector3D start = toVertex(m_data->vertlist.at(index));
            const QVector3D end = toVertex(m_data->vertlist.at(index - 1));
            const double bulge = -m_data->vertlist.at(index - 1)->bulge;

            if (hasPlaneBasis)
            {
                appendBulgeRawPathPoints
                (
                    m_rawPathPoints3D,
                    origin,
                    axisU,
                    axisV,
                    normal,
                    start,
                    end,
                    bulge
                );
            }
            else
            {
                appendRawPathPoint(m_rawPathPoints3D, end);
            }
        }
    }
    else
    {
        appendRawPathPoint(m_rawPathPoints3D, toVertex(m_data->vertlist.front()));

        for (size_t index = 0; index + 1 < vertexCount; ++index)
        {
            const QVector3D start = toVertex(m_data->vertlist.at(index));
            const QVector3D end = toVertex(m_data->vertlist.at(index + 1));
            const double bulge = m_data->vertlist.at(index)->bulge;

            if (hasPlaneBasis)
            {
                appendBulgeRawPathPoints
                (
                    m_rawPathPoints3D,
                    origin,
                    axisU,
                    axisV,
                    normal,
                    start,
                    end,
                    bulge
                );
            }
            else
            {
                appendRawPathPoint(m_rawPathPoints3D, end);
            }
        }
    }
}

bool CadPolylineItem::rebuildControlPoints4Axis
(
    double axisY,
    double axisZ,
    double judgeCenterY,
    double judgeCenterZ,
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
            *errorMessage = QStringLiteral("多段线原始路径点集为空。");
        }

        return false;
    }

    m_controlPoints4Axis.clear();
    m_controlPoints4Axis.reserve(m_rawPathPoints3D.size());

    auto applyAnglePolicy = [&](double rawA, bool hasPrevious, double previousA) -> double
        {
            if (invertAAxisDirection)
            {
                rawA = -rawA;
            }

            rawA += aAxisOffsetDegrees;
            rawA = normalizeAngle180(rawA);

            if (hasPrevious && keepContinuousAngle)
            {
                rawA = unwrapAngleNear(previousA, rawA);
            }

            return rawA;
        };

    constexpr double kPlaneEps = 1.0e-8;

    // 先判断整条 polyline 是否基本处于固定姿态平面
    double minY = m_rawPathPoints3D.front().y;
    double maxY = m_rawPathPoints3D.front().y;
    double minZ = m_rawPathPoints3D.front().z;
    double maxZ = m_rawPathPoints3D.front().z;

    double sumY = 0.0;
    double sumZ = 0.0;

    for (const RawPathPoint3D& point : m_rawPathPoints3D)
    {
        minY = std::min(minY, point.y);
        maxY = std::max(maxY, point.y);
        minZ = std::min(minZ, point.z);
        maxZ = std::max(maxZ, point.z);

        sumY += point.y;
        sumZ += point.z;
    }

    const double avgY = sumY / static_cast<double>(m_rawPathPoints3D.size());
    const double avgZ = sumZ / static_cast<double>(m_rawPathPoints3D.size());

    const bool inPlaneParallelXZ = (maxY - minY) < kPlaneEps; // y 基本恒定
    const bool inPlaneParallelXY = (maxZ - minZ) < kPlaneEps; // z 基本恒定

    bool useFixedA = false;
    double fixedRawA = 0.0;

    // 规则 1：整条 polyline 位于平行 XZ 的平面
    if (inPlaneParallelXZ)
    {
        if (avgY > judgeCenterY + kPlaneEps)
        {
            fixedRawA = 90.0;     // +Y
            useFixedA = true;
        }
        else if (avgY < judgeCenterY - kPlaneEps)
        {
            fixedRawA = -90.0;    // -Y
            useFixedA = true;
        }
        else
        {
            fixedRawA = (avgZ >= judgeCenterZ) ? 0.0 : 180.0;
            useFixedA = true;
        }
    }
    // 规则 2：整条 polyline 位于平行 XY 的平面
    else if (inPlaneParallelXY)
    {
        if (avgZ > judgeCenterZ + kPlaneEps)
        {
            fixedRawA = 0.0;      // +Z
            useFixedA = true;
        }
        else if (avgZ < judgeCenterZ - kPlaneEps)
        {
            fixedRawA = 180.0;    // -Z
            useFixedA = true;
        }
        else
        {
            fixedRawA = (avgY >= judgeCenterY) ? 90.0 : -90.0;
            useFixedA = true;
        }
    }

    if (useFixedA)
    {
        const double fixedA = applyAnglePolicy(fixedRawA, false, 0.0);
        const double angleRad = fixedA / kRadToDeg;
        const double c = std::cos(angleRad);
        const double s = std::sin(angleRad);

        for (const RawPathPoint3D& point : m_rawPathPoints3D)
        {
            const double dy = point.y - axisY;
            const double dz = point.z - axisZ;

            const double machineX = point.x;
            const double machineY = axisY + dy * c - dz * s;
            const double machineZ = axisZ + dy * s + dz * c;

            m_controlPoints4Axis.push_back({ machineX, machineY, machineZ, fixedA });
        }
    }
    else
    {
        // 一般空间 polyline：逐点 A + 顶部对齐
        bool hasPrevious = false;
        double previousA = 0.0;

        for (const RawPathPoint3D& point : m_rawPathPoints3D)
        {
            const double dy = point.y - axisY;
            const double dz = point.z - axisZ;
            const double radiusSquared = dy * dy + dz * dz;
            const double radius = std::sqrt(radiusSquared);

            double aDeg = 0.0;

            if (radiusSquared < kAxisEps * kAxisEps)
            {
                if (hasPrevious)
                {
                    aDeg = previousA;
                }
                else
                {
                    aDeg = applyAnglePolicy(0.0, false, 0.0);
                }
            }
            else
            {
                const double rawA = std::atan2(dy, dz) * kRadToDeg;
                aDeg = applyAnglePolicy(rawA, hasPrevious, previousA);
            }

            const double machineX = point.x;
            const double machineY = axisY;
            const double machineZ = axisZ + radius;

            m_controlPoints4Axis.push_back({ machineX, machineY, machineZ, aDeg });

            previousA = aDeg;
            hasPrevious = true;
        }
    }

    if (errorMessage != nullptr)
    {
        errorMessage->clear();
    }

    return true;
}
