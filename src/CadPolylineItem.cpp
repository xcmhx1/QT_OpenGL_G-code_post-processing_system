// 实现 CadPolylineItem 模块，对应头文件中声明的主要行为和协作流程。
// 多段线图元模块，负责多段线的离散显示和 bulge 圆弧段解释。
#include "pch.h"

#include "CadPolylineItem.h"
#include "application/messaging/DebugMessageSink.h"
#include "compatibility/legacy/LegacyCadItemPathBridge.h"

#include <cmath>

namespace
{

    constexpr double kTwoPi = 6.28318530717958647692;
    // bulge 圆弧展开时参考整圆采样密度。
    constexpr int kFullCircleSegments = 128;

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
    cadcam::geometry::PathCompileOptions options;
    options.reverse = m_isReverse;
    if (m_hasCustomProcessStart)
    {
        options.startParameter = m_processStartParameter;
    }
    const OperationResult<cadcam::geometry::Path3D> result = LegacyCadItemPathBridge::compile
    (
        *this,
        LegacyCadItemPathBridge::legacySamplingPolicy(*this),
        options,
        createOperationContext(QStringLiteral("rebuild-polyline-raw-path"))
    );
    if (result.succeeded() && result.value.has_value())
    {
        LegacyCadItemPathBridge::copyToLegacyRawPath(*result.value, m_rawPathPoints3D);
        return;
    }

    DebugMessageSink sink;
    for (const Diagnostic& diagnostic : result.diagnostics)
    {
        sink.publish(diagnostic);
    }
}
