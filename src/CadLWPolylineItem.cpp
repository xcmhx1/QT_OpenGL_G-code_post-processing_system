// 实现 CadLWPolylineItem 模块，对应头文件中声明的主要行为和协作流程。
// 轻量多段线图元模块，负责轻量多段线的离散显示和 bulge 解释。
#include "pch.h"

#include "CadLWPolylineItem.h"
#include "application/messaging/DebugMessageSink.h"
#include "compatibility/legacy/LegacyCadItemPathBridge.h"

#include <cmath>

namespace
{
    constexpr double kTwoPi = 6.28318530717958647692;
    // 轻量多段线的 bulge 圆弧离散策略与普通多段线保持一致。
    constexpr int kFullCircleSegments = 128;

    QVector3D resolveLWPolylineNormal(const DRW_Coord& extPoint)
    {
        QVector3D normal(extPoint.x, extPoint.y, extPoint.z);

        if (normal.lengthSquared() <= 1.0e-12f)
        {
            return QVector3D(0.0f, 0.0f, 1.0f);
        }

        normal.normalize();
        return normal;
    }

    QVector3D inferPlaneNormalFromVertices(const std::vector<std::shared_ptr<DRW_Vertex2D>>& vertices, float z)
    {
        if (vertices.size() < 3)
        {
            return QVector3D();
        }

        const QVector3D p0
        (
            static_cast<float>(vertices.front()->x),
            static_cast<float>(vertices.front()->y),
            z
        );

        for (size_t i = 1; i + 1 < vertices.size(); ++i)
        {
            const QVector3D p1
            (
                static_cast<float>(vertices[i]->x),
                static_cast<float>(vertices[i]->y),
                z
            );
            const QVector3D p2
            (
                static_cast<float>(vertices[i + 1]->x),
                static_cast<float>(vertices[i + 1]->y),
                z
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

    bool buildLWPolylinePlaneBasis(const DRW_LWPolyline* polyline,
        QVector3D& origin,
        QVector3D& axisU,
        QVector3D& axisV,
        QVector3D& normal)
    {
        if (polyline == nullptr)
        {
            return false;
        }

        // LWPOLYLINE 的顶点坐标在 OCS 中，elevation 是 OCS 的 z，
        // extrusion direction 定义该 OCS 平面在 WCS 中的朝向。
        normal = resolveLWPolylineNormal(polyline->extPoint);
        buildPlaneBasis(normal, axisU, axisV);

        // OCS 中 z = elevation 对应到 WCS 中，是沿 normal 偏移 elevation
        origin = normal * static_cast<float>(polyline->elevation);
        return true;
    }

    QVector3D lwPolylineVertexToWcs(const DRW_LWPolyline* polyline,
        const std::shared_ptr<DRW_Vertex2D>& vertex)
    {
        QVector3D origin;
        QVector3D axisU;
        QVector3D axisV;
        QVector3D normal;

        if (!buildLWPolylinePlaneBasis(polyline, origin, axisU, axisV, normal))
        {
            return QVector3D
            (
                static_cast<float>(vertex->x),
                static_cast<float>(vertex->y),
                static_cast<float>(polyline != nullptr ? polyline->elevation : 0.0)
            );
        }

        return origin
            + axisU * static_cast<float>(vertex->x)
            + axisV * static_cast<float>(vertex->y);
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

}


CadLWPolylineItem::CadLWPolylineItem(DRW_Entity* entity, QObject* parent)
    : CadItem(entity, parent)
{
    // 绑定原生轻量多段线实体并立即准备好离散缓存。
    m_data = static_cast<DRW_LWPolyline*>(m_nativeEntity);
    buildGeometryDatay();
}

void CadLWPolylineItem::buildGeometryDatay()
{
    m_geometry.vertices.clear();
    clearPathCaches();

    if (m_data == nullptr || m_data->vertlist.empty())
    {
        return;
    }

    const bool isClosed = (m_data->flags & 1) != 0;

    const auto toVertex = [this](const std::shared_ptr<DRW_Vertex2D>& vertex)
        {
            return lwPolylineVertexToWcs(m_data, vertex);
        };

    QVector3D origin;
    QVector3D axisU;
    QVector3D axisV;
    QVector3D normal;
    const bool hasPlaneBasis = buildLWPolylinePlaneBasis(m_data, origin, axisU, axisV, normal);

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

        if (hasPlaneBasis)
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
void CadLWPolylineItem::rebuildRawPathPoints3D()
{
    m_rawPathPoints3D.clear();
    cadcam::geometry::PathCompileOptions options;
    const OperationResult<cadcam::geometry::Path3D> result = LegacyCadItemPathBridge::compile
    (
        *this,
        LegacyCadItemPathBridge::legacySamplingPolicy(*this),
        options,
        createOperationContext(QStringLiteral("rebuild-lwpolyline-raw-path"))
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
