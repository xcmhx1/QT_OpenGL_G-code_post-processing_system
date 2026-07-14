// 实现 CadArcItem 模块，对应头文件中声明的主要行为和协作流程。
// 圆弧图元模块，封装圆弧实体的几何离散、颜色解析和方向信息。
#include "pch.h"

#include "CadArcItem.h"
#include "CadOcsGeometry.h"
#include "application/messaging/DebugMessageSink.h"
#include "compatibility/legacy/LegacyCadItemPathBridge.h"

#include <cmath>

namespace
{
constexpr double kAxisEps = 1.0e-8;
constexpr double kRadToDeg = 57.2957795130823208768;
constexpr double kTwoPi = 6.28318530717958647692;
// 显示圆弧与整圆共用采样密度，再按弧长比例折算最终段数。
constexpr int kFullCircleSegments = 128;

}

CadArcItem::CadArcItem(DRW_Entity* entity, QObject* parent)
    : CadItem(entity, parent)
{
    // 绑定原生圆弧实体并在构造时完成首次离散。
    m_data = static_cast<DRW_Arc*>(m_nativeEntity);
    buildGeometryDatay();
    buildProcessDirection();
}

void CadArcItem::buildGeometryDatay()
{
    m_geometry.vertices.clear();
    clearPathCaches();

    if (m_data == nullptr || m_data->radious <= 0.0)
    {
        return;
    }

    // 圆弧和圆共享圆心/半径/法向定义，只是额外多了起止角范围。
    const QVector3D center = CadOcsGeometry::center(m_data);
    QVector3D normal = CadOcsGeometry::normal(m_data->extPoint);

    QVector3D axisX;
    QVector3D axisY;
    CadOcsGeometry::basis(m_data->extPoint, axisX, axisY, normal);

    double startAngle = m_data->staangle;
    double endAngle = m_data->endangle;

    // libdxfrw 中结束角可能小于开始角，这里统一展开到同一正向周期。
    while (endAngle <= startAngle)
    {
        endAngle += kTwoPi;
    }

    const double span = endAngle - startAngle;
    // 至少保留 16 段，避免很短的圆弧看起来过于粗糙。
    const int segments = std::max(16, static_cast<int>(std::ceil(span / kTwoPi * kFullCircleSegments)));

    m_geometry.vertices.reserve(segments + 1);

    for (int i = 0; i <= segments; ++i)
    {
        // 沿弧长均匀插值角度，生成折线化顶点。
        const double t = startAngle + span * static_cast<double>(i) / static_cast<double>(segments);
        const QVector3D offset =
            axisX * static_cast<float>(std::cos(t) * m_data->radious) +
            axisY * static_cast<float>(std::sin(t) * m_data->radious);

        m_geometry.vertices.append(center + offset);
    }
}

void CadArcItem::rebuildRawPathPoints3D()
{
    m_rawPathPoints3D.clear();
    cadcam::geometry::PathCompileOptions options;
    options.reverse = m_isReverse;
    const OperationResult<cadcam::geometry::Path3D> result = LegacyCadItemPathBridge::compile
    (
        *this,
        LegacyCadItemPathBridge::legacySamplingPolicy(*this),
        options,
        createOperationContext(QStringLiteral("rebuild-arc-raw-path"))
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
