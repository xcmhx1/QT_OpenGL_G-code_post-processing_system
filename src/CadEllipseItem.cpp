// 实现 CadEllipseItem 模块，对应头文件中声明的主要行为和协作流程。
// 椭圆图元模块，负责椭圆实体的离散显示数据和方向信息构建。
#include "pch.h"

#include "CadEllipseItem.h"
#include "CadEllipseGeometry.h"
#include "application/messaging/DebugMessageSink.h"
#include "compatibility/legacy/LegacyCadItemPathBridge.h"

#include <cmath>

namespace
{
constexpr double kAxisEps = 1.0e-8;
constexpr double kRadToDeg = 57.2957795130823208768;
constexpr double kTwoPi = 6.28318530717958647692;
// 整椭圆的基础采样密度，局部弧段会按参数跨度缩放。
constexpr int kFullEllipseSegments = 128;

bool isFullEllipsePath(const DRW_Ellipse* ellipse)
{
    return ellipse != nullptr && CadEllipseGeometryUtils::isFullEllipseParameterRange
    (
        ellipse->staparam,
        ellipse->endparam
    );
}

}

CadEllipseItem::CadEllipseItem(DRW_Entity* entity, QObject* parent)
    : CadItem(entity, parent)
{
    // 绑定原生椭圆实体，构造时同步生成离散几何和加工方向。
    m_data = static_cast<DRW_Ellipse*>(m_nativeEntity);
    buildGeometryDatay();
}

void CadEllipseItem::buildGeometryDatay()
{
    m_geometry.vertices.clear();
    clearPathCaches();

    if (m_data == nullptr)
    {
        return;
    }

    // DXF 椭圆以中心点和“从中心指向长轴端点”的向量表示长轴。
    CadEllipseGeometry geometry;

    if (!CadEllipseGeometryUtils::buildEllipseGeometry(m_data, geometry))
    {
        return;
    }

    double startParam = m_data->staparam;
    double endParam = m_data->endparam;

    // 完整椭圆在 DXF 中可能以相同起止参数或完整 2π 参数区间表示。
    if (geometry.full)
    {
        endParam = startParam + kTwoPi;
    }

    // 统一把结束参数展开到开始参数之后，便于正向采样。
    while (endParam <= startParam)
    {
        endParam += kTwoPi;
    }

    const double span = endParam - startParam;
    // 至少保留 16 段，避免局部椭圆弧过稀。
    const int segments = std::max(16, static_cast<int>(std::ceil(span / kTwoPi * kFullEllipseSegments)));

    m_geometry.vertices.reserve(segments + 1);

    for (int i = 0; i <= segments; ++i)
    {
        // 椭圆参数方程：center + major*cos(t) + minor*sin(t)。
        const double t = startParam + span * static_cast<double>(i) / static_cast<double>(segments);
        m_geometry.vertices.append(CadEllipseGeometryUtils::ellipsePointAt(geometry, t));
    }
}

double CadEllipseItem::defaultProcessStartParameter() const
{
    return M_PI_2;
}

void CadEllipseItem::rebuildRawPathPoints3D()
{
    m_rawPathPoints3D.clear();
    cadcam::geometry::PathCompileOptions options;
    const OperationResult<cadcam::geometry::Path3D> result = LegacyCadItemPathBridge::compile
    (
        *this,
        LegacyCadItemPathBridge::legacySamplingPolicy(*this),
        options,
        createOperationContext(QStringLiteral("rebuild-ellipse-raw-path"))
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
