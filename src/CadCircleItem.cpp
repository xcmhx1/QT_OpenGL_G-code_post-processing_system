// 实现 CadCircleItem 模块，对应头文件中声明的主要行为和协作流程。
// 圆图元模块，负责圆实体的离散显示数据和加工方向生成。
#include "pch.h"

#include "CadCircleItem.h"
#include "CadOcsGeometry.h"
#include "application/messaging/DebugMessageSink.h"
#include "compatibility/legacy/LegacyCadItemPathBridge.h"

#include <cmath>

namespace
{
constexpr double kAxisEps = 1.0e-8;
constexpr double kRadToDeg = 57.2957795130823208768;
// 统一使用 2π，避免在采样时重复书写常量。
constexpr double kTwoPi = 6.28318530717958647692;
// 圆显示默认采样为 128 段，兼顾平滑度和顶点数量。
constexpr int kCircleSegments = 128;
}

CadCircleItem::CadCircleItem(DRW_Entity* entity, QObject* parent)
    : CadItem(entity, parent)
{
    // 绑定原生圆实体，并立即生成渲染所需的离散数据。
    m_data = static_cast<DRW_Circle*>(m_nativeEntity);
    buildGeometryDatay();
    buildProcessDirection();
}

void CadCircleItem::buildGeometryDatay()
{
    m_geometry.vertices.clear();
    clearPathCaches();

    if (m_data == nullptr || m_data->radious <= 0.0)
    {
        return;
    }

    // DXF 圆心在 OCS 中，使用时统一转换到 WCS，避免 extrusion=-Z 时 X/Z 符号错误。
    const QVector3D center = CadOcsGeometry::center(m_data);
    QVector3D axisX;
    QVector3D axisY;
    QVector3D axisZ;
    CadOcsGeometry::basis(m_data->extPoint, axisX, axisY, axisZ);

    // 多预留一个点，把首尾闭合成完整圆环折线。
    m_geometry.vertices.reserve(kCircleSegments + 1);

    for (int i = 0; i <= kCircleSegments; ++i)
    {
        // 沿局部平面参数化采样：center + cos(t)*axisX + sin(t)*axisY。
        const double angle = kTwoPi * static_cast<double>(i) / static_cast<double>(kCircleSegments);
        const QVector3D offset =
            axisX * static_cast<float>(std::cos(angle) * m_data->radious) +
            axisY * static_cast<float>(std::sin(angle) * m_data->radious);

        m_geometry.vertices.append(center + offset);
    }
}

double CadCircleItem::defaultProcessStartParameter() const
{
    return M_PI_2;
}

void CadCircleItem::rebuildRawPathPoints3D()
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
        createOperationContext(QStringLiteral("rebuild-circle-raw-path"))
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
