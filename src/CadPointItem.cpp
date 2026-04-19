// 实现 CadPointItem 模块，对应头文件中声明的主要行为和协作流程。
// 点图元模块，负责点实体的显示数据构建和基础图元属性整理。
#include "pch.h"

#include "CadPointItem.h"

#include <cmath>

namespace
{
constexpr double kAxisEps = 1.0e-8;
constexpr double kRadToDeg = 57.2957795130823208768;
}

CadPointItem::CadPointItem(DRW_Entity* entity, QObject* parent)
    : CadItem(entity, parent)
{
    // 绑定原生点实体，后续所有几何数据都直接从 basePoint 提取。
    m_data = static_cast<DRW_Point*>(m_nativeEntity);
    buildGeometryDatay();
    buildProcessDirection();
}

void CadPointItem::buildGeometryDatay()
{
    m_geometry.vertices.clear();
    clearPathCaches();

    if (m_data == nullptr)
    {
        return;
    }

    // 点图元没有边，只保留一个位置顶点供渲染层绘制。
    m_geometry.vertices.append(QVector3D(m_data->basePoint.x, m_data->basePoint.y, m_data->basePoint.z));
}

void CadPointItem::rebuildRawPathPoints3D()
{
    m_rawPathPoints3D.clear();

    if (m_data == nullptr)
    {
        return;
    }

    m_rawPathPoints3D.push_back({ m_data->basePoint.x, m_data->basePoint.y, m_data->basePoint.z });
}

bool CadPointItem::rebuildControlPoints4Axis
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
            *errorMessage = QStringLiteral("点图元原始路径点集为空。");
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
