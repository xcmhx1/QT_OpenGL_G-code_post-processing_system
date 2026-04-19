// 实现 CadLineItem 模块，对应头文件中声明的主要行为和协作流程。
// 直线图元模块，负责直线实体的几何离散、颜色解析和方向生成。
#include "pch.h"

#include "CadLineItem.h"

#include <cmath>

namespace
{
constexpr double kAxisEps = 1.0e-8;
constexpr double kRadToDeg = 57.2957795130823208768;
}

CadLineItem::CadLineItem(DRW_Entity* entity, QObject* parent)
    : CadItem(entity, parent)
{
    // 直线图元只接受 DRW_Line，这里在构造阶段完成强类型绑定。
    m_data = static_cast<DRW_Line*>(m_nativeEntity);
    // 构造后立即构建几何和方向，保证图元一进入场景即可渲染。
    buildGeometryDatay();
    buildProcessDirection();
}

void CadLineItem::buildGeometryDatay()
{
    // 重建前先清空旧几何，避免刷新实体时残留历史顶点。
    m_geometry.vertices.clear();
    clearPathCaches();

    if (m_data == nullptr)
    {
        return;
    }

    // 直线渲染只需要起点和终点两个顶点。
    m_geometry.vertices.reserve(2);
    m_geometry.vertices.append(QVector3D(m_data->basePoint.x, m_data->basePoint.y, m_data->basePoint.z));
    m_geometry.vertices.append(QVector3D(m_data->secPoint.x, m_data->secPoint.y, m_data->secPoint.z));
}

void CadLineItem::rebuildRawPathPoints3D()
{
    m_rawPathPoints3D.clear();

    if (m_data == nullptr)
    {
        return;
    }

    m_rawPathPoints3D.reserve(2);

    const DRW_Coord& startPoint = m_isReverse ? m_data->secPoint : m_data->basePoint;
    const DRW_Coord& endPoint = m_isReverse ? m_data->basePoint : m_data->secPoint;

    m_rawPathPoints3D.push_back({ startPoint.x, startPoint.y, startPoint.z });
    m_rawPathPoints3D.push_back({ endPoint.x, endPoint.y, endPoint.z });
}

bool CadLineItem::rebuildControlPoints4Axis
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
            *errorMessage = QStringLiteral("直线原始路径点集为空。");
        }

        return false;
    }

    m_controlPoints4Axis.clear();
    m_controlPoints4Axis.reserve(m_rawPathPoints3D.size());

    bool hasPrevious = false;
    double previousA = 0.0;

    auto applyAnglePolicy = [&](double rawA) -> double
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
                aDeg = applyAnglePolicy(0.0);
            }
        }
        else
        {
            const double rawA = std::atan2(dy, dz) * kRadToDeg;
            aDeg = applyAnglePolicy(rawA);
        }

        const double machineX = point.x;
        const double machineY = axisY;
        const double machineZ = axisZ + radius;

        m_controlPoints4Axis.push_back({ machineX, machineY, machineZ, aDeg });
        previousA = aDeg;
        hasPrevious = true;
    }

    if (errorMessage != nullptr)
    {
        errorMessage->clear();
    }

    return true;
}
