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

    if (m_rawPathPoints3D.size() < 2)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("直线原始路径点集不足，无法生成 4 轴控制点。");
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

    const RawPathPoint3D& p0 = m_rawPathPoints3D.front();
    const RawPathPoint3D& p1 = m_rawPathPoints3D.back();

    const double lineDy = p1.y - p0.y;
    const double lineDz = p1.z - p0.z;

    constexpr double kPlaneEps = 1.0e-8;

    const bool inPlaneParallelXZ = std::abs(lineDy) < kPlaneEps; // y 基本恒定
    const bool inPlaneParallelXY = std::abs(lineDz) < kPlaneEps; // z 基本恒定

    bool useFixedA = false;
    double fixedRawA = 0.0;

    // 规则 1：整条线位于平行 XZ 的平面（y 基本恒定）
    // 工艺约定：
    //   +Y -> A = 90
    //   -Y -> A = -90（等价于 270）
    if (inPlaneParallelXZ)
    {
        const double midY = 0.5 * (p0.y + p1.y);

        if (midY > axisY + kPlaneEps)
        {
            fixedRawA = 90.0;
            useFixedA = true;
        }
        else if (midY < axisY - kPlaneEps)
        {
            fixedRawA = -90.0;
            useFixedA = true;
        }
        else
        {
            // 线恰好落在 axisY 对应的 Y 平面附近时，退化到看 Z 位置
            const double midZ = 0.5 * (p0.z + p1.z);
            fixedRawA = (midZ >= axisZ) ? 0.0 : 180.0;
            useFixedA = true;
        }
    }
    // 规则 2：整条线位于平行 XY 的平面（z 基本恒定）
    // 工艺约定：
    //   +Z -> A = 0
    //   -Z -> A = 180
    else if (inPlaneParallelXY)
    {
        const double midZ = 0.5 * (p0.z + p1.z);

        if (midZ > axisZ + kPlaneEps)
        {
            fixedRawA = 0.0;
            useFixedA = true;
        }
        else if (midZ < axisZ - kPlaneEps)
        {
            fixedRawA = 180.0;
            useFixedA = true;
        }
        else
        {
            // 线恰好落在 axisZ 对应的 Z 平面附近时，退化到看 Y 位置
            const double midY = 0.5 * (p0.y + p1.y);
            fixedRawA = (midY >= axisY) ? 90.0 : -90.0;
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
