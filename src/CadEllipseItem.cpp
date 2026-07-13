// 实现 CadEllipseItem 模块，对应头文件中声明的主要行为和协作流程。
// 椭圆图元模块，负责椭圆实体的离散显示数据和方向信息构建。
#include "pch.h"

#include "CadEllipseItem.h"
#include "CadEllipseGeometry.h"

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

double effectiveClosedEllipseStartParameter(const CadEllipseItem* item)
{
    return item != nullptr ? item->defaultProcessStartParameter() : M_PI_2;
}
}

CadEllipseItem::CadEllipseItem(DRW_Entity* entity, QObject* parent)
    : CadItem(entity, parent)
{
    // 绑定原生椭圆实体，构造时同步生成离散几何和加工方向。
    m_data = static_cast<DRW_Ellipse*>(m_nativeEntity);
    buildGeometryDatay();
    buildProcessDirection();
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

    if (m_data == nullptr)
    {
        return;
    }

    CadEllipseGeometry geometry;

    if (!CadEllipseGeometryUtils::buildEllipseGeometry(m_data, geometry))
    {
        return;
    }

    const bool closedPath = isFullEllipsePath(m_data);
    double startParam = m_data->staparam;
    double endParam = m_data->endparam;

    if (closedPath)
    {
        startParam = effectiveClosedEllipseStartParameter(this);
        endParam = startParam + (m_isReverse ? -kTwoPi : kTwoPi);
    }
    else if (m_isReverse)
    {
        startParam = m_data->endparam;
        endParam = m_data->staparam;

        while (endParam >= startParam)
        {
            endParam -= kTwoPi;
        }
    }
    else
    {
        while (endParam <= startParam)
        {
            endParam += kTwoPi;
        }
    }

    const double span = endParam - startParam;
    const int segments = std::max(16, static_cast<int>(std::ceil(std::abs(span) / kTwoPi * kFullEllipseSegments)));

    m_rawPathPoints3D.reserve(static_cast<size_t>(segments) + 1);

    for (int index = 0; index <= segments; ++index)
    {
        const double parameter = startParam + span * static_cast<double>(index) / static_cast<double>(segments);
        const QVector3D point = CadEllipseGeometryUtils::ellipsePointAt(geometry, parameter);
        m_rawPathPoints3D.push_back({ point.x(), point.y(), point.z() });
    }
}

bool CadEllipseItem::rebuildControlPoints4Axis
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
            *errorMessage = QStringLiteral("椭圆原始路径点集为空。");
        }

        return false;
    }

    if (m_data == nullptr)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("椭圆原始实体为空。");
        }

        return false;
    }

    m_controlPoints4Axis.clear();
    m_controlPoints4Axis.reserve(m_rawPathPoints3D.size());

    CadEllipseGeometry geometry;

    if (!CadEllipseGeometryUtils::buildEllipseGeometry(m_data, geometry))
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("椭圆几何参数无效。");
        }

        return false;
    }

    const QVector3D normal = geometry.normal;

    const double centerY = geometry.center.y();
    const double centerZ = geometry.center.z();

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

    constexpr double kNormalEps = 1.0e-8;
    constexpr double kSideEps = 1.0e-8;

    bool useFixedA = false;
    double fixedRawA = 0.0;

    // 规则 1：椭圆平面平行于 XY 平面（法向约等于 ±Z）
    // A 由椭圆中心位于 +Z / -Z 决定，而不是由法向正负决定。
    if (std::abs(normal.x()) < kNormalEps && std::abs(normal.y()) < kNormalEps)
    {
        const double relativeZ = centerZ - judgeCenterZ;

        if (relativeZ > kSideEps)
        {
            fixedRawA = 0.0;      // +Z
        }
        else if (relativeZ < -kSideEps)
        {
            fixedRawA = 180.0;    // -Z
        }
        else
        {
            // 椭圆中心恰好落在 axisZ 上时，退化到看法向
            fixedRawA = (normal.z() >= 0.0) ? 0.0 : 180.0;
        }

        useFixedA = true;
    }
    // 规则 2：椭圆平面平行于 XZ 平面（法向约等于 ±Y）
    // A 由椭圆中心位于 +Y / -Y 决定，而不是由法向正负决定。
    else if (std::abs(normal.x()) < kNormalEps && std::abs(normal.z()) < kNormalEps)
    {
        const double relativeY = centerY - judgeCenterY;

        if (relativeY > kSideEps)
        {
            fixedRawA = 90.0;     // +Y
        }
        else if (relativeY < -kSideEps)
        {
            fixedRawA = -90.0;    // -Y
        }
        else
        {
            // 椭圆中心恰好落在 axisY 上时，退化到看法向
            fixedRawA = (normal.y() >= 0.0) ? 90.0 : -90.0;
        }

        useFixedA = true;
    }

    if (useFixedA)
    {
        const double fixedA = applyAnglePolicy(fixedRawA);

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
        // 其它姿态：回退到“刀头指向椭圆心”的逐点 A 模型
        for (const RawPathPoint3D& point : m_rawPathPoints3D)
        {
            const double dirY = point.y - centerY;
            const double dirZ = point.z - centerZ;
            const double dirLenSquared = dirY * dirY + dirZ * dirZ;

            double aDeg = 0.0;

            if (dirLenSquared < kAxisEps * kAxisEps)
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
                const double rawA = std::atan2(dirY, dirZ) * kRadToDeg;
                aDeg = applyAnglePolicy(rawA);
            }

            const double angleRad = aDeg / kRadToDeg;
            const double c = std::cos(angleRad);
            const double s = std::sin(angleRad);

            const double dy = point.y - axisY;
            const double dz = point.z - axisZ;

            const double machineX = point.x;
            const double machineY = axisY + dy * c - dz * s;
            const double machineZ = axisZ + dy * s + dz * c;

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
