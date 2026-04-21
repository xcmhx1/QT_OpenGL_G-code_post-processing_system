// 实现 CadArcItem 模块，对应头文件中声明的主要行为和协作流程。
// 圆弧图元模块，封装圆弧实体的几何离散、颜色解析和方向信息。
#include "pch.h"

#include "CadArcItem.h"

#include <cmath>

namespace
{
constexpr double kAxisEps = 1.0e-8;
constexpr double kRadToDeg = 57.2957795130823208768;
constexpr double kTwoPi = 6.28318530717958647692;
constexpr double kMaxArcStep = 5.0 * kTwoPi / 360.0;
// 圆弧与整圆共用采样密度，再按弧长比例折算最终段数。
constexpr int kFullCircleSegments = 128;
constexpr int kMinRawArcSegments = 8;

QVector3D resolveNormal(const DRW_Coord& extPoint)
{
    // extrusion direction 决定圆弧所在平面法向。
    QVector3D normal(extPoint.x, extPoint.y, extPoint.z);

    if (normal.lengthSquared() <= 1.0e-12f)
    {
        return QVector3D(0.0f, 0.0f, 1.0f);
    }

    normal.normalize();
    return normal;
}

void buildPlaneBasis(const QVector3D& normal, QVector3D& axisX, QVector3D& axisY)
{
    // 法向接近世界 Z 轴时直接选用世界 X/Y 作为稳定基底。
    if (std::abs(normal.x()) <= 1.0e-6f && std::abs(normal.y()) <= 1.0e-6f)
    {
        axisX = QVector3D(1.0f, 0.0f, 0.0f);
        axisY = QVector3D::crossProduct(normal, axisX);

        if (axisY.lengthSquared() <= 1.0e-12f)
        {
            axisY = QVector3D(0.0f, 1.0f, 0.0f);
        }
        else
        {
            axisY.normalize();
        }

        return;
    }

    // 一般情况先构造一个与法向不平行的辅助向量，再通过叉乘得到局部坐标系。
    const QVector3D helper = std::abs(normal.z()) < 0.999f
        ? QVector3D(0.0f, 0.0f, 1.0f)
        : QVector3D(0.0f, 1.0f, 0.0f);

    axisX = QVector3D::crossProduct(helper, normal);

    if (axisX.lengthSquared() <= 1.0e-12f)
    {
        axisX = QVector3D(1.0f, 0.0f, 0.0f);
    }
    else
    {
        axisX.normalize();
    }

    axisY = QVector3D::crossProduct(normal, axisX);

    if (axisY.lengthSquared() <= 1.0e-12f)
    {
        axisY = QVector3D(0.0f, 1.0f, 0.0f);
    }
    else
    {
        axisY.normalize();
    }
}
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
    const QVector3D center(m_data->basePoint.x, m_data->basePoint.y, m_data->basePoint.z);
    const QVector3D normal = resolveNormal(m_data->extPoint);

    QVector3D axisX;
    QVector3D axisY;
    buildPlaneBasis(normal, axisX, axisY);

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

    if (m_data == nullptr || m_data->radious <= 0.0)
    {
        return;
    }

    const QVector3D center(m_data->basePoint.x, m_data->basePoint.y, m_data->basePoint.z);
    const QVector3D normal = resolveNormal(m_data->extPoint);

    QVector3D axisX;
    QVector3D axisY;
    buildPlaneBasis(normal, axisX, axisY);

    double startAngle = m_isReverse ? m_data->endangle : m_data->staangle;
    double endAngle = m_isReverse ? m_data->staangle : m_data->endangle;

    if (m_isReverse)
    {
        while (endAngle >= startAngle)
        {
            endAngle -= kTwoPi;
        }
    }
    else
    {
        while (endAngle <= startAngle)
        {
            endAngle += kTwoPi;
        }
    }

    const double span = endAngle - startAngle;
    const int segments = std::max
    (
        kMinRawArcSegments,
        static_cast<int>(std::ceil(std::abs(span) / kMaxArcStep))
    );

    m_rawPathPoints3D.reserve(static_cast<size_t>(segments) + 1);

    for (int index = 0; index <= segments; ++index)
    {
        const double angle = startAngle + span * static_cast<double>(index) / static_cast<double>(segments);
        const QVector3D offset =
            axisX * static_cast<float>(std::cos(angle) * m_data->radious) +
            axisY * static_cast<float>(std::sin(angle) * m_data->radious);
        const QVector3D point = center + offset;
        m_rawPathPoints3D.push_back({ point.x(), point.y(), point.z() });
    }
}

bool CadArcItem::rebuildControlPoints4Axis
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
            *errorMessage = QStringLiteral("圆弧原始路径点集为空。");
        }

        return false;
    }

    if (m_data == nullptr)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("圆弧原始实体为空。");
        }

        return false;
    }

    m_controlPoints4Axis.clear();
    m_controlPoints4Axis.reserve(m_rawPathPoints3D.size());

    const QVector3D normal = resolveNormal(m_data->extPoint);

    const double centerY = m_data->basePoint.y;
    const double centerZ = m_data->basePoint.z;

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

    // 规则 1：圆弧平面平行于 XY 平面（法向约等于 ±Z）
    // A 由圆心位于 +Z / -Z 决定。
    if (std::abs(normal.x()) < kNormalEps && std::abs(normal.y()) < kNormalEps)
    {
        const double relativeZ = centerZ - axisZ;

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
            // 圆心恰好落在 axisZ 上时，退化到看法向
            fixedRawA = (normal.z() >= 0.0) ? 0.0 : 180.0;
        }

        useFixedA = true;
    }
    // 规则 2：圆弧平面平行于 XZ 平面（法向约等于 ±Y）
    // A 由圆心位于 +Y / -Y 决定。
    else if (std::abs(normal.x()) < kNormalEps && std::abs(normal.z()) < kNormalEps)
    {
        const double relativeY = centerY - axisY;

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
            // 圆心恰好落在 axisY 上时，退化到看法向
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
        // 其它姿态：回退到“刀头指向圆心”的逐点 A 模型
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