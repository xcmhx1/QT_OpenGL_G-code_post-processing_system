// 实现 CadEllipseItem 模块，对应头文件中声明的主要行为和协作流程。
// 椭圆图元模块，负责椭圆实体的离散显示数据和方向信息构建。
#include "pch.h"

#include "CadEllipseItem.h"

#include <cmath>

namespace
{
constexpr double kAxisEps = 1.0e-8;
constexpr double kRadToDeg = 57.2957795130823208768;
constexpr double kTwoPi = 6.28318530717958647692;
// 整椭圆的基础采样密度，局部弧段会按参数跨度缩放。
constexpr int kFullEllipseSegments = 128;

double normalizeAnglePositive(double angle)
{
    double normalized = std::fmod(angle, kTwoPi);

    if (normalized < 0.0)
    {
        normalized += kTwoPi;
    }

    return normalized;
}

QVector3D resolveNormal(const DRW_Coord& extPoint)
{
    // 椭圆所在平面同样由 extrusion direction 给出。
    QVector3D normal(extPoint.x, extPoint.y, extPoint.z);

    if (normal.lengthSquared() <= 1.0e-12f)
    {
        return QVector3D(0.0f, 0.0f, 1.0f);
    }

    normal.normalize();
    return normal;
}

QVector3D buildMinorAxis(const QVector3D& normal, const QVector3D& majorAxis, double ratio)
{
    // 短轴方向由法向与长轴叉乘得到，长度再乘以 ratio。
    QVector3D minorDirection = QVector3D::crossProduct(normal, majorAxis);

    if (minorDirection.lengthSquared() <= 1.0e-12f)
    {
        // 如果长轴与法向导致叉乘退化，换辅助轴再尝试一次。
        const QVector3D helper = std::abs(majorAxis.z()) < 0.999f
            ? QVector3D(0.0f, 0.0f, 1.0f)
            : QVector3D(0.0f, 1.0f, 0.0f);

        minorDirection = QVector3D::crossProduct(helper, majorAxis);
    }

    if (minorDirection.lengthSquared() <= 1.0e-12f)
    {
        return QVector3D();
    }

    minorDirection.normalize();
    return minorDirection * static_cast<float>(majorAxis.length() * ratio);
}

bool isFullEllipsePath(const DRW_Ellipse* ellipse)
{
    if (ellipse == nullptr)
    {
        return false;
    }

    const double span = ellipse->endparam - ellipse->staparam;
    return std::abs(span) < 1.0e-10
        || std::abs(std::abs(span) - kTwoPi) < 1.0e-10;
}

double effectiveClosedEllipseStartParameter(const CadEllipseItem* item)
{
    if (item != nullptr && item->m_hasCustomProcessStart)
    {
        return normalizeAnglePositive(item->m_processStartParameter);
    }

    return (item != nullptr && item->m_data != nullptr)
        ? normalizeAnglePositive(item->m_data->staparam)
        : 0.0;
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
    const QVector3D center(m_data->basePoint.x, m_data->basePoint.y, m_data->basePoint.z);
    const QVector3D majorAxis(m_data->secPoint.x, m_data->secPoint.y, m_data->secPoint.z);

    // 没有有效长轴或 ratio 非法时无法构建椭圆。
    if (majorAxis.lengthSquared() <= 1.0e-12f || m_data->ratio <= 0.0)
    {
        return;
    }

    const QVector3D normal = resolveNormal(m_data->extPoint);
    const QVector3D minorAxis = buildMinorAxis(normal, majorAxis, m_data->ratio);

    // 短轴构建失败通常意味着几何定义退化，不再继续采样。
    if (minorAxis.lengthSquared() <= 1.0e-12f)
    {
        return;
    }

    double startParam = m_data->staparam;
    double endParam = m_data->endparam;

    // 完整椭圆在 DXF 中可能以相同起止参数或完整 2π 参数区间表示。
    if (std::abs(endParam - startParam) < 1.0e-10 || std::abs(std::abs(endParam - startParam) - kTwoPi) < 1.0e-10)
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
        const QVector3D point =
            center +
            majorAxis * static_cast<float>(std::cos(t)) +
            minorAxis * static_cast<float>(std::sin(t));

        m_geometry.vertices.append(point);
    }
}

void CadEllipseItem::rebuildRawPathPoints3D()
{
    m_rawPathPoints3D.clear();

    if (m_data == nullptr)
    {
        return;
    }

    const QVector3D center(m_data->basePoint.x, m_data->basePoint.y, m_data->basePoint.z);
    const QVector3D majorAxis(m_data->secPoint.x, m_data->secPoint.y, m_data->secPoint.z);

    if (majorAxis.lengthSquared() <= 1.0e-12f || m_data->ratio <= 0.0)
    {
        return;
    }

    const QVector3D normal = resolveNormal(m_data->extPoint);
    const QVector3D minorAxis = buildMinorAxis(normal, majorAxis, m_data->ratio);

    if (minorAxis.lengthSquared() <= 1.0e-12f)
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
        const QVector3D point =
            center +
            majorAxis * static_cast<float>(std::cos(parameter)) +
            minorAxis * static_cast<float>(std::sin(parameter));
        m_rawPathPoints3D.push_back({ point.x(), point.y(), point.z() });
    }
}

bool CadEllipseItem::rebuildControlPoints4Axis
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
            *errorMessage = QStringLiteral("椭圆原始路径点集为空。");
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

        // 机床坐标：把当前原始点在对应 A 旋转后对齐到顶部
        // 顶部对齐后：
        //   X 保持原值
        //   Y 固定到回转轴的 axisY
        //   Z 变为 axisZ + 半径
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