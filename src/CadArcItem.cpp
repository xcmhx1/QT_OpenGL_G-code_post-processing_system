// 实现 CadArcItem 模块，对应头文件中声明的主要行为和协作流程。
// 圆弧图元模块，封装圆弧实体的几何离散、颜色解析和方向信息。
#include "pch.h"

#include "CadArcItem.h"

#include <cmath>

namespace
{
constexpr double kTwoPi = 6.28318530717958647692;
// 圆弧与整圆共用采样密度，再按弧长比例折算最终段数。
constexpr int kFullCircleSegments = 128;

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
