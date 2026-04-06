// 实现 CadCircleItem 模块，对应头文件中声明的主要行为和协作流程。
// 圆图元模块，负责圆实体的离散显示数据和加工方向生成。
#include "pch.h"

#include "CadCircleItem.h"

#include <cmath>

namespace
{
// 统一使用 2π，避免在采样时重复书写常量。
constexpr double kTwoPi = 6.28318530717958647692;
// 圆默认采样为 128 段，兼顾显示平滑度和顶点数量。
constexpr int kCircleSegments = 128;

QVector3D resolveNormal(const DRW_Coord& extPoint)
{
    // DXF 圆使用 extrusion direction 表示所在平面法向。
    QVector3D normal(extPoint.x, extPoint.y, extPoint.z);

    // 极端情况下法向可能缺失，回退到世界 Z 轴。
    if (normal.lengthSquared() <= 1.0e-12f)
    {
        return QVector3D(0.0f, 0.0f, 1.0f);
    }

    normal.normalize();
    return normal;
}

void buildPlaneBasis(const QVector3D& normal, QVector3D& axisX, QVector3D& axisY)
{
    // 当法向几乎平行于 Z 轴时，直接使用世界 X 轴作为首选基向量。
    if (std::abs(normal.x()) <= 1.0e-6f && std::abs(normal.y()) <= 1.0e-6f)
    {
        axisX = QVector3D(1.0f, 0.0f, 0.0f);
        axisY = QVector3D::crossProduct(normal, axisX);

        // 如果叉乘退化，直接回退到世界 Y 轴，保证基底可用。
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

    // 根据法向挑选一个不平行的辅助轴，构造稳定的平面基。
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

    if (m_data == nullptr || m_data->radious <= 0.0)
    {
        return;
    }

    // DXF 圆由圆心、法向和半径定义，这里先恢复局部平面坐标系。
    const QVector3D center(m_data->basePoint.x, m_data->basePoint.y, m_data->basePoint.z);
    const QVector3D normal = resolveNormal(m_data->extPoint);

    QVector3D axisX;
    QVector3D axisY;
    buildPlaneBasis(normal, axisX, axisY);

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
