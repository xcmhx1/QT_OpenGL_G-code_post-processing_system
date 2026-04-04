#pragma once

#include <QColor>
#include <QObject>
#include <QVector>
#include <QVector3D>

#include <include/libdxfrw/libdxfrw.h>

// 几何数据
struct GeometryData
{
    // 三维顶点
    QVector<QVector3D> vertices;      
};

// Cad图元基类
class CadItem : public QObject
{
    Q_OBJECT

public:
    explicit CadItem(DRW_Entity* entity, QObject* parent = nullptr);
    virtual ~CadItem() = default;

    // 生成几何数据
    virtual void buildGeometryDatay() = 0;

    // 生成加工方向（单个三维向量）,根据几何数据生成
    void buildProcessDirection();

    // 生成颜色(根据颜色综合解析)
    QColor buildColor();

    ///颜色解析
    // 由AutoCad颜色索引得到颜色
    QColor colorFromIndex();
    // 由Color24得到颜色
    QColor colorFromTrueColor();
    // 由图层颜色得到颜色
    QColor colorFromLayer();

    // 原始数据指针
    DRW_Entity* m_nativeEntity = nullptr;
    // 图元类型
    DRW::ETYPE m_type;
    // 加工顺序
    int m_processOrder = -1;
    // 是否反向加工
    bool m_isReverse = false;
    // 集合形状
    GeometryData m_geometry;
    // 加工方向矢量
    QVector3D m_processDirection;
    // 颜色
    QColor m_color;
};