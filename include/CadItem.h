#pragma once

// 声明 CadItem 模块，对外暴露当前组件的核心类型、接口和协作边界。
// 图元基类模块，定义原生实体绑定、几何缓存和公共图元行为。
#include <QColor>
#include <QObject>
#include <QString>
#include <QVector>
#include <QVector3D>

#include <libdxfrw.h>

// 几何数据
struct GeometryData
{
    // 按渲染顺序存放离散后的三维顶点。
    // 对线段类图元通常是关键点，对圆弧/椭圆/多段线则是采样后的折线点列。
    QVector<QVector3D> vertices;      
};

// Cad图元基类
class CadItem : public QObject
{
    Q_OBJECT

public:
    // entity 由文档层持有，CadItem 只保存原生实体指针并围绕它构建显示数据。
    explicit CadItem(DRW_Entity* entity, QObject* parent = nullptr);
    virtual ~CadItem() = default;

    // 从原生 DXF 实体重建当前图元的离散几何。
    // 每个派生类都需要把自身实体转换为适合 OpenGL 绘制的顶点序列。
    virtual void buildGeometryDatay() = 0;

    // 根据离散后的几何顶点推导一个加工方向向量。
    // 当前实现取首个有效边方向，并按 m_isReverse 决定是否翻转。
    void buildProcessDirection();

    // 综合 true color、ACI 索引色和图层色规则得到最终显示颜色。
    QColor buildColor();

    ///颜色解析
    // 按 AutoCAD ACI 索引解析颜色。
    QColor colorFromIndex();
    // 解析 24 位真彩色。
    QColor colorFromTrueColor();
    // 解析图层颜色；当前基类里仅提供兜底行为。
    QColor colorFromLayer();

    // 指向原始 libdxfrw 实体，几何和颜色都从这里读取。
    DRW_Entity* m_nativeEntity = nullptr;
    // 缓存实体类型，避免每次都回查原生对象。
    DRW::ETYPE m_type;
    // 供后续排序/后处理使用的加工顺序标记。
    int m_processOrder = -1;
    // 标记当前图元是否采用反向加工方向。
    bool m_isReverse = false;
    // 标记当前图元是否显式指定了闭合路径的起刀缝点参数。
    bool m_hasCustomProcessStart = false;
    // 闭合路径的起刀缝点参数；圆使用弧度，完整椭圆使用参数方程角。
    double m_processStartParameter = 0.0;
    // 记录当前图元是否处于选中状态。
    bool m_isSelected = false;
    // 渲染层直接消费的离散几何缓存。
    GeometryData m_geometry;
    // 由几何推导出的标准化加工方向。
    QVector3D m_processDirection;
    // 当前图元的最终显示颜色缓存。
    QColor m_color;

    // 最近一次 4 轴导出规划得到的最终点序（单一真源：导出与预览共用）。
    QVector<QVector3D> m_exportPathPoints;
    bool m_hasExportPathPoints = false;
    bool m_exportDirectionReversed = false;
    QString m_exportProcessMode;
    bool m_exportRequiresA = false;
    double m_exportAStartDeg = 0.0;
    double m_exportAEndDeg = 0.0;
    double m_exportARangeDeg = 0.0;
    QString m_exportRegionSummary;
};
