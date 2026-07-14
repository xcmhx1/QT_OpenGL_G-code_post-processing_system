#pragma once

// 声明 CadItem 模块，对外暴露当前组件的核心类型、接口和协作边界。
// 图元基类模块，定义原生实体绑定、几何缓存和公共图元行为。
#include <QColor>
#include <QObject>
#include <QString>
#include <QVector>
#include <QVector3D>
#include <vector>

#include <libdxfrw.h>

#include "core/geometry/GeometryTypes.h"

// 几何数据
struct GeometryData
{
    // 按渲染顺序存放离散后的三维顶点。
    // 对线段类图元通常是关键点，对圆弧/椭圆/多段线则是采样后的折线点列。
    QVector<QVector3D> vertices;      
};

struct RawPathPoint3D
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

// 用户为四轴方管排序显式指定的加工边界角色。
enum class RotaryEndCutRole
{
    None,
    Break,
    Waste
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

    // 按当前加工语义重建图元的原始三维路径点集。
    virtual void rebuildRawPathPoints3D() = 0;

    const std::vector<RawPathPoint3D>& rawPathPoints3D() const;

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
    // 兼容字段：文档生命周期内稳定的核心图元编号，未来迁出 CadItem。
    cadcam::geometry::EntityId m_entityId = 0;
    // Compatibility projection of the current ProcessPlan. Not a planning or NC source of truth.
    int m_processOrder = -1;
    // Compatibility projection of the current ProcessPlan. Not a planning or NC source of truth.
    int m_processContinuousGroupId = -1;
    // Compatibility projection of the current ProcessPlan. Not a planning or NC source of truth.
    bool m_isReverse = false;
    // 标记当前图元是否显式指定了闭合路径的起刀缝点参数。
    bool m_hasCustomProcessStart = false;
    // 闭合路径的起刀缝点参数；圆使用弧度，完整椭圆使用参数方程角。
    double m_processStartParameter = 0.0;
    // 记录当前图元是否处于选中状态。
    bool m_isSelected = false;
    // 用户指定的方管加工边界；仅作为当前文档的加工排序参考，不写入 DXF。
    int m_rotaryEndCutPairId = -1;
    RotaryEndCutRole m_rotaryEndCutRole = RotaryEndCutRole::None;
    // 由废面边界区间或内部线识别推导，不参与排序、加工可视化和 G 代码输出。
    bool m_excludedFromProcessing = false;
    // 用户执行内部线识别后保留的独立排除来源，刷新废面规则时不会丢失。
    bool m_excludedAsInternalGeometry = false;
    // 渲染层直接消费的离散几何缓存。
    GeometryData m_geometry;
    // 图元按当前加工顺序离散后的原始三维点集缓存。
    std::vector<RawPathPoint3D> m_rawPathPoints3D;
    // 由几何推导出的标准化加工方向。
    QVector3D m_processDirection;
    // 当前图元的最终显示颜色缓存。
    QColor m_color;

protected:
    void clearPathCaches();
};
