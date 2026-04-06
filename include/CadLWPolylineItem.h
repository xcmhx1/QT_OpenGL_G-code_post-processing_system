#pragma once

// 声明 CadLWPolylineItem 模块，对外暴露当前组件的核心类型、接口和协作边界。
// 轻量多段线图元模块，负责轻量多段线的离散显示和 bulge 解释。
#include "CadItem.h"

class CadLWPolylineItem : public CadItem
{
public:
    explicit CadLWPolylineItem(DRW_Entity* entity, QObject* parent = nullptr);

    // 轻量多段线和普通多段线处理思路一致，但顶点来源是 2D 顶点列表加统一 elevation。
    void buildGeometryDatay() override;

    // 指向原生轻量多段线实体，包含 2D 顶点、bulge 和 elevation。
    DRW_LWPolyline* m_data = nullptr;
};
