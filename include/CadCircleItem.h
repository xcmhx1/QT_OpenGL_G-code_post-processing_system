#pragma once

// 声明 CadCircleItem 模块，对外暴露当前组件的核心类型、接口和协作边界。
// 圆图元模块，负责圆实体的离散显示数据和加工方向生成。
#include "CadItem.h"

class CadCircleItem : public CadItem
{
public:
    explicit CadCircleItem(DRW_Entity* entity, QObject* parent = nullptr);

    void buildGeometryDatay() override;

    DRW_Circle* m_data = nullptr;
};
