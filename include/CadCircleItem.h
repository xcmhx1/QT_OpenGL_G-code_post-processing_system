#pragma once

// 声明 CadCircleItem 模块，对外暴露当前组件的核心类型、接口和协作边界。
// 圆图元模块，负责圆实体的离散显示数据和加工方向生成。
#include "CadItem.h"

class CadCircleItem : public CadItem
{
public:
    explicit CadCircleItem(DRW_Entity* entity, QObject* parent = nullptr);

    // 圆会被离散为闭合折线，顶点数由实现文件中的采样常量控制。
    void buildGeometryDatay() override;

    // 指向原生圆实体，圆心、半径和法向都从这里读取。
    DRW_Circle* m_data = nullptr;
};
