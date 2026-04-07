// CadPreviewBuilder 头文件
// 声明 CadPreviewBuilder 模块，对外暴露当前组件的核心类型、接口和协作边界。
// 预览构建模块，负责根据当前命令状态生成 transient 预览图元。

#pragma once

// 标准库
#include <vector>

// CAD 模块内部依赖
#include "CadRenderTypes.h"

// 前向声明
class CadItem;
class DrawStateMachine;

// CadPreviewBuilder 命名空间：
// 包含预览图元构建相关的静态函数，用于生成临时预览图元，这些图元只用于显示，不保存到文档中。
namespace CadPreviewBuilder
{
    // 根据当前命令状态生成 transient 图元。
    // 这些图元只参与预览渲染，不会写入文档模型。
    // 该函数根据绘图状态机的当前状态（如绘制点、线、圆等）和选中的实体，构建预览图元。
    // 预览图元用于在用户执行命令时提供视觉反馈，例如绘制过程中的临时线段、圆等。
    // @param state 当前绘图状态机，包含命令状态、子模式、已输入的点等信息
    // @param selectedItem 当前选中的实体，用于在某些编辑命令中生成预览
    // @return 临时图元向量，包含用于预览的图元数据
    std::vector<TransientPrimitive> buildTransientPrimitives
    (
        const DrawStateMachine& state,
        CadItem* selectedItem
    );
}