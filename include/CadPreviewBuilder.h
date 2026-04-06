// 声明 CadPreviewBuilder 模块，对外暴露当前组件的核心类型、接口和协作边界。
// 预览构建模块，负责根据当前命令状态生成 transient 预览图元。
#pragma once

#include <vector>

#include "CadRenderTypes.h"

class CadItem;
class DrawStateMachine;

namespace CadPreviewBuilder
{
    // 根据当前命令状态生成 transient 图元。
    // 这些图元只参与预览渲染，不会写入文档模型。
    std::vector<TransientPrimitive> buildTransientPrimitives
    (
        const DrawStateMachine& state,
        CadItem* selectedItem
    );
}
