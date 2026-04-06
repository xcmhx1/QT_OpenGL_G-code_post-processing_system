// 声明 CadEntityRenderer 模块，对外暴露当前组件的核心类型、接口和协作边界。
// 实体渲染模块，负责把缓存好的图元数据提交给 OpenGL 绘制。
#pragma once

#include <memory>
#include <vector>

#include <QMatrix4x4>
#include <QOpenGLShaderProgram>

#include "CadRenderTypes.h"

class CadItem;
class CadSceneRenderCache;

class CadEntityRenderer
{
public:
    static void renderEntities
    (
        QOpenGLShaderProgram& shader,
        const QMatrix4x4& mvp,
        const std::vector<std::unique_ptr<CadItem>>& entities,
        CadSceneRenderCache& sceneRenderCache,
        EntityId selectedEntityId
    );
};
