// CadEntityRenderer 头文件
// 声明 CadEntityRenderer 模块，对外暴露当前组件的核心类型、接口和协作边界。
// 实体渲染模块，负责把缓存好的图元数据提交给 OpenGL 绘制。
#pragma once

#include <memory>
#include <vector>

#include <QMatrix4x4>
#include <QOpenGLShaderProgram>

#include "AppTheme.h"
#include "CadRenderTypes.h"

class CadItem;
class CadSceneRenderCache;

class CadEntityRenderer
{
public:
    // 绘制场景实体
    // 遍历场景中的所有图元，按缓存好的 GPU 资源逐个发起绘制
    // selectedEntityId 用于在绘制阶段直接叠加选中高亮效果
    // @param shader 通用绘制 Shader
    // @param mvp 当前视图使用的模型视图投影矩阵
    // @param entities 场景实体列表
    // @param sceneRenderCache 实体对应的 GPU 缓冲缓存
    // @param selectedEntityId 当前选中的实体 ID
    static void renderEntities
    (
        QOpenGLShaderProgram& shader,
        const QMatrix4x4& mvp,
        const std::vector<std::unique_ptr<CadItem>>& entities,
        CadSceneRenderCache& sceneRenderCache,
        EntityId selectedEntityId,
        const AppThemeColors& theme
    );
};
