// CadSceneRenderCache 头文件
// 声明 CadSceneRenderCache 模块，对外暴露当前组件的核心类型、接口和协作边界。
// 场景渲染缓存模块，负责维护实体对应的 GPU 资源与缓存数据。
#pragma once

#include <memory>
#include <unordered_map>
#include <vector>

#include "CadRenderTypes.h"

class CadItem;

class CadSceneRenderCache
{
public:
    // 为单个图元创建或刷新 GPU 缓冲
    // @param entity 待上传的实体对象
    void uploadEntity(const CadItem* entity);

    // 删除指定实体对应的 GPU 资源
    // @param id 目标实体 ID
    void removeEntityBuffer(EntityId id);

    // 按当前实体列表整体重建缓存
    // @param entities 当前场景实体列表
    void rebuildAllBuffers(const std::vector<std::unique_ptr<CadItem>>& entities);

    // 清空全部 GPU 缓冲
    void clearAllBuffers();

    // 获取实体缓冲表
    // @return 只读实体缓冲映射表引用
    const std::unordered_map<EntityId, EntityGpuBuffer>& entityBuffers() const;

    // 获取实体缓冲表
    // @return 可修改的实体缓冲映射表引用
    std::unordered_map<EntityId, EntityGpuBuffer>& entityBuffers();

private:
    // 实体 ID 到 GPU 缓冲的映射表
    std::unordered_map<EntityId, EntityGpuBuffer> m_entityBuffers;
};
