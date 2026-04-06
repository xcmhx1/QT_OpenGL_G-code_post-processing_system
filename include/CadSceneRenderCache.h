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
    // 为单个图元创建或刷新 GPU 缓冲。
    void uploadEntity(const CadItem* entity);
    // 删除指定实体对应的 GPU 资源。
    void removeEntityBuffer(EntityId id);
    // 按当前实体列表整体重建缓存，常用于场景整体刷新。
    void rebuildAllBuffers(const std::vector<std::unique_ptr<CadItem>>& entities);
    // 清空全部 GPU 缓冲。
    void clearAllBuffers();

    // 提供缓存表访问接口，供渲染器和协调器共享使用。
    const std::unordered_map<EntityId, EntityGpuBuffer>& entityBuffers() const;
    std::unordered_map<EntityId, EntityGpuBuffer>& entityBuffers();

private:
    std::unordered_map<EntityId, EntityGpuBuffer> m_entityBuffers;
};
