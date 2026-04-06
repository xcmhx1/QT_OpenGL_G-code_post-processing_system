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
    void uploadEntity(const CadItem* entity);
    void removeEntityBuffer(EntityId id);
    void rebuildAllBuffers(const std::vector<std::unique_ptr<CadItem>>& entities);
    void clearAllBuffers();

    const std::unordered_map<EntityId, EntityGpuBuffer>& entityBuffers() const;
    std::unordered_map<EntityId, EntityGpuBuffer>& entityBuffers();

private:
    std::unordered_map<EntityId, EntityGpuBuffer> m_entityBuffers;
};
