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
