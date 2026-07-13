#pragma once

#include "core/geometry/GeometryTypes.h"

#include <algorithm>
#include <limits>

namespace cadcam::geometry
{
    class EntityIdAllocator
    {
    public:
        EntityId ensure(EntityId existingId)
        {
            if (existingId != 0)
            {
                if (existingId < std::numeric_limits<EntityId>::max())
                {
                    m_nextId = std::max(m_nextId, existingId + 1);
                }
                return existingId;
            }

            return m_nextId++;
        }

        void reset()
        {
            m_nextId = 1;
        }

    private:
        EntityId m_nextId = 1;
    };
}
