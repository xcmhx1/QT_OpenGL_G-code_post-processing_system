#pragma once

#include <vector>

#include "CadRenderTypes.h"

class CadItem;
class DrawStateMachine;

namespace CadPreviewBuilder
{
    std::vector<TransientPrimitive> buildTransientPrimitives
    (
        const DrawStateMachine& state,
        CadItem* selectedItem
    );
}
