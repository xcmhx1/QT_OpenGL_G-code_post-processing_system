#include "compatibility/legacy/SplineEntityClone.h"

#include <libdxfrw.h>

std::unique_ptr<DRW_Spline> cloneSplineEntity(const DRW_Spline* source)
{
    if (source == nullptr)
    {
        return nullptr;
    }

    auto clone = std::make_unique<DRW_Spline>(*source);
    clone->controllist.clear();
    clone->fitlist.clear();
    clone->controllist.reserve(source->controllist.size());
    clone->fitlist.reserve(source->fitlist.size());

    for (const std::shared_ptr<DRW_Coord>& point : source->controllist)
    {
        clone->controllist.push_back
            (point != nullptr ? std::make_shared<DRW_Coord>(*point) : nullptr);
    }
    for (const std::shared_ptr<DRW_Coord>& point : source->fitlist)
    {
        clone->fitlist.push_back
            (point != nullptr ? std::make_shared<DRW_Coord>(*point) : nullptr);
    }
    return clone;
}
