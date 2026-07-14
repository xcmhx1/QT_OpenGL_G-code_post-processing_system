#pragma once

#include <memory>

class DRW_Spline;

std::unique_ptr<DRW_Spline> cloneSplineEntity(const DRW_Spline* source);
