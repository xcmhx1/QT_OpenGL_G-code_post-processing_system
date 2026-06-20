#pragma once

#include "libdxfrw/drw_entities.h"

#include <memory>

// Converts a DXF spline into a 3D WCS polyline so all downstream behavior uses
// the existing polyline rendering, editing, sorting and G-code paths.
std::unique_ptr<DRW_Polyline> convertSplineToPolyline(const DRW_Spline* spline);
