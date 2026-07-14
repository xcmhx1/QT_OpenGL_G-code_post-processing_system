#pragma once

#include "core/diagnostics/Diagnostic.h"

#include <QVector>

#include <cstddef>

class DRW_Spline;

struct SplineParityReport
{
    bool equivalent = false;
    std::size_t legacyPointCount = 0;
    std::size_t corePointCount = 0;
    double maximumPointDistance = 0.0;
    int firstDifferentIndex = -1;
    QVector<Diagnostic> diagnostics;
};

SplineParityReport compareSplineWithLegacy(const DRW_Spline& spline);
