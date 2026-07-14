#pragma once

#include "application/geometry/GeometrySnapshot.h"

class CadDocument;

struct GeometrySnapshotParityEntry
{
    std::size_t sourceIndex = 0;
    cadcam::geometry::EntityId entityId = 0;
    bool equivalent = false;
    bool closedMatches = false;
    std::size_t legacyPointCount = 0;
    std::size_t snapshotPointCount = 0;
    double legacyPathLength = 0.0;
    double snapshotPathLength = 0.0;
    double maximumPointDistance = 0.0;
    int firstDifferentIndex = -1;
};

struct GeometrySnapshotParityReport
{
    bool equivalent = false;
    double maximumPointDistance = 0.0;
    std::vector<GeometrySnapshotParityEntry> entries;
    QVector<Diagnostic> diagnostics;
};

class GeometrySnapshotParityVerifier
{
public:
    OperationResult<GeometrySnapshotParityReport> verify
    (
        const CadDocument& document,
        const GeometrySnapshot& snapshot,
        const OperationContext& context
    ) const;
};
