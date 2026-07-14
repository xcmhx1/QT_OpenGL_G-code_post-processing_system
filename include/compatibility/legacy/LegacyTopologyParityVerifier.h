#pragma once

#include "application/geometry/GeometrySnapshot.h"
#include "application/tasks/TaskContext.h"
#include "core/topology/PathTopology.h"

class CadDocument;

struct LegacyTopologyParityOptions
{
    std::vector<cadcam::geometry::EntityId> candidates;
    std::vector<cadcam::geometry::EntityId> seeds;
    std::vector<cadcam::geometry::EntityId> preferred;
};

struct LegacyTopologyParityReport
{
    bool equivalent = false;
    bool recordsEquivalent = false;
    bool adjacencyEquivalent = false;
    bool componentsEquivalent = false;
    bool seededLoopEquivalent = false;
    bool bestLoopEquivalent = false;
    bool exactEquivalent = false;
    bool equivalentAfterCyclicRotation = false;
    bool equivalentAfterReverseAndRotation = false;
    double maximumPointDistance = 0.0;
    double closureGapDifference = 0.0;
    QVector<Diagnostic> diagnostics;
};

class LegacyTopologyParityVerifier
{
public:
    OperationResult<LegacyTopologyParityReport> verify
    (
        const CadDocument& document,
        const GeometrySnapshot& snapshot,
        const LegacyTopologyParityOptions& options,
        const cadcam::topology::PathTopologyTolerance& tolerance,
        const TaskContext& taskContext
    ) const;
};
