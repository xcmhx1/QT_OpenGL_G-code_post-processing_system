#pragma once

#include "core/diagnostics/OperationContext.h"
#include "core/diagnostics/OperationResult.h"
#include "core/machining/TubeCutBoundary.h"
#include "core/topology/PathTopology.h"

#include <cstdint>
#include <vector>

namespace cadcam::machining
{
    enum class TubeSurfaceRegion
    {
        Top,
        TopRightCorner,
        Right,
        BottomRightCorner,
        Bottom,
        BottomLeftCorner,
        Left,
        TopLeftCorner,
        Mixed,
        Unknown
    };

    struct TubeCornerGeometry
    {
        geometry::Vector2d center;
        double radius = 0.0;
        int yDirection = 0;
        int zDirection = 0;
    };

    struct TubeSectionModel
    {
        std::uint64_t contentRevision = 0;
        TubeSectionGeometry geometry;
        double centerX = 0.0;
        double maximumPlaneDeviation = 0.0;
        std::vector<geometry::Vector3d> orderedBoundary3D;
        std::vector<geometry::EntityId> outerBoundaryEntityIds;
        int roundedCornerCount = 0;
        std::vector<double> cornerRadii;
        double cornerRadius = 0.0;
        double cornerConfidence = 0.0;
        std::vector<TubeCornerGeometry> corners;
    };

    enum class InternalPathClassificationMode
    {
        BranchedClosedUnit,
        TubeSectionInset
    };

    struct InternalPathClassification
    {
        InternalPathClassificationMode mode =
            InternalPathClassificationMode::BranchedClosedUnit;
        std::vector<geometry::EntityId> removableEntityIds;
        int candidateComponentCount = 0;
        int eligibleComponentCount = 0;
        int outerBoundaryEntityCount = 0;
        double insetDistance = 0.0;
        int preservedSafetyBandCount = 0;
        int ambiguousComponentCount = 0;
        int skippedComponentCount = 0;
    };

    struct TubeSectionPolicy
    {
        double connectionTolerance = 1.0;
        double numericalEpsilon = 1.0e-5;
        double maximumPlaneDeviation = 0.1;
        double boundaryDistanceTolerance = 0.1;
        double interiorDistanceTolerance = 0.1;
        double minimumSectionArea = 1.0e-6;
    };

    class TubeSectionAnalyzer
    {
    public:
        static OperationResult<TubeSectionModel> buildFromSelection
        (
            const topology::TopologyInput& input,
            const topology::PathTopology& topology,
            const std::vector<geometry::EntityId>& selectedEntityIds,
            const TubeSectionPolicy& policy,
            const OperationContext& context
        );

        static OperationResult<TubeSectionModel> findBest
        (
            const topology::TopologyInput& input,
            const topology::PathTopology& topology,
            const TubeSectionPolicy& policy,
            const OperationContext& context
        );

        static OperationResult<InternalPathClassification> classifyInternalPaths
        (
            const topology::TopologyInput& input,
            const topology::PathTopology& topology,
            const TubeSectionModel& section,
            const TubeSectionPolicy& policy,
            const OperationContext& context
        );

        static OperationResult<InternalPathClassification> classifyTopologicalInteriorPaths
        (
            const topology::TopologyInput& input,
            const topology::PathTopology& topology,
            const TubeSectionPolicy& policy,
            const OperationContext& context
        );
    };
}
