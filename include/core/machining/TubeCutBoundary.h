#pragma once

#include "core/diagnostics/OperationContext.h"
#include "core/diagnostics/OperationResult.h"
#include "core/geometry/GeometryTypes.h"

#include <array>
#include <vector>

namespace cadcam::machining
{
    using geometry::EntityId;
    using geometry::Vector2d;
    using geometry::Vector3d;

    enum class TubeCutResult
    {
        CutsLeftAndRight,
        KeepsLeftAndRight,
        Indeterminate
    };

    struct SeamWindingResult
    {
        bool usable = false;
        int winding = 0;
        int positiveCrossingCount = 0;
        int negativeCrossingCount = 0;
        int touchCount = 0;
        int overlapRunCount = 0;
    };

    struct TubeSectionGeometry
    {
        std::vector<Vector2d> boundary;
        double centerY = 0.0;
        double centerZ = 0.0;
        double yLength = 0.0;
        double zWidth = 0.0;
        double perimeter = 0.0;
        std::array<double, 4> seamPositions{};
    };

    struct SurfaceSpan
    {
        std::size_t sectionSegmentIndex = 0;
        double sectionParameterStart = 0.0;
        double sectionParameterEnd = 0.0;
        double xStart = 0.0;
        double xEnd = 0.0;
    };

    struct UnwrappedBoundaryPoint
    {
        double x = 0.0;
        double perimeterPosition = 0.0;
    };

    struct TubeCutAnalysis
    {
        TubeCutResult result = TubeCutResult::Indeterminate;
        bool projectionMatchesSection = false;
        bool surfaceMappingValid = false;
        int winding = 0;
        std::array<SeamWindingResult, 4> seamResults;
        double maximumJoinGap = 0.0;
        double maximumSurfaceDeviation = 0.0;
        double maximumProjectionCoverageGap = 0.0;
        double projectedCenterY = 0.0;
        double projectedCenterZ = 0.0;
        double projectedYLength = 0.0;
        double projectedZWidth = 0.0;
        std::vector<Vector3d> orderedPath;
        std::vector<EntityId> boundaryEntityIds;
        std::vector<UnwrappedBoundaryPoint> unwrappedBoundary;
    };

    class TubeCutBoundaryClassifier
    {
    public:
        static TubeCutResult classifyWinding
        (
            int globalWinding,
            const std::array<SeamWindingResult, 4>& seamResults
        );

        static OperationResult<TubeSectionGeometry> prepareSection
        (
            const TubeSectionGeometry& source,
            const OperationContext& context,
            double sectionMatchEpsilon = 1.0e-4
        );

        static OperationResult<TubeCutAnalysis> analyze
        (
            const std::vector<Vector3d>& orderedPath,
            const std::vector<EntityId>& boundaryEntityIds,
            double maximumJoinGap,
            const TubeSectionGeometry& section,
            const OperationContext& context,
            double surfaceMappingEpsilon = 1.0e-4,
            double sectionMatchEpsilon = 1.0e-4
        );
    };
}
