#pragma once

#include "core/diagnostics/OperationContext.h"
#include "core/diagnostics/OperationResult.h"
#include "core/geometry/Path3D.h"
#include "core/machining/TubeSection.h"

#include <QString>

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace cadcam::machining
{
    enum class TubeZone16 : std::uint8_t
    {
        TopFace = 0,
        TopToTopRightBoundary,
        TopRightCorner,
        TopRightToRightBoundary,
        RightFace,
        RightToBottomRightBoundary,
        BottomRightCorner,
        BottomRightToBottomBoundary,
        BottomFace,
        BottomToBottomLeftBoundary,
        BottomLeftCorner,
        BottomLeftToLeftBoundary,
        LeftFace,
        LeftToTopLeftBoundary,
        TopLeftCorner,
        TopLeftToTopBoundary
    };

    using TubeZoneMask = std::uint16_t;
    constexpr std::size_t kTubeZone16Count = 16U;

    constexpr TubeZoneMask tubeZoneBit(TubeZone16 zone)
    {
        return static_cast<TubeZoneMask>
            (TubeZoneMask{ 1U } << static_cast<std::uint8_t>(zone));
    }

    QString tubeZoneName(TubeZone16 zone);

    struct TubeSectionProjection
    {
        TubeZone16 zone = TubeZone16::TopFace;
        double perimeterPosition = 0.0;
        double signedDistanceToShell = 0.0;
        double absoluteDistanceToShell = 0.0;
        double confidence = 0.0;
        bool valid = false;
        bool onBoundary = false;
        bool ambiguous = false;
    };

    struct TubeZoneSpan
    {
        bool occupied = false;
        double minimumX = 0.0;
        double maximumX = 0.0;
        double projectedLength = 0.0;
        double maximumShellDeviation = 0.0;
        std::size_t contributingSegmentCount = 0U;
    };

    struct ProcessUnitZoneProfile
    {
        TubeZoneMask occupancyMask = 0U;
        std::array<TubeZoneSpan, kTubeZone16Count> zoneSpans;
        TubeZone16 entryZone = TubeZone16::TopFace;
        TubeZone16 exitZone = TubeZone16::TopFace;
        double entryPerimeterPosition = 0.0;
        double exitPerimeterPosition = 0.0;
        double maximumShellDeviation = 0.0;
        double averageShellDeviation = 0.0;
        bool closed = false;
        bool uncertain = false;
    };

    class TubeSectionProjector
    {
    public:
        static TubeSectionProjection project
        (
            const TubeSectionModel& section,
            const geometry::Vector2d& yzPoint,
            double projectionTolerance
        );

        static OperationResult<ProcessUnitZoneProfile> buildProfile
        (
            const TubeSectionModel& section,
            const std::vector<geometry::Path3D>& orderedPaths,
            bool closed,
            double projectionTolerance,
            const OperationContext& context
        );
    };
}
