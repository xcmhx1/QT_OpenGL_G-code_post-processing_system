#include "pch.h"

#include "core/machining/TubeCutBoundary.h"

#include <QVariantList>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace cadcam::machining
{
    namespace
    {
        constexpr double kCalculationEpsilon = 1.0e-12;

        struct SectionProjection
        {
            bool valid = false;
            std::size_t segmentIndex = 0;
            double segmentFactor = 0.0;
            double perimeterPosition = 0.0;
            double distance = std::numeric_limits<double>::max();
            Vector2d point;
        };

        struct MappedPath
        {
            enum class Failure
            {
                None,
                PointOffSurface,
                SegmentLeavesBoundary
            };

            bool valid = false;
            std::vector<SurfaceSpan> spans;
            double maximumDeviation = 0.0;
            Failure failure = Failure::None;
        };

        double distance2D(const Vector2d& left, const Vector2d& right)
        {
            return std::hypot(left.x - right.x, left.y - right.y);
        }

        double distance3D(const Vector3d& left, const Vector3d& right)
        {
            return std::sqrt
            (
                (left.x - right.x) * (left.x - right.x)
                + (left.y - right.y) * (left.y - right.y)
                + (left.z - right.z) * (left.z - right.z)
            );
        }

        bool finite(const Vector2d& point)
        {
            return std::isfinite(point.x) && std::isfinite(point.y);
        }

        bool finite(const Vector3d& point)
        {
            return std::isfinite(point.x)
                && std::isfinite(point.y)
                && std::isfinite(point.z);
        }

        std::vector<double> cumulativeLengths(const std::vector<Vector2d>& boundary)
        {
            std::vector<double> cumulative(boundary.size() + 1, 0.0);

            for (std::size_t index = 0; index < boundary.size(); ++index)
            {
                cumulative[index + 1] = cumulative[index]
                    + distance2D(boundary[index], boundary[(index + 1) % boundary.size()]);
            }

            return cumulative;
        }

        SectionProjection projectToBoundary
        (
            const Vector2d& point,
            const std::vector<Vector2d>& boundary,
            const std::vector<double>& cumulative
        )
        {
            SectionProjection best;

            for (std::size_t index = 0; index < boundary.size(); ++index)
            {
                const Vector2d& start = boundary[index];
                const Vector2d& end = boundary[(index + 1) % boundary.size()];
                const double deltaY = end.x - start.x;
                const double deltaZ = end.y - start.y;
                const double lengthSquared = deltaY * deltaY + deltaZ * deltaZ;

                if (lengthSquared <= kCalculationEpsilon)
                {
                    continue;
                }

                const double factor = std::clamp
                (
                    ((point.x - start.x) * deltaY + (point.y - start.y) * deltaZ)
                        / lengthSquared,
                    0.0,
                    1.0
                );
                const Vector2d projected
                {
                    start.x + deltaY * factor,
                    start.y + deltaZ * factor
                };
                const double candidateDistance = distance2D(point, projected);

                if (candidateDistance < best.distance)
                {
                    best.valid = true;
                    best.segmentIndex = index;
                    best.segmentFactor = factor;
                    best.perimeterPosition = cumulative[index]
                        + std::sqrt(lengthSquared) * factor;
                    best.distance = candidateDistance;
                    best.point = projected;
                }
            }

            return best;
        }

        Vector2d pointAtParameter
        (
            const std::vector<Vector2d>& boundary,
            const std::vector<double>& cumulative,
            double parameter
        )
        {
            const double perimeter = cumulative.back();
            double wrapped = std::fmod(parameter, perimeter);

            if (wrapped < 0.0)
            {
                wrapped += perimeter;
            }

            const auto upper = std::upper_bound(cumulative.begin(), cumulative.end(), wrapped);
            const std::size_t index = std::min
            (
                boundary.size() - 1,
                static_cast<std::size_t>(std::distance(cumulative.begin(), upper) - 1)
            );
            const double length = cumulative[index + 1] - cumulative[index];
            const double factor = length > kCalculationEpsilon
                ? (wrapped - cumulative[index]) / length
                : 0.0;
            const Vector2d& start = boundary[index];
            const Vector2d& end = boundary[(index + 1) % boundary.size()];
            return
            {
                start.x + (end.x - start.x) * factor,
                start.y + (end.y - start.y) * factor
            };
        }

        double wrapParameter(double value, double perimeter)
        {
            double wrapped = std::fmod(value, perimeter);

            if (wrapped < 0.0)
            {
                wrapped += perimeter;
            }

            return wrapped;
        }

        bool parameterAtSeam(double parameter, double seam, double perimeter, double epsilon)
        {
            const double wrappedParameter = wrapParameter(parameter, perimeter);
            const double wrappedSeam = wrapParameter(seam, perimeter);
            const double difference = std::abs(wrappedParameter - wrappedSeam);
            return std::min(difference, perimeter - difference) <= epsilon;
        }

        int spanRegion(const SurfaceSpan& span, const TubeSectionGeometry& section, double epsilon)
        {
            if (std::abs(span.sectionParameterEnd - span.sectionParameterStart) <= epsilon)
            {
                return -1;
            }

            const double middle = wrapParameter
            (
                (span.sectionParameterStart + span.sectionParameterEnd) * 0.5,
                section.perimeter
            );
            if (middle > epsilon && middle < section.seamPositions[0] - epsilon) return 0;
            if (middle > section.seamPositions[0] + epsilon
                && middle < section.seamPositions[1] - epsilon) return 1;
            if (middle > section.seamPositions[1] + epsilon
                && middle < section.seamPositions[2] - epsilon) return 2;
            if (middle > section.seamPositions[2] + epsilon
                && middle < section.perimeter - epsilon) return 3;
            return -1;
        }

        QVariantList entityIdList(const std::vector<EntityId>& entityIds)
        {
            QVariantList values;
            values.reserve(static_cast<qsizetype>(entityIds.size()));

            for (const EntityId entityId : entityIds)
            {
                values.push_back(QVariant::fromValue<qulonglong>(entityId));
            }

            return values;
        }

        QVariantMap diagnosticContext
        (
            const TubeCutAnalysis& analysis,
            const TubeSectionGeometry& section
        )
        {
            QVariantMap values;
            values.insert(QStringLiteral("boundaryEntityIds"), entityIdList(analysis.boundaryEntityIds));
            values.insert(QStringLiteral("maximumJoinGap"), analysis.maximumJoinGap);
            values.insert(QStringLiteral("maximumSurfaceDeviation"), analysis.maximumSurfaceDeviation);
            values.insert(QStringLiteral("maximumProjectionCoverageGap"), analysis.maximumProjectionCoverageGap);
            values.insert(QStringLiteral("projectedCenterY"), analysis.projectedCenterY);
            values.insert(QStringLiteral("projectedCenterZ"), analysis.projectedCenterZ);
            values.insert(QStringLiteral("expectedCenterY"), section.centerY);
            values.insert(QStringLiteral("expectedCenterZ"), section.centerZ);
            values.insert(QStringLiteral("projectedYLength"), analysis.projectedYLength);
            values.insert(QStringLiteral("projectedZWidth"), analysis.projectedZWidth);
            values.insert(QStringLiteral("expectedYLength"), section.yLength);
            values.insert(QStringLiteral("expectedZWidth"), section.zWidth);
            values.insert(QStringLiteral("globalWinding"), analysis.winding);
            values.insert(QStringLiteral("seam0Winding"), analysis.seamResults[0].winding);
            values.insert(QStringLiteral("seam1Winding"), analysis.seamResults[1].winding);
            values.insert(QStringLiteral("seam2Winding"), analysis.seamResults[2].winding);
            values.insert(QStringLiteral("seam3Winding"), analysis.seamResults[3].winding);
            int positiveCrossings = 0;
            int negativeCrossings = 0;
            int touches = 0;
            int overlaps = 0;

            for (const SeamWindingResult& seam : analysis.seamResults)
            {
                positiveCrossings += seam.positiveCrossingCount;
                negativeCrossings += seam.negativeCrossingCount;
                touches += seam.touchCount;
                overlaps += seam.overlapRunCount;
            }

            values.insert(QStringLiteral("positiveCrossingCount"), positiveCrossings);
            values.insert(QStringLiteral("negativeCrossingCount"), negativeCrossings);
            values.insert(QStringLiteral("touchCount"), touches);
            values.insert(QStringLiteral("overlapRunCount"), overlaps);
            return values;
        }

        Diagnostic makeDiagnostic
        (
            DiagnosticCode code,
            DiagnosticSeverity severity,
            const QString& userMessage,
            const QString& technicalDetail,
            const OperationContext& context,
            const TubeCutAnalysis& analysis,
            const TubeSectionGeometry& section
        )
        {
            Diagnostic diagnostic;
            diagnostic.code = code;
            diagnostic.severity = severity;
            diagnostic.component = QStringLiteral("TubeCutBoundaryClassifier");
            diagnostic.operation = context.operationName;
            diagnostic.stage = QStringLiteral("cut-boundary-classification");
            diagnostic.userMessage = userMessage;
            diagnostic.technicalDetail = technicalDetail;
            diagnostic.correlationId = context.correlationId;
            diagnostic.context = diagnosticContext(analysis, section);
            return diagnostic;
        }

        OperationResult<TubeCutAnalysis> fail
        (
            TubeCutAnalysis analysis,
            DiagnosticCode code,
            const QString& userMessage,
            const QString& technicalDetail,
            const OperationContext& context,
            const TubeSectionGeometry& section
        )
        {
            OperationResult<TubeCutAnalysis> result;
            result.status = OperationStatus::Failed;
            result.value = std::move(analysis);
            result.addDiagnostic(makeDiagnostic
            (
                code,
                DiagnosticSeverity::Error,
                userMessage,
                technicalDetail,
                context,
                *result.value,
                section
            ));
            return result;
        }

        MappedPath mapPathToSection
        (
            const std::vector<Vector3d>& path,
            const TubeSectionGeometry& section,
            double epsilon
        )
        {
            MappedPath mapped;
            const std::vector<double> cumulative = cumulativeLengths(section.boundary);

            for (std::size_t pathIndex = 0; pathIndex < path.size(); ++pathIndex)
            {
                const Vector3d& start3D = path[pathIndex];
                const Vector3d& end3D = path[(pathIndex + 1) % path.size()];
                const Vector2d start{ start3D.y, start3D.z };
                const Vector2d end{ end3D.y, end3D.z };
                const double deltaY = end.x - start.x;
                const double deltaZ = end.y - start.y;
                const double projectedLengthSquared = deltaY * deltaY + deltaZ * deltaZ;

                if (projectedLengthSquared <= epsilon * epsilon)
                {
                    const SectionProjection projection = projectToBoundary(start, section.boundary, cumulative);
                    mapped.maximumDeviation = std::max
                        (mapped.maximumDeviation, projection.distance);

                    if (!projection.valid || projection.distance > epsilon)
                    {
                        mapped.failure = MappedPath::Failure::PointOffSurface;
                        return mapped;
                    }

                    mapped.spans.push_back
                    ({
                        projection.segmentIndex,
                        projection.perimeterPosition,
                        projection.perimeterPosition,
                        start3D.x,
                        end3D.x
                    });
                    continue;
                }

                std::vector<double> splits{ 0.0, 1.0 };

                for (const Vector2d& vertex : section.boundary)
                {
                    const double factor = ((vertex.x - start.x) * deltaY
                        + (vertex.y - start.y) * deltaZ) / projectedLengthSquared;

                    if (factor <= epsilon || factor >= 1.0 - epsilon)
                    {
                        continue;
                    }

                    const Vector2d projected
                    {
                        start.x + deltaY * factor,
                        start.y + deltaZ * factor
                    };

                    if (distance2D(projected, vertex) <= epsilon)
                    {
                        splits.push_back(factor);
                    }
                }

                std::sort(splits.begin(), splits.end());
                splits.erase(std::unique(splits.begin(), splits.end(), [epsilon](double left, double right)
                {
                    return std::abs(left - right) <= epsilon;
                }), splits.end());

                for (std::size_t splitIndex = 0; splitIndex + 1 < splits.size(); ++splitIndex)
                {
                    const double startFactor = splits[splitIndex];
                    const double endFactor = splits[splitIndex + 1];
                    const double middleFactor = (startFactor + endFactor) * 0.5;
                    const Vector2d subStart
                    {
                        start.x + deltaY * startFactor,
                        start.y + deltaZ * startFactor
                    };
                    const Vector2d subEnd
                    {
                        start.x + deltaY * endFactor,
                        start.y + deltaZ * endFactor
                    };
                    const Vector2d middle
                    {
                        start.x + deltaY * middleFactor,
                        start.y + deltaZ * middleFactor
                    };
                    const SectionProjection startProjection = projectToBoundary
                        (subStart, section.boundary, cumulative);
                    const SectionProjection endProjection = projectToBoundary
                        (subEnd, section.boundary, cumulative);
                    const SectionProjection middleProjection = projectToBoundary
                        (middle, section.boundary, cumulative);

                    const auto projectToSelectedSegment =
                        [&section, &cumulative, &middleProjection](const Vector2d& point)
                    {
                        SectionProjection projection;
                        const std::size_t index = middleProjection.segmentIndex;
                        const Vector2d& segmentStart = section.boundary[index];
                        const Vector2d& segmentEnd = section.boundary
                            [(index + 1) % section.boundary.size()];
                        const double segmentY = segmentEnd.x - segmentStart.x;
                        const double segmentZ = segmentEnd.y - segmentStart.y;
                        const double lengthSquared = segmentY * segmentY + segmentZ * segmentZ;

                        if (lengthSquared <= kCalculationEpsilon)
                        {
                            return projection;
                        }

                        const double factor = std::clamp
                        (
                            ((point.x - segmentStart.x) * segmentY
                                + (point.y - segmentStart.y) * segmentZ) / lengthSquared,
                            0.0,
                            1.0
                        );
                        projection.valid = true;
                        projection.segmentIndex = index;
                        projection.segmentFactor = factor;
                        projection.point =
                        {
                            segmentStart.x + segmentY * factor,
                            segmentStart.y + segmentZ * factor
                        };
                        projection.distance = distance2D(point, projection.point);
                        projection.perimeterPosition = cumulative[index]
                            + std::sqrt(lengthSquared) * factor;
                        return projection;
                    };

                    const SectionProjection selectedStartProjection =
                        projectToSelectedSegment(subStart);
                    const SectionProjection selectedEndProjection =
                        projectToSelectedSegment(subEnd);
                    mapped.maximumDeviation = std::max
                    ({
                        mapped.maximumDeviation,
                        startProjection.distance,
                        endProjection.distance,
                        middleProjection.distance,
                        selectedStartProjection.distance,
                        selectedEndProjection.distance
                    });

                    if (!startProjection.valid || !endProjection.valid
                        || startProjection.distance > epsilon || endProjection.distance > epsilon)
                    {
                        mapped.failure = MappedPath::Failure::PointOffSurface;
                        return mapped;
                    }

                    if (!middleProjection.valid
                        || !selectedStartProjection.valid || !selectedEndProjection.valid
                        || selectedStartProjection.distance > epsilon
                        || selectedEndProjection.distance > epsilon
                        || middleProjection.distance > epsilon)
                    {
                        mapped.failure = MappedPath::Failure::SegmentLeavesBoundary;
                        return mapped;
                    }

                    mapped.spans.push_back
                    ({
                        middleProjection.segmentIndex,
                        selectedStartProjection.perimeterPosition,
                        selectedEndProjection.perimeterPosition,
                        start3D.x + (end3D.x - start3D.x) * startFactor,
                        start3D.x + (end3D.x - start3D.x) * endFactor
                    });
                }
            }

            mapped.valid = !mapped.spans.empty();
            return mapped;
        }

        double maximumCoverageGap(const std::vector<SurfaceSpan>& spans, double perimeter, double epsilon)
        {
            std::vector<std::pair<double, double>> intervals;

            for (const SurfaceSpan& span : spans)
            {
                const double start = std::min(span.sectionParameterStart, span.sectionParameterEnd);
                const double end = std::max(span.sectionParameterStart, span.sectionParameterEnd);

                if (end - start > epsilon)
                {
                    intervals.emplace_back(start, end);
                }
            }

            if (intervals.empty())
            {
                return perimeter;
            }

            std::sort(intervals.begin(), intervals.end());
            double maximumGap = intervals.front().first;
            double coveredEnd = intervals.front().second;

            for (std::size_t index = 1; index < intervals.size(); ++index)
            {
                if (intervals[index].first > coveredEnd)
                {
                    maximumGap = std::max(maximumGap, intervals[index].first - coveredEnd);
                }

                coveredEnd = std::max(coveredEnd, intervals[index].second);
            }

            return std::max(maximumGap, perimeter - coveredEnd);
        }

        SeamWindingResult analyzeSeam
        (
            const std::vector<SurfaceSpan>& spans,
            const TubeSectionGeometry& section,
            int seamIndex,
            double epsilon
        )
        {
            SeamWindingResult result;
            std::vector<std::size_t> nonZeroIndices;

            for (std::size_t index = 0; index < spans.size(); ++index)
            {
                if (std::abs(spans[index].sectionParameterEnd
                    - spans[index].sectionParameterStart) > epsilon)
                {
                    nonZeroIndices.push_back(index);
                }
            }

            if (nonZeroIndices.empty())
            {
                return result;
            }

            const double seam = section.seamPositions[static_cast<std::size_t>(seamIndex)];
            const int beforeRegion = seamIndex;
            const int afterRegion = (seamIndex + 1) % 4;
            bool invalid = false;

            for (std::size_t eventIndex = 0; eventIndex < nonZeroIndices.size(); ++eventIndex)
            {
                const std::size_t previousIndex = nonZeroIndices[eventIndex];
                const std::size_t nextIndex = nonZeroIndices[(eventIndex + 1) % nonZeroIndices.size()];
                const SurfaceSpan& previous = spans[previousIndex];
                const SurfaceSpan& next = spans[nextIndex];

                if (!parameterAtSeam(previous.sectionParameterEnd, seam, section.perimeter, epsilon)
                    || !parameterAtSeam(next.sectionParameterStart, seam, section.perimeter, epsilon))
                {
                    continue;
                }

                bool hasOverlap = false;
                std::size_t cursor = (previousIndex + 1) % spans.size();

                while (cursor != nextIndex)
                {
                    const SurfaceSpan& between = spans[cursor];

                    if (std::abs(between.sectionParameterEnd
                        - between.sectionParameterStart) > epsilon
                        || !parameterAtSeam
                        (
                            between.sectionParameterStart,
                            seam,
                            section.perimeter,
                            epsilon
                        ))
                    {
                        invalid = true;
                        break;
                    }

                    hasOverlap = hasOverlap
                        || std::abs(between.xEnd - between.xStart) > epsilon;
                    cursor = (cursor + 1) % spans.size();
                }

                if (invalid)
                {
                    break;
                }

                const int previousRegion = spanRegion(previous, section, epsilon);
                const int nextRegion = spanRegion(next, section, epsilon);

                if (previousRegion < 0 || nextRegion < 0)
                {
                    invalid = true;
                    break;
                }

                if (hasOverlap)
                {
                    ++result.overlapRunCount;
                }

                if (previousRegion == beforeRegion && nextRegion == afterRegion)
                {
                    ++result.positiveCrossingCount;
                }
                else if (previousRegion == afterRegion && nextRegion == beforeRegion)
                {
                    ++result.negativeCrossingCount;
                }
                else if (previousRegion == nextRegion
                    && (previousRegion == beforeRegion || previousRegion == afterRegion))
                {
                    ++result.touchCount;
                }
                else
                {
                    invalid = true;
                    break;
                }
            }

            result.winding = result.positiveCrossingCount - result.negativeCrossingCount;
            result.usable = !invalid
                && result.positiveCrossingCount + result.negativeCrossingCount
                    + result.touchCount + result.overlapRunCount > 0;
            return result;
        }
    }

    TubeCutResult TubeCutBoundaryClassifier::classifyWinding
    (
        int globalWinding,
        const std::array<SeamWindingResult, 4>& seamResults
    )
    {
        int usableCount = 0;

        for (const SeamWindingResult& seam : seamResults)
        {
            if (!seam.usable)
            {
                continue;
            }

            ++usableCount;

            if (seam.winding != globalWinding)
            {
                return TubeCutResult::Indeterminate;
            }
        }

        if (usableCount < 2 || std::abs(globalWinding) > 1)
        {
            return TubeCutResult::Indeterminate;
        }

        return globalWinding == 0
            ? TubeCutResult::KeepsLeftAndRight
            : TubeCutResult::CutsLeftAndRight;
    }

    OperationResult<TubeSectionGeometry> TubeCutBoundaryClassifier::prepareSection
    (
        const TubeSectionGeometry& source,
        const OperationContext& context,
        double sectionMatchEpsilon
    )
    {
        OperationResult<TubeSectionGeometry> result;
        TubeSectionGeometry section = source;
        const double epsilon = std::max(sectionMatchEpsilon, 1.0e-9);
        std::vector<Vector2d> boundary;
        boundary.reserve(source.boundary.size());

        for (const Vector2d& point : source.boundary)
        {
            if (!finite(point))
            {
                result.status = OperationStatus::InvalidInput;
                return result;
            }

            if (boundary.empty() || distance2D(boundary.back(), point) > epsilon)
            {
                boundary.push_back(point);
            }
        }

        if (boundary.size() > 1 && distance2D(boundary.front(), boundary.back()) <= epsilon)
        {
            boundary.pop_back();
        }

        if (boundary.size() < 4)
        {
            result.status = OperationStatus::InvalidInput;
            return result;
        }

        double signedArea = 0.0;
        double minimumY = std::numeric_limits<double>::max();
        double maximumY = std::numeric_limits<double>::lowest();
        double minimumZ = std::numeric_limits<double>::max();
        double maximumZ = std::numeric_limits<double>::lowest();

        for (std::size_t index = 0; index < boundary.size(); ++index)
        {
            const Vector2d& point = boundary[index];
            const Vector2d& next = boundary[(index + 1) % boundary.size()];
            signedArea += point.x * next.y - next.x * point.y;
            minimumY = std::min(minimumY, point.x);
            maximumY = std::max(maximumY, point.x);
            minimumZ = std::min(minimumZ, point.y);
            maximumZ = std::max(maximumZ, point.y);
        }

        if (std::abs(signedArea) <= epsilon * epsilon)
        {
            result.status = OperationStatus::InvalidInput;
            return result;
        }

        if (signedArea > 0.0)
        {
            std::reverse(boundary.begin(), boundary.end());
        }

        const std::vector<double> originalCumulative = cumulativeLengths(boundary);
        const double perimeter = originalCumulative.back();

        if (!std::isfinite(perimeter) || perimeter <= epsilon)
        {
            result.status = OperationStatus::InvalidInput;
            return result;
        }

        const std::array<Vector2d, 4> cornerTargets
        {{
            { maximumY, maximumZ },
            { maximumY, minimumZ },
            { minimumY, minimumZ },
            { minimumY, maximumZ }
        }};
        std::array<SectionProjection, 4> seams;

        for (std::size_t index = 0; index < seams.size(); ++index)
        {
            seams[index] = projectToBoundary(cornerTargets[index], boundary, originalCumulative);

            if (!seams[index].valid)
            {
                result.status = OperationStatus::InvalidInput;
                return result;
            }
        }

        const double origin = seams[3].perimeterPosition;
        std::array<double, 4> seamPositions{};

        for (std::size_t index = 0; index < seamPositions.size(); ++index)
        {
            seamPositions[index] = wrapParameter(seams[index].perimeterPosition - origin, perimeter);
        }

        seamPositions[3] = 0.0;

        if (!(seamPositions[0] > epsilon
            && seamPositions[1] > seamPositions[0] + epsilon
            && seamPositions[2] > seamPositions[1] + epsilon
            && perimeter > seamPositions[2] + epsilon))
        {
            result.status = OperationStatus::InvalidInput;
            return result;
        }

        std::vector<double> parameters{ 0.0, seamPositions[0], seamPositions[1], seamPositions[2] };

        for (std::size_t index = 0; index < boundary.size(); ++index)
        {
            parameters.push_back(wrapParameter(originalCumulative[index] - origin, perimeter));
        }

        std::sort(parameters.begin(), parameters.end());
        parameters.erase(std::unique(parameters.begin(), parameters.end(), [epsilon](double left, double right)
        {
            return std::abs(left - right) <= epsilon;
        }), parameters.end());

        section.boundary.clear();
        section.boundary.reserve(parameters.size());

        for (const double parameter : parameters)
        {
            section.boundary.push_back(pointAtParameter
            (
                boundary,
                originalCumulative,
                origin + parameter
            ));
        }

        section.perimeter = perimeter;
        section.seamPositions = seamPositions;
        section.centerY = source.centerY;
        section.centerZ = source.centerZ;
        section.yLength = source.yLength;
        section.zWidth = source.zWidth;

        if (!std::isfinite(section.centerY) || !std::isfinite(section.centerZ)
            || !std::isfinite(section.yLength) || !std::isfinite(section.zWidth)
            || section.yLength <= epsilon || section.zWidth <= epsilon)
        {
            result.status = OperationStatus::InvalidInput;
            return result;
        }

        result.status = OperationStatus::Success;
        result.value = std::move(section);
        Q_UNUSED(context);
        return result;
    }

    OperationResult<TubeCutAnalysis> TubeCutBoundaryClassifier::analyze
    (
        const std::vector<Vector3d>& orderedPath,
        const std::vector<EntityId>& boundaryEntityIds,
        double maximumJoinGap,
        const TubeSectionGeometry& sourceSection,
        const OperationContext& context,
        double surfaceMappingEpsilon,
        double sectionMatchEpsilon
    )
    {
        TubeCutAnalysis analysis;
        analysis.maximumJoinGap = maximumJoinGap;
        analysis.boundaryEntityIds = boundaryEntityIds;
        const auto preparedSection = prepareSection(sourceSection, context, sectionMatchEpsilon);

        if (!preparedSection.succeeded() || !preparedSection.value.has_value())
        {
            return fail
            (
                std::move(analysis),
                DiagnosticCode::CutBoundarySectionUnavailable,
                QStringLiteral("已识别的方管外截面不可用，无法判断加工断面。"),
                QStringLiteral("TubeSectionGeometry normalization failed."),
                context,
                sourceSection
            );
        }

        const TubeSectionGeometry& section = *preparedSection.value;
        const double mappingEpsilon = std::max(surfaceMappingEpsilon, 1.0e-9);
        const double matchEpsilon = std::max(sectionMatchEpsilon, 1.0e-9);
        analysis.orderedPath.reserve(orderedPath.size());

        for (const Vector3d& point : orderedPath)
        {
            if (!finite(point))
            {
                return fail
                (
                    std::move(analysis),
                    DiagnosticCode::CutBoundaryTopologyInvalid,
                    QStringLiteral("加工断面闭环包含无效坐标。"),
                    QStringLiteral("Non-finite ordered path point."),
                    context,
                    section
                );
            }

            if (analysis.orderedPath.empty()
                || distance3D(analysis.orderedPath.back(), point) > matchEpsilon)
            {
                analysis.orderedPath.push_back(point);
            }
        }

        if (analysis.orderedPath.size() > 1
            && distance3D(analysis.orderedPath.front(), analysis.orderedPath.back()) <= matchEpsilon)
        {
            analysis.orderedPath.pop_back();
        }

        if (analysis.orderedPath.size() < 3)
        {
            return fail
            (
                std::move(analysis),
                DiagnosticCode::CutBoundaryTopologyInvalid,
                QStringLiteral("加工断面闭环点数不足。"),
                QStringLiteral("Strict topology loop has fewer than three points."),
                context,
                section
            );
        }

        double minimumY = std::numeric_limits<double>::max();
        double maximumY = std::numeric_limits<double>::lowest();
        double minimumZ = std::numeric_limits<double>::max();
        double maximumZ = std::numeric_limits<double>::lowest();

        for (const Vector3d& point : analysis.orderedPath)
        {
            minimumY = std::min(minimumY, point.y);
            maximumY = std::max(maximumY, point.y);
            minimumZ = std::min(minimumZ, point.z);
            maximumZ = std::max(maximumZ, point.z);
        }

        analysis.projectedYLength = maximumY - minimumY;
        analysis.projectedZWidth = maximumZ - minimumZ;
        analysis.projectedCenterY = (minimumY + maximumY) * 0.5;
        analysis.projectedCenterZ = (minimumZ + maximumZ) * 0.5;

        if (std::abs(analysis.projectedYLength - section.yLength) > matchEpsilon
            || std::abs(analysis.projectedZWidth - section.zWidth) > matchEpsilon)
        {
            return fail
            (
                std::move(analysis),
                DiagnosticCode::CutBoundaryDimensionMismatch,
                QStringLiteral("候选曲线的 YZ 投影尺寸与方管外截面不一致。"),
                QStringLiteral("Projected section dimensions differ from the recognized tube section."),
                context,
                section
            );
        }

        if (std::abs(analysis.projectedCenterY - section.centerY) > matchEpsilon
            || std::abs(analysis.projectedCenterZ - section.centerZ) > matchEpsilon)
        {
            return fail
            (
                std::move(analysis),
                DiagnosticCode::CutBoundaryCenterMismatch,
                QStringLiteral("候选曲线的 YZ 投影中心与方管外截面中心不一致。"),
                QStringLiteral("Projected section center differs from the recognized tube section."),
                context,
                section
            );
        }

        const MappedPath mapped = mapPathToSection
            (analysis.orderedPath, section, mappingEpsilon);
        analysis.maximumSurfaceDeviation = mapped.maximumDeviation;

        if (!mapped.valid)
        {
            const bool pointOffSurface = mapped.failure == MappedPath::Failure::PointOffSurface;
            return fail
            (
                std::move(analysis),
                pointOffSurface
                    ? DiagnosticCode::CutBoundarySurfaceMappingFailed
                    : DiagnosticCode::CutBoundaryProjectionMismatch,
                pointOffSurface
                    ? QStringLiteral("候选曲线存在无法映射到方管外表面的投影点。")
                    : QStringLiteral("候选曲线的 YZ 投影没有沿方管真实外截面连续运行。"),
                pointOffSurface
                    ? QStringLiteral("At least one projected point is outside the section boundary tolerance.")
                    : QStringLiteral("At least one projected path segment is not a section-boundary interval."),
                context,
                section
            );
        }

        analysis.surfaceMappingValid = true;
        const double traversalEpsilon = std::max(matchEpsilon, mappingEpsilon);
        analysis.maximumProjectionCoverageGap = maximumCoverageGap
            (mapped.spans, section.perimeter, traversalEpsilon);

        if (analysis.maximumProjectionCoverageGap > traversalEpsilon)
        {
            return fail
            (
                std::move(analysis),
                DiagnosticCode::CutBoundaryCoverageGap,
                QStringLiteral("候选曲线没有覆盖完整的方管外截面投影。"),
                QStringLiteral("Merged section intervals contain an uncovered gap."),
                context,
                section
            );
        }

        analysis.projectionMatchesSection = true;
        double signedTravel = 0.0;
        const double initialPerimeterPosition = mapped.spans.front().sectionParameterStart;

        for (const SurfaceSpan& span : mapped.spans)
        {
            if (analysis.unwrappedBoundary.empty())
            {
                analysis.unwrappedBoundary.push_back
                    ({ span.xStart, initialPerimeterPosition });
            }

            signedTravel += span.sectionParameterEnd - span.sectionParameterStart;
            analysis.unwrappedBoundary.push_back
                ({ span.xEnd, initialPerimeterPosition + signedTravel });
        }

        const double windingValue = signedTravel / section.perimeter;
        analysis.winding = static_cast<int>(std::llround(windingValue));

        if (std::abs(signedTravel - analysis.winding * section.perimeter) > traversalEpsilon)
        {
            return fail
            (
                std::move(analysis),
                DiagnosticCode::CutBoundaryWindingMismatch,
                QStringLiteral("候选曲线的周向行程无法形成稳定整数绕数。"),
                QStringLiteral("Unwrapped perimeter travel has a non-numerical winding remainder."),
                context,
                section
            );
        }

        for (int seamIndex = 0; seamIndex < 4; ++seamIndex)
        {
            analysis.seamResults[static_cast<std::size_t>(seamIndex)] = analyzeSeam
                (mapped.spans, section, seamIndex, traversalEpsilon);
        }

        int usableSeamCount = 0;

        for (const SeamWindingResult& seam : analysis.seamResults)
        {
            if (!seam.usable)
            {
                continue;
            }

            ++usableSeamCount;

            if (seam.winding != analysis.winding)
            {
                return fail
                (
                    std::move(analysis),
                    DiagnosticCode::CutBoundarySeamDisagreement,
                    QStringLiteral("四条接缝的有向穿越结果与全局绕数不一致。"),
                    QStringLiteral("A usable seam winding differs from global winding."),
                    context,
                    section
                );
            }
        }

        if (usableSeamCount < 2)
        {
            return fail
            (
                std::move(analysis),
                DiagnosticCode::CutBoundarySeamDegenerate,
                QStringLiteral("可用接缝不足，无法确定候选曲线是否切断方管。"),
                QStringLiteral("Fewer than two seams have unambiguous events."),
                context,
                section
            );
        }

        if (std::abs(analysis.winding) > 1)
        {
            return fail
            (
                std::move(analysis),
                DiagnosticCode::CutBoundaryMultipleWinding,
                QStringLiteral("候选曲线绕方管多圈，无法作为单一加工断面。"),
                QStringLiteral("Absolute integer winding exceeds one."),
                context,
                section
            );
        }

        analysis.result = classifyWinding(analysis.winding, analysis.seamResults);
        OperationResult<TubeCutAnalysis> result;
        result.status = OperationStatus::Success;

        if (analysis.result == TubeCutResult::KeepsLeftAndRight)
        {
            result.addDiagnostic(makeDiagnostic
            (
                DiagnosticCode::CutBoundaryKeepsTubeConnected,
                DiagnosticSeverity::Warning,
                QStringLiteral("候选曲线的 YZ 投影覆盖完整方管截面，但周向有向绕数为 0，切割后仍存在连接左右两端的材料桥。"),
                QStringLiteral("Complete projection with zero signed seam winding."),
                context,
                analysis,
                section
            ));
        }
        result.value = std::move(analysis);
        return result;
    }
}
