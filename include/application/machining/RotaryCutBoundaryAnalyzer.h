#pragma once

#include "core/machining/TubeCutBoundary.h"

#include <QVector>
#include <QVector2D>
#include <QVector3D>
#include <QString>

#include <functional>

class CadItem;
class RotaryPathTopology;
struct RotaryTubeSectionModel;

using TubeCutResult = cadcam::machining::TubeCutResult;
using SeamWindingResult = cadcam::machining::SeamWindingResult;

struct RotaryCutBoundaryUnwrappedSample
{
    double x = 0.0;
    double perimeterPosition = 0.0;
};

enum class RotaryBoundarySide
{
    Before,
    OnBoundary,
    After,
    Ambiguous
};

struct RotaryBoundaryPointClassificationDiagnostics
{
    bool validProjection = false;
    double mappedPerimeterPosition = 0.0;
    double distanceToPerimeterSeam = 0.0;
    double minimumBoundaryDistance = 0.0;
    QVector<double> rawRayIntersectionXs;
    QVector<double> deduplicatedRayIntersectionXs;
};

struct RotaryCutBoundaryAnalysis
{
    bool valid = false;
    bool connectedLoop = false;
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
    double sectionPerimeter = 0.0;
    std::array<double, 4> seamPositions{};
    QVector<QVector3D> orderedPath;
    QVector<CadItem*> boundaryItems;
    QVector<QVector2D> sectionBoundary;
    QVector<RotaryCutBoundaryUnwrappedSample> unwrappedBoundary;
    QVector<Diagnostic> diagnostics;
    QString errorMessage;
};

class RotaryCutBoundaryAnalyzer
{
public:
    static RotaryCutBoundaryAnalysis analyze
    (
        const QVector<CadItem*>& candidateItems,
        const QVector<CadItem*>& sceneItems,
        const RotaryTubeSectionModel& sectionModel,
        double connectionTolerance = 1.0,
        const std::function<void(double)>& pathRebuildObserver = {}
    );

    static RotaryCutBoundaryAnalysis analyze
    (
        const QVector<CadItem*>& candidateItems,
        const RotaryPathTopology& topology,
        const RotaryTubeSectionModel& sectionModel,
        double connectionTolerance = 1.0
    );

    static RotaryBoundarySide classifyPointRelativeToBoundary
    (
        const RotaryCutBoundaryAnalysis& analysis,
        const QVector3D& point,
        double tolerance = 1.0,
        RotaryBoundaryPointClassificationDiagnostics* diagnostics = nullptr
    );

    static QVector<QVector3D> buildBoundaryOrderTestPoints
    (
        const RotaryCutBoundaryAnalysis& analysis,
        double tolerance = 1.0
    );

    static bool boundariesIntersect
    (
        const RotaryCutBoundaryAnalysis& left,
        const RotaryCutBoundaryAnalysis& right,
        double tolerance = 1.0e-6
    );
};
