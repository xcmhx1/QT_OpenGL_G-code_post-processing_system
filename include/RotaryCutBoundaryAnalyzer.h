#pragma once

#include <QVector>
#include <QVector2D>
#include <QVector3D>
#include <QString>

class CadItem;

struct RotaryCutBoundaryProfileSample
{
    double phase = 0.0;
    double x = 0.0;
};

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

struct RotaryCutPlaneFit
{
    bool valid = false;
    double a = 0.0;
    double b = 0.0;
    double c = 0.0;
    double rmsDeviation = 0.0;
    double maximumDeviation = 0.0;
};

struct RotaryCutBoundaryAnalysis
{
    bool valid = false;
    bool connectedLoop = false;
    bool surfaceConforming = false;
    bool separating = false;
    double maximumJoinGap = 0.0;
    int connectedComponentCount = 0;
    int openNodeCount = 0;
    int branchNodeCount = 0;
    int ignoredBranchItemCount = 0;
    double signedPerimeterTravel = 0.0;
    double windingNumber = 0.0;
    double perimeterCoverage = 0.0;
    double perimeterTravelRatio = 0.0;
    double backtrackRatio = 0.0;
    double sectionPerimeter = 0.0;
    double initialPerimeterPosition = 0.0;
    bool singleValuedProfile = false;
    int multiValuePhaseCount = 0;
    double maximumMultiValueXSpan = 0.0;
    int ambiguousProjectionPointCount = 0;
    double maximumPerimeterJump = 0.0;
    double maximumSurfaceDeviation = 0.0;
    QVector<QVector3D> orderedPath;
    QVector<CadItem*> boundaryItems;
    QVector<QVector2D> sectionHull;
    QVector<RotaryCutBoundaryProfileSample> boundaryProfile;
    QVector<RotaryCutBoundaryUnwrappedSample> unwrappedBoundary;
    RotaryCutPlaneFit planeFit;
    QString errorMessage;
};

class RotaryCutBoundaryAnalyzer
{
public:
    static RotaryCutBoundaryAnalysis analyze
    (
        const QVector<CadItem*>& candidateItems,
        const QVector<CadItem*>& sceneItems,
        double connectionTolerance = 1.0,
        const QVector<QVector2D>& sectionHull = {}
    );

    static bool boundaryXAtPoint
    (
        const RotaryCutBoundaryAnalysis& analysis,
        const QVector3D& point,
        double& boundaryX
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
