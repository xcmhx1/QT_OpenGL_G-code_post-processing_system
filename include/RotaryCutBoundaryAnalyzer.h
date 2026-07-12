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

struct RotaryCutBoundaryAnalysis
{
    bool valid = false;
    bool connectedLoop = false;
    bool surfaceConforming = false;
    bool separating = false;
    double closureGap = 0.0;
    double windingNumber = 0.0;
    double maximumSurfaceDeviation = 0.0;
    QVector<QVector3D> orderedPath;
    QVector<QVector2D> sectionHull;
    QVector<RotaryCutBoundaryProfileSample> boundaryProfile;
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
};
