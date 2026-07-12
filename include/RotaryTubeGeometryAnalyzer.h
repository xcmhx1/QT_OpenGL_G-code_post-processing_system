#pragma once

#include <QVector>
#include <QVector2D>
#include <QString>

class CadItem;

struct RotaryTubeSectionModel
{
    bool valid = false;
    QVector<QVector2D> sectionHull;
    QVector<CadItem*> outerBoundaryItems;
    double yLength = 0.0;
    double zWidth = 0.0;
    double cornerRadius = 0.0;
    QString errorMessage;
};

struct RotaryInternalPathResult
{
    QVector<CadItem*> physicalInteriorItems;
    QVector<CadItem*> topologicalInteriorItems;
};

class RotaryTubeGeometryAnalyzer
{
public:
    static RotaryTubeSectionModel buildSectionModel
    (
        const QVector<CadItem*>& selectedItems,
        const QVector<CadItem*>& sceneItems,
        double connectionTolerance = 1.0
    );

    static RotaryInternalPathResult findInternalPaths
    (
        const RotaryTubeSectionModel& model,
        const QVector<CadItem*>& sceneItems,
        double connectionTolerance = 1.0
    );
};
