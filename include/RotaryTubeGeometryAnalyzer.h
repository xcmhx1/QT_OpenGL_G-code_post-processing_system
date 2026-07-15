#pragma once

#include "core/machining/TubeSection.h"

#include <QVector>
#include <QVector2D>
#include <QString>

#include <cstdint>
#include <optional>

class CadItem;

struct RotaryTubeSectionModel
{
    bool valid = false;
    std::optional<cadcam::machining::TubeSectionModel> coreModel;
    QVector<QVector2D> sectionBoundary;
    QVector<CadItem*> outerBoundaryItems;
    double yLength = 0.0;
    double zWidth = 0.0;
    double cornerRadius = 0.0;
    int roundedCornerCount = 0;
    QVector<double> cornerRadii;
    double cornerConfidence = 0.0;
    double centerX = 0.0;
    bool centerValid = false;
    double centerY = 0.0;
    double centerZ = 0.0;
    int inspectedCandidateCount = 0;
    int validCandidateCount = 0;
    int roundedCandidateCount = 0;
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
        double connectionTolerance = 1.0,
        std::uint64_t contentRevision = 1U
    );

    static RotaryTubeSectionModel findBestSectionModel
    (
        const QVector<CadItem*>& sceneItems,
        double connectionTolerance = 1.0,
        std::uint64_t contentRevision = 1U
    );

    static RotaryInternalPathResult findInternalPaths
    (
        const RotaryTubeSectionModel& model,
        const QVector<CadItem*>& sceneItems,
        double connectionTolerance = 1.0
    );
};
