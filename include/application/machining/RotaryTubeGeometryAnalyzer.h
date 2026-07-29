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
    std::optional<cadcam::geometry::Vector2d> automaticCenter;
    std::optional<cadcam::geometry::Vector2d> userCenter;
    int inspectedCandidateCount = 0;
    int validCandidateCount = 0;
    int roundedCandidateCount = 0;
    bool manuallyConfigured = false;
    QString errorMessage;

    cadcam::geometry::Vector2d effectiveCenter() const;
    bool setAutomaticCenter
        (std::optional<cadcam::geometry::Vector2d> center);
    bool setUserCenter
        (std::optional<cadcam::geometry::Vector2d> center);
};

struct RotaryInternalPathResult
{
    bool sectionAvailable = false;
    QVector<CadItem*> removableItems;
    QVector<Diagnostic> diagnostics;
    int candidatePathCount = 0;
    int outerBoundaryEntityCount = 0;
    double insetDistance = 0.0;
    int preservedSafetyBandCount = 0;
    int skippedPathCount = 0;
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

    static RotaryTubeSectionModel buildManualSectionModel
    (
        double yLength,
        double zWidth,
        double cornerRadius,
        double centerX,
        double centerY,
        double centerZ,
        std::uint64_t contentRevision = 1U
    );

    static RotaryInternalPathResult findInternalPaths
    (
        const RotaryTubeSectionModel& model,
        const QVector<CadItem*>& sceneItems,
        double connectionTolerance = 1.0
    );
};
