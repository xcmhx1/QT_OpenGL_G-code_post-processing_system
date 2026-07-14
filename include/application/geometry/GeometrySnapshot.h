#pragma once

#include "core/geometry/GeometryCompiler.h"

#include <QColor>
#include <QString>
#include <QVector>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

struct EntitySnapshotAttributes
{
    cadcam::geometry::EntityId entityId = 0;
    int originalDxfType = 0;
    QString layer;
    QColor color;
    bool visible = true;
};

struct GeometrySourceEntry
{
    std::size_t sourceIndex = 0;
    EntitySnapshotAttributes attributes;
    cadcam::geometry::SourceGeometryKind sourceKind =
        cadcam::geometry::SourceGeometryKind::Unknown;
    OperationStatus status = OperationStatus::InternalError;
    std::optional<cadcam::geometry::SourceEntity> sourceEntity;
    QVector<Diagnostic> diagnostics;
};

struct GeometrySourceSnapshot
{
    std::uint64_t contentRevision = 0;
    std::vector<GeometrySourceEntry> entries;
};

struct GeometrySnapshotEntry
{
    std::size_t sourceIndex = 0;
    EntitySnapshotAttributes attributes;
    cadcam::geometry::SourceGeometryKind sourceKind =
        cadcam::geometry::SourceGeometryKind::Unknown;
    OperationStatus status = OperationStatus::InternalError;
    std::optional<cadcam::geometry::Path3D> path;
    QVector<Diagnostic> diagnostics;
};

struct GeometrySnapshot
{
    std::uint64_t contentRevision = 0;
    cadcam::geometry::SamplingPolicy samplingPolicy;
    std::vector<GeometrySnapshotEntry> entries;

    bool matchesRevision(std::uint64_t currentRevision) const
    {
        return contentRevision == currentRevision;
    }
};
