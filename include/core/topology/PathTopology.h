#pragma once

#include "application/tasks/TaskContext.h"
#include "core/diagnostics/OperationResult.h"
#include "core/geometry/GeometryTypes.h"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace cadcam::topology
{
    struct PathTopologyTolerance
    {
        double nodeSnap = 1.0;
        double closure = 1.0;
        double intersection = 0.01;
        double minimumEdgeLength = 1.0e-6;

        static PathTopologyTolerance fromConnectionTolerance(double connectionTolerance);
    };

    struct TopologyPathRecord
    {
        std::size_t sourceIndex = 0;
        geometry::EntityId entityId = 0;
        geometry::SourceGeometryKind sourceKind = geometry::SourceGeometryKind::Unknown;
        std::vector<geometry::Vector3d> points;
        bool semanticallyClosed = false;
    };

    struct TopologyInput
    {
        std::uint64_t contentRevision = 0;
        std::vector<TopologyPathRecord> records;
        QVector<Diagnostic> diagnostics;
    };

    struct TopologyLoopResult
    {
        bool connectedLoop = false;
        bool approximatelyClosed = false;
        double closureGap = 0.0;
        int connectedComponentCount = 0;
        int openNodeCount = 0;
        int branchNodeCount = 0;
        int ignoredBranchRecordCount = 0;
        std::vector<geometry::Vector3d> orderedPath;
        std::vector<geometry::EntityId> usedEntityIds;
        std::vector<geometry::EntityId> ignoredBranchEntityIds;
    };

    struct PathTopologyStatistics
    {
        std::size_t recordCount = 0;
        std::size_t nodeCount = 0;
        std::size_t edgeCount = 0;
        int componentCount = 0;
    };

    class PathTopology
    {
    public:
        const std::vector<TopologyPathRecord>& records() const;
        const std::vector<std::vector<int>>& adjacency() const;
        const PathTopologyStatistics& statistics() const;

        std::vector<int> componentIds
        (
            const std::vector<geometry::EntityId>& subset = {}
        ) const;

        bool directlyConnected(geometry::EntityId left, geometry::EntityId right) const;

        OperationResult<TopologyLoopResult> extractSeededLoop
        (
            const std::vector<geometry::EntityId>& seeds
        ) const;

        OperationResult<TopologyLoopResult> extractBestLoop
        (
            const std::vector<geometry::EntityId>& candidates,
            const std::vector<geometry::EntityId>& preferred = {}
        ) const;

    private:
        PathTopologyTolerance m_tolerance;
        std::vector<TopologyPathRecord> m_records;
        std::vector<std::vector<int>> m_adjacency;
        PathTopologyStatistics m_statistics;
        OperationContext m_context;
        CancellationToken m_cancellationToken;

        friend class PathTopologyBuilder;
    };

    class PathTopologyBuilder
    {
    public:
        OperationResult<PathTopology> build
        (
            const TopologyInput& input,
            const PathTopologyTolerance& tolerance,
            const TaskContext& taskContext
        ) const;
    };
}
