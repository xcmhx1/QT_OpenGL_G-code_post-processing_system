#pragma once

#include "core/topology/PathTopology.h"

#include <QString>
#include <QVector>
#include <QVector3D>

#include <map>
#include <optional>
#include <vector>

class CadItem;

using RotaryPathTopologyTolerance = cadcam::topology::PathTopologyTolerance;

struct RotaryPathTopologyRecord
{
    CadItem* sourceItem = nullptr;
    int sourceItemIndex = -1;
    QVector<QVector3D> points;
    bool semanticallyClosed = false;
};

struct RotaryPathLoopResult
{
    bool valid = false;
    bool connectedLoop = false;
    bool approximatelyClosed = false;
    double closureGap = 0.0;
    int connectedComponentCount = 0;
    int openNodeCount = 0;
    int branchNodeCount = 0;
    int ignoredBranchItemCount = 0;
    QVector<QVector3D> orderedPath;
    QVector<CadItem*> usedItems;
    QVector<CadItem*> ignoredBranchItems;
    QString errorMessage;
};

class RotaryPathTopology
{
public:
    RotaryPathTopology
    (
        const QVector<CadItem*>& items,
        const RotaryPathTopologyTolerance& tolerance
    );

    const QVector<RotaryPathTopologyRecord>& records() const;
    std::vector<int> itemComponentIds(const QVector<CadItem*>& subset = {}) const;
    bool itemsDirectlyConnected(CadItem* left, CadItem* right) const;

    RotaryPathLoopResult extractSeededLoop
    (
        const QVector<CadItem*>& seedItems,
        QVector<CadItem*>* expandedItems = nullptr
    ) const;

    RotaryPathLoopResult extractBestLoop
    (
        const QVector<CadItem*>& candidateItems,
        const QVector<CadItem*>& preferredItems = {}
    ) const;

    OperationStatus status() const;
    const QVector<Diagnostic>& diagnostics() const;

private:
    std::vector<cadcam::geometry::EntityId> entityIds
        (const QVector<CadItem*>& items) const;
    RotaryPathLoopResult mapLoopResult
        (const OperationResult<cadcam::topology::TopologyLoopResult>& coreResult) const;
    void addMappingDiagnostic
        (cadcam::geometry::EntityId entityId, const QString& detail) const;

    QVector<RotaryPathTopologyRecord> m_records;
    std::optional<cadcam::topology::PathTopology> m_topology;
    std::map<cadcam::geometry::EntityId, CadItem*> m_itemById;
    std::map<CadItem*, cadcam::geometry::EntityId> m_idByItem;
    mutable OperationStatus m_status = OperationStatus::InternalError;
    mutable QVector<Diagnostic> m_diagnostics;
    OperationContext m_context;
};

QString describeRotaryPathItems(const QVector<CadItem*>& items);
