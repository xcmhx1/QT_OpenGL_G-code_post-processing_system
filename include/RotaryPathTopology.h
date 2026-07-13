#pragma once

#include <QString>
#include <QVector>
#include <QVector3D>

#include <vector>

class CadItem;

struct RotaryPathTopologyTolerance
{
    double nodeSnap = 1.0;
    double closure = 1.0;
    double intersection = 0.01;
    double minimumEdgeLength = 1.0e-6;

    static RotaryPathTopologyTolerance fromConnectionTolerance(double connectionTolerance);
};

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

// Shared path topology used by section recognition, cut-boundary validation and sorting.
// Raw paths are rebuilt once when this object is constructed.
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

    RotaryPathLoopResult extractBestLoop
    (
        const QVector<CadItem*>& candidateItems,
        const QVector<CadItem*>& preferredItems = {}
    ) const;

private:
    RotaryPathTopologyTolerance m_tolerance;
    QVector<RotaryPathTopologyRecord> m_records;
    QVector<QVector<int>> m_itemAdjacency;
};
