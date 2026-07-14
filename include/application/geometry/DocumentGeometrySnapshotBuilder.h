#pragma once

#include "application/geometry/GeometrySnapshot.h"

class CadDocument;

class DocumentGeometrySnapshotBuilder
{
public:
    OperationResult<GeometrySourceSnapshot> capture
    (
        const CadDocument& document,
        const OperationContext& context
    ) const;
};
