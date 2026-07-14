#pragma once

#include "CadItem.h"

class CadSplineItem : public CadItem
{
public:
    explicit CadSplineItem(DRW_Entity* entity, QObject* parent = nullptr);

    void buildGeometryDatay() override;
    void rebuildRawPathPoints3D() override;
    bool rebuildControlPoints4Axis
    (
        double axisY = 0.0,
        double axisZ = 0.0,
        double judgeCenterY = 0.0,
        double judgeCenterZ = 0.0,
        bool invertAAxisDirection = false,
        double aAxisOffsetDegrees = 0.0,
        bool keepContinuousAngle = true,
        QString* errorMessage = nullptr
    ) override;

    DRW_Spline* m_data = nullptr;
};
