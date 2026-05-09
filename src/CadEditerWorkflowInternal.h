#pragma once

#include "CadItem.h"

#include <memory>
#include <vector>

namespace CadEditerWorkflowInternal
{
    inline constexpr double kPi = 3.14159265358979323846;
    inline constexpr double kTwoPi = 6.28318530717958647692;
    inline constexpr double kGeometryEpsilon = 1.0e-9;
    inline constexpr double kMinEllipseRatio = 1.0e-4;

    QVector3D flattenToDrawingPlane(const QVector3D& point);
    QVector3D normalizedXlineDirection(const DRW_Xline* xline);
    double normalizeAnglePositive(double angle);
    QVector3D resolveEntityNormal(const DRW_Coord& extPoint);
    void buildPlaneBasis(const QVector3D& normal, QVector3D& axisX, QVector3D& axisY);
    QVector3D circlePointAt(const DRW_Circle* circle, double parameter);
    QVector3D arcPointAt(const DRW_Arc* arc, double angle);
    double arcMidAngle(const DRW_Arc* arc);
    double angleFromPointOnCircle(const DRW_Circle* circle, const QVector3D& point, bool* valid = nullptr);
    bool tryBuildEllipseAxes(const DRW_Ellipse* ellipse, QVector3D& majorAxis, QVector3D& minorAxis);
    QVector3D ellipsePointAt(const DRW_Ellipse* ellipse, double parameter);
    bool ellipseParameterFromPoint(const DRW_Ellipse* ellipse, const QVector3D& worldPoint, double& parameter);
    void translateEntity(DRW_Entity* entity, const QVector3D& delta);
    QVector3D rotatePlanarPoint(const QVector3D& point, const QVector3D& basePoint, double radians);
    void rotateEntity(DRW_Entity* entity, const QVector3D& basePoint, double angleDegrees);
    void scaleEntity(DRW_Entity* entity, const QVector3D& basePoint, double scaleFactor);
    bool readEditableControlPoint(const CadItem* item, int pointIndex, QVector3D& point);
    bool applyEditableControlPoint(DRW_Entity* entity, int pointIndex, const QVector3D& worldPos);
    std::unique_ptr<DRW_Entity> cloneEntity(const DRW_Entity* entity);
    bool pointsNear(const QVector3D& first, const QVector3D& second, double tolerance = 1.0e-4);
    bool mirrorEntityGeometry(DRW_Entity* entity, const QVector3D& lineStart, const QVector3D& lineEnd);
    std::unique_ptr<DRW_Entity> createOffsetEntity(const CadItem* item, double distance);
    bool trimOrExtendLineEntity(DRW_Entity* targetEntity, const DRW_Entity* boundaryEntity, bool startSide, bool trimMode);
    QVector3D itemGeometryCenter(const CadItem* item);
    bool buildJoinedPolylineEntity(const QVector<CadItem*>& items, std::unique_ptr<DRW_Entity>& joinedEntity);
    bool buildChamferReplacementEntities
    (
        const CadItem* firstItem,
        const CadItem* secondItem,
        double firstDistance,
        double secondDistance,
        std::vector<std::unique_ptr<DRW_Entity>>& replacements
    );
    bool buildFilletReplacementEntities
    (
        const CadItem* firstItem,
        const CadItem* secondItem,
        double radius,
        std::vector<std::unique_ptr<DRW_Entity>>& replacements
    );
    int colorToTrueColor(const QColor& color);
    QVector3D polylineEndTangent(const QVector<QVector3D>& points, const QVector<double>& bulges);
    double bulgeFromTangent(const QVector3D& startPoint, const QVector3D& tangentDirection, const QVector3D& endPoint);
    std::unique_ptr<DRW_Entity> createPointEntity(const QVector3D& position, const QString& layerName, const QColor& color, int colorIndex);
    std::unique_ptr<DRW_Entity> createLineEntity
    (
        const QVector3D& startPoint,
        const QVector3D& endPoint,
        const QString& layerName,
        const QColor& color,
        int colorIndex
    );
    std::unique_ptr<DRW_Entity> createXlineEntity
    (
        const QVector3D& basePoint,
        const QVector3D& throughPoint,
        const QString& layerName,
        const QColor& color,
        int colorIndex
    );
    std::unique_ptr<DRW_Entity> createRectangleEntity
    (
        const QVector3D& firstCorner,
        const QVector3D& secondCorner,
        const QString& layerName,
        const QColor& color,
        int colorIndex
    );
    std::unique_ptr<DRW_Entity> createPolygonEntity
    (
        const QVector3D& center,
        const QVector3D& radiusPoint,
        int sideCount,
        bool circumscribedAboutCircle,
        const QString& layerName,
        const QColor& color,
        int colorIndex
    );
    std::unique_ptr<DRW_Entity> createCircleEntity
    (
        const QVector3D& center,
        const QVector3D& radiusPoint,
        const QString& layerName,
        const QColor& color,
        int colorIndex
    );
    std::unique_ptr<DRW_Entity> createArcEntity
    (
        const QVector3D& center,
        const QVector3D& radiusPoint,
        const QVector3D& startAnglePoint,
        const QVector3D& endAnglePoint,
        const QString& layerName,
        const QColor& color,
        int colorIndex,
        bool useComplementArc = false
    );
    std::unique_ptr<DRW_Entity> createEllipseEntity
    (
        const QVector3D& center,
        const QVector3D& majorAxisPoint,
        const QVector3D& ratioPoint,
        const QString& layerName,
        const QColor& color,
        int colorIndex
    );
    std::unique_ptr<DRW_Entity> createPolylineEntity
    (
        const QVector<QVector3D>& points,
        const QVector<double>& bulges,
        const QString& layerName,
        const QColor& color,
        int colorIndex,
        bool closePolyline,
        bool lightweight
    );
}
