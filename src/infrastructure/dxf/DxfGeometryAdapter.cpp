#include "infrastructure/dxf/DxfGeometryAdapter.h"

#include "CadOcsGeometry.h"
#include "libdxfrw/drw_entities.h"

#include <QVector3D>

#include <cmath>
#include <limits>
#include <memory>
#include <vector>

namespace
{
    using namespace cadcam::geometry;
    constexpr double kTwoPi = 6.28318530717958647692;
    constexpr double kPolylineTolerance = 1.0e-10;

    struct AdaptedPolylineVertex
    {
        Vector3d position;
        double bulge = 0.0;
    };

    Diagnostic makeAdapterDiagnostic
    (
        EntityId entityId,
        const OperationContext& context,
        DiagnosticCode code,
        const QString& userMessage,
        const QString& technicalDetail,
        const QVariantMap& diagnosticContext = QVariantMap()
    );

    Vector3d toVector3d(const QVector3D& value)
    {
        return { value.x(), value.y(), value.z() };
    }

    Vector3d toVector3d(const DRW_Coord& value)
    {
        return { value.x, value.y, value.z };
    }

    double squaredLength(const Vector3d& value)
    {
        return value.x * value.x + value.y * value.y + value.z * value.z;
    }

    Vector3d crossProduct(const Vector3d& left, const Vector3d& right)
    {
        return
        {
            left.y * right.z - left.z * right.y,
            left.z * right.x - left.x * right.z,
            left.x * right.y - left.y * right.x
        };
    }

    bool normalize(Vector3d& value)
    {
        const double lengthSquared = squaredLength(value);
        if (!std::isfinite(lengthSquared) || lengthSquared <= 1.0e-24)
        {
            return false;
        }
        const double inverseLength = 1.0 / std::sqrt(lengthSquared);
        value.x *= inverseLength;
        value.y *= inverseLength;
        value.z *= inverseLength;
        return true;
    }

    bool finiteVector(const Vector3d& value)
    {
        return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
    }

    Vector3d subtract(const Vector3d& left, const Vector3d& right)
    {
        return { left.x - right.x, left.y - right.y, left.z - right.z };
    }

    QVariantMap polylineContext
    (
        const QString& dxfType,
        std::size_t vertexCount,
        bool closed,
        bool is3DPolyline,
        int segmentIndex = -1,
        double bulge = 0.0
    )
    {
        QVariantMap values
        {
            { QStringLiteral("dxfType"), dxfType },
            { QStringLiteral("vertexCount"), static_cast<qulonglong>(vertexCount) },
            { QStringLiteral("segmentIndex"), segmentIndex },
            { QStringLiteral("bulge"), bulge },
            { QStringLiteral("closed"), closed },
            { QStringLiteral("is3DPolyline"), is3DPolyline }
        };
        return values;
    }

    QVariantMap splineContext(EntityId entityId, const SplineGeometry& spline)
    {
        return
        {
            { QStringLiteral("entityId"), static_cast<qulonglong>(entityId) },
            { QStringLiteral("degree"), spline.degree },
            { QStringLiteral("controlPointCount"),
                static_cast<qulonglong>(spline.controlPoints.size()) },
            { QStringLiteral("knotCount"), static_cast<qulonglong>(spline.knots.size()) },
            { QStringLiteral("weightCount"), static_cast<qulonglong>(spline.weights.size()) },
            { QStringLiteral("fitPointCount"), static_cast<qulonglong>(spline.fitPoints.size()) },
            { QStringLiteral("parameterStart"), spline.parameterStart },
            { QStringLiteral("parameterEnd"), spline.parameterEnd },
            { QStringLiteral("subdivisionDepth"), -1 },
            { QStringLiteral("generatedPointCount"), 0 },
            { QStringLiteral("closed"), spline.closed },
            { QStringLiteral("periodic"), spline.periodic },
            { QStringLiteral("rational"), spline.rational }
        };
    }

    Diagnostic makeSplineAdapterDiagnostic
    (
        EntityId entityId,
        const SplineGeometry& spline,
        const OperationContext& context,
        DiagnosticCode code,
        DiagnosticSeverity severity,
        const QString& detail
    )
    {
        Diagnostic diagnostic = makeAdapterDiagnostic
        (
            entityId,
            context,
            code,
            QStringLiteral("DXF 样条曲线数据无效。"),
            detail,
            splineContext(entityId, spline)
        );
        diagnostic.severity = severity;
        return diagnostic;
    }

    bool buildPolylinePlane
    (
        const DRW_Coord& extrusion,
        double elevation,
        QVector3D& origin,
        QVector3D& axisU,
        QVector3D& axisV,
        QVector3D& normal
    )
    {
        if (!std::isfinite(extrusion.x) || !std::isfinite(extrusion.y)
            || !std::isfinite(extrusion.z) || !std::isfinite(elevation))
        {
            return false;
        }

        normal = QVector3D
        (
            static_cast<float>(extrusion.x),
            static_cast<float>(extrusion.y),
            static_cast<float>(extrusion.z)
        );
        if (normal.lengthSquared() <= 1.0e-12f)
        {
            normal = QVector3D(0.0f, 0.0f, 1.0f);
        }
        else
        {
            normal.normalize();
        }

        const QVector3D helper = std::abs(normal.z()) < 0.999f
            ? QVector3D(0.0f, 0.0f, 1.0f)
            : QVector3D(0.0f, 1.0f, 0.0f);
        axisU = QVector3D::crossProduct(helper, normal);
        if (axisU.lengthSquared() <= 1.0e-12f)
        {
            return false;
        }
        axisU.normalize();
        axisV = QVector3D::crossProduct(normal, axisU);
        if (axisV.lengthSquared() <= 1.0e-12f)
        {
            return false;
        }
        axisV.normalize();
        origin = normal * static_cast<float>(elevation);
        return true;
    }

    Vector3d ocsVertexToWcs
    (
        double x,
        double y,
        const QVector3D& origin,
        const QVector3D& axisU,
        const QVector3D& axisV
    )
    {
        return toVector3d
        (
            origin
            + axisU * static_cast<float>(x)
            + axisV * static_cast<float>(y)
        );
    }

    bool makeBulgeArc
    (
        const AdaptedPolylineVertex& start,
        const AdaptedPolylineVertex& end,
        const QVector3D& origin,
        const QVector3D& axisU,
        const QVector3D& axisV,
        double bulge,
        ArcGeometry& arc
    )
    {
        if (!std::isfinite(bulge) || std::abs(bulge) < 1.0e-8)
        {
            return false;
        }

        const QVector3D startPoint
        (
            static_cast<float>(start.position.x),
            static_cast<float>(start.position.y),
            static_cast<float>(start.position.z)
        );
        const QVector3D endPoint
        (
            static_cast<float>(end.position.x),
            static_cast<float>(end.position.y),
            static_cast<float>(end.position.z)
        );
        const QVector3D startDelta = startPoint - origin;
        const QVector3D endDelta = endPoint - origin;
        const double startU = QVector3D::dotProduct(startDelta, axisU);
        const double startV = QVector3D::dotProduct(startDelta, axisV);
        const double endU = QVector3D::dotProduct(endDelta, axisU);
        const double endV = QVector3D::dotProduct(endDelta, axisV);
        const double dx = endU - startU;
        const double dy = endV - startV;
        const double chordLength = std::hypot(dx, dy);
        if (!std::isfinite(chordLength) || chordLength <= kPolylineTolerance)
        {
            return false;
        }

        const double middleU = (startU + endU) * 0.5;
        const double middleV = (startV + endV) * 0.5;
        const double centerOffset = chordLength * (1.0 / bulge - bulge) * 0.25;
        const double centerU = middleU - centerOffset * (dy / chordLength);
        const double centerV = middleV + centerOffset * (dx / chordLength);
        const double radius = std::hypot(startU - centerU, startV - centerV);
        const double startParameter = std::atan2(startV - centerV, startU - centerU);
        const double sweep = 4.0 * std::atan(bulge);
        if (!std::isfinite(radius) || radius <= kPolylineTolerance
            || !std::isfinite(startParameter) || !std::isfinite(sweep)
            || std::abs(sweep) <= kPolylineTolerance)
        {
            return false;
        }

        arc.center = toVector3d
        (
            origin
            + axisU * static_cast<float>(centerU)
            + axisV * static_cast<float>(centerV)
        );
        arc.axisU = toVector3d(axisU);
        arc.axisV = toVector3d(axisV);
        arc.radius = radius;
        arc.startParameter = startParameter;
        arc.endParameter = startParameter + sweep;
        return finiteVector(arc.center);
    }

    OperationResult<SourceEntity> buildPolylineSource
    (
        EntityId entityId,
        const QString& dxfType,
        std::vector<AdaptedPolylineVertex> vertices,
        bool closed,
        bool is3DPolyline,
        const QVector3D& origin,
        const QVector3D& axisU,
        const QVector3D& axisV,
        const OperationContext& context
    )
    {
        OperationResult<SourceEntity> result;
        const std::size_t vertexCount = vertices.size();
        if (vertexCount < 2)
        {
            result.status = OperationStatus::InvalidInput;
            result.addDiagnostic(makeAdapterDiagnostic
            (
                entityId, context, DiagnosticCode::InvalidPolyline,
                QStringLiteral("DXF 多段线顶点数量不足。"),
                QStringLiteral("Polyline requires at least two vertices"),
                polylineContext(dxfType, vertexCount, closed, is3DPolyline)
            ));
            return result;
        }

        PolylineGeometry geometry;
        geometry.sourceVertexCount = vertexCount;
        geometry.closed = closed;
        const std::size_t segmentCount = closed ? vertexCount : vertexCount - 1U;
        geometry.segments.reserve(segmentCount);
        for (std::size_t index = 0; index < segmentCount; ++index)
        {
            const AdaptedPolylineVertex& start = vertices[index];
            const AdaptedPolylineVertex& end = vertices[(index + 1U) % vertexCount];
            const double bulge = start.bulge;
            if (!finiteVector(start.position) || !finiteVector(end.position))
            {
                result.status = OperationStatus::InvalidInput;
                result.addDiagnostic(makeAdapterDiagnostic
                (
                    entityId, context, DiagnosticCode::InvalidPolylineVertex,
                    QStringLiteral("DXF 多段线包含无效顶点。"),
                    QStringLiteral("Polyline vertex contains NaN or infinity"),
                    polylineContext(dxfType, vertexCount, closed, is3DPolyline,
                        static_cast<int>(index), bulge)
                ));
                return result;
            }
            if (squaredLength(subtract(end.position, start.position))
                <= kPolylineTolerance * kPolylineTolerance)
            {
                result.status = OperationStatus::InvalidInput;
                result.addDiagnostic(makeAdapterDiagnostic
                (
                    entityId, context, DiagnosticCode::InvalidPolyline,
                    QStringLiteral("DXF 多段线包含退化线段。"),
                    QStringLiteral("Polyline segment has zero length"),
                    polylineContext(dxfType, vertexCount, closed, is3DPolyline,
                        static_cast<int>(index), bulge)
                ));
                return result;
            }

            if (!is3DPolyline && std::abs(bulge) >= 1.0e-8)
            {
                ArcGeometry arc;
                if (!makeBulgeArc(start, end, origin, axisU, axisV, bulge, arc))
                {
                    result.status = OperationStatus::InvalidInput;
                    result.addDiagnostic(makeAdapterDiagnostic
                    (
                        entityId, context, DiagnosticCode::InvalidBulge,
                        QStringLiteral("DXF 多段线 bulge 圆弧参数无效。"),
                        QStringLiteral("Bulge arc construction failed"),
                        polylineContext(dxfType, vertexCount, closed, is3DPolyline,
                            static_cast<int>(index), bulge)
                    ));
                    return result;
                }
                geometry.segments.emplace_back(std::move(arc));
            }
            else
            {
                if (!is3DPolyline && !std::isfinite(bulge))
                {
                    result.status = OperationStatus::InvalidInput;
                    result.addDiagnostic(makeAdapterDiagnostic
                    (
                        entityId, context, DiagnosticCode::InvalidBulge,
                        QStringLiteral("DXF 多段线 bulge 参数无效。"),
                        QStringLiteral("Bulge is NaN or infinity"),
                        polylineContext(dxfType, vertexCount, closed, is3DPolyline,
                            static_cast<int>(index), bulge)
                    ));
                    return result;
                }
                geometry.segments.emplace_back(LineGeometry{ start.position, end.position });
            }
        }

        SourceEntity source;
        source.id = entityId;
        source.kind = SourceGeometryKind::Polyline;
        source.geometry = std::move(geometry);
        result.status = OperationStatus::Success;
        result.value = std::move(source);
        return result;
    }

    double normalizedPositiveEnd(double start, double end)
    {
        while (end <= start)
        {
            end += kTwoPi;
        }
        return end;
    }

    Diagnostic makeAdapterDiagnostic
    (
        EntityId entityId,
        const OperationContext& context,
        DiagnosticCode code,
        const QString& userMessage,
        const QString& technicalDetail,
        const QVariantMap& diagnosticContext
    )
    {
        Diagnostic diagnostic;
        diagnostic.code = code;
        diagnostic.severity = DiagnosticSeverity::Error;
        diagnostic.component = QStringLiteral("DxfGeometryAdapter");
        diagnostic.operation = QStringLiteral("ConvertDxfGeometry");
        diagnostic.stage = QStringLiteral("AdaptSourceGeometry");
        diagnostic.userMessage = userMessage;
        diagnostic.technicalDetail = technicalDetail;
        diagnostic.correlationId = context.correlationId;
        diagnostic.entityId = entityId;
        diagnostic.context = diagnosticContext;
        diagnostic.context.insert(QStringLiteral("entityId"), static_cast<qulonglong>(entityId));
        return diagnostic;
    }
}

OperationResult<cadcam::geometry::SourceEntity> DxfGeometryAdapter::convert
(
    cadcam::geometry::EntityId entityId,
    const DRW_Entity& entity,
    const OperationContext& context
)
{
    using namespace cadcam::geometry;

    OperationResult<SourceEntity> result;
    if (entityId == 0)
    {
        result.status = OperationStatus::InvalidInput;
        result.addDiagnostic(makeAdapterDiagnostic
        (
            entityId,
            context,
            DiagnosticCode::InvalidArgument,
            QStringLiteral("DXF 图元缺少有效的内部编号。"),
            QStringLiteral("entityId is zero")
        ));
        return result;
    }

    SourceEntity source;
    source.id = entityId;

    switch (entity.eType)
    {
    case DRW::ETYPE::LINE:
    {
        const auto& line = static_cast<const DRW_Line&>(entity);
        source.kind = SourceGeometryKind::Line;
        source.geometry = LineGeometry
        {
            toVector3d(line.basePoint),
            toVector3d(line.secPoint)
        };
        break;
    }
    case DRW::ETYPE::CIRCLE:
    {
        const auto& circle = static_cast<const DRW_Circle&>(entity);
        QVector3D axisU;
        QVector3D axisV;
        QVector3D normal;
        CadOcsGeometry::basis(circle.extPoint, axisU, axisV, normal);
        const Vector3d center = toVector3d(CadOcsGeometry::center(&circle));
        const Vector3d worldAxisU = toVector3d(axisU);
        const Vector3d worldAxisV = toVector3d(axisV);
        if (!std::isfinite(circle.radious) || circle.radious <= 0.0
            || !finiteVector(center) || !finiteVector(worldAxisU) || !finiteVector(worldAxisV))
        {
            result.status = OperationStatus::InvalidInput;
            result.addDiagnostic(makeAdapterDiagnostic
            (
                entityId,
                context,
                DiagnosticCode::DegenerateGeometry,
                QStringLiteral("DXF 圆半径无效。"),
                QStringLiteral("DRW_Circle::radious is not positive"),
                { { QStringLiteral("radius"), circle.radious } }
            ));
            return result;
        }
        source.kind = SourceGeometryKind::Circle;
        source.geometry = CircleGeometry
        {
            center,
            worldAxisU,
            worldAxisV,
            circle.radious
        };
        break;
    }
    case DRW::ETYPE::ARC:
    {
        const auto& arc = static_cast<const DRW_Arc&>(entity);
        QVector3D axisU;
        QVector3D axisV;
        QVector3D normal;
        CadOcsGeometry::basis(arc.extPoint, axisU, axisV, normal);
        const Vector3d center = toVector3d(CadOcsGeometry::center(&arc));
        const Vector3d worldAxisU = toVector3d(axisU);
        const Vector3d worldAxisV = toVector3d(axisV);
        if (!std::isfinite(arc.radious) || arc.radious <= 0.0
            || !std::isfinite(arc.staangle) || !std::isfinite(arc.endangle)
            || !finiteVector(center) || !finiteVector(worldAxisU) || !finiteVector(worldAxisV))
        {
            result.status = OperationStatus::InvalidInput;
            result.addDiagnostic(makeAdapterDiagnostic
            (
                entityId,
                context,
                DiagnosticCode::DegenerateGeometry,
                QStringLiteral("DXF 圆弧参数无效。"),
                QStringLiteral("Arc radius or parameters are not finite"),
                {
                    { QStringLiteral("radius"), arc.radious },
                    { QStringLiteral("startParameter"), arc.staangle },
                    { QStringLiteral("endParameter"), arc.endangle }
                }
            ));
            return result;
        }
        source.kind = SourceGeometryKind::Arc;
        source.geometry = ArcGeometry
        {
            center,
            worldAxisU,
            worldAxisV,
            arc.radious,
            arc.staangle,
            normalizedPositiveEnd(arc.staangle, arc.endangle)
        };
        break;
    }
    case DRW::ETYPE::ELLIPSE:
    {
        const auto& ellipse = static_cast<const DRW_Ellipse&>(entity);
        const Vector3d center = toVector3d(ellipse.basePoint);
        const Vector3d majorAxis = toVector3d(ellipse.secPoint);
        Vector3d normal = toVector3d(ellipse.extPoint);
        if (squaredLength(normal) <= 1.0e-24)
        {
            normal = { 0.0, 0.0, 1.0 };
        }
        const double majorLength = std::sqrt(squaredLength(majorAxis));
        Vector3d minorAxis = crossProduct(normal, majorAxis);
        const bool validMinorDirection = normalize(minorAxis);
        if (validMinorDirection)
        {
            const double minorLength = majorLength * ellipse.ratio;
            minorAxis.x *= minorLength;
            minorAxis.y *= minorLength;
            minorAxis.z *= minorLength;
        }
        const double parameterSpan = ellipse.endparam - ellipse.staparam;
        const bool fullEllipse = std::abs(parameterSpan) <= 1.0e-6
            || std::abs(std::abs(parameterSpan) - kTwoPi) <= 1.0e-6;
        if (!std::isfinite(ellipse.ratio) || ellipse.ratio <= 0.0 || ellipse.ratio > 1.0
            || !std::isfinite(ellipse.staparam) || !std::isfinite(ellipse.endparam)
            || !finiteVector(center) || !finiteVector(majorAxis) || !finiteVector(normal)
            || !validMinorDirection || majorLength <= 1.0e-12
            || squaredLength(minorAxis) <= 1.0e-12)
        {
            result.status = OperationStatus::InvalidInput;
            result.addDiagnostic(makeAdapterDiagnostic
            (
                entityId,
                context,
                DiagnosticCode::GeometryAdapterFailure,
                QStringLiteral("DXF 椭圆轴参数无效。"),
                QStringLiteral("Ellipse major/minor axis construction failed"),
                {
                    { QStringLiteral("ratio"), ellipse.ratio },
                    { QStringLiteral("startParameter"), ellipse.staparam },
                    { QStringLiteral("endParameter"), ellipse.endparam }
                }
            ));
            return result;
        }
        source.kind = SourceGeometryKind::Ellipse;
        const double normalizedEnd = fullEllipse
            ? ellipse.staparam + kTwoPi
            : normalizedPositiveEnd(ellipse.staparam, ellipse.endparam);
        source.geometry = EllipseGeometry
        {
            center,
            majorAxis,
            minorAxis,
            ellipse.staparam,
            normalizedEnd,
            fullEllipse
        };
        break;
    }
    case DRW::ETYPE::POLYLINE:
    {
        const auto& polyline = static_cast<const DRW_Polyline&>(entity);
        const bool closed = (polyline.flags & 1) != 0;
        const bool is3DPolyline = (polyline.flags & 8) != 0;
        const std::size_t vertexCount = polyline.vertlist.size();
        QVector3D origin;
        QVector3D axisU;
        QVector3D axisV;
        QVector3D normal;
        if (!is3DPolyline && !buildPolylinePlane
            (polyline.extPoint, polyline.basePoint.z, origin, axisU, axisV, normal))
        {
            result.status = OperationStatus::InvalidInput;
            result.addDiagnostic(makeAdapterDiagnostic
            (
                entityId, context, DiagnosticCode::PolylinePlaneFailure,
                QStringLiteral("DXF 多段线 OCS 平面无效。"),
                QStringLiteral("POLYLINE OCS to WCS basis construction failed"),
                polylineContext(QStringLiteral("POLYLINE"), vertexCount, closed, false)
            ));
            return result;
        }

        std::vector<AdaptedPolylineVertex> vertices;
        vertices.reserve(vertexCount);
        for (std::size_t index = 0; index < vertexCount; ++index)
        {
            const std::shared_ptr<DRW_Vertex>& vertex = polyline.vertlist[index];
            if (!vertex)
            {
                result.status = OperationStatus::InvalidInput;
                result.addDiagnostic(makeAdapterDiagnostic
                (
                    entityId, context, DiagnosticCode::InvalidPolylineVertex,
                    QStringLiteral("DXF 多段线包含空顶点。"),
                    QStringLiteral("POLYLINE vertex pointer is null"),
                    polylineContext(QStringLiteral("POLYLINE"), vertexCount, closed,
                        is3DPolyline, static_cast<int>(index))
                ));
                return result;
            }
            vertices.push_back
            ({
                is3DPolyline
                    ? Vector3d{ vertex->basePoint.x, vertex->basePoint.y, vertex->basePoint.z }
                    : ocsVertexToWcs(vertex->basePoint.x, vertex->basePoint.y,
                        origin, axisU, axisV),
                vertex->bulge
            });
        }
        return buildPolylineSource
        (
            entityId, QStringLiteral("POLYLINE"), std::move(vertices), closed,
            is3DPolyline, origin, axisU, axisV, context
        );
    }
    case DRW::ETYPE::LWPOLYLINE:
    {
        const auto& polyline = static_cast<const DRW_LWPolyline&>(entity);
        const bool closed = (polyline.flags & 1) != 0;
        const std::size_t vertexCount = polyline.vertlist.size();
        QVector3D origin;
        QVector3D axisU;
        QVector3D axisV;
        QVector3D normal;
        if (!buildPolylinePlane
            (polyline.extPoint, polyline.elevation, origin, axisU, axisV, normal))
        {
            result.status = OperationStatus::InvalidInput;
            result.addDiagnostic(makeAdapterDiagnostic
            (
                entityId, context, DiagnosticCode::PolylinePlaneFailure,
                QStringLiteral("DXF 轻量多段线 OCS 平面无效。"),
                QStringLiteral("LWPOLYLINE OCS to WCS basis construction failed"),
                polylineContext(QStringLiteral("LWPOLYLINE"), vertexCount, closed, false)
            ));
            return result;
        }

        std::vector<AdaptedPolylineVertex> vertices;
        vertices.reserve(vertexCount);
        for (std::size_t index = 0; index < vertexCount; ++index)
        {
            const std::shared_ptr<DRW_Vertex2D>& vertex = polyline.vertlist[index];
            if (!vertex)
            {
                result.status = OperationStatus::InvalidInput;
                result.addDiagnostic(makeAdapterDiagnostic
                (
                    entityId, context, DiagnosticCode::InvalidPolylineVertex,
                    QStringLiteral("DXF 轻量多段线包含空顶点。"),
                    QStringLiteral("LWPOLYLINE vertex pointer is null"),
                    polylineContext(QStringLiteral("LWPOLYLINE"), vertexCount, closed,
                        false, static_cast<int>(index))
                ));
                return result;
            }
            vertices.push_back
            ({
                ocsVertexToWcs(vertex->x, vertex->y, origin, axisU, axisV),
                vertex->bulge
            });
        }
        return buildPolylineSource
        (
            entityId, QStringLiteral("LWPOLYLINE"), std::move(vertices), closed,
            false, origin, axisU, axisV, context
        );
    }
    case DRW::ETYPE::SPLINE:
    {
        const auto& spline = static_cast<const DRW_Spline&>(entity);
        SplineGeometry geometry;
        geometry.degree = spline.degree;
        geometry.closed = (spline.flags & 1) != 0;
        geometry.periodic = (spline.flags & 2) != 0;
        geometry.rational = (spline.flags & 4) != 0;
        geometry.knots = spline.knotslist;
        geometry.weights = spline.weightlist;
        geometry.controlPoints.reserve(spline.controllist.size());
        geometry.fitPoints.reserve(spline.fitlist.size());

        const double nan = (std::numeric_limits<double>::quiet_NaN)();
        bool controlPointsFinite = true;
        for (const std::shared_ptr<DRW_Coord>& point : spline.controllist)
        {
            if (!point)
            {
                geometry.controlPoints.push_back({ nan, nan, nan });
                controlPointsFinite = false;
                continue;
            }
            const Vector3d converted = toVector3d(*point);
            geometry.controlPoints.push_back(converted);
            controlPointsFinite = controlPointsFinite && finiteVector(converted);
        }

        std::size_t validFitPointCount = 0U;
        bool fitPointsFinite = true;
        for (const std::shared_ptr<DRW_Coord>& point : spline.fitlist)
        {
            if (!point)
            {
                geometry.fitPoints.push_back({ nan, nan, nan });
                fitPointsFinite = false;
                continue;
            }
            const Vector3d converted = toVector3d(*point);
            geometry.fitPoints.push_back(converted);
            if (finiteVector(converted))
            {
                ++validFitPointCount;
            }
            else
            {
                fitPointsFinite = false;
            }
        }

        if (geometry.weights.size() < geometry.controlPoints.size())
        {
            geometry.weights.resize(geometry.controlPoints.size(), 1.0);
        }
        bool weightsValid = true;
        for (double weight : geometry.weights)
        {
            weightsValid = weightsValid
                && std::isfinite(weight)
                && std::abs(weight) > 1.0e-15;
        }
        bool knotsValid = !geometry.knots.empty();
        for (std::size_t index = 0; index < geometry.knots.size(); ++index)
        {
            knotsValid = knotsValid
                && std::isfinite(geometry.knots[index])
                && (index == 0U || geometry.knots[index] >= geometry.knots[index - 1U]);
        }

        const std::size_t controlCount = geometry.controlPoints.size();
        const bool degreeValid = geometry.degree >= 1
            && controlCount > static_cast<std::size_t>(geometry.degree);
        const std::size_t requiredKnotCount = degreeValid
            ? controlCount + static_cast<std::size_t>(geometry.degree) + 1U
            : 0U;
        knotsValid = knotsValid && degreeValid
            && geometry.knots.size() >= requiredKnotCount;
        if (knotsValid)
        {
            geometry.parameterStart = geometry.knots[static_cast<std::size_t>(geometry.degree)];
            geometry.parameterEnd = geometry.knots[controlCount];
        }
        const bool domainValid = knotsValid
            && std::isfinite(geometry.parameterStart)
            && std::isfinite(geometry.parameterEnd)
            && geometry.parameterEnd > geometry.parameterStart;
        const bool exactValid = degreeValid
            && controlPointsFinite
            && knotsValid
            && weightsValid
            && domainValid;
        const bool fallbackAvailable = validFitPointCount >= 2U;

        SourceEntity splineSource;
        splineSource.id = entityId;
        splineSource.kind = SourceGeometryKind::Spline;
        splineSource.geometry = geometry;

        const DiagnosticSeverity invalidSeverity = fallbackAvailable
            ? DiagnosticSeverity::Warning
            : DiagnosticSeverity::Error;
        if (!degreeValid)
        {
            result.addDiagnostic(makeSplineAdapterDiagnostic
            (entityId, geometry, context, DiagnosticCode::InvalidSplineDegree,
                invalidSeverity, QStringLiteral("degree or control point count is invalid")));
        }
        if (!controlPointsFinite)
        {
            result.addDiagnostic(makeSplineAdapterDiagnostic
            (entityId, geometry, context, DiagnosticCode::InvalidSplineControlPoints,
                invalidSeverity, QStringLiteral("control point is null, NaN or infinity")));
        }
        if (!knotsValid)
        {
            result.addDiagnostic(makeSplineAdapterDiagnostic
            (entityId, geometry, context, DiagnosticCode::InvalidSplineKnots,
                invalidSeverity, QStringLiteral("knot vector is invalid")));
        }
        if (!weightsValid)
        {
            result.addDiagnostic(makeSplineAdapterDiagnostic
            (entityId, geometry, context, DiagnosticCode::InvalidSplineWeights,
                invalidSeverity, QStringLiteral("weight is zero, NaN or infinity")));
        }
        if (!domainValid)
        {
            result.addDiagnostic(makeSplineAdapterDiagnostic
            (entityId, geometry, context, DiagnosticCode::InvalidSplineParameterDomain,
                invalidSeverity, QStringLiteral("effective parameter domain is invalid")));
        }
        if (!fitPointsFinite)
        {
            result.addDiagnostic(makeSplineAdapterDiagnostic
            (entityId, geometry, context, DiagnosticCode::InvalidSpline,
                exactValid ? DiagnosticSeverity::Warning : invalidSeverity,
                QStringLiteral("fit point is null, NaN or infinity")));
        }

        if (!exactValid && !fallbackAvailable)
        {
            result.status = OperationStatus::InvalidInput;
            if (result.diagnostics.isEmpty())
            {
                result.addDiagnostic(makeSplineAdapterDiagnostic
                (entityId, geometry, context, DiagnosticCode::InvalidSpline,
                    DiagnosticSeverity::Error,
                    QStringLiteral("exact NURBS and fit-point fallback are both unavailable")));
            }
            return result;
        }

        result.status = exactValid && fitPointsFinite
            ? OperationStatus::Success
            : OperationStatus::PartialSuccess;
        result.value = std::move(splineSource);
        return result;
    }
    default:
        result.status = OperationStatus::NotSupported;
        result.addDiagnostic(makeAdapterDiagnostic
        (
            entityId,
            context,
            DiagnosticCode::UnsupportedGeometry,
            QStringLiteral("当前几何内核尚不支持该 DXF 图元。"),
            QStringLiteral("DXF entity type is outside the phase-two migration scope"),
            { { QStringLiteral("dxfEntityType"), static_cast<int>(entity.eType) } }
        ));
        return result;
    }

    result.status = OperationStatus::Success;
    result.value = std::move(source);
    return result;
}
