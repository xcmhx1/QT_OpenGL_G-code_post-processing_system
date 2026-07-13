#include "infrastructure/dxf/DxfGeometryAdapter.h"

#include "CadEllipseGeometry.h"
#include "CadOcsGeometry.h"
#include "libdxfrw/drw_entities.h"

#include <QVector3D>

#include <cmath>

namespace
{
    using namespace cadcam::geometry;
    constexpr double kTwoPi = 6.28318530717958647692;

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

    bool finiteVector(const Vector3d& value)
    {
        return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
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
        const QVariantMap& diagnosticContext = QVariantMap()
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
        CadEllipseGeometry geometry;
        if (!std::isfinite(ellipse.ratio) || ellipse.ratio <= 0.0 || ellipse.ratio > 1.0
            || !CadEllipseGeometryUtils::buildEllipseGeometry(&ellipse, geometry)
            || !finiteVector(toVector3d(geometry.center))
            || !finiteVector(toVector3d(geometry.majorAxis))
            || !finiteVector(toVector3d(geometry.minorAxis))
            || squaredLength(toVector3d(geometry.majorAxis)) <= 1.0e-12
            || squaredLength(toVector3d(geometry.minorAxis)) <= 1.0e-12)
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
        const double normalizedEnd = geometry.full
            ? geometry.startParameter + kTwoPi
            : normalizedPositiveEnd(geometry.startParameter, geometry.endParameter);
        source.geometry = EllipseGeometry
        {
            toVector3d(geometry.center),
            toVector3d(geometry.majorAxis),
            toVector3d(geometry.minorAxis),
            geometry.startParameter,
            normalizedEnd,
            geometry.full
        };
        break;
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
