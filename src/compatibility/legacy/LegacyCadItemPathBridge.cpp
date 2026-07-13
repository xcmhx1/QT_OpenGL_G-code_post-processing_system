#include "compatibility/legacy/LegacyCadItemPathBridge.h"

#include "infrastructure/dxf/DxfGeometryAdapter.h"

namespace
{
    constexpr double kTwoPi = 6.28318530717958647692;
}

cadcam::geometry::SamplingPolicy LegacyCadItemPathBridge::legacySamplingPolicy(const CadItem& item)
{
    cadcam::geometry::SamplingPolicy policy;
    policy.chordTolerance = 0.0;
    policy.singlePrecisionEvaluation = true;

    switch (item.m_type)
    {
    case DRW::ETYPE::CIRCLE:
        policy.minimumSegments = 128;
        policy.fullTurnSegments = 128;
        break;
    case DRW::ETYPE::ARC:
        policy.minimumSegments = 8;
        policy.maximumAngularStep = 5.0 * kTwoPi / 360.0;
        policy.fullTurnSegments = 128;
        break;
    case DRW::ETYPE::ELLIPSE:
        policy.minimumSegments = 16;
        policy.fullTurnSegments = 128;
        break;
    case DRW::ETYPE::LINE:
    default:
        policy.minimumSegments = 1;
        break;
    }

    return policy;
}

OperationResult<cadcam::geometry::Path3D> LegacyCadItemPathBridge::compile
(
    const CadItem& item,
    const cadcam::geometry::SamplingPolicy& policy,
    const cadcam::geometry::PathCompileOptions& options,
    const OperationContext& context
)
{
    if (item.m_nativeEntity == nullptr)
    {
        OperationResult<cadcam::geometry::Path3D> result;
        result.status = OperationStatus::InvalidInput;
        Diagnostic diagnostic;
        diagnostic.code = DiagnosticCode::GeometryAdapterFailure;
        diagnostic.severity = DiagnosticSeverity::Error;
        diagnostic.component = QStringLiteral("LegacyCadItemPathBridge");
        diagnostic.operation = QStringLiteral("CompileLegacyPath");
        diagnostic.stage = QStringLiteral("AdaptSourceGeometry");
        diagnostic.userMessage = QStringLiteral("旧图元缺少原始 DXF 实体。");
        diagnostic.technicalDetail = QStringLiteral("CadItem::m_nativeEntity is nullptr");
        diagnostic.correlationId = context.correlationId;
        diagnostic.entityId = item.m_entityId;
        result.addDiagnostic(diagnostic);
        return result;
    }

    OperationResult<cadcam::geometry::SourceEntity> sourceResult = DxfGeometryAdapter::convert
    (
        item.m_entityId,
        *item.m_nativeEntity,
        context
    );
    if (!sourceResult.succeeded() || !sourceResult.value.has_value())
    {
        OperationResult<cadcam::geometry::Path3D> result;
        result.status = sourceResult.status;
        result.mergeDiagnostics(sourceResult);
        return result;
    }

    cadcam::geometry::GeometryCompiler compiler;
    OperationResult<cadcam::geometry::Path3D> result = compiler.compile
    (
        *sourceResult.value,
        policy,
        options,
        context
    );
    result.mergeDiagnostics(sourceResult);
    return result;
}

void LegacyCadItemPathBridge::copyToLegacyRawPath
(
    const cadcam::geometry::Path3D& path,
    std::vector<RawPathPoint3D>& destination
)
{
    destination.clear();
    destination.reserve(path.vertices.size() + (path.closed ? 1U : 0U));

    for (const cadcam::geometry::PathVertex3D& vertex : path.vertices)
    {
        destination.push_back
        ({
            vertex.position.x,
            vertex.position.y,
            vertex.position.z
        });
    }

    if (path.closed && !path.vertices.empty())
    {
        const cadcam::geometry::Vector3d& first = path.vertices.front().position;
        destination.push_back({ first.x, first.y, first.z });
    }
}
