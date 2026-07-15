#include "platform/pch.h"

#include "cad/items/CadSplineItem.h"
#include "application/messaging/DebugMessageSink.h"
#include "compatibility/legacy/LegacyCadItemPathBridge.h"
#include "compatibility/legacy/SplineProductionPathProvider.h"

namespace
{
    Diagnostic splineItemDiagnostic
    (
        const CadSplineItem& item,
        DiagnosticCode code,
        const QString& detail
    )
    {
        Diagnostic diagnostic;
        diagnostic.code = code;
        diagnostic.severity = DiagnosticSeverity::Error;
        diagnostic.component = QStringLiteral("CadSplineItem");
        diagnostic.operation = QStringLiteral("BuildSplineItemPath");
        diagnostic.stage = QStringLiteral("BuildCadItemCache");
        diagnostic.userMessage = code == DiagnosticCode::SplineDisplayPathFailure
            ? QStringLiteral("样条曲线显示路径生成失败。")
            : QStringLiteral("样条曲线四轴控制点生成失败。");
        diagnostic.technicalDetail = detail;
        diagnostic.entityId = item.m_entityId;
        return diagnostic;
    }

    void publishDiagnostics(const QVector<Diagnostic>& diagnostics)
    {
        DebugMessageSink sink;
        for (const Diagnostic& diagnostic : diagnostics)
        {
            sink.publish(diagnostic);
        }
    }
}

CadSplineItem::CadSplineItem(DRW_Entity* entity, QObject* parent)
    : CadItem(entity, parent)
{
    m_data = static_cast<DRW_Spline*>(m_nativeEntity);
    buildGeometryDatay();
}

void CadSplineItem::buildGeometryDatay()
{
    m_geometry.vertices.clear();
    clearPathCaches();
    if (m_data == nullptr)
    {
        return;
    }

    const OperationResult<cadcam::geometry::Path3D> result =
        SplineProductionPathProvider::build
        (
            m_entityId,
            *m_data,
            LegacyCadItemPathBridge::legacySamplingPolicy(*this),
            {},
            createOperationContext(QStringLiteral("build-spline-display-path"))
        );
    if (!result.succeeded() || !result.value.has_value())
    {
        QVector<Diagnostic> diagnostics = result.diagnostics;
        diagnostics.push_back(splineItemDiagnostic
            (*this, DiagnosticCode::SplineDisplayPathFailure,
                QStringLiteral("production path provider returned no display path")));
        publishDiagnostics(diagnostics);
        return;
    }

    m_geometry.vertices.reserve
        (static_cast<int>(result.value->vertices.size()) + (result.value->closed ? 1 : 0));
    for (const cadcam::geometry::PathVertex3D& vertex : result.value->vertices)
    {
        m_geometry.vertices.push_back(QVector3D
        (
            static_cast<float>(vertex.position.x),
            static_cast<float>(vertex.position.y),
            static_cast<float>(vertex.position.z)
        ));
    }
    if (result.value->closed && !m_geometry.vertices.isEmpty())
    {
        m_geometry.vertices.push_back(m_geometry.vertices.constFirst());
    }
    publishDiagnostics(result.diagnostics);
}

void CadSplineItem::rebuildRawPathPoints3D()
{
    m_rawPathPoints3D.clear();
    cadcam::geometry::PathCompileOptions options;
    const OperationResult<cadcam::geometry::Path3D> result = LegacyCadItemPathBridge::compile
    (
        *this,
        LegacyCadItemPathBridge::legacySamplingPolicy(*this),
        options,
        createOperationContext(QStringLiteral("rebuild-spline-raw-path"))
    );
    if (result.succeeded() && result.value.has_value())
    {
        LegacyCadItemPathBridge::copyToLegacyRawPath
            (*result.value, m_rawPathPoints3D);
    }
    publishDiagnostics(result.diagnostics);
}
