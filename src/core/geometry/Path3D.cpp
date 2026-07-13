#include "core/geometry/Path3D.h"

#include <cmath>

namespace cadcam::geometry
{
    namespace
    {
        constexpr double kLengthTolerance = 1.0e-12;

        double squaredDistance(const Vector3d& left, const Vector3d& right)
        {
            const double dx = left.x - right.x;
            const double dy = left.y - right.y;
            const double dz = left.z - right.z;
            return dx * dx + dy * dy + dz * dz;
        }

        Diagnostic makePathDiagnostic
        (
            const Path3D& path,
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
            diagnostic.component = QStringLiteral("GeometryCore");
            diagnostic.operation = QStringLiteral("ValidatePath3D");
            diagnostic.stage = QStringLiteral("PathInvariant");
            diagnostic.userMessage = userMessage;
            diagnostic.technicalDetail = technicalDetail;
            diagnostic.correlationId = context.correlationId;
            diagnostic.entityId = path.sourceEntityId;
            diagnostic.context = diagnosticContext;
            diagnostic.context.insert
            (
                QStringLiteral("sourceKind"),
                QString::fromLatin1(sourceGeometryKindName(path.sourceKind))
            );
            diagnostic.context.insert
            (
                QStringLiteral("vertexCount"),
                static_cast<qulonglong>(path.vertices.size())
            );
            return diagnostic;
        }
    }

    OperationReport validatePath3D(const Path3D& path, const OperationContext& context)
    {
        OperationReport report;

        if (path.sourceEntityId == 0)
        {
            report.status = OperationStatus::InvalidInput;
            report.addDiagnostic(makePathDiagnostic
            (
                path,
                context,
                DiagnosticCode::PathInvariantViolation,
                QStringLiteral("路径缺少有效的源图元编号。"),
                QStringLiteral("sourceEntityId is zero")
            ));
            return report;
        }

        const std::size_t minimumVertexCount = path.closed ? 3U : 2U;
        if (path.vertices.size() < minimumVertexCount)
        {
            report.status = OperationStatus::Failed;
            report.addDiagnostic(makePathDiagnostic
            (
                path,
                context,
                DiagnosticCode::DegenerateGeometry,
                QStringLiteral("路径顶点数量不足。"),
                QStringLiteral("Path does not contain enough vertices")
            ));
            return report;
        }

        double accumulatedLength = 0.0;
        for (std::size_t index = 0; index < path.vertices.size(); ++index)
        {
            const Vector3d& point = path.vertices[index].position;
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
            {
                report.status = OperationStatus::InvalidInput;
                report.addDiagnostic(makePathDiagnostic
                (
                    path,
                    context,
                    DiagnosticCode::PathInvariantViolation,
                    QStringLiteral("路径包含非有限坐标。"),
                    QStringLiteral("A path coordinate is NaN or infinity"),
                    { { QStringLiteral("vertexIndex"), static_cast<qulonglong>(index) } }
                ));
                return report;
            }

            if (index == 0)
            {
                continue;
            }

            const double segmentLengthSquared = squaredDistance
            (
                path.vertices[index - 1].position,
                point
            );
            if (segmentLengthSquared <= kLengthTolerance * kLengthTolerance)
            {
                report.status = OperationStatus::Failed;
                report.addDiagnostic(makePathDiagnostic
                (
                    path,
                    context,
                    DiagnosticCode::PathInvariantViolation,
                    QStringLiteral("路径包含连续重复点。"),
                    QStringLiteral("Adjacent path vertices are coincident"),
                    { { QStringLiteral("vertexIndex"), static_cast<qulonglong>(index) } }
                ));
                return report;
            }
            accumulatedLength += std::sqrt(segmentLengthSquared);
        }

        if (path.closed
            && squaredDistance(path.vertices.front().position, path.vertices.back().position)
                <= kLengthTolerance * kLengthTolerance)
        {
            report.status = OperationStatus::Failed;
            report.addDiagnostic(makePathDiagnostic
            (
                path,
                context,
                DiagnosticCode::PathInvariantViolation,
                QStringLiteral("闭合核心路径不得重复保存首点。"),
                QStringLiteral("The last closed-path vertex duplicates the first vertex")
            ));
            return report;
        }

        if (accumulatedLength <= kLengthTolerance)
        {
            report.status = OperationStatus::Failed;
            report.addDiagnostic(makePathDiagnostic
            (
                path,
                context,
                DiagnosticCode::DegenerateGeometry,
                QStringLiteral("路径总长度退化为零。"),
                QStringLiteral("Accumulated path length is zero")
            ));
            return report;
        }

        report.status = OperationStatus::Success;
        report.value = std::monostate{};
        return report;
    }
}
