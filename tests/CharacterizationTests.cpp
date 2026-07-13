#include "CadArcItem.h"
#include "CadCircleItem.h"
#include "CadDocument.h"
#include "CadEllipseItem.h"
#include "CadItem.h"
#include "CadLineItem.h"
#include "CadOcsGeometry.h"
#include "GGenerator.h"
#include "GProfile.h"
#include "application/messaging/MessageCenter.h"
#include "compatibility/legacy/LegacyCadItemPathBridge.h"
#include "core/diagnostics/OperationResult.h"
#include "core/geometry/EntityIdAllocator.h"
#include "core/geometry/GeometryCompiler.h"
#include "infrastructure/dxf/DxfGeometryAdapter.h"
#include "dx_data.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>

namespace
{
    constexpr double kTolerance = 1.0e-6;
    constexpr double kHalfPi = 1.57079632679489661923;
    int failureCount = 0;
    bool updateGoldenFiles = false;

    void check(bool condition, const char* name)
    {
        if (!condition)
        {
            ++failureCount;
            std::cerr << "FAILED: " << name << '\n';
        }
    }

    bool hasDiagnosticCode(const QVector<Diagnostic>& diagnostics, DiagnosticCode code)
    {
        return std::any_of
        (
            diagnostics.cbegin(),
            diagnostics.cend(),
            [code](const Diagnostic& diagnostic)
            {
                return diagnostic.code == code;
            }
        );
    }

    template<typename Entity, typename Item>
    Item* appendItem(CadDocument& document, std::unique_ptr<Entity> entity)
    {
        auto item = std::make_unique<Item>(entity.get());
        Item* rawItem = item.get();
        rawItem->m_entityId = static_cast<cadcam::geometry::EntityId>(document.m_entities.size()) + 1;
        document.m_data->mBlock->ent.push_back(entity.release());
        document.m_entities.push_back(std::move(item));
        return rawItem;
    }

    std::unique_ptr<DRW_Line> makeLine
    (
        double x1,
        double y1,
        double z1,
        double x2,
        double y2,
        double z2
    )
    {
        auto line = std::make_unique<DRW_Line>();
        line->basePoint = DRW_Coord(x1, y1, z1);
        line->secPoint = DRW_Coord(x2, y2, z2);
        return line;
    }

    std::unique_ptr<DRW_Circle> makeCircle()
    {
        auto circle = std::make_unique<DRW_Circle>();
        circle->basePoint = DRW_Coord(20.0, 0.0, 50.0);
        circle->radious = 5.0;
        circle->extPoint = DRW_Coord(0.0, 0.0, 1.0);
        return circle;
    }

    std::unique_ptr<DRW_Ellipse> makeEllipse()
    {
        auto ellipse = std::make_unique<DRW_Ellipse>();
        ellipse->basePoint = DRW_Coord(20.0, 0.0, 50.0);
        ellipse->secPoint = DRW_Coord(8.0, 0.0, 0.0);
        ellipse->ratio = 0.5;
        ellipse->staparam = 0.0;
        ellipse->endparam = 6.28318530717958647692;
        ellipse->extPoint = DRW_Coord(0.0, 0.0, 1.0);
        return ellipse;
    }

    std::unique_ptr<DRW_Arc> makeArc(double startParameter, double endParameter)
    {
        auto arc = std::make_unique<DRW_Arc>();
        arc->basePoint = DRW_Coord(0.0, 0.0, 0.0);
        arc->radious = 10.0;
        arc->staangle = startParameter;
        arc->endangle = endParameter;
        arc->extPoint = DRW_Coord(0.0, 0.0, 1.0);
        return arc;
    }

    OperationContext testContext(const QString& operation)
    {
        return { QStringLiteral("characterization-test"), operation };
    }

    OperationResult<QString> buildProgram
    (
        CadDocument& document,
        GProfile& profile,
        GGenerator::GenerationMode mode
    )
    {
        GGenerator generator;
        generator.setDocument(&document);
        generator.setProfile(&profile);
        generator.setGenerationMode(mode);
        generator.setRotaryTubeCenter(0.0, 0.0, true);
        return generator.buildProgramText(testContext(QStringLiteral("build-program")));
    }

    QString goldenDirectory()
    {
        return QDir(QFileInfo(QString::fromUtf8(__FILE__)).absolutePath()).filePath(QStringLiteral("golden"));
    }

    void verifyGolden(const QString& fileName, const QString& actual)
    {
        const QString filePath = QDir(goldenDirectory()).filePath(fileName);

        if (updateGoldenFiles)
        {
            QDir().mkpath(goldenDirectory());
            QFile output(filePath);
            check(output.open(QIODevice::WriteOnly | QIODevice::Truncate), "open golden output");
            if (output.isOpen())
            {
                output.write(actual.toUtf8());
                output.close();
            }
            return;
        }

        QFile input(filePath);
        check(input.open(QIODevice::ReadOnly), "open golden input");
        if (!input.isOpen())
        {
            return;
        }

        const QString expected = QString::fromUtf8(input.readAll());
        if (actual != expected)
        {
            const QStringList expectedLines = expected.split(QStringLiteral("\r\n"));
            const QStringList actualLines = actual.split(QStringLiteral("\r\n"));
            const int comparedLineCount = std::min(expectedLines.size(), actualLines.size());
            for (int index = 0; index < comparedLineCount; ++index)
            {
                if (expectedLines[index] == actualLines[index])
                {
                    continue;
                }
                std::cerr << "Golden mismatch " << fileName.toStdString()
                    << " line " << (index + 1)
                    << "\n  expected: " << expectedLines[index].toStdString()
                    << "\n  actual:   " << actualLines[index].toStdString() << '\n';
                break;
            }
        }
        check(actual == expected, fileName.toUtf8().constData());
    }

    void testOperationResult()
    {
        OperationResult<int> success;
        success.status = OperationStatus::Success;
        success.value = 42;
        check(success.succeeded(), "OperationResult Success");
        check(!success.hasErrors(), "OperationResult Success has no errors");

        OperationResult<int> partial;
        partial.status = OperationStatus::PartialSuccess;
        partial.addDiagnostic({ DiagnosticCode::InvalidGeometry, DiagnosticSeverity::Warning });
        check(partial.succeeded(), "OperationResult PartialSuccess");

        OperationResult<int> failed;
        failed.status = OperationStatus::Failed;
        failed.addDiagnostic({ DiagnosticCode::InternalInvariantViolation, DiagnosticSeverity::Error });
        check(!failed.succeeded(), "OperationResult Failed");
        check(failed.hasErrors(), "OperationResult error severity");

        success.mergeDiagnostics(failed);
        check(success.diagnostics.size() == 1, "OperationResult diagnostic merge");
    }

    class RecordingSink : public IMessageSink
    {
    public:
        void publish(const Diagnostic& diagnostic) override
        {
            received.push_back(diagnostic);
        }

        QVector<Diagnostic> received;
    };

    void testMessageCenter()
    {
        RecordingSink first;
        RecordingSink second;
        MessageCenter center;
        center.addSink(&first);
        center.addSink(&second);

        OperationReport report;
        report.status = OperationStatus::PartialSuccess;
        Diagnostic diagnostic;
        diagnostic.correlationId = QStringLiteral("same-correlation-id");
        report.addDiagnostic(diagnostic);
        center.publish(report);

        check(first.received.size() == 1 && second.received.size() == 1, "MessageCenter fan-out");
        check(first.received.front().correlationId == second.received.front().correlationId, "MessageCenter correlation id");
        check(report.status == OperationStatus::PartialSuccess, "MessageCenter does not mutate report");
    }

    void testPath3DContract()
    {
        using namespace cadcam::geometry;

        Path3D closed;
        closed.sourceEntityId = 1;
        closed.sourceKind = SourceGeometryKind::Circle;
        closed.closed = true;
        closed.vertices =
        {
            { { 0.0, 0.0, 0.0 }, 0.0 },
            { { 1.0, 0.0, 0.0 }, 1.0 },
            { { 0.0, 1.0, 0.0 }, 2.0 }
        };
        check(validatePath3D(closed, testContext(QStringLiteral("path-contract"))).succeeded(),
            "closed Path3D without duplicate start");

        closed.vertices.push_back(closed.vertices.front());
        const OperationReport repeatedClosure = validatePath3D
            (closed, testContext(QStringLiteral("path-contract")));
        check(!repeatedClosure.succeeded()
            && hasDiagnosticCode(repeatedClosure.diagnostics, DiagnosticCode::PathInvariantViolation),
            "closed Path3D rejects duplicate start");

        Path3D nonFinite;
        nonFinite.sourceEntityId = 2;
        nonFinite.sourceKind = SourceGeometryKind::Line;
        nonFinite.vertices =
        {
            { { 0.0, 0.0, 0.0 }, 0.0 },
            { { std::numeric_limits<double>::infinity(), 0.0, 0.0 }, 1.0 }
        };
        const OperationReport nonFiniteResult = validatePath3D
            (nonFinite, testContext(QStringLiteral("path-contract")));
        check(!nonFiniteResult.succeeded()
            && hasDiagnosticCode(nonFiniteResult.diagnostics, DiagnosticCode::PathInvariantViolation),
            "Path3D rejects non-finite coordinates");

        Path3D repeated;
        repeated.sourceEntityId = 3;
        repeated.sourceKind = SourceGeometryKind::Line;
        repeated.vertices =
        {
            { { 1.0, 2.0, 3.0 }, 0.0 },
            { { 1.0, 2.0, 3.0 }, 1.0 }
        };
        const OperationReport repeatedResult = validatePath3D
            (repeated, testContext(QStringLiteral("path-contract")));
        check(!repeatedResult.succeeded()
            && hasDiagnosticCode(repeatedResult.diagnostics, DiagnosticCode::PathInvariantViolation),
            "Path3D detects adjacent duplicate vertices");
    }

    void testGeometryCompilerLineAndCircle()
    {
        using namespace cadcam::geometry;

        GeometryCompiler compiler;
        SamplingPolicy linePolicy;
        SourceEntity lineSource;
        lineSource.id = 10;
        lineSource.kind = SourceGeometryKind::Line;
        lineSource.geometry = LineGeometry{ { 1.0, 2.0, 3.0 }, { 4.0, 5.0, 6.0 } };

        OperationResult<Path3D> lineForward = compiler.compile
        (
            lineSource,
            linePolicy,
            {},
            testContext(QStringLiteral("compile-line"))
        );
        check(lineForward.succeeded() && lineForward.value->vertices.size() == 2
            && lineForward.value->vertices.front().position.x == 1.0
            && lineForward.value->vertices.back().position.x == 4.0,
            "GeometryCompiler line forward");

        PathCompileOptions reverse;
        reverse.reverse = true;
        OperationResult<Path3D> lineReverse = compiler.compile
        (
            lineSource,
            linePolicy,
            reverse,
            testContext(QStringLiteral("compile-line"))
        );
        check(lineReverse.succeeded()
            && lineReverse.value->vertices.front().position.x == 4.0
            && lineReverse.value->vertices.back().position.x == 1.0,
            "GeometryCompiler line reverse");

        lineSource.geometry = LineGeometry{ { 1.0, 2.0, 3.0 }, { 1.0, 2.0, 3.0 } };
        const OperationResult<Path3D> degenerate = compiler.compile
        (
            lineSource,
            linePolicy,
            {},
            testContext(QStringLiteral("compile-line"))
        );
        check(!degenerate.succeeded()
            && hasDiagnosticCode(degenerate.diagnostics, DiagnosticCode::DegenerateGeometry),
            "GeometryCompiler zero-length line diagnostic");

        SamplingPolicy invalidPolicy;
        invalidPolicy.minimumSegments = 0;
        const OperationResult<Path3D> invalidPolicyResult = compiler.compile
        (
            lineSource,
            invalidPolicy,
            {},
            testContext(QStringLiteral("compile-invalid-policy"))
        );
        check(!invalidPolicyResult.succeeded()
            && hasDiagnosticCode
            (invalidPolicyResult.diagnostics, DiagnosticCode::InvalidSamplingPolicy),
            "GeometryCompiler invalid sampling policy diagnostic");

        SourceEntity circleSource;
        circleSource.id = 11;
        circleSource.kind = SourceGeometryKind::Circle;
        circleSource.geometry = CircleGeometry
        {
            { 0.0, 0.0, 0.0 },
            { 1.0, 0.0, 0.0 },
            { 0.0, 1.0, 0.0 },
            5.0
        };
        SamplingPolicy circlePolicy;
        circlePolicy.chordTolerance = 0.0;
        circlePolicy.minimumSegments = 128;
        circlePolicy.fullTurnSegments = 128;
        const OperationResult<Path3D> circleForward = compiler.compile
        (
            circleSource,
            circlePolicy,
            {},
            testContext(QStringLiteral("compile-circle"))
        );
        const OperationResult<Path3D> circleReverse = compiler.compile
        (
            circleSource,
            circlePolicy,
            reverse,
            testContext(QStringLiteral("compile-circle"))
        );
        check(circleForward.succeeded() && circleForward.value->closed
            && circleForward.value->vertices.size() == 128,
            "GeometryCompiler closed circle vertex count");
        check(std::abs(circleForward.value->vertices.front().position.x) <= kTolerance
            && std::abs(circleForward.value->vertices.front().position.y - 5.0) <= kTolerance,
            "GeometryCompiler circle north start");
        check(std::abs(circleForward.value->vertices.front().position.x
                - circleReverse.value->vertices.front().position.x) <= kTolerance
            && std::abs(circleForward.value->vertices.front().position.y
                - circleReverse.value->vertices.front().position.y) <= kTolerance,
            "GeometryCompiler circle same reverse start");
        check(circleForward.value->vertices[1].position.x < 0.0
            && circleReverse.value->vertices[1].position.x > 0.0,
            "GeometryCompiler circle reverse traversal");
        check(std::abs(circleForward.value->vertices.front().position.x
                - circleForward.value->vertices.back().position.x) > 1.0e-3
            || std::abs(circleForward.value->vertices.front().position.y
                - circleForward.value->vertices.back().position.y) > 1.0e-3,
            "GeometryCompiler circle omits duplicate closure");
    }

    void testGeometryCompilerArcAndEllipse()
    {
        using namespace cadcam::geometry;

        GeometryCompiler compiler;
        const double degrees = 3.14159265358979323846 / 180.0;
        SourceEntity arcSource;
        arcSource.id = 20;
        arcSource.kind = SourceGeometryKind::Arc;
        arcSource.geometry = ArcGeometry
        {
            { 0.0, 0.0, 0.0 },
            { 1.0, 0.0, 0.0 },
            { 0.0, 1.0, 0.0 },
            10.0,
            0.0,
            90.0 * degrees
        };
        SamplingPolicy arcPolicy;
        arcPolicy.chordTolerance = 0.0;
        arcPolicy.minimumSegments = 8;
        arcPolicy.maximumAngularStep = 5.0 * degrees;
        const OperationResult<Path3D> ordinaryArc = compiler.compile
        (
            arcSource,
            arcPolicy,
            {},
            testContext(QStringLiteral("compile-ordinary-arc"))
        );
        check(ordinaryArc.succeeded() && ordinaryArc.value->vertices.size() == 19,
            "GeometryCompiler ordinary arc");

        std::get<ArcGeometry>(arcSource.geometry).startParameter = 350.0 * degrees;
        std::get<ArcGeometry>(arcSource.geometry).endParameter = 370.0 * degrees;
        OperationResult<Path3D> arcForward = compiler.compile
        (
            arcSource,
            arcPolicy,
            {},
            testContext(QStringLiteral("compile-arc"))
        );
        PathCompileOptions reverse;
        reverse.reverse = true;
        OperationResult<Path3D> arcReverse = compiler.compile
        (
            arcSource,
            arcPolicy,
            reverse,
            testContext(QStringLiteral("compile-arc"))
        );
        check(arcForward.succeeded() && !arcForward.value->closed
            && arcForward.value->vertices.size() == 9,
            "GeometryCompiler zero-crossing arc");
        check(arcReverse.succeeded()
            && std::abs(arcForward.value->vertices.front().position.x
                - arcReverse.value->vertices.back().position.x) <= kTolerance
            && std::abs(arcForward.value->vertices.back().position.x
                - arcReverse.value->vertices.front().position.x) <= kTolerance,
            "GeometryCompiler arc reverse endpoints");

        std::get<ArcGeometry>(arcSource.geometry).endParameter = 350.0 * degrees + 1.0e-6;
        const OperationResult<Path3D> shortArc = compiler.compile
        (
            arcSource,
            arcPolicy,
            {},
            testContext(QStringLiteral("compile-short-arc"))
        );
        check(shortArc.succeeded() && shortArc.value->vertices.size() == 9,
            "GeometryCompiler short arc minimum segments");

        SourceEntity ellipseSource;
        ellipseSource.id = 21;
        ellipseSource.kind = SourceGeometryKind::Ellipse;
        ellipseSource.geometry = EllipseGeometry
        {
            { 3.0, 4.0, 5.0 },
            { 6.0, 8.0, 0.0 },
            { -4.0, 3.0, 0.0 },
            0.0,
            6.28318530717958647692,
            true
        };
        SamplingPolicy ellipsePolicy;
        ellipsePolicy.chordTolerance = 0.0;
        ellipsePolicy.minimumSegments = 16;
        ellipsePolicy.fullTurnSegments = 128;
        const OperationResult<Path3D> ellipseForward = compiler.compile
        (
            ellipseSource,
            ellipsePolicy,
            {},
            testContext(QStringLiteral("compile-ellipse"))
        );
        const OperationResult<Path3D> ellipseReverse = compiler.compile
        (
            ellipseSource,
            ellipsePolicy,
            reverse,
            testContext(QStringLiteral("compile-ellipse"))
        );
        check(ellipseForward.succeeded() && ellipseForward.value->closed
            && ellipseForward.value->vertices.size() == 128,
            "GeometryCompiler rotated full ellipse");
        check(std::abs(ellipseForward.value->vertices.front().position.x + 1.0) <= kTolerance
            && std::abs(ellipseForward.value->vertices.front().position.y - 7.0) <= kTolerance,
            "GeometryCompiler ellipse north parameter");
        check(std::abs(ellipseForward.value->vertices.front().position.x
                - ellipseReverse.value->vertices.front().position.x) <= kTolerance
            && std::abs(ellipseForward.value->vertices.front().position.y
                - ellipseReverse.value->vertices.front().position.y) <= kTolerance,
            "GeometryCompiler ellipse same reverse start");

        EllipseGeometry partial = std::get<EllipseGeometry>(ellipseSource.geometry);
        partial.fullEllipse = false;
        partial.startParameter = 0.0;
        partial.endParameter = 1.57079632679489661923;
        ellipseSource.geometry = partial;
        const OperationResult<Path3D> partialForward = compiler.compile
        (
            ellipseSource,
            ellipsePolicy,
            {},
            testContext(QStringLiteral("compile-partial-ellipse"))
        );
        const OperationResult<Path3D> partialReverse = compiler.compile
        (
            ellipseSource,
            ellipsePolicy,
            reverse,
            testContext(QStringLiteral("compile-partial-ellipse"))
        );
        check(partialForward.succeeded() && !partialForward.value->closed
            && partialForward.value->vertices.size() == 33,
            "GeometryCompiler partial ellipse point count");
        check(std::abs(partialForward.value->vertices.front().position.x
                - partialReverse.value->vertices.back().position.x) <= kTolerance
            && std::abs(partialForward.value->vertices.back().position.y
                - partialReverse.value->vertices.front().position.y) <= kTolerance,
            "GeometryCompiler partial ellipse reverse endpoints");
    }

    void testDxfAdapterAndLegacyBridge()
    {
        using namespace cadcam::geometry;

        CadDocument document;
        CadCircleItem* circle = appendItem<DRW_Circle, CadCircleItem>(document, makeCircle());
        const OperationResult<SourceEntity> adaptedCircle = DxfGeometryAdapter::convert
        (
            circle->m_entityId,
            *circle->m_nativeEntity,
            testContext(QStringLiteral("adapt-circle"))
        );
        check(adaptedCircle.succeeded()
            && adaptedCircle.value->kind == SourceGeometryKind::Circle,
            "DxfGeometryAdapter circle");

        PathCompileOptions options;
        const OperationResult<Path3D> coreCircle = LegacyCadItemPathBridge::compile
        (
            *circle,
            LegacyCadItemPathBridge::legacySamplingPolicy(*circle),
            options,
            testContext(QStringLiteral("legacy-circle"))
        );
        std::vector<RawPathPoint3D> legacyPath;
        LegacyCadItemPathBridge::copyToLegacyRawPath(*coreCircle.value, legacyPath);
        check(coreCircle.value->vertices.size() == 128 && legacyPath.size() == 129,
            "Legacy bridge restores circle closure point");
        check(legacyPath.front().x == legacyPath.back().x
            && legacyPath.front().y == legacyPath.back().y
            && legacyPath.front().z == legacyPath.back().z,
            "Legacy bridge closure is exact copy");

        const double degrees = 3.14159265358979323846 / 180.0;
        CadArcItem* arc = appendItem<DRW_Arc, CadArcItem>
            (document, makeArc(350.0 * degrees, 10.0 * degrees));
        arc->rebuildRawPathPoints3D();
        check(arc->rawPathPoints3D().size() == 9,
            "Legacy arc keeps old point count");
        const double expectedStartX = 10.0 * std::cos(350.0 * degrees);
        const double expectedEndX = 10.0 * std::cos(10.0 * degrees);
        check(std::abs(arc->rawPathPoints3D().front().x - expectedStartX) <= kTolerance
            && std::abs(arc->rawPathPoints3D().back().x - expectedEndX) <= kTolerance,
            "Legacy arc keeps exact endpoints");

        auto invalidEllipse = makeEllipse();
        invalidEllipse->ratio = 0.0;
        const OperationResult<SourceEntity> invalidEllipseResult = DxfGeometryAdapter::convert
        (
            99,
            *invalidEllipse,
            testContext(QStringLiteral("adapt-invalid-ellipse"))
        );
        check(!invalidEllipseResult.succeeded()
            && hasDiagnosticCode
            (invalidEllipseResult.diagnostics, DiagnosticCode::GeometryAdapterFailure),
            "DxfGeometryAdapter rejects invalid ellipse ratio");

        invalidEllipse->ratio = 0.5;
        invalidEllipse->secPoint = DRW_Coord(0.0, 0.0, 0.0);
        const OperationResult<SourceEntity> invalidAxisResult = DxfGeometryAdapter::convert
        (
            100,
            *invalidEllipse,
            testContext(QStringLiteral("adapt-invalid-ellipse-axis"))
        );
        check(!invalidAxisResult.succeeded()
            && hasDiagnosticCode
            (invalidAxisResult.diagnostics, DiagnosticCode::GeometryAdapterFailure),
            "DxfGeometryAdapter rejects degenerate ellipse axis");

        DRW_Point point;
        const OperationResult<SourceEntity> unsupported = DxfGeometryAdapter::convert
        (
            101,
            point,
            testContext(QStringLiteral("adapt-unsupported"))
        );
        check(unsupported.status == OperationStatus::NotSupported
            && hasDiagnosticCode(unsupported.diagnostics, DiagnosticCode::UnsupportedGeometry),
            "DxfGeometryAdapter unsupported geometry diagnostic");
    }

    void testEntityIdAllocator()
    {
        using namespace cadcam::geometry;

        EntityIdAllocator allocator;
        const EntityId first = allocator.ensure(0);
        const EntityId second = allocator.ensure(0);
        check(first != 0 && second != 0 && first != second, "EntityId uniqueness");

        const EntityId removedId = first;
        const EntityId restoredId = allocator.ensure(removedId);
        check(restoredId == removedId, "EntityId delete/undo stability");

        const EntityId copyId = allocator.ensure(0);
        check(copyId != restoredId && copyId != second, "EntityId copy gets new id");

        const EntityId highRestoredId = 5000;
        check(allocator.ensure(highRestoredId) == highRestoredId
            && allocator.ensure(0) > highRestoredId,
            "EntityId reinsertion advances allocator");

        auto originalEntity = makeLine(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
        CadLineItem originalItem(originalEntity.get());
        originalItem.m_entityId = allocator.ensure(0);
        const EntityId originalItemId = originalItem.m_entityId;
        const EntityId undoRestoredItemId = allocator.ensure(originalItem.m_entityId);
        auto copiedEntity = makeLine(0.0, 1.0, 0.0, 1.0, 1.0, 0.0);
        CadLineItem copiedItem(copiedEntity.get());
        copiedItem.m_entityId = allocator.ensure(copiedItem.m_entityId);
        check(undoRestoredItemId == originalItemId, "CadItem EntityId survives undo reinsertion");
        check(copiedItem.m_entityId != originalItemId, "copied CadItem receives a new EntityId");
    }

    void testCircleAndEllipseNorthStart()
    {
        CadDocument document;
        CadCircleItem* circle = appendItem<DRW_Circle, CadCircleItem>(document, makeCircle());
        circle->m_isReverse = false;
        circle->rebuildRawPathPoints3D();
        const RawPathPoint3D circleForward = circle->rawPathPoints3D().front();
        circle->m_isReverse = true;
        circle->rebuildRawPathPoints3D();
        const RawPathPoint3D circleReverse = circle->rawPathPoints3D().front();
        check(std::abs(circle->defaultProcessStartParameter() - kHalfPi) <= kTolerance, "circle M_PI_2 start parameter");
        check(std::abs(circleForward.x - circleReverse.x) <= kTolerance
            && std::abs(circleForward.y - circleReverse.y) <= kTolerance
            && std::abs(circleForward.z - circleReverse.z) <= kTolerance,
            "circle forward/reverse same start");

        CadEllipseItem* ellipse = appendItem<DRW_Ellipse, CadEllipseItem>(document, makeEllipse());
        ellipse->m_isReverse = false;
        ellipse->rebuildRawPathPoints3D();
        const RawPathPoint3D ellipseForward = ellipse->rawPathPoints3D().front();
        ellipse->m_isReverse = true;
        ellipse->rebuildRawPathPoints3D();
        const RawPathPoint3D ellipseReverse = ellipse->rawPathPoints3D().front();
        check(std::abs(ellipse->defaultProcessStartParameter() - kHalfPi) <= kTolerance, "ellipse M_PI_2 start parameter");
        check(std::abs(ellipseForward.x - ellipseReverse.x) <= kTolerance
            && std::abs(ellipseForward.y - ellipseReverse.y) <= kTolerance
            && std::abs(ellipseForward.z - ellipseReverse.z) <= kTolerance,
            "ellipse forward/reverse same start");
    }

    void testSimpleLineAndMCodeOptimization()
    {
        CadDocument connectedDocument;
        CadLineItem* first = appendItem<DRW_Line, CadLineItem>
            (connectedDocument, makeLine(0.0, 0.0, 0.0, 10.0, 0.0, 0.0));
        CadLineItem* second = appendItem<DRW_Line, CadLineItem>
            (connectedDocument, makeLine(10.0, 0.0, 0.0, 20.0, 0.0, 0.0));
        first->m_processOrder = 0;
        second->m_processOrder = 1;
        GProfile profile = GProfile::createDefaultLaserProfile();
        const OperationResult<QString> connected = buildProgram
            (connectedDocument, profile, GGenerator::GenerationMode::Mode2D);
        check(connected.succeeded() && connected.value.has_value(), "connected line program builds");
        if (!connected.value.has_value())
        {
            return;
        }

        const QString& connectedProgram = *connected.value;
        check(!connectedProgram.contains(QStringLiteral("\r\r\n")), "single CRLF");
        check(!connectedProgram.contains(QStringLiteral("\r\n\r\n")), "no blank lines");
        check(!connectedProgram.contains(QStringLiteral("M05\r\nM03")), "adjacent M05/M03 removed");
        check(connectedProgram.lastIndexOf(QStringLiteral("M05"))
            > connectedProgram.lastIndexOf(QStringLiteral("G01")), "final M05 retained");

        CadDocument disconnectedDocument;
        CadLineItem* disconnectedFirst = appendItem<DRW_Line, CadLineItem>
            (disconnectedDocument, makeLine(0.0, 0.0, 0.0, 10.0, 0.0, 0.0));
        CadLineItem* disconnectedSecond = appendItem<DRW_Line, CadLineItem>
            (disconnectedDocument, makeLine(100.0, 0.0, 0.0, 110.0, 0.0, 0.0));
        disconnectedFirst->m_processOrder = 0;
        disconnectedSecond->m_processOrder = 1;
        const OperationResult<QString> disconnected = buildProgram
            (disconnectedDocument, profile, GGenerator::GenerationMode::Mode2D);
        check(disconnected.succeeded() && disconnected.value.has_value(), "disconnected line program builds");
        if (disconnected.value.has_value())
        {
            const QString& program = *disconnected.value;
            const int firstStop = program.indexOf(QStringLiteral("M05"));
            const int nextRapid = program.indexOf(QStringLiteral("G00"), firstStop + 1);
            const int nextStart = program.indexOf(QStringLiteral("M03"), nextRapid + 1);
            check(firstStop >= 0 && nextRapid > firstStop && nextStart > nextRapid, "M05/G00/M03 retained");
        }

        CadDocument goldenDocument;
        CadLineItem* goldenLine = appendItem<DRW_Line, CadLineItem>
            (goldenDocument, makeLine(0.0, 0.0, 0.0, 25.0, 10.0, 0.0));
        goldenLine->m_processOrder = 0;
        const OperationResult<QString> golden = buildProgram
            (goldenDocument, profile, GGenerator::GenerationMode::Mode2D);
        if (golden.value.has_value())
        {
            verifyGolden(QStringLiteral("simple_line_3axis.nc"), *golden.value);
        }
    }

    void testRotaryGoldenPrograms()
    {
        GProfile rotaryProfile = GProfile::createDefaultRotaryProfile();

        CadDocument circleDocument;
        CadCircleItem* circle = appendItem<DRW_Circle, CadCircleItem>(circleDocument, makeCircle());
        circle->m_processOrder = 0;
        circle->m_processContinuousGroupId = 0;
        const OperationResult<QString> circleProgram = buildProgram
            (circleDocument, rotaryProfile, GGenerator::GenerationMode::Mode3D);
        check(circleProgram.succeeded() && circleProgram.value.has_value(), "circle rotary program builds");
        if (circleProgram.value.has_value())
        {
            verifyGolden(QStringLiteral("closed_circle_4axis.nc"), *circleProgram.value);
        }

        CadDocument ellipseDocument;
        CadEllipseItem* ellipse = appendItem<DRW_Ellipse, CadEllipseItem>(ellipseDocument, makeEllipse());
        ellipse->m_processOrder = 0;
        ellipse->m_processContinuousGroupId = 0;
        const OperationResult<QString> ellipseProgram = buildProgram
            (ellipseDocument, rotaryProfile, GGenerator::GenerationMode::Mode3D);
        check(ellipseProgram.succeeded() && ellipseProgram.value.has_value(), "ellipse rotary program builds");
        if (ellipseProgram.value.has_value())
        {
            verifyGolden(QStringLiteral("closed_ellipse_4axis.nc"), *ellipseProgram.value);
        }

        CadDocument groupDocument;
        const double points[4][3] =
        {
            { 0.0, -10.0, 50.0 },
            { 20.0, -10.0, 50.0 },
            { 20.0, 10.0, 50.0 },
            { 0.0, 10.0, 50.0 }
        };
        for (int index = 0; index < 4; ++index)
        {
            const int next = (index + 1) % 4;
            CadLineItem* line = appendItem<DRW_Line, CadLineItem>
            (
                groupDocument,
                makeLine
                (
                    points[index][0], points[index][1], points[index][2],
                    points[next][0], points[next][1], points[next][2]
                )
            );
            line->m_processOrder = index;
            line->m_processContinuousGroupId = 0;
        }

        const OperationResult<QString> groupProgram = buildProgram
            (groupDocument, rotaryProfile, GGenerator::GenerationMode::Mode3D);
        check(groupProgram.succeeded() && groupProgram.value.has_value(), "continuous group program builds");
        if (groupProgram.value.has_value())
        {
            const QString& program = *groupProgram.value;
            const int laserStart = program.indexOf(QStringLiteral("M03"));
            const int laserStop = program.indexOf(QStringLiteral("M05"), laserStart);
            check(laserStart >= 0 && laserStop > laserStart, "continuous group laser section");
            if (laserStart >= 0 && laserStop > laserStart)
            {
                const QString cuttingSection = program.mid(laserStart, laserStop - laserStart);
                check(!cuttingSection.contains(QStringLiteral("G00")), "continuous group has no safety lift");
                check(!cuttingSection.contains(QStringLiteral("M05"))
                    && cuttingSection.count(QStringLiteral("M03")) == 1,
                    "overcut has no laser restart");
                check(program.lastIndexOf(QStringLiteral("G01"), laserStop) < laserStop,
                    "overcut before final M05");
            }
            verifyGolden(QStringLiteral("continuous_group_overcut.nc"), program);
        }
    }

    void testFailures()
    {
        GProfile profile = GProfile::createDefaultRotaryProfile();
        CadDocument emptyDocument;
        const OperationResult<QString> empty = buildProgram
            (emptyDocument, profile, GGenerator::GenerationMode::Mode3D);
        check(empty.status == OperationStatus::InvalidInput, "empty document InvalidInput");
        check(hasDiagnosticCode(empty.diagnostics, DiagnosticCode::InvalidGeometry), "empty document diagnostic code");

        CadDocument invalidPathDocument;
        auto invalidCircle = makeCircle();
        invalidCircle->radious = 0.0;
        CadCircleItem* circle = appendItem<DRW_Circle, CadCircleItem>
            (invalidPathDocument, std::move(invalidCircle));
        circle->m_processOrder = 0;
        circle->m_processContinuousGroupId = 0;
        const OperationResult<QString> invalidPath = buildProgram
            (invalidPathDocument, profile, GGenerator::GenerationMode::Mode3D);
        check(!invalidPath.succeeded(), "empty path fails");
        check(hasDiagnosticCode(invalidPath.diagnostics, DiagnosticCode::EmptyPath), "empty path diagnostic code");

        GGenerator generator;
        const OperationReport writeFailure = generator.writeProgramText
        (
            QDir::tempPath(),
            QStringLiteral("M05\r\n"),
            testContext(QStringLiteral("write-program"))
        );
        check(!writeFailure.succeeded(), "invalid output path fails");
        check(hasDiagnosticCode(writeFailure.diagnostics, DiagnosticCode::FileOpenFailure), "file open diagnostic code");
    }
}

int main(int argc, char* argv[])
{
    for (int index = 1; index < argc; ++index)
    {
        updateGoldenFiles = QString::fromLocal8Bit(argv[index]) == QStringLiteral("--update-golden");
    }

    testOperationResult();
    testMessageCenter();
    testPath3DContract();
    testGeometryCompilerLineAndCircle();
    testGeometryCompilerArcAndEllipse();
    testDxfAdapterAndLegacyBridge();
    testEntityIdAllocator();
    testCircleAndEllipseNorthStart();
    testSimpleLineAndMCodeOptimization();
    testRotaryGoldenPrograms();
    testFailures();

    if (failureCount == 0)
    {
        std::cout << "All characterization tests passed.\n";
    }
    return failureCount == 0 ? 0 : 1;
}
