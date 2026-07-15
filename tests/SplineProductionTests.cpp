#include "SplineProductionTests.h"

#include "cad/document/CadDocument.h"
#include "cad/items/CadPolylineItem.h"
#include "cad/items/CadSplineItem.h"
#include "application/export/GGenerator.h"
#include "infrastructure/config/GProfile.h"
#include "SplineParity.h"
#include "core/machine/RotaryKinematics.h"
#include "application/process/DocumentProcessState.h"
#include "compatibility/legacy/SplineEntityClone.h"
#include "compatibility/legacy/SplineProductionPathProvider.h"
#include "core/geometry/GeometryCompiler.h"
#include "infrastructure/dxf/legacy/dx_data.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTemporaryDir>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>

namespace
{
    int failures = 0;
    bool updateGoldens = false;

    void check(bool condition, const char* name)
    {
        if (!condition)
        {
            ++failures;
            std::cerr << "FAILED: " << name << '\n';
        }
    }

    OperationContext context(const QString& operation)
    {
        return { QStringLiteral("spline-production-test"), operation };
    }

    std::unique_ptr<DRW_Spline> makeProductionSpline()
    {
        auto spline = std::make_unique<DRW_Spline>();
        spline->degree = 3;
        spline->flags = 4;
        spline->layer = "0";
        spline->color = 1;
        spline->knotslist = { 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0 };
        spline->weightlist = { 1.0, 0.8, 0.8, 1.0 };
        spline->controllist =
        {
            std::make_shared<DRW_Coord>(0.0, -10.0, 50.0),
            std::make_shared<DRW_Coord>(10.0, -5.0, 55.0),
            std::make_shared<DRW_Coord>(20.0, 5.0, 55.0),
            std::make_shared<DRW_Coord>(30.0, 10.0, 50.0)
        };
        spline->fitlist =
        {
            std::make_shared<DRW_Coord>(0.0, -10.0, 50.0),
            std::make_shared<DRW_Coord>(30.0, 10.0, 50.0)
        };
        spline->ncontrol = static_cast<dint32>(spline->controllist.size());
        spline->nknots = static_cast<dint32>(spline->knotslist.size());
        spline->nfit = static_cast<dint32>(spline->fitlist.size());
        return spline;
    }

    std::unique_ptr<DRW_Spline> makeClosedSpline()
    {
        auto spline = std::make_unique<DRW_Spline>();
        spline->degree = 2;
        spline->flags = 1;
        spline->layer = "0";
        spline->knotslist = { 0.0, 0.0, 0.0, 0.33, 0.66, 1.0, 1.0, 1.0 };
        spline->controllist =
        {
            std::make_shared<DRW_Coord>(0.0, 0.0, 50.0),
            std::make_shared<DRW_Coord>(5.0, 8.0, 50.0),
            std::make_shared<DRW_Coord>(10.0, 0.0, 50.0),
            std::make_shared<DRW_Coord>(5.0, -8.0, 50.0),
            std::make_shared<DRW_Coord>(0.0, 0.0, 50.0)
        };
        spline->ncontrol = static_cast<dint32>(spline->controllist.size());
        spline->nknots = static_cast<dint32>(spline->knotslist.size());
        return spline;
    }

    std::unique_ptr<DRW_Spline> makeFitOnlySpline()
    {
        auto spline = std::make_unique<DRW_Spline>();
        spline->degree = 0;
        spline->layer = "0";
        spline->fitlist =
        {
            std::make_shared<DRW_Coord>(0.0, 0.0, 50.0),
            std::make_shared<DRW_Coord>(5.0, 4.0, 52.0),
            std::make_shared<DRW_Coord>(10.0, 0.0, 50.0)
        };
        spline->nfit = static_cast<dint32>(spline->fitlist.size());
        return spline;
    }

    QString goldenDirectory()
    {
        return QDir(QFileInfo(QString::fromUtf8(__FILE__)).absolutePath())
            .filePath(QStringLiteral("golden"));
    }

    void verifyGolden(const QString& fileName, const QByteArray& actual)
    {
        const QString path = QDir(goldenDirectory()).filePath(fileName);
        if (updateGoldens)
        {
            QDir().mkpath(goldenDirectory());
            QFile output(path);
            check(output.open(QIODevice::WriteOnly | QIODevice::Truncate),
                "open spline production golden output");
            if (output.isOpen())
            {
                output.write(actual);
            }
            return;
        }
        QFile input(path);
        check(input.open(QIODevice::ReadOnly), "open spline production golden input");
        if (input.isOpen())
        {
            const QByteArray expected = input.readAll();
            if (expected != actual)
            {
                const QList<QByteArray> expectedLines = expected.split('\n');
                const QList<QByteArray> actualLines = actual.split('\n');
                const int count = std::min(expectedLines.size(), actualLines.size());
                for (int index = 0; index < count; ++index)
                {
                    if (expectedLines[index] == actualLines[index]) continue;
                    std::cerr << "Golden mismatch " << fileName.toStdString()
                        << " line " << index + 1 << "\n  expected: "
                        << expectedLines[index].constData() << "\n  actual:   "
                        << actualLines[index].constData() << '\n';
                    break;
                }
            }
            check(expected == actual, fileName.toUtf8().constData());
        }
    }

    OperationResult<QString> buildProgram
    (
        CadDocument& document,
        GGenerator::GenerationMode mode
    )
    {
        GProfile profile = mode == GGenerator::GenerationMode::Mode3D
            ? GProfile::createDefaultRotaryProfile()
            : GProfile::createDefaultLaserProfile();
        GGenerator generator;
        std::optional<cadcam::planning::ProcessPlan> plan;
        cadcam::process::DocumentProcessState processState;
        std::optional<cadcam::machining::TubeSectionModel> section;
        if (!document.m_entities.empty())
        {
            plan.emplace();
            plan->contentRevision = document.contentRevision();
            plan->processStateRevision = processState.revision();
            plan->mode = mode == GGenerator::GenerationMode::Mode3D
                ? cadcam::planning::ProcessPlanMode::Rotary4Axis
                : cadcam::planning::ProcessPlanMode::Planar3Axis;
            int processOrder = 0;
            for (const auto& item : document.m_entities)
            {
                if (item == nullptr) continue;
                plan->assignments.push_back
                    ({ item->m_entityId, processOrder++, -1, false, std::nullopt });
            }
        }
        if (mode == GGenerator::GenerationMode::Mode3D)
        {
            section.emplace();
            section->contentRevision = document.contentRevision();
            section->geometry.centerY = 0.0;
            section->geometry.centerZ = 0.0;
            section->geometry.boundary =
            {
                { -10.0, 50.0 }, { 10.0, 50.0 },
                { 10.0, 53.529412 }, { -10.0, 53.529412 }
            };
            section->corners =
            {
                { { 0.0, 50.0 }, 0.0, 1, 1 },
                { { 0.0, 50.0 }, 0.0, -1, 1 }
            };
        }
        generator.setDocument(&document);
        generator.setProfile(&profile);
        generator.setGenerationMode(mode);
        generator.setProcessState(&processState);
        generator.setProcessPlan(plan.has_value() ? &*plan : nullptr);
        generator.setTubeSectionModel(section);
        generator.setRotaryTubeCenter(0.0, 0.0, true);
        return generator.buildProgramText(context(QStringLiteral("build-spline-program")));
    }

    bool samePoint(const RawPathPoint3D& left, const RawPathPoint3D& right)
    {
        return left.x == right.x && left.y == right.y && left.z == right.z;
    }

    void testCadSplineItemAndProvider()
    {
        std::unique_ptr<DRW_Spline> source = makeProductionSpline();
        const int originalFlags = source->flags;
        const std::vector<double> originalKnots = source->knotslist;
        const std::vector<double> originalWeights = source->weightlist;

        CadDocument document;
        CadItem* appended = document.appendEntity(std::move(source));
        auto* item = dynamic_cast<CadSplineItem*>(appended);
        check(item != nullptr && item->m_type == DRW::ETYPE::SPLINE,
            "CadDocument creates CadSplineItem");
        check(item != nullptr && item->m_entityId != 0 && !item->m_geometry.vertices.isEmpty(),
            "CadSplineItem builds display path before and after id assignment");
        check(item != nullptr && item->m_data->flags == originalFlags
            && item->m_data->knotslist == originalKnots
            && item->m_data->weightlist == originalWeights,
            "CadSplineItem preserves flags knots and weights");

        cadcam::geometry::PathCompileOptions forwardOptions;
        cadcam::geometry::PathCompileOptions reverseOptions;
        reverseOptions.reverse = true;
        const auto forward = SplineProductionPathProvider::build
            (item->m_entityId, *item->m_data, {}, forwardOptions,
                context(QStringLiteral("spline-forward")));
        const auto reverse = SplineProductionPathProvider::build
            (item->m_entityId, *item->m_data, {}, reverseOptions,
                context(QStringLiteral("spline-reverse")));
        check(forward.value.has_value() && reverse.value.has_value()
            && forward.value->vertices.size() == reverse.value->vertices.size()
            && !forward.value->vertices.empty()
            && std::abs(forward.value->vertices.front().position.x - reverse.value->vertices.back().position.x) <= 1.0e-9
            && std::abs(forward.value->vertices.front().position.y - reverse.value->vertices.back().position.y) <= 1.0e-9
            && std::abs(forward.value->vertices.front().position.z - reverse.value->vertices.back().position.z) <= 1.0e-9
            && std::abs(forward.value->vertices.back().position.x - reverse.value->vertices.front().position.x) <= 1.0e-9
            && std::abs(forward.value->vertices.back().position.y - reverse.value->vertices.front().position.y) <= 1.0e-9
            && std::abs(forward.value->vertices.back().position.z - reverse.value->vertices.front().position.z) <= 1.0e-9,
            "CadSplineItem supports forward and reverse production paths");

        std::unique_ptr<DRW_Spline> closedEntity = makeClosedSpline();
        CadSplineItem closedItem(closedEntity.get());
        closedItem.m_entityId = 42;
        closedItem.rebuildRawPathPoints3D();
        check(closedItem.rawPathPoints3D().size() > 2U
            && samePoint(closedItem.rawPathPoints3D().front(),
                closedItem.rawPathPoints3D().back()),
            "Closed CadSplineItem legacy cache appends start point");

        std::unique_ptr<DRW_Spline> periodicEntity = makeClosedSpline();
        periodicEntity->flags = 3;
        CadSplineItem periodicItem(periodicEntity.get());
        periodicItem.m_entityId = 44;
        periodicItem.rebuildRawPathPoints3D();
        check(periodicItem.rawPathPoints3D().size() > 2U
            && samePoint(periodicItem.rawPathPoints3D().front(),
                periodicItem.rawPathPoints3D().back()),
            "Periodic CadSplineItem uses closed production path");

        const std::unique_ptr<DRW_Spline> fitOnly = makeFitOnlySpline();
        const OperationResult<cadcam::geometry::Path3D> fallback =
            SplineProductionPathProvider::build
            (
                43, *fitOnly, cadcam::geometry::SamplingPolicy{}, {},
                context(QStringLiteral("fit-only-production-path"))
            );
        const bool warned = std::any_of
        (
            fallback.diagnostics.cbegin(), fallback.diagnostics.cend(),
            [](const Diagnostic& diagnostic)
            {
                return diagnostic.code == DiagnosticCode::SplineFitPointFallbackUsed;
            }
        );
        check(fallback.status == OperationStatus::PartialSuccess && warned,
            "Fit-point fallback is explicit PartialSuccess");

        cadcam::geometry::SamplingPolicy limitedPolicy;
        limitedPolicy.spline.maximumPoints = 2;
        const std::unique_ptr<DRW_Spline> fallbackSource = makeProductionSpline();
        const OperationResult<cadcam::geometry::Path3D> legacyFallback =
            SplineProductionPathProvider::build
            (
                45, *fallbackSource, limitedPolicy, {},
                context(QStringLiteral("forced-legacy-production-fallback"))
            );
        const bool legacyFallbackWarned = std::any_of
        (
            legacyFallback.diagnostics.cbegin(), legacyFallback.diagnostics.cend(),
            [](const Diagnostic& diagnostic)
            {
                return diagnostic.code == DiagnosticCode::SplineLegacyFallbackUsed;
            }
        );
        check(legacyFallback.status == OperationStatus::PartialSuccess
            && legacyFallback.value.has_value() && legacyFallbackWarned,
            "New kernel failure uses explicit temporary legacy fallback");
    }

    void testDocumentSaveAndIdentity()
    {
        CadDocument document;
        CadItem* original = document.appendEntity(makeProductionSpline());
        check(original != nullptr, "append production spline");
        const cadcam::geometry::EntityId originalId = original->m_entityId;

        auto removed = document.takeEntity(original);
        CadItem* restored = document.appendEntity
            (std::move(removed.first), std::move(removed.second));
        check(restored != nullptr && restored->m_entityId == originalId,
            "Spline undo redo preserves EntityId");

        const auto* restoredSpline = static_cast<const DRW_Spline*>(restored->m_nativeEntity);
        std::unique_ptr<DRW_Spline> copied = cloneSplineEntity(restoredSpline);
        check(copied != nullptr
            && copied->controllist.front().get() != restoredSpline->controllist.front().get()
            && copied->knotslist == restoredSpline->knotslist
            && copied->weightlist == restoredSpline->weightlist,
            "Spline copy is deep and preserves exact data");
        CadItem* copiedItem = document.appendEntity(std::move(copied));
        check(copiedItem != nullptr && copiedItem->m_entityId != originalId,
            "Spline copy receives new EntityId");

        QTemporaryDir directory;
        check(directory.isValid(), "temporary DXF directory");
        const QString exactPath = directory.filePath(QStringLiteral("exact_spline.dxf"));
        const QString safePath = directory.filePath(QStringLiteral("safe_spline.dxf"));
        check(document.saveDxfDocument(exactPath, false), "save exact spline DXF");

        CadDocument exactReload;
        exactReload.readDxfDocument(exactPath);
        check(!exactReload.m_entities.empty()
            && std::all_of
            (
                exactReload.m_entities.cbegin(), exactReload.m_entities.cend(),
                [](const std::unique_ptr<CadItem>& item)
                {
                    return item != nullptr && item->m_type == DRW::ETYPE::SPLINE
                        && dynamic_cast<CadSplineItem*>(item.get()) != nullptr;
                }
            ),
            "Normal DXF save reload preserves SPLINE entities");
        if (!exactReload.m_entities.empty()
            && exactReload.m_entities.front()->m_type == DRW::ETYPE::SPLINE)
        {
            const auto* reloaded = static_cast<const DRW_Spline*>(
                exactReload.m_entities.front()->m_nativeEntity);
            check(reloaded->degree == restoredSpline->degree
                && reloaded->flags == restoredSpline->flags
                && reloaded->knotslist == restoredSpline->knotslist
                && reloaded->weightlist == restoredSpline->weightlist
                && reloaded->controllist.size() == restoredSpline->controllist.size(),
                "Exact DXF reload preserves spline definition data");
        }

        check(document.saveDxfDocument(safePath, true), "save safe spline DXF");
        check(std::all_of
            (
                document.m_entities.cbegin(), document.m_entities.cend(),
                [](const std::unique_ptr<CadItem>& item)
                {
                    return item != nullptr && item->m_type == DRW::ETYPE::SPLINE;
                }
            ),
            "Safe DXF export does not mutate source document");
        CadDocument safeReload;
        safeReload.readDxfDocument(safePath);
        check(!safeReload.m_entities.empty()
            && std::all_of
            (
                safeReload.m_entities.cbegin(), safeReload.m_entities.cend(),
                [](const std::unique_ptr<CadItem>& item)
                {
                    return item != nullptr && item->m_type == DRW::ETYPE::POLYLINE;
                }
            ),
            "Safe DXF export reloads as temporary 3D POLYLINE");
    }

    void testGCodeProfileAndParity()
    {
        CadDocument document;
        auto* item = static_cast<CadSplineItem*>(document.appendEntity(makeProductionSpline()));
        check(item != nullptr, "append spline for G-code");

        const OperationResult<QString> threeAxis =
            buildProgram(document, GGenerator::GenerationMode::Mode2D);
        const OperationResult<QString> fourAxis =
            buildProgram(document, GGenerator::GenerationMode::Mode3D);
        check(threeAxis.succeeded() && threeAxis.value.has_value(),
            "Spline 3-axis G-code builds");
        check(fourAxis.succeeded() && fourAxis.value.has_value(),
            "Spline 4-axis G-code builds");
        if (threeAxis.value.has_value())
        {
            verifyGolden(QStringLiteral("spline_3axis.nc"), threeAxis.value->toUtf8());
        }
        if (fourAxis.value.has_value())
        {
            verifyGolden(QStringLiteral("spline_4axis.nc"), fourAxis.value->toUtf8());
        }

        GProfile oldProfile;
        GProfileCodeBlock polylineCode;
        polylineCode.header = QStringLiteral("M101");
        polylineCode.footer = QStringLiteral("M102");
        oldProfile.setEntityTypeCode(QStringLiteral("POLYLINE"), polylineCode);
        check(!oldProfile.containsEntityTypeCode(QStringLiteral("SPLINE"))
            && oldProfile.entityTypeCode(QStringLiteral("SPLINE")).header
                == polylineCode.header,
            "Old profile SPLINE rule falls back to POLYLINE");

        const std::unique_ptr<DRW_Spline> spline = makeProductionSpline();
        const SplineParityReport report = compareSplineWithLegacy(*spline);
        QJsonObject parity;
        parity.insert(QStringLiteral("equivalent"), report.equivalent);
        parity.insert(QStringLiteral("legacyPointCount"),
            static_cast<qint64>(report.legacyPointCount));
        parity.insert(QStringLiteral("corePointCount"),
            static_cast<qint64>(report.corePointCount));
        parity.insert(QStringLiteral("maximumPointDistance"), report.maximumPointDistance);
        parity.insert(QStringLiteral("firstDifferentIndex"), report.firstDifferentIndex);
        verifyGolden
        (
            QStringLiteral("spline_path_error_report.json"),
            QJsonDocument(parity).toJson(QJsonDocument::Indented)
        );
        check(report.equivalent && report.maximumPointDistance == 0.0,
            "Production spline path remains byte-quantized legacy equivalent");
    }

    void testSharedFourAxisBuilder()
    {
        cadcam::geometry::Path3D path;
        path.sourceEntityId = 99;
        path.vertices =
        {
            { { 0.0, 0.0, 10.0 }, 0.0 },
            { { 5.0, 10.0, 0.0 }, 1.0 },
            { { 10.0, 0.0, -10.0 }, 2.0 }
        };
        cadcam::machine::RotaryMachinePolicy options;
        options.keepContinuousAngle = true;
        const auto result = cadcam::machine::RotaryKinematics::transform
            (path, options, std::nullopt, context(QStringLiteral("shared-four-axis")));
        check(result.succeeded() && result.value->size() == path.vertices.size(),
            "Rotary kinematics returns one pose per path point");
        check(result.succeeded()
            && std::abs((*result.value)[0].aDegrees) <= 1.0e-9
            && std::abs((*result.value)[1].aDegrees - 90.0) <= 1.0e-9
            && std::abs((*result.value)[2].aDegrees - 180.0) <= 1.0e-9,
            "Rotary kinematics preserves angle policy");
    }
}

int runSplineProductionTests(bool updateGoldenFiles)
{
    failures = 0;
    updateGoldens = updateGoldenFiles;
    testCadSplineItemAndProvider();
    testDocumentSaveAndIdentity();
    testGCodeProfileAndParity();
    testSharedFourAxisBuilder();
    return failures;
}
