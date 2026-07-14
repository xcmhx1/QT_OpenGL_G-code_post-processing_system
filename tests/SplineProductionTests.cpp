#include "SplineProductionTests.h"

#include "CadDocument.h"
#include "CadPolylineItem.h"
#include "CadSplineItem.h"
#include "GGenerator.h"
#include "GProfile.h"
#include "SplineParity.h"
#include "compatibility/legacy/LegacyFourAxisPathBuilder.h"
#include "compatibility/legacy/SplineEntityClone.h"
#include "compatibility/legacy/SplineProductionPathProvider.h"
#include "core/geometry/GeometryCompiler.h"
#include "dx_data.h"

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
            check(input.readAll() == actual, fileName.toUtf8().constData());
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
        generator.setDocument(&document);
        generator.setProfile(&profile);
        generator.setGenerationMode(mode);
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

        item->m_isReverse = false;
        item->rebuildRawPathPoints3D();
        const std::vector<RawPathPoint3D> forward = item->rawPathPoints3D();
        item->m_isReverse = true;
        item->rebuildRawPathPoints3D();
        const std::vector<RawPathPoint3D> reverse = item->rawPathPoints3D();
        check(forward.size() == reverse.size() && !forward.empty()
            && samePoint(forward.front(), reverse.back())
            && samePoint(forward.back(), reverse.front()),
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
        item->m_processOrder = 0;
        item->m_processContinuousGroupId = -1;

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
        const std::vector<RawPathPoint3D> raw
        {
            { 0.0, 0.0, 10.0 },
            { 5.0, 10.0, 0.0 },
            { 10.0, 0.0, -10.0 }
        };
        LegacyFourAxisPathOptions options;
        options.keepContinuousAngle = true;
        const OperationResult<std::vector<ControlPoint4Axis>> result =
            LegacyFourAxisPathBuilder::build
            (raw, options, 99, context(QStringLiteral("shared-four-axis")));
        check(result.succeeded() && result.value->size() == raw.size(),
            "Shared four-axis builder returns one point per raw point");
        check(result.succeeded()
            && std::abs((*result.value)[0].aDeg) <= 1.0e-9
            && std::abs((*result.value)[1].aDeg - 90.0) <= 1.0e-9
            && std::abs((*result.value)[2].aDeg - 180.0) <= 1.0e-9,
            "Shared four-axis builder preserves legacy angle policy");
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
