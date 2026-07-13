#include "CadCircleItem.h"
#include "CadDocument.h"
#include "CadEllipseItem.h"
#include "CadItem.h"
#include "CadLineItem.h"
#include "CadOcsGeometry.h"
#include "GGenerator.h"
#include "GProfile.h"
#include "application/messaging/MessageCenter.h"
#include "core/diagnostics/OperationResult.h"
#include "dx_data.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>

#include <algorithm>
#include <cmath>
#include <iostream>
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
