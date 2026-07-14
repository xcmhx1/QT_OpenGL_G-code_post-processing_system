#include "pch.h"

#include "GGenerator.h"
#include "application/nc/NcProgramService.h"
#include "infrastructure/nc/GCodePostProcessor.h"

#include "CadDocument.h"

#include <QDir>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QSettings>
#include <QStandardPaths>
#include <QWidget>

namespace
{
    QString defaultNcPath()
    {
        QString baseDirectory = QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation);
        if (baseDirectory.trimmed().isEmpty()) baseDirectory = QDir::homePath();
        return QDir(baseDirectory).filePath(QStringLiteral("output.nc"));
    }

    QString loadLastExportPath()
    {
        QSettings settings(QStringLiteral("GCodePostProcessingSystem"),
            QStringLiteral("GCodePostProcessingSystem"));
        return settings.value(QStringLiteral("gcode/lastExportPath"), QString()).toString().trimmed();
    }

    void saveLastExportPath(const QString& filePath)
    {
        QSettings settings(QStringLiteral("GCodePostProcessingSystem"),
            QStringLiteral("GCodePostProcessingSystem"));
        settings.setValue(QStringLiteral("gcode/lastExportPath"), filePath.trimmed());
    }

    QString resolveInitialExportPath(QWidget*)
    {
        const QString lastExportPath = loadLastExportPath();
        if (!lastExportPath.isEmpty()
            && QDir(QFileInfo(lastExportPath).absolutePath()).exists())
        {
            return lastExportPath;
        }
        return defaultNcPath();
    }

    Diagnostic makeGeneratorDiagnostic
    (
        const OperationContext& context,
        DiagnosticCode code,
        DiagnosticSeverity severity,
        const QString& operation,
        const QString& stage,
        const QString& userMessage,
        const QString& technicalDetail = QString(),
        const QVariantMap& diagnosticContext = QVariantMap()
    )
    {
        Diagnostic diagnostic;
        diagnostic.code = code;
        diagnostic.severity = severity;
        diagnostic.component = QStringLiteral("GGenerator");
        diagnostic.operation = operation;
        diagnostic.stage = stage;
        diagnostic.userMessage = userMessage;
        diagnostic.technicalDetail = technicalDetail;
        diagnostic.correlationId = context.correlationId;
        diagnostic.context = diagnosticContext;
        return diagnostic;
    }

    QString legacyErrorMessage(const QVector<Diagnostic>& diagnostics)
    {
        for (const Diagnostic& diagnostic : diagnostics)
        {
            if (isErrorSeverity(diagnostic.severity))
            {
                return diagnostic.userMessage.trimmed().isEmpty()
                    ? diagnostic.technicalDetail : diagnostic.userMessage;
            }
        }
        return diagnostics.isEmpty() ? QString() : diagnostics.constFirst().userMessage;
    }
}

GGenerator::GGenerator()
    : m_defaultProfile(GProfile::createDefaultLaserProfile())
    , m_profile(&m_defaultProfile)
{
}

void GGenerator::setDocument(CadDocument* document)
{
    m_document = document;
}

CadDocument* GGenerator::document() const
{
    return m_document;
}

void GGenerator::setProfile(GProfile* profile)
{
    m_profile = profile != nullptr ? profile : &m_defaultProfile;
}

GProfile* GGenerator::profile() const
{
    return m_profile;
}

void GGenerator::setGenerationMode(GenerationMode generationMode)
{
    m_generationMode = generationMode;
}

GGenerator::GenerationMode GGenerator::generationMode() const
{
    return m_generationMode;
}

void GGenerator::setRotaryTubeCenter(double centerY, double centerZ, bool valid)
{
    m_rotaryTubeCenterY = centerY;
    m_rotaryTubeCenterZ = centerZ;
    m_rotaryTubeCenterValid = valid;
}

void GGenerator::setProcessPlan(const cadcam::planning::ProcessPlan* processPlan)
{
    m_processPlan = processPlan;
}

void GGenerator::setProcessState(const cadcam::process::DocumentProcessState* processState)
{
    m_processState = processState;
}

void GGenerator::setTubeSectionModel
(
    const std::optional<cadcam::machining::TubeSectionModel>& tubeSectionModel
)
{
    m_tubeSectionModel = tubeSectionModel;
}

bool GGenerator::generate(QWidget* parent, QString* errorMessage) const
{
    const QString filePath = QFileDialog::getSaveFileName
    (
        parent,
        QStringLiteral("导出 G 代码"),
        resolveInitialExportPath(parent),
        QStringLiteral("NC 文件 (*.nc);;GCode 文件 (*.gcode);;文本文件 (*.txt)")
    );
    if (filePath.isEmpty())
    {
        if (errorMessage != nullptr) errorMessage->clear();
        return false;
    }

    QString resolvedPath = filePath;
    if (QFileInfo(resolvedPath).suffix().isEmpty()) resolvedPath.append(QStringLiteral(".nc"));
    const bool generated = generateToFile(resolvedPath, errorMessage);
    if (generated) saveLastExportPath(resolvedPath);
    return generated;
}

OperationResult<QString> GGenerator::buildRotaryProgramText(const OperationContext& context) const
{
    OperationResult<QString> result;
    if (m_document->m_entities.empty())
    {
        result.status = OperationStatus::InvalidInput;
        result.addDiagnostic(makeGeneratorDiagnostic
        (
            context,
            DiagnosticCode::InvalidGeometry,
            DiagnosticSeverity::Error,
            QStringLiteral("BuildRotaryProgramText"),
            QStringLiteral("ValidateDocument"),
            QStringLiteral("文档中没有可生成 G 代码的图元。")
        ));
        return result;
    }
    if (m_processPlan == nullptr || m_processState == nullptr)
    {
        result.status = OperationStatus::InvalidInput;
        result.addDiagnostic(makeGeneratorDiagnostic
        (
            context,
            DiagnosticCode::MachineTrajectoryInputInvalid,
            DiagnosticSeverity::Error,
            QStringLiteral("BuildRotaryProgramText"),
            QStringLiteral("ValidateProcessPlan"),
            QStringLiteral("当前没有有效的四轴加工计划，无法导出。")
        ));
        return result;
    }
    if (m_processPlan->mode != cadcam::planning::ProcessPlanMode::Rotary4Axis)
    {
        result.status = OperationStatus::Conflict;
        result.addDiagnostic(makeGeneratorDiagnostic
        (
            context, DiagnosticCode::ProcessPlanModeMismatch, DiagnosticSeverity::Error,
            QStringLiteral("BuildRotaryProgramText"), QStringLiteral("ValidateProcessPlan"),
            QStringLiteral("四轴 G 代码只能使用四轴加工计划。")
        ));
        return result;
    }

    std::optional<cadcam::geometry::Vector2d> explicitCenter;
    if (m_rotaryTubeCenterValid)
        explicitCenter = cadcam::geometry::Vector2d{ m_rotaryTubeCenterY, m_rotaryTubeCenterZ };
    NcProgramService service;
    auto program = service.buildRotaryProgram
    (
        *m_document,
        *m_processState,
        *m_processPlan,
        m_tubeSectionModel,
        m_profile->rotaryAxisConfig(),
        context,
        explicitCenter
    );
    result.mergeDiagnostics(program);
    if (!program.succeeded() || !program.value.has_value())
    {
        result.status = program.status;
        return result;
    }

    const auto postProfile = cadcam::infrastructure::nc::makeGCodePostProcessorProfile(*m_profile);
    auto rendered = cadcam::infrastructure::nc::GCodePostProcessor::render
        (*program.value, postProfile, context);
    result.mergeDiagnostics(rendered);
    if (!rendered.succeeded() || !rendered.value.has_value())
    {
        result.status = rendered.status;
        return result;
    }
    result.status = program.status == OperationStatus::PartialSuccess
        ? OperationStatus::PartialSuccess : rendered.status;
    result.value = std::move(rendered.value);
    return result;
}

OperationResult<QString> GGenerator::buildProgramText(const OperationContext& context) const
{
    OperationResult<QString> result;
    if (m_document == nullptr || m_profile == nullptr)
    {
        result.status = OperationStatus::InvalidInput;
        result.addDiagnostic(makeGeneratorDiagnostic
        (
            context,
            m_document == nullptr ? DiagnosticCode::MissingDocument : DiagnosticCode::MissingProfile,
            DiagnosticSeverity::Error,
            QStringLiteral("BuildProgramText"),
            QStringLiteral("ValidateInput"),
            m_document == nullptr
                ? QStringLiteral("未设置文档，无法生成 G 代码。")
                : QStringLiteral("未设置 G 代码配置，无法生成 G 代码。")
        ));
        return result;
    }
    if (m_generationMode == GenerationMode::Mode3D) return buildRotaryProgramText(context);

    if (m_processPlan == nullptr || m_processState == nullptr
        || m_processPlan->mode != cadcam::planning::ProcessPlanMode::Planar3Axis)
    {
        result.status = OperationStatus::Conflict;
        result.addDiagnostic(makeGeneratorDiagnostic
        (
            context, DiagnosticCode::ProcessPlanModeMismatch, DiagnosticSeverity::Error,
            QStringLiteral("BuildProgramText"), QStringLiteral("ValidateProcessPlan"),
            QStringLiteral("三轴 G 代码需要有效的三轴加工计划。")
        ));
        return result;
    }

    NcProgramService service;
    auto program = service.buildPlanarProgram
        (*m_document, *m_processState, *m_processPlan, context);
    result.mergeDiagnostics(program);
    if (!program.succeeded() || !program.value.has_value())
    {
        result.status = program.status;
        return result;
    }
    const auto postProfile = cadcam::infrastructure::nc::makeGCodePostProcessorProfile(*m_profile);
    auto rendered = cadcam::infrastructure::nc::GCodePostProcessor::render
        (*program.value, postProfile, context);
    result.mergeDiagnostics(rendered);
    if (!rendered.succeeded() || !rendered.value.has_value())
    {
        result.status = rendered.status;
        return result;
    }
    result.status = program.status == OperationStatus::PartialSuccess
        ? OperationStatus::PartialSuccess : rendered.status;
    result.value = std::move(rendered.value);
    return result;
}

OperationReport GGenerator::writeProgramText
(
    const QString& filePath,
    const QString& program,
    const OperationContext& context
) const
{
    OperationReport report;
    if (filePath.trimmed().isEmpty())
    {
        report.status = OperationStatus::InvalidInput;
        report.addDiagnostic(makeGeneratorDiagnostic
        (
            context, DiagnosticCode::InvalidArgument, DiagnosticSeverity::Error,
            QStringLiteral("WriteProgramText"), QStringLiteral("ValidateInput"),
            QStringLiteral("G 代码输出路径为空。"), QStringLiteral("filePath is empty")
        ));
        return report;
    }
    if (program.isEmpty())
    {
        report.status = OperationStatus::InvalidInput;
        report.addDiagnostic(makeGeneratorDiagnostic
        (
            context, DiagnosticCode::OutputVerificationFailure, DiagnosticSeverity::Error,
            QStringLiteral("WriteProgramText"), QStringLiteral("ValidateInput"),
            QStringLiteral("不能写入空的 G 代码程序。"), QStringLiteral("program is empty"),
            { { QStringLiteral("filePath"), filePath } }
        ));
        return report;
    }

    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate))
    {
        report.status = OperationStatus::Failed;
        report.addDiagnostic(makeGeneratorDiagnostic
        (
            context, DiagnosticCode::FileOpenFailure, DiagnosticSeverity::Error,
            QStringLiteral("WriteProgramText"), QStringLiteral("OpenOutputFile"),
            QStringLiteral("无法打开 G 代码输出文件。"), file.errorString(),
            { { QStringLiteral("filePath"), filePath } }
        ));
        return report;
    }

    const QByteArray encodedProgram = program.toUtf8();
    const qint64 writtenLength = file.write(encodedProgram);
    file.close();
    if (writtenLength != encodedProgram.size())
    {
        report.status = OperationStatus::Failed;
        report.addDiagnostic(makeGeneratorDiagnostic
        (
            context, DiagnosticCode::FileWriteFailure, DiagnosticSeverity::Error,
            QStringLiteral("WriteProgramText"), QStringLiteral("WriteOutputFile"),
            QStringLiteral("G 代码文件写入不完整。"),
            QStringLiteral("QFile::write returned an unexpected byte count"),
            {
                { QStringLiteral("filePath"), filePath },
                { QStringLiteral("expectedCount"), encodedProgram.size() },
                { QStringLiteral("actualCount"), writtenLength }
            }
        ));
        return report;
    }

    report.status = OperationStatus::Success;
    report.value = std::monostate{};
    return report;
}

bool GGenerator::generateToFile(const QString& filePath, QString* errorMessage) const
{
    const OperationContext context = createOperationContext(QStringLiteral("generate-gcode"));
    const OperationResult<QString> buildResult = buildProgramText(context);
    if (!buildResult.succeeded() || !buildResult.value.has_value())
    {
        if (errorMessage != nullptr) *errorMessage = legacyErrorMessage(buildResult.diagnostics);
        return false;
    }

    const OperationReport writeResult = writeProgramText(filePath, *buildResult.value, context);
    if (!writeResult.succeeded())
    {
        if (errorMessage != nullptr) *errorMessage = legacyErrorMessage(writeResult.diagnostics);
        return false;
    }
    if (errorMessage != nullptr) errorMessage->clear();
    return true;
}
