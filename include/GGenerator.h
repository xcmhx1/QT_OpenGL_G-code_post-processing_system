#pragma once

#include "GProfile.h"
#include "core/diagnostics/OperationContext.h"
#include "core/diagnostics/OperationResult.h"
#include "core/machining/TubeSection.h"
#include "core/planning/ProcessPlan.h"
#include "application/process/DocumentProcessState.h"

#include <QString>

class CadDocument;
class QWidget;

class GGenerator
{
public:
    enum class GenerationMode
    {
        Mode2D,
        Mode3D
    };

public:
    GGenerator();

    void setDocument(CadDocument* document);
    CadDocument* document() const;

    void setProfile(GProfile* profile);
    GProfile* profile() const;

    void setGenerationMode(GenerationMode generationMode);
    GenerationMode generationMode() const;

    void setRotaryTubeCenter(double centerY, double centerZ, bool valid);
    void setProcessPlan(const cadcam::planning::ProcessPlan* processPlan);
    void setProcessState(const cadcam::process::DocumentProcessState* processState);
    void setTubeSectionModel
    (
        const std::optional<cadcam::machining::TubeSectionModel>& tubeSectionModel
    );

    OperationResult<QString> buildProgramText(const OperationContext& context) const;
    OperationReport writeProgramText
    (
        const QString& filePath,
        const QString& program,
        const OperationContext& context
    ) const;

    bool generate(QWidget* parent = nullptr, QString* errorMessage = nullptr) const;
    bool generateToFile(const QString& filePath, QString* errorMessage = nullptr) const;

private:
    OperationResult<QString> buildRotaryProgramText(const OperationContext& context) const;

    CadDocument* m_document = nullptr;
    GProfile m_defaultProfile;
    GProfile* m_profile = nullptr;
    GenerationMode m_generationMode = GenerationMode::Mode2D;
    double m_rotaryTubeCenterY = 0.0;
    double m_rotaryTubeCenterZ = 0.0;
    bool m_rotaryTubeCenterValid = false;
    const cadcam::planning::ProcessPlan* m_processPlan = nullptr;
    const cadcam::process::DocumentProcessState* m_processState = nullptr;
    std::optional<cadcam::machining::TubeSectionModel> m_tubeSectionModel;
};
