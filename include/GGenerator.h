#pragma once

#include "GProfile.h"

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

    bool generate(QWidget* parent = nullptr, QString* errorMessage = nullptr) const;
    bool generateToFile(const QString& filePath, QString* errorMessage = nullptr) const;

private:
    CadDocument* m_document = nullptr;
    GProfile m_defaultProfile;
    GProfile* m_profile = nullptr;
    GenerationMode m_generationMode = GenerationMode::Mode2D;
};
