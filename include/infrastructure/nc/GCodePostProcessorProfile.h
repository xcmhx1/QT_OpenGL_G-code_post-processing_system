#pragma once

#include <QHash>
#include <QString>

class GProfile;

namespace cadcam::infrastructure::nc
{
    struct GCodeBlock
    {
        QString header;
        QString footer;
    };

    struct GCodePostProcessorProfile
    {
        QString programHeader;
        QString programFooter;
        QHash<QString, GCodeBlock> entityTypeBlocks;
        QHash<QString, GCodeBlock> layerBlocks;
        QHash<QString, GCodeBlock> colorBlocks;
        int coordinatePrecision = 5;
        int anglePrecision = 5;
        QString rapidCode = QStringLiteral("G00");
        QString linearCode = QStringLiteral("G01");
    };

    GCodePostProcessorProfile makeGCodePostProcessorProfile(const GProfile& profile);
}
