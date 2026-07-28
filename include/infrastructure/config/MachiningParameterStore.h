#pragma once

#include "infrastructure/config/GProfile.h"

class MachiningParameterStore final
{
public:
    static void applyTo(GProfile& profile);
    static void save
    (
        const GProfileToolTransferConfig& toolTransferConfig,
        const GProfileRotaryAxisConfig& rotaryAxisConfig
    );
};
