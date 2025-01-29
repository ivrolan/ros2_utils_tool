#pragma once

#include "AdvancedInputSettings.hpp"

// Store bag creation parameters
class BagInputSettings : public AdvancedInputSettings {
public:
    BagInputSettings(Utils::UI::BagInputParameters& parameters,
                     const QString&                 groupName);

    bool
    write() override;

private:
    bool
    read() override;

private:
    Utils::UI::BagInputParameters& m_parameters;
};
