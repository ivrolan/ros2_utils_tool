#pragma once

#include "InputSettings.hpp"

// Store advanced settings
class AdvancedInputSettings : public InputSettings {
public:
    AdvancedInputSettings(Utils::UI::AdvancedInputParameters& parameters,
                          const QString&                      groupName);

    bool
    write() override;

protected:
    bool
    read() override;

private:
    Utils::UI::AdvancedInputParameters& m_parameters;
};
