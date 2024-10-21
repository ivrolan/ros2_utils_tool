#pragma once

#include "BasicParamSettings.hpp"

// Store advanced settings
class AdvancedParamSettings : public BasicParamSettings {
public:
    AdvancedParamSettings(Utils::UI::AdvancedParameters& advancedParameters,
                          const QString&                 groupName);

    void
    write() override;

protected:
    bool
    read() override;

private:
    Utils::UI::AdvancedParameters& m_advancedParameters;
};
