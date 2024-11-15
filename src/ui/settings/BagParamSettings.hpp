#pragma once

#include "AdvancedParamSettings.hpp"

// Store bag parameter settings
class BagParamSettings : public AdvancedParamSettings {
public:
    BagParamSettings(Utils::UI::BagParameters& bagParameters,
                     const QString&            groupName);

    bool
    write() override;

private:
    bool
    read() override;

private:
    Utils::UI::BagParameters& m_bagParameters;
};
