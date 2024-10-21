#pragma once

#include "VideoParamSettings.hpp"

// Store bag parameter settings
class BagParamSettings : public VideoParamSettings {
public:
    BagParamSettings(Utils::UI::BagParameters& bagParameters,
                     const QString&            groupName);

    void
    write() override;

private:
    bool
    read() override;

private:
    Utils::UI::BagParameters& m_bagParameters;
};
