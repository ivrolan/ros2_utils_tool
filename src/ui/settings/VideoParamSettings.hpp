#pragma once

#include "AdvancedParamSettings.hpp"

// Store advanced settings
class VideoParamSettings : public AdvancedParamSettings {
public:
    VideoParamSettings(Utils::UI::VideoParameters& videoParameters,
                       const QString&              groupName);

    bool
    write() override;

protected:
    bool
    read() override;

private:
    Utils::UI::VideoParameters& m_videoParameters;
};
