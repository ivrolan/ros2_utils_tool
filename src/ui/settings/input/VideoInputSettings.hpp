#pragma once

#include "AdvancedInputSettings.hpp"

// Store advanced settings
class VideoInputSettings : public AdvancedInputSettings {
public:
    VideoInputSettings(Utils::UI::VideoInputParameters& parameters,
                       const QString&                   groupName);

    bool
    write() override;

protected:
    bool
    read() override;

private:
    Utils::UI::VideoInputParameters& m_parameters;
};
