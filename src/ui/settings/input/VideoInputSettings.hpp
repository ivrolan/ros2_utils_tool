#pragma once

#include "AdvancedInputSettings.hpp"

// Store video out of ROS bag creation parameters
class VideoInputSettings : public AdvancedInputSettings {
public:
    VideoInputSettings(Utils::UI::VideoInputParameters& parameters,
                       const QString&                   groupName);

    bool
    write() override;

private:
    bool
    read() override;

private:
    Utils::UI::VideoInputParameters& m_parameters;
};
