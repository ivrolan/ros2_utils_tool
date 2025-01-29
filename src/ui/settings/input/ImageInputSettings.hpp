#pragma once

#include "AdvancedInputSettings.hpp"

// Store parameters for image sequence out of ROS bag creation
class ImageInputSettings : public AdvancedInputSettings {
public:
    ImageInputSettings(Utils::UI::ImageInputParameters& parameters,
                       const QString&                   groupName);

    bool
    write() override;

private:
    bool
    read() override;

private:
    Utils::UI::ImageInputParameters& m_parameters;
};
