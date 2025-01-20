#pragma once

#include "AdvancedInputSettings.hpp"

// Store advanced settings
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
