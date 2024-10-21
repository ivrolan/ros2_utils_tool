#pragma once

#include "AdvancedParamSettings.hpp"

// Store advanced settings
class ImageParamSettings : public AdvancedParamSettings {
public:
    ImageParamSettings(Utils::UI::ImageParameters& imageParameters,
                       const QString&              groupName);

    void
    write() override;

private:
    bool
    read() override;

private:
    Utils::UI::ImageParameters& m_imageParameters;
};
