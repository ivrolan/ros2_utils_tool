#pragma once

#include "AdvancedInputSettings.hpp"

// Store publishing parameters
class PublishSettings : public AdvancedInputSettings {
public:
    PublishSettings(Utils::UI::PublishParameters& parameters,
                    const QString&                groupName);

    bool
    write() override;

private:
    bool
    read() override;

private:
    Utils::UI::PublishParameters& m_parameters;
};
