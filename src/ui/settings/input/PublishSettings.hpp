#pragma once

#include "AdvancedInputSettings.hpp"

// Store publishing settings
class PublishSettings : public AdvancedInputSettings {
public:
    PublishSettings(Utils::UI::PublishParameters& parameters,
                    const QString&                groupName);

    bool
    write() override;

protected:
    bool
    read() override;

private:
    Utils::UI::PublishParameters& m_parameters;
};
