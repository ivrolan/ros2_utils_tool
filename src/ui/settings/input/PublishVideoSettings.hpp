#pragma once

#include "AdvancedInputSettings.hpp"

// Store video publishing settings
class PublishVideoSettings : public AdvancedInputSettings {
public:
    PublishVideoSettings(Utils::UI::PublishVideoParameters& parameters,
                         const QString&                     groupName);

    bool
    write() override;

protected:
    bool
    read() override;

private:
    Utils::UI::PublishVideoParameters& m_parameters;
};
