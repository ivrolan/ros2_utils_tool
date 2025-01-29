#pragma once

#include "InputSettings.hpp"

// Store dummy bag creation parameters
class DummyBagInputSettings : public InputSettings {
public:
    DummyBagInputSettings(Utils::UI::DummyBagInputParameters& parameters,
                          const QString&                      groupName);

    bool
    write() override;

private:
    bool
    read() override;

private:
    Utils::UI::DummyBagInputParameters& m_parameters;
};
