#pragma once

#include "InputSettings.hpp"

// Basic settings, from which all other settings derive
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
