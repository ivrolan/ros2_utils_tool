#pragma once

#include "BasicParamSettings.hpp"

// Basic settings, from which all other settings derive
class DummyBagParamSettings : public BasicParamSettings {
public:
    DummyBagParamSettings(Utils::UI::DummyBagParameters& dummyBagParameters,
                          const QString&                 groupName);

    void
    write() override;

private:
    bool
    read() override;

private:
    Utils::UI::DummyBagParameters& m_dummyBagParameters;
};
