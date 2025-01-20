#pragma once

#include "BasicSettings.hpp"
#include "UtilsUI.hpp"

// Basic parameter settings, which are used to store all input widget parameters
class InputSettings : public BasicSettings {
public:
    InputSettings(Utils::UI::InputParameters& parameters,
                  const QString&              groupName);

    virtual bool
    write() override;

protected:
    virtual bool
    read() override;

private:
    Utils::UI::InputParameters& m_parameters;
};
