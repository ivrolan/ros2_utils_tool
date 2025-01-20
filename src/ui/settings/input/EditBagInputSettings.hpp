#pragma once

#include "AdvancedInputSettings.hpp"

// Store edit bag parameter settings
class EditBagInputSettings : public AdvancedInputSettings {
public:
    EditBagInputSettings(Utils::UI::EditBagInputParameters& parameters,
                         const QString&                     groupName);

    bool
    write() override;

private:
    bool
    read() override;

private:
    Utils::UI::EditBagInputParameters& m_parameters;
};
