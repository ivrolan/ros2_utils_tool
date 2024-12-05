#pragma once

#include "AdvancedParamSettings.hpp"

// Store edit bag parameter settings
class EditBagParamSettings : public AdvancedParamSettings {
public:
    EditBagParamSettings(Utils::UI::EditBagParameters& editBagParameters,
                         const QString&                groupName);

    bool
    write() override;

private:
    bool
    read() override;

private:
    Utils::UI::EditBagParameters& m_editBagParameters;
};
