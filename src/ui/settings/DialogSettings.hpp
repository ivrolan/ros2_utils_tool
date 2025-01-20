#pragma once

#include "BasicSettings.hpp"
#include "UtilsUI.hpp"

// Settings modified from settings dialog
class DialogSettings : public BasicSettings {
public:
    DialogSettings(Utils::UI::DialogParameters& parameters,
                   const QString&               groupName);

    [[nodiscard]] static bool
    areParametersSaved();

    bool
    write() override;

private:
    bool
    read() override;

private:
    Utils::UI::DialogParameters& m_parameters;
};
