#include "AdvancedInputSettings.hpp"

AdvancedInputSettings::AdvancedInputSettings(Utils::UI::AdvancedInputParameters& parameters, const QString& groupName) :
    InputSettings(parameters, groupName), m_parameters(parameters)
{
    read();
}


bool
AdvancedInputSettings::write()
{
    if (!InputSettings::write()) {
        return false;
    }

    QSettings settings;
    settings.beginGroup(m_groupName);
    setSettingsParameter(settings, m_parameters.targetDirectory, "target_dir");
    settings.endGroup();

    return true;
}


bool
AdvancedInputSettings::read()
{
    if (!InputSettings::read()) {
        return false;
    }

    QSettings settings;
    settings.beginGroup(m_groupName);
    m_parameters.targetDirectory = settings.value("target_dir").isValid() ? settings.value("target_dir").toString() : "";
    settings.endGroup();

    return true;
}
