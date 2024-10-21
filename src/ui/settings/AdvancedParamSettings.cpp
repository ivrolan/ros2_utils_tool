#include "AdvancedParamSettings.hpp"

AdvancedParamSettings::AdvancedParamSettings(Utils::UI::AdvancedParameters& advancedParameters, const QString& groupName) :
    BasicParamSettings(advancedParameters, groupName), m_advancedParameters(advancedParameters)
{
    read();
}


void
AdvancedParamSettings::write()
{
    BasicParamSettings::write();

    QSettings settings;

    settings.beginGroup(m_groupName);
    setSettingsParameter(settings, m_advancedParameters.targetDirectory, "target_dir");
    settings.endGroup();
}


bool
AdvancedParamSettings::read()
{
    if (!BasicParamSettings::read()) {
        return false;
    }

    QSettings settings;
    settings.beginGroup(m_groupName);
    m_advancedParameters.targetDirectory = settings.value("target_dir").isValid() ? settings.value("target_dir").toString() : "";
    settings.endGroup();

    return true;
}
