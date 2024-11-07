#include "BagParamSettings.hpp"

BagParamSettings::BagParamSettings(Utils::UI::BagParameters& bagParameters,
                                   const QString&            groupName) :
    VideoParamSettings(bagParameters, groupName), m_bagParameters(bagParameters)
{
    read();
}


bool
BagParamSettings::write()
{
    if (!VideoParamSettings::write()) {
        return false;
    }

    QSettings settings;
    settings.beginGroup(m_groupName);
    setSettingsParameter(settings, m_bagParameters.useCustomFPS, "custom_fps");
    settings.endGroup();

    return true;
}


bool
BagParamSettings::read()
{
    if (!VideoParamSettings::read()) {
        return false;
    }

    QSettings settings;
    settings.beginGroup(m_groupName);
    m_bagParameters.useCustomFPS = settings.value("custom_fps").isValid() ? settings.value("custom_fps").toBool() : false;
    settings.endGroup();

    return true;
}
