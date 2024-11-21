#include "BagParamSettings.hpp"

BagParamSettings::BagParamSettings(Utils::UI::BagParameters& bagParameters,
                                   const QString&            groupName) :
    AdvancedParamSettings(bagParameters, groupName), m_bagParameters(bagParameters)
{
    read();
}


bool
BagParamSettings::write()
{
    if (!AdvancedParamSettings::write()) {
        return false;
    }

    QSettings settings;
    settings.beginGroup(m_groupName);
    setSettingsParameter(settings, m_bagParameters.fps, "fps");
    setSettingsParameter(settings, m_bagParameters.useCustomFPS, "custom_fps");
    setSettingsParameter(settings, m_bagParameters.useHardwareAcceleration, "hw_acc");
    settings.endGroup();

    return true;
}


bool
BagParamSettings::read()
{
    if (!AdvancedParamSettings::read()) {
        return false;
    }

    QSettings settings;
    settings.beginGroup(m_groupName);
    m_bagParameters.fps = settings.value("fps").isValid() ? settings.value("fps").toInt() : 30;
    m_bagParameters.useCustomFPS = settings.value("custom_fps").isValid() ? settings.value("custom_fps").toBool() : false;
    m_bagParameters.useHardwareAcceleration = settings.value("hw_acc").isValid() ? settings.value("hw_acc").toBool() : false;
    settings.endGroup();

    return true;
}
