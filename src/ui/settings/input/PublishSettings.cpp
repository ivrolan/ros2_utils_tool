#include "PublishSettings.hpp"

PublishSettings::PublishSettings(Utils::UI::PublishParameters& parameters, const QString& groupName) :
    AdvancedInputSettings(parameters, groupName), m_parameters(parameters)
{
    read();
}


bool
PublishSettings::write()
{
    if (!AdvancedInputSettings::write()) {
        return false;
    }

    QSettings settings;
    settings.beginGroup(m_groupName);
    setSettingsParameter(settings, m_parameters.switchRedBlueValues, "switch_red_blue");
    setSettingsParameter(settings, m_parameters.loop, "loop");
    setSettingsParameter(settings, m_parameters.useHardwareAcceleration, "hw_acc");
    setSettingsParameter(settings, m_parameters.fps, "fps");
    settings.endGroup();

    return true;
}


bool
PublishSettings::read()
{
    if (!AdvancedInputSettings::read()) {
        return false;
    }

    QSettings settings;
    settings.beginGroup(m_groupName);
    m_parameters.switchRedBlueValues = settings.value("switch_red_blue").isValid() ? settings.value("switch_red_blue").toBool() : false;
    m_parameters.loop = settings.value("loop").isValid() ? settings.value("loop").toBool() : false;
    m_parameters.useHardwareAcceleration = settings.value("hw_acc").isValid() ? settings.value("hw_acc").toBool() : false;
    m_parameters.fps = settings.value("fps").isValid() ? settings.value("fps").toInt() : 30;
    settings.endGroup();

    return true;
}
