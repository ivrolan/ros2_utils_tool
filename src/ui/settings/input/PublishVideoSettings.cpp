#include "PublishVideoSettings.hpp"

PublishVideoSettings::PublishVideoSettings(Utils::UI::PublishVideoParameters& parameters, const QString& groupName) :
    AdvancedInputSettings(parameters, groupName), m_parameters(parameters)
{
    read();
}


bool
PublishVideoSettings::write()
{
    if (!AdvancedInputSettings::write()) {
        return false;
    }

    QSettings settings;
    settings.beginGroup(m_groupName);
    setSettingsParameter(settings, m_parameters.useHardwareAcceleration, "hw_acc");
    setSettingsParameter(settings, m_parameters.switchRedBlueValues, "switch_red_blue");
    setSettingsParameter(settings, m_parameters.loop, "loop");
    settings.endGroup();

    return true;
}


bool
PublishVideoSettings::read()
{
    if (!AdvancedInputSettings::read()) {
        return false;
    }

    QSettings settings;
    settings.beginGroup(m_groupName);
    m_parameters.useHardwareAcceleration = settings.value("hw_acc").isValid() ? settings.value("hw_acc").toBool() : false;
    m_parameters.switchRedBlueValues = settings.value("switch_red_blue").isValid() ? settings.value("switch_red_blue").toBool() : false;
    m_parameters.loop = settings.value("loop").isValid() ? settings.value("loop").toBool() : false;
    settings.endGroup();

    return true;
}
