#include "DialogSettings.hpp"

DialogSettings::DialogSettings(Utils::UI::DialogParameters& parameters, const QString& groupName) :
    BasicSettings(groupName), m_parameters(parameters)
{
    read();
}


bool
DialogSettings::areParametersSaved()
{
    QSettings settings;
    settings.beginGroup("dialog");
    const auto savedValue = settings.value("save_parameters").isValid() ? settings.value("save_parameters").toBool() : false;
    settings.endGroup();

    return savedValue;
}


bool
DialogSettings::write()
{
    QSettings settings;
    settings.beginGroup(m_groupName);
    settings.setValue("save_parameters", m_parameters.saveParameters);
    settings.setValue("check_ros2_naming_convention", m_parameters.checkROS2NameConform);
    settings.endGroup();

    return true;
}


bool
DialogSettings::read()
{
    QSettings settings;
    settings.beginGroup(m_groupName);
    m_parameters.saveParameters = settings.value("save_parameters").isValid() ? settings.value("save_parameters").toBool() : false;
    m_parameters.checkROS2NameConform = settings.value("check_ros2_naming_convention").isValid() ?
                                        settings.value("check_ros2_naming_convention").toBool() :
                                        false;
    settings.endGroup();

    return true;
}
