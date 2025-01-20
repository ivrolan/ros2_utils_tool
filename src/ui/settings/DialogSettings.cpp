#include "DialogSettings.hpp"

DialogSettings::DialogSettings(Utils::UI::DialogParameters& parameters, const QString& groupName) :
    BasicSettings(groupName), m_parameters(parameters)
{
    read();
}


// Make this static because we need to access the variable from many different places
// in the application without wanting to use this as extra dependency
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
    settings.endGroup();

    return true;
}


bool
DialogSettings::read()
{
    QSettings settings;
    settings.beginGroup(m_groupName);
    m_parameters.saveParameters = settings.value("save_parameters").isValid() ? settings.value("save_parameters").toBool() : false;
    settings.endGroup();

    return true;
}
