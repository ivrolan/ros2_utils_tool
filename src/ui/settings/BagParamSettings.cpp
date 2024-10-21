#include "BagParamSettings.hpp"

#include "UtilsSettings.hpp"

#include <QSettings>
BagParamSettings::BagParamSettings(Utils::UI::BagParameters& bagParameters,
                                   const QString&            groupName) :
    VideoParamSettings(bagParameters, groupName), m_bagParameters(bagParameters)
{
    read();
}


void
BagParamSettings::write()
{
    VideoParamSettings::write();

    QSettings settings;

    settings.beginGroup(m_groupName);
    setSettingsParameter(settings, m_bagParameters.useCDRForSerialization, "cdr");
    settings.endGroup();
}


bool
BagParamSettings::read()
{
    if (!VideoParamSettings::read()) {
        return false;
    }

    QSettings settings;
    settings.beginGroup(m_groupName);
    m_bagParameters.useCDRForSerialization = settings.value("cdr").isValid() ? settings.value("cdr").toBool() : false;
    settings.endGroup();

    return true;
}
