#include "BasicParamSettings.hpp"

#include "UtilsSettings.hpp"

BasicParamSettings::BasicParamSettings(Utils::UI::BasicParameters& basicParameters, const QString& groupName) :
    m_groupName(groupName), m_basicParameters(basicParameters)
{
}


void
BasicParamSettings::write()
{
    QSettings settings;

    settings.beginGroup(m_groupName);
    setSettingsParameter(settings, m_basicParameters.sourceDirectory, "source_dir");
    setSettingsParameter(settings, m_basicParameters.topicName, "topic_name");
    settings.endGroup();
}


bool
BasicParamSettings::read()
{
    if (const auto parametersSaved = Utils::Settings::readAreParametersSaved(); !parametersSaved) {
        return false;
    }

    QSettings settings;
    settings.beginGroup(m_groupName);
    m_basicParameters.sourceDirectory = settings.value("source_dir").isValid() ? settings.value("source_dir").toString() : "";
    m_basicParameters.topicName = settings.value("topic_name").isValid() ? settings.value("topic_name").toString() : "";
    settings.endGroup();

    return true;
}
