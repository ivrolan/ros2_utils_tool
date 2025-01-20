#include "DummyBagInputSettings.hpp"

DummyBagInputSettings::DummyBagInputSettings(Utils::UI::DummyBagInputParameters& parameters, const QString& groupName) :
    InputSettings(parameters, groupName), m_parameters(parameters)
{
    read();
}


bool
DummyBagInputSettings::write()
{
    if (!InputSettings::write()) {
        return false;
    }

    QSettings settings;
    settings.beginGroup(m_groupName);

    settings.beginWriteArray("topics");
    for (auto i = 0; i < m_parameters.topics.size(); ++i) {
        settings.setArrayIndex(i);
        settings.setValue("type", m_parameters.topics.at(i).type);
        settings.setValue("name", m_parameters.topics.at(i).name);
    }
    settings.endArray();

    setSettingsParameter(settings, m_parameters.messageCount, "msg_count");
    settings.endGroup();

    return true;
}


bool
DummyBagInputSettings::read()
{
    if (!InputSettings::read()) {
        return false;
    }

    QSettings settings;

    settings.beginGroup(m_groupName);
    m_parameters.topics.clear();

    const auto size = settings.beginReadArray("topics");
    for (auto i = 0; i < size; ++i) {
        settings.setArrayIndex(i);
        m_parameters.topics.append({ settings.value("type").toString(), settings.value("name").toString() });
    }
    settings.endArray();

    m_parameters.messageCount = settings.value("msg_count").isValid() ? settings.value("msg_count").toInt() : 100;
    settings.endGroup();

    return true;
}
