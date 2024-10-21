#include "DummyBagParamSettings.hpp"

DummyBagParamSettings::DummyBagParamSettings(Utils::UI::DummyBagParameters& dummyBagParameters, const QString& groupName) :
    BasicParamSettings(dummyBagParameters, groupName), m_dummyBagParameters(dummyBagParameters)
{
    read();
}


void
DummyBagParamSettings::write()
{
    BasicParamSettings::write();

    QSettings settings;
    settings.beginGroup(m_groupName);

    settings.beginWriteArray("topics");
    for (auto i = 0; i < m_dummyBagParameters.topics.size(); ++i) {
        settings.setArrayIndex(i);
        settings.setValue("type", m_dummyBagParameters.topics.at(i).type);
        settings.setValue("name", m_dummyBagParameters.topics.at(i).name);
    }
    settings.endArray();

    setSettingsParameter(settings, m_dummyBagParameters.messageCount, "msg_count");
    settings.endGroup();
}


bool
DummyBagParamSettings::read()
{
    if (!BasicParamSettings::read()) {
        return false;
    }

    QSettings settings;

    settings.beginGroup(m_groupName);
    m_dummyBagParameters.topics.clear();

    const auto size = settings.beginReadArray("topics");
    for (auto i = 0; i < size; ++i) {
        settings.setArrayIndex(i);
        m_dummyBagParameters.topics.append({ settings.value("type").toString(), settings.value("name").toString() });
    }
    settings.endArray();

    m_dummyBagParameters.messageCount = settings.value("msg_count").isValid() ? settings.value("msg_count").toInt() : 100;
    settings.endGroup();

    return true;
}
