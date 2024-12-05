#include "EditBagParamSettings.hpp"

EditBagParamSettings::EditBagParamSettings(Utils::UI::EditBagParameters& editBagParameters,
                                           const QString&                groupName) :
    AdvancedParamSettings(editBagParameters, groupName), m_editBagParameters(editBagParameters)
{
    read();
}


bool
EditBagParamSettings::write()
{
    if (!AdvancedParamSettings::write()) {
        return false;
    }

    QSettings settings;
    settings.beginGroup(m_groupName);

    settings.beginWriteArray("topics");
    for (auto i = 0; i < m_editBagParameters.topics.size(); ++i) {
        settings.setArrayIndex(i);
        setSettingsParameter(settings, m_editBagParameters.topics.at(i).renamedTopicName, "renamed_name");
        setSettingsParameter(settings, m_editBagParameters.topics.at(i).originalTopicName, "original_name");
        setSettingsParameter(settings, m_editBagParameters.topics.at(i).lowerBoundary, "lower_boundary");
        setSettingsParameter(settings, m_editBagParameters.topics.at(i).upperBoundary, "upper_boundary");
        setSettingsParameter(settings, m_editBagParameters.topics.at(i).isSelected, "is_selected");
    }
    settings.endArray();

    settings.endGroup();
    return true;
}


bool
EditBagParamSettings::read()
{
    if (!AdvancedParamSettings::read()) {
        return false;
    }

    QSettings settings;
    settings.beginGroup(m_groupName);
    m_editBagParameters.topics.clear();

    const auto size = settings.beginReadArray("topics");
    for (auto i = 0; i < size; ++i) {
        settings.setArrayIndex(i);
        m_editBagParameters.topics.append({ settings.value("renamed_name").toString(), settings.value("original_name").toString(),
                                            settings.value("lower_boundary").value<size_t>(), settings.value("upper_boundary").value<size_t>(),
                                            settings.value("is_selected").toBool() });
    }
    settings.endArray();

    settings.endGroup();
    return true;
}
