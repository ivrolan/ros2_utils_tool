#include "VideoParamSettings.hpp"

VideoParamSettings::VideoParamSettings(Utils::UI::VideoParameters& videoParameters, const QString& groupName) :
    AdvancedParamSettings(videoParameters, groupName), m_videoParameters(videoParameters)
{
    read();
}


bool
VideoParamSettings::write()
{
    if (!AdvancedParamSettings::write()) {
        return false;
    }

    QSettings settings;
    settings.beginGroup(m_groupName);
    setSettingsParameter(settings, m_videoParameters.fps, "fps");
    setSettingsParameter(settings, m_videoParameters.useHardwareAcceleration, "hw_acc");
    setSettingsParameter(settings, m_videoParameters.useBWImages, "bw_images");
    settings.endGroup();

    return true;
}


bool
VideoParamSettings::read()
{
    if (!AdvancedParamSettings::read()) {
        return false;
    }

    QSettings settings;
    settings.beginGroup(m_groupName);
    m_videoParameters.fps = settings.value("fps").isValid() ? settings.value("fps").toInt() : 30;
    m_videoParameters.useHardwareAcceleration = settings.value("hw_acc").isValid() ? settings.value("hw_acc").toBool() : false;
    m_videoParameters.useBWImages = settings.value("bw_images").isValid() ? settings.value("bw_images").toBool() : false;
    settings.endGroup();

    return true;
}
