#include "ImageParamSettings.hpp"

#include <QSettings>

ImageParamSettings::ImageParamSettings(Utils::UI::ImageParameters& imageParameters, const QString& groupName) :
    AdvancedParamSettings(imageParameters, groupName), m_imageParameters(imageParameters)
{
    read();
}


void
ImageParamSettings::write()
{
    AdvancedParamSettings::write();

    QSettings settings;

    settings.beginGroup(m_groupName);
    setSettingsParameter(settings, m_imageParameters.format, "format");
    setSettingsParameter(settings, m_imageParameters.quality, "quality");
    setSettingsParameter(settings, m_imageParameters.useBWImages, "bw_images");
    setSettingsParameter(settings, m_imageParameters.jpgOptimize, "jpg_optimize");
    setSettingsParameter(settings, m_imageParameters.pngBilevel, "png_bilevel");
    settings.endGroup();
}


bool
ImageParamSettings::read()
{
    if (!AdvancedParamSettings::read()) {
        return false;
    }

    QSettings settings;
    settings.beginGroup(m_groupName);
    m_imageParameters.format = settings.value("format").isValid() ? settings.value("format").toString() : "jpg";
    m_imageParameters.quality = settings.value("quality").isValid() ? settings.value("quality").toInt() : 8;
    m_imageParameters.useBWImages = settings.value("bw_images").isValid() ? settings.value("bw_images").toBool() : false;
    m_imageParameters.jpgOptimize = settings.value("jpg_optimize").isValid() ? settings.value("jpg_optimize").toBool() : false;
    m_imageParameters.pngBilevel = settings.value("png_bilevel").isValid() ? settings.value("png_bilevel").toBool() : false;
    settings.endGroup();

    return true;
}
