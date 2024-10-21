#include "UtilsSettings.hpp"

#include <QSettings>

namespace Utils::Settings
{
bool
readAreParametersSaved()
{
    QSettings settings;
    return settings.value("save").isValid() ? settings.value("save").toBool() : false;
}
}
