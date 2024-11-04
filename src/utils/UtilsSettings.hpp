#pragma once

#include "UtilsUI.hpp"

#include <QSettings>

// Util functions for all settings related stuff
namespace Utils::Settings
{
// Checks the parameters saved setting
[[nodiscard]] bool
readAreParametersSaved();
}
