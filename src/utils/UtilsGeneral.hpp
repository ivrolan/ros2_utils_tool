#pragma once

#include <string>

// Util functions for general, non-specific contents
namespace UtilsGeneral
{
// Draws a small progress string in the following format:
// ############################--------------------
// 50 charactes, # shows the progress
[[nodiscard]] std::string
drawProgressString(int progress);
}
