#pragma once

#include <QString>
#include <QStringList>

#include <string>

// Util functions for the cli tools
namespace Utils::CLI
{
// Checks if the stringlist contains either the short or long argument and returns the index
bool
containsArguments(const QStringList& stringList,
                  const QString&     shortArg,
                  const QString&     longArg);

// Get the index of arguments in a stringlist
int
getArgumentsIndex(const QStringList& stringList,
                  const QString&     shortArg,
                  const QString&     longArg);

// Check if an argument is correctly called
bool
checkArgumentValidity(const QStringList& stringList,
                      const QString&     shortArg,
                      const QString&     longArg,
                      int&               parameter,
                      int                lowerRange,
                      int                higherRange);

bool
continueForExistingSourceDir(const std::string& message);

// Draws a small progress string in the following format:
// ############################--------------------
// 50 charactes, # shows the progress
[[nodiscard]] std::string
drawProgressString(int progress);
}
