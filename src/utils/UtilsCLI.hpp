#pragma once

#include <QString>
#include <QStringList>

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
}
