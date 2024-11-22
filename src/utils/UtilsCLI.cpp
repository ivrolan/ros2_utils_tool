#include "UtilsCLI.hpp"

namespace Utils::CLI
{
bool
containsArguments(const QStringList& stringList, const QString& shortArg, const QString& longArg)
{
    return stringList.contains(shortArg) || stringList.contains(longArg);
}


int
getArgumentsIndex(const QStringList& stringList, const QString& shortArg, const QString& longArg)
{
    return std::max(stringList.lastIndexOf(shortArg), stringList.lastIndexOf(longArg));
}


bool
checkArgumentValidity(const QStringList& stringList, const QString& shortArg, const QString& longArg, int& parameter, int lowerRange, int higherRange)
{
    if (!containsArguments(stringList, shortArg, longArg)) {
        return true;
    }

    const auto argumentIndex = std::max(stringList.lastIndexOf(shortArg), stringList.lastIndexOf(longArg));
    if (stringList.at(argumentIndex) == stringList.last()) {
        return false;
    }
    if (parameter = stringList.at(argumentIndex + 1).toInt(); parameter < lowerRange || parameter > higherRange) {
        return false;
    }

    return true;
}
}
