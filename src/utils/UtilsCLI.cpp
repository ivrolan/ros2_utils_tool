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
}
