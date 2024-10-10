#include "UtilsCLI.hpp"

namespace Utils::CLI
{
bool
containsArguments(const QStringList& stringList, const QString& shortArg, const QString& longArg)
{
    return stringList.contains(shortArg) || stringList.contains(longArg);
}
}
