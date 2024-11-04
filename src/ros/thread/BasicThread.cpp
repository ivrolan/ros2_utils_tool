#include "BasicThread.hpp"

BasicThread::BasicThread(const QString& sourceDirectory,
                         const QString& topicName,
                         QObject*       parent) :
    QThread(parent), m_sourceDirectory(sourceDirectory.toStdString()), m_topicName(topicName.toStdString())
{
}
