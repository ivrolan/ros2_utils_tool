#include "BasicThread.hpp"

BasicThread::BasicThread(const QString& bagDirectory,
                         const QString& topicName,
                         QObject*       parent) :
    QThread(parent), m_bagDirectory(bagDirectory.toStdString()), m_topicName(topicName.toStdString())
{
}
