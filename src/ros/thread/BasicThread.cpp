#include "BasicThread.hpp"

BasicThread::BasicThread(const QString& bagDirectory,
                         const QString& topicName,
                         const QString& vidDirectory,
                         bool           useHardwareAcceleration,
                         QObject*       parent) :
    QThread(parent), m_bagDirectory(bagDirectory), m_topicName(topicName), m_vidDirectory(vidDirectory),
    m_useHardwareAcceleration(useHardwareAcceleration)
{
}
