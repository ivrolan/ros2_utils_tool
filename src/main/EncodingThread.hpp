#pragma once

#include "BasicThread.hpp"

// Encoding thread, encoding a video out of a ROSBag
class EncodingThread : public BasicThread {
    Q_OBJECT
public:
    explicit
    EncodingThread(const QString& bagDirectory,
                   const QString& topicName,
                   const QString& vidDirectory,
                   bool           useHardwareAcceleration,
                   QObject*       parent = nullptr);

    void
    run() override;
};
