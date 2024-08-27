#pragma once

#include "BasicThread.hpp"

// Encoding thread, encoding a video out of a ROSBag
class WriteToBagThread : public BasicThread {
    Q_OBJECT

public:
    explicit
    WriteToBagThread(const QString& bagDirectory,
                     const QString& topicName,
                     const QString& videoDirectory,
                     bool           useHardwareAcceleration,
                     QObject*       parent = nullptr);

    void
    run() override;

private:
    const QString m_videoDirectory;
    const bool m_useHardwareAcceleration;
};
