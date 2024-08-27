#pragma once

#include "BasicThread.hpp"

// Encoding thread, encoding a video out of a ROSBag
class WriteToImageThread : public BasicThread {
    Q_OBJECT
public:
    explicit
    WriteToImageThread(const QString& bagDirectory,
                       const QString& topicName,
                       const QString& imagesDirectory,
                       const QString& format,
                       const int      quality,
                       QObject*       parent = nullptr);

    void
    run() override;

private:
    const QString m_imagesDirectory;
    const QString m_format;
    int m_quality;
};
