#pragma once

#include <QThread>

// Encoding thread, encoding a video out of a ROSBag
class EncodingThread : public QThread {
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

signals:
    void
    calculatedTopicMessageCount(int count);

    void
    openingVideoWriterFailed();

    void
    encodingProgressChanged(int iteration,
                            int progress);

    void
    finished();

private:
    const QString m_bagDirectory;
    const QString m_topicName;
    const QString m_vidDirectory;

    bool m_useHardwareAcceleration;
};
