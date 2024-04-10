#pragma once

#include <QThread>

// Basic thread class, overridden by custom classes
class BasicThread : public QThread {
    Q_OBJECT
public:
    explicit
    BasicThread(const QString& bagDirectory,
                const QString& topicName,
                const QString& vidDirectory,
                bool           useHardwareAcceleration,
                QObject*       parent = nullptr);

signals:
    void
    calculatedMaximumInstances(int count);

    void
    openingCVInstanceFailed();

    void
    progressChanged(int iteration,
                    int progress);

    void
    finished();

protected:
    const QString m_bagDirectory;
    const QString m_topicName;
    const QString m_vidDirectory;
    const bool m_useHardwareAcceleration;
};
