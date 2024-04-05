#pragma once

#include <QPointer>
#include <QWidget>

class EncodingThread;

/**
 * @brief The widget showing the progress of videos encoded out of a ROSBag
 */
class ProgressWidget : public QWidget
{
    Q_OBJECT

public:
    ProgressWidget(const QString& bagDirectory,
                   const QString& topicName,
                   const QString& vidDirectory,
                   bool           useHardwareAcceleration,
                   QWidget*       parent = 0);

    ~ProgressWidget();

signals:
    void
    encodingStopped();

    void
    finished();

private:
    QPointer<EncodingThread> m_encodingThread;

    int m_messageCount;
};
