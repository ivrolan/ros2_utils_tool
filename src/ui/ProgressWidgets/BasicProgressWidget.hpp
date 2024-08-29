#pragma once

#include <QLabel>
#include <QPointer>
#include <QWidget>

class BasicThread;

class QLabel;
class QProgressBar;
class QPushButton;

/**
 * @brief Base widget showing encoding or writing process
 */
class BasicProgressWidget : public QWidget
{
    Q_OBJECT

public:
    BasicProgressWidget(QWidget* parent = 0);

    ~BasicProgressWidget();

    void
    startThread();

signals:
    void
    progressStopped();

    void
    finished();

protected:
    void
    connectThread();

protected:
    QPointer<BasicThread> m_thread;

    QPointer<QLabel> m_headerLabel;
    QPointer<QLabel> m_headerPixmapLabel;

    int m_maximumCount;

private:
    QPointer<QLabel> m_progressLabel;
    QPointer<QProgressBar> m_progressBar;
    QPointer<QPushButton> m_cancelButton;
    QPointer<QPushButton> m_finishedButton;
};
