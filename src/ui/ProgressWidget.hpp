#pragma once

#include "UtilsUI.hpp"

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
class ProgressWidget : public QWidget
{
    Q_OBJECT

public:
    ProgressWidget(const QString&              headerPixmapLabelTextBlack,
                   const QString&              headerPixmapLabelTextWhite,
                   const QString&              headerLabelText,
                   Utils::UI::BasicParameters& parameters,
                   const int                   threadTypeId,
                   QWidget*                    parent = 0);

    ~ProgressWidget();

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
