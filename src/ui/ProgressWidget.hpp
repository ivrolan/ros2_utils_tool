#pragma once

#include "UtilsUI.hpp"

#include <QPointer>
#include <QWidget>

class BasicThread;

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

private:
    QPointer<BasicThread> m_thread;

    int m_maximumCount;
};
