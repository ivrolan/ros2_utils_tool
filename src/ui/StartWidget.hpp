#pragma once

#include <QPointer>
#include <QWidget>

class QToolButton;

// The starting widget showing the two basic options
class StartWidget : public QWidget
{
    Q_OBJECT
public:
    explicit
    StartWidget(QWidget* parent = 0);

signals:
    void
    bagToVideoRequested();

    void
    videoToBagRequested();

private:
    QPointer<QToolButton>
    createToolButton(const QString& buttonText);

    void
    setButtonIcons();

private:
    QPointer<QToolButton> m_bagToVideoPushButton;
    QPointer<QToolButton> m_videoToBagPushButton;
};
