#pragma once

#include <QPointer>
#include <QWidget>

class QPushButton;
class QToolButton;

// The starting widget showing all possible ui tools
class StartWidget : public QWidget
{
    Q_OBJECT
public:
    explicit
    StartWidget(QWidget* parent = 0);

signals:
    void
    functionRequested(int id);

private slots:
    void
    openSettingsDialog();

private:
    QPointer<QToolButton>
    createToolButton(const QString& buttonText);

    void
    setButtonIcons();

    bool
    event(QEvent *event) override;

private:
    QPointer<QPushButton> m_settingsButton;

    QPointer<QToolButton> m_bagToVideoPushButton;
    QPointer<QToolButton> m_bagToImagesPushButton;
    QPointer<QToolButton> m_videoToBagPushButton;
    QPointer<QToolButton> m_dummyBagButton;
    QPointer<QToolButton> m_editROSBagButton;
    QPointer<QToolButton> m_bagInfoButton;
};
