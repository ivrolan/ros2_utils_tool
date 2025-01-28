#pragma once

#include "UtilsUI.hpp"

#include <QPointer>
#include <QWidget>

class QLabel;
class QPushButton;
class QToolButton;
class QVBoxLayout;

// The starting widget showing all possible ui tools
class StartWidget : public QWidget
{
    Q_OBJECT
public:
    explicit
    StartWidget(Utils::UI::DialogParameters& dialogParameters,
                QWidget*                     parent = 0);

signals:
    void
    toolRequested(int id);

private slots:
    void
    openSettingsDialog();

private:
    void
    replaceWidgets(QWidget* fromWidget,
                   QWidget* toWidget,
                   bool     otherItemVisibility);

    QPointer<QToolButton>
    createToolButton(const QString& buttonText);

    void
    setButtonIcons();

    bool
    event(QEvent *event) override;

private:
    QPointer<QPushButton> m_settingsButton;

    QPointer<QToolButton> m_bagToolsButton;
    QPointer<QToolButton> m_publishingToolsButton;

    QPointer<QToolButton> m_editROSBagButton;
    QPointer<QToolButton> m_bagInfoButton;
    QPointer<QToolButton> m_bagToVideoPushButton;
    QPointer<QToolButton> m_videoToBagPushButton;
    QPointer<QToolButton> m_bagToImagesPushButton;
    QPointer<QToolButton> m_dummyBagButton;

    QPointer<QToolButton> m_publishVideoButton;
    QPointer<QToolButton> m_publishImagesButton;

    QPointer<QVBoxLayout> m_mainLayout;

    QPointer<QPushButton> m_backButton;
    QPointer<QLabel> m_versionLabel;

    Utils::UI::DialogParameters& m_dialogParameters;

    bool m_isBagToolsWidgetSelected;
};
