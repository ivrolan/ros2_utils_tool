#pragma once

#include "UtilsUI.hpp"

#include <QPointer>
#include <QWidget>

class QCheckBox;
class QHBoxLayout;
class QLabel;
class QLineEdit;
class QPushButton;

/**
 * @brief The widget used to write a video file into a ROSBag
 */
class VideoToBagWidget : public QWidget
{
    Q_OBJECT

public:
    VideoToBagWidget(Utils::UI::VideoParameters& videoParameters,
                     QWidget*                    parent = 0);

signals:
    void
    back();

    void
    okPressed();

private slots:
    void
    searchButtonPressed();

    void
    bagLocationButtonPressed();

    void
    okButtonPressed();

private:
    void
    enableOkButton();

    void
    setPixmapLabelIcon();

    bool
    event(QEvent *event) override;

private:
    QPointer<QLabel> m_headerPixmapLabel;
    QPointer<QLineEdit> m_videoNameLineEdit;
    QPointer<QLineEdit> m_bagNameLineEdit;
    QPointer<QLineEdit> m_topicNameLineEdit;
    QPointer<QCheckBox> m_useHardwareAccCheckBox;

    QPointer<QPushButton> m_okButton;
    Utils::UI::VideoParameters& m_videoParameters;
};
