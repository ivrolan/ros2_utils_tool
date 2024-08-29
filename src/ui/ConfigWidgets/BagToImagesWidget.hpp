#pragma once

#include "UtilsUI.hpp"

#include <QPointer>
#include <QWidget>

class QComboBox;
class QHBoxLayout;
class QLabel;
class QLineEdit;
class QPushButton;
class QSlider;

/**
 * @brief The widget used to manage writing images out of a ROS bag
 */
class BagToImagesWidget : public QWidget
{
    Q_OBJECT

public:
    BagToImagesWidget(const Utils::UI::ImageParameters& imageParameters,
                      QWidget*                          parent = 0);

signals:
    void
    back();

    void
    parametersSet(const QString& bagDirectory,
                  const QString& topicName,
                  const QString& imagesDirectory,
                  const QString& format,
                  const int      quality);

private slots:
    void
    searchButtonPressed();

    void
    imagesLocationButtonPressed();

    void
    adjustSliderToChangedFormat(const QString& text);

    void
    formatComboBoxTextChanged(const QString& text);

    void
    okButtonPressed();

private:
    void
    setPixmapLabelIcon();

    bool
    event(QEvent *event) override;

private:
    QPointer<QLabel> m_headerPixmapLabel;
    QPointer<QLineEdit> m_bagNameLineEdit;
    QPointer<QComboBox> m_topicNameComboBox;
    QPointer<QLineEdit> m_imagesNameLineEdit;
    QPointer<QComboBox> m_formatComboBox;
    QPointer<QSlider> m_slider;
    QPointer<QLabel> m_formLayoutSliderLabel;

    QPointer<QPushButton> m_okButton;

    bool m_fileDialogOpened = false;
};
