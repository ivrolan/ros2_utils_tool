#pragma once

#include "BasicConfigWidget.hpp"
#include "UtilsUI.hpp"

class QComboBox;
class QLabel;
class QLineEdit;
class QSlider;

/**
 * @brief The widget used to manage writing images out of a ROS bag
 */
class BagToImagesWidget : public BasicConfigWidget
{
    Q_OBJECT

public:
    BagToImagesWidget(Utils::UI::ImageParameters& imageParameters,
                      QWidget*                    parent = 0);

private slots:
    void
    searchButtonPressed();

    void
    imagesLocationButtonPressed();

    void
    adjustSliderToChangedFormat(const QString& text);

    void
    okButtonPressed();

private:
    QPointer<QLineEdit> m_bagNameLineEdit;
    QPointer<QComboBox> m_topicNameComboBox;
    QPointer<QLineEdit> m_imagesNameLineEdit;
    QPointer<QComboBox> m_formatComboBox;
    QPointer<QSlider> m_slider;
    QPointer<QLabel> m_formLayoutSliderLabel;

    Utils::UI::ImageParameters& m_imageParameters;

    bool m_fileDialogOpened = false;
};
