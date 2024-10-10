#pragma once

#include "BasicInputWidget.hpp"
#include "UtilsUI.hpp"

class QCheckBox;
class QComboBox;
class QFormLayout;
class QLabel;
class QLineEdit;
class QSlider;

/**
 * @brief The widget used to manage writing images out of a ROS bag
 */
class BagToImagesWidget : public BasicInputWidget
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
    adjustWidgetsToChangedFormat(const QString& text);

    void
    okButtonPressed();

private:
    QPointer<QLineEdit> m_bagNameLineEdit;
    QPointer<QComboBox> m_topicNameComboBox;
    QPointer<QLineEdit> m_imagesNameLineEdit;
    QPointer<QSlider> m_slider;
    QPointer<QLabel> m_formLayoutSliderLabel;
    QPointer<QCheckBox> m_useBWCheckBox;
    QPointer<QCheckBox> m_optimizeBilevelCheckBox;

    QPointer<QFormLayout> m_advancedOptionsFormLayout;

    Utils::UI::ImageParameters& m_imageParameters;

    bool m_fileDialogOpened = false;
};
