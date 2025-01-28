#pragma once

#include "BasicInputWidget.hpp"
#include "ImageInputSettings.hpp"
#include "UtilsUI.hpp"

class QCheckBox;
class QComboBox;
class QFormLayout;
class QLineEdit;
class QSlider;

/**
 * @brief The widget used to manage writing images out of a ROS bag
 */
class BagToImagesWidget : public BasicInputWidget
{
    Q_OBJECT

public:
    BagToImagesWidget(Utils::UI::ImageInputParameters& parameters,
                      QWidget*                         parent = 0);

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
    QPointer<QComboBox> m_topicNameComboBox;
    QPointer<QLineEdit> m_imagesNameLineEdit;
    QPointer<QSlider> m_slider;
    QPointer<QCheckBox> m_optimizeOrBilevelCheckBox;

    QPointer<QFormLayout> m_advancedOptionsFormLayout;

    Utils::UI::ImageInputParameters& m_parameters;

    ImageInputSettings m_settings;

    bool m_fileDialogOpened = false;
};
