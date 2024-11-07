#pragma once

#include "BagParamSettings.hpp"
#include "BasicInputWidget.hpp"
#include "UtilsUI.hpp"

#include <QPointer>
#include <QWidget>

class QFormLayout;
class QLineEdit;
class QSpinBox;

/**
 * @brief The widget used to write a video file into a ROSBag
 */
class VideoToBagWidget : public BasicInputWidget
{
    Q_OBJECT

public:
    VideoToBagWidget(Utils::UI::BagParameters& bagParameters,
                     QWidget*                  parent = 0);

private slots:
    void
    searchButtonPressed();

    void
    bagLocationButtonPressed();

    void
    useCustomFPSCheckBoxPressed(int state);

    void
    okButtonPressed();

private:
    QPointer<QLineEdit> m_bagNameLineEdit;
    QPointer<QFormLayout> m_advancedOptionsFormLayout;
    QPointer<QSpinBox> m_fpsSpinBox;

    Utils::UI::BagParameters& m_bagParameters;

    BagParamSettings m_bagParamSettings;
};
