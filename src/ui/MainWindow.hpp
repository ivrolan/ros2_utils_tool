#pragma once

#include "UtilsUI.hpp"

#include <QMainWindow>

/**
 * @brief Main window displaying the main user interface
 */
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow();

private slots:
    void
    setStartWidget();

    void
    setConfigWidget(int mode);

    void
    setProgressWidget(int mode);

private:
    void
    closeEvent(QCloseEvent *event) override;

private:
    Utils::UI::VideoInputParameters m_parametersBagToVideo;
    Utils::UI::BagInputParameters m_parametersVideoToBag;
    Utils::UI::ImageInputParameters m_parametersBagToImages;
    Utils::UI::DummyBagInputParameters m_dummyBagParameters;
    Utils::UI::EditBagInputParameters m_editBagParameters;

    Utils::UI::PublishParameters m_publishParametersVideo;
    Utils::UI::PublishParameters m_publishParametersImages;

    Utils::UI::DialogParameters m_dialogParameters;

    bool m_saveParametersOnChange;

    static constexpr int DEFAULT_WIDTH = 450;
    static constexpr int DEFAULT_HEIGHT = 600;
};
