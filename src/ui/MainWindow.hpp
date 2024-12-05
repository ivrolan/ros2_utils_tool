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
    Utils::UI::VideoParameters m_parametersBagToVideo;
    Utils::UI::BagParameters m_parametersVideoToBag;
    Utils::UI::ImageParameters m_parametersBagToImages;
    Utils::UI::DummyBagParameters m_dummyBagParameters;
    Utils::UI::EditBagParameters m_editBagParameters;
    Utils::UI::BasicParameters m_basicInfoParameters;

    bool m_saveParametersOnChange;

    static constexpr int DEFAULT_WIDTH = 450;
    static constexpr int DEFAULT_HEIGHT = 600;
};
