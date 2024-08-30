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
    setBagToVideoWidget();

    void
    setVideoToBagWidget();

    void
    setBagToImagesWidget();

    void
    setProgressWidget(const Utils::UI::VideoParameters& videoParameters,
                      bool                              isEncoding);

    void
    setProgressWidget();

private:
    void
    closeEvent(QCloseEvent *event) override;

private:
    Utils::UI::VideoParameters m_parametersBagToVideo;
    Utils::UI::VideoParameters m_parametersVideoToBag;
    Utils::UI::ImageParameters m_parametersBagToImages;

    // We only need this one for encoding a video, but not for writing to a bag file
    QString m_encodingFormat = "mp4";
};
