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
    setProgressWidget(const QString& bagDirectory,
                      const QString& topicName,
                      const QString& vidDirectory,
                      bool           useHardwareAcceleration,
                      bool           isEncoding);

    void
    setProgressWidget(const QString& bagDirectory,
                      const QString& topicName,
                      const QString& vidDirectory,
                      const QString& format,
                      int            quality);

private:
    void
    closeEvent(QCloseEvent *event) override;

private:
    Utils::UI::VideoParameters m_parametersBagToVideo;
    Utils::UI::VideoParameters m_parametersVideoToBag;
    Utils::UI::ImageParameters m_parametersBagToImages;
};
