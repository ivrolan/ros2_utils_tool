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
    setProgressWidget(const QString bagDirectory,
                      const QString topicName,
                      const QString vidDirectory,
                      bool          useHardwareAcceleration,
                      bool          useEncode);

private:
    void
    closeEvent(QCloseEvent *event) override;

private:
    Utils::UI::WidgetParameters m_parametersBagToVideo;
    Utils::UI::WidgetParameters m_parametersVideoToBag;
};
