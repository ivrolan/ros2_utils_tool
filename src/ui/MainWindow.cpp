#include "MainWindow.hpp"

#include "BagToVideoWidget.hpp"
#include "ProgressWidget.hpp"
#include "StartWidget.hpp"
#include "VideoToBagWidget.hpp"

#include <QCloseEvent>

#include "rclcpp/rclcpp.hpp"

MainWindow::MainWindow()
{
    setWindowTitle("ROS Tools");

    setStartWidget();

    resize(450, 450);
}


void
MainWindow::setStartWidget()
{
    auto* const startWidget = new StartWidget;
    connect(startWidget, &StartWidget::bagToVideoRequested, this, &MainWindow::setBagToVideoWidget);
    connect(startWidget, &StartWidget::videoToBagRequested, this, &MainWindow::setVideoToBagWidget);

    setCentralWidget(startWidget);
}


void
MainWindow::setBagToVideoWidget()
{
    auto* const bagToVideoWidget = new BagToVideoWidget(m_parametersBagToVideo);
    setCentralWidget(bagToVideoWidget);

    connect(bagToVideoWidget, &BagToVideoWidget::back, this, &MainWindow::setStartWidget);
    connect(bagToVideoWidget, &BagToVideoWidget::parametersSet, this, [this] (const QString& bagDirectory, const QString& topicName,
                                                                              const QString& vidDirectory, bool useHardwareAcceleration) {
        m_parametersBagToVideo.bagDirectory = bagDirectory;
        m_parametersBagToVideo.vidDirectory = vidDirectory;
        m_parametersBagToVideo.topicName = topicName;
        m_parametersBagToVideo.useHardwareAcceleration = useHardwareAcceleration;
        setProgressWidget(bagDirectory, topicName, vidDirectory, useHardwareAcceleration, true);
    });
}


void
MainWindow::setVideoToBagWidget()
{
    auto* const videoToBagWidget = new VideoToBagWidget(m_parametersVideoToBag);
    setCentralWidget(videoToBagWidget);

    connect(videoToBagWidget, &VideoToBagWidget::back, this, &MainWindow::setStartWidget);
    connect(videoToBagWidget, &VideoToBagWidget::parametersSet, this, [this] (const QString& vidDirectory, const QString& bagDirectory,
                                                                              const QString& topicName, bool useHardwareAcceleration) {
        m_parametersVideoToBag.bagDirectory = bagDirectory;
        m_parametersVideoToBag.vidDirectory = vidDirectory;
        m_parametersVideoToBag.topicName = topicName;
        m_parametersVideoToBag.useHardwareAcceleration = useHardwareAcceleration;
        setProgressWidget(bagDirectory, topicName, vidDirectory, useHardwareAcceleration, false);
    });
}


void
MainWindow::setProgressWidget(const QString bagDirectory, const QString topicName, const QString vidDirectory,
                              bool useHardwareAcceleration, bool useEncode)
{
    auto* const progressWidget = new ProgressWidget(bagDirectory, topicName, vidDirectory, useHardwareAcceleration, useEncode);
    setCentralWidget(progressWidget);

    connect(progressWidget, &ProgressWidget::encodingStopped, this, useEncode ? &MainWindow::setBagToVideoWidget : &MainWindow::setVideoToBagWidget);
    connect(progressWidget, &ProgressWidget::finished, this, &MainWindow::setStartWidget);
}


void
MainWindow::closeEvent(QCloseEvent *event)
{
    rclcpp::shutdown();
    event->accept();
}
