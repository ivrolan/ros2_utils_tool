#include "MainWindow.hpp"

#include "BagToImagesWidget.hpp"
#include "BagToVideoWidget.hpp"
#include "ImagesProgressWidget.hpp"
#include "StartWidget.hpp"
#include "VideoToBagWidget.hpp"
#include "VideoProgressWidget.hpp"

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
    connect(startWidget, &StartWidget::bagToImagesRequested, this, &MainWindow::setBagToImagesWidget);

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
        m_parametersBagToVideo.videoDirectory = vidDirectory;
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
        m_parametersVideoToBag.videoDirectory = vidDirectory;
        m_parametersVideoToBag.topicName = topicName;
        m_parametersVideoToBag.useHardwareAcceleration = useHardwareAcceleration;
        setProgressWidget(bagDirectory, topicName, vidDirectory, useHardwareAcceleration, false);
    });
}


void
MainWindow::setBagToImagesWidget()
{
    auto* const bagToImagesWidget = new BagToImagesWidget(m_parametersBagToImages);
    setCentralWidget(bagToImagesWidget);

    connect(bagToImagesWidget, &BagToImagesWidget::back, this, &MainWindow::setStartWidget);
    connect(bagToImagesWidget, &BagToImagesWidget::parametersSet, this, [this] (const QString& bagDirectory, const QString& topicName,
                                                                                const QString& imagesDirectory, const QString& format,
                                                                                const int quality) {
        m_parametersBagToImages.bagDirectory = bagDirectory;
        m_parametersBagToImages.imagesDirectory = imagesDirectory;
        m_parametersBagToImages.topicName = topicName;
        m_parametersBagToImages.format = format;
        m_parametersBagToImages.quality = quality;
        setProgressWidget(bagDirectory, topicName, imagesDirectory, format, quality);
    });
}


void
MainWindow::setProgressWidget(const QString& bagDirectory, const QString& topicName, const QString& vidDirectory,
                              bool useHardwareAcceleration, bool isEncoding)
{
    auto* const progressWidget = new VideoProgressWidget(bagDirectory, topicName, vidDirectory, useHardwareAcceleration, isEncoding);
    setCentralWidget(progressWidget);

    connect(progressWidget, &BasicProgressWidget::progressStopped, this, [this, isEncoding] {
        isEncoding ? setBagToVideoWidget() : setVideoToBagWidget();
    });
    connect(progressWidget, &BasicProgressWidget::finished, this, &MainWindow::setStartWidget);

    progressWidget->startThread();
}


void
MainWindow::setProgressWidget(const QString& bagDirectory, const QString& topicName, const QString& vidDirectory,
                              const QString& format, int compressionLevel)
{
    auto* const progressWidget = new ImagesProgressWidget(bagDirectory, topicName, vidDirectory, format, compressionLevel);
    setCentralWidget(progressWidget);

    connect(progressWidget, &BasicProgressWidget::progressStopped, this, &MainWindow::setBagToImagesWidget);
    connect(progressWidget, &BasicProgressWidget::finished, this, &MainWindow::setStartWidget);

    progressWidget->startThread();
}


void
MainWindow::closeEvent(QCloseEvent *event)
{
    rclcpp::shutdown();
    event->accept();
}
