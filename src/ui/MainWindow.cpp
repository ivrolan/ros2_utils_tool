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
    auto* const bagToVideoWidget = new BagToVideoWidget(m_parametersBagToVideo, m_encodingFormat);
    setCentralWidget(bagToVideoWidget);

    connect(bagToVideoWidget, &BagToVideoWidget::back, this, &MainWindow::setStartWidget);
    connect(bagToVideoWidget, &BagToVideoWidget::okPressed, this, [this] {
        setProgressWidget(m_parametersBagToVideo, true);
    });
}


void
MainWindow::setVideoToBagWidget()
{
    auto* const videoToBagWidget = new VideoToBagWidget(m_parametersVideoToBag);
    setCentralWidget(videoToBagWidget);

    connect(videoToBagWidget, &VideoToBagWidget::back, this, &MainWindow::setStartWidget);
    connect(videoToBagWidget, &VideoToBagWidget::okPressed, this, [this] {
        setProgressWidget(m_parametersVideoToBag, false);
    });
}


void
MainWindow::setBagToImagesWidget()
{
    auto* const bagToImagesWidget = new BagToImagesWidget(m_parametersBagToImages);
    setCentralWidget(bagToImagesWidget);

    connect(bagToImagesWidget, &BagToImagesWidget::back, this, &MainWindow::setStartWidget);
    connect(bagToImagesWidget, &BagToImagesWidget::okPressed, this, [this] {
        setProgressWidget();
    });
}


void
MainWindow::setProgressWidget(const Utils::UI::VideoParameters& videoParameters, bool isEncoding)
{
    auto* const progressWidget = new VideoProgressWidget(videoParameters, isEncoding);
    setCentralWidget(progressWidget);

    connect(progressWidget, &BasicProgressWidget::progressStopped, this, [this, isEncoding] {
        isEncoding ? setBagToVideoWidget() : setVideoToBagWidget();
    });
    connect(progressWidget, &BasicProgressWidget::finished, this, &MainWindow::setStartWidget);

    progressWidget->startThread();
}


void
MainWindow::setProgressWidget()
{
    auto* const progressWidget = new ImagesProgressWidget(m_parametersBagToImages);
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
