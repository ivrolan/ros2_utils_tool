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
    connect(startWidget, &StartWidget::bagToVideoRequested, this, [this] {
        setConfigWidget(0);
    });
    connect(startWidget, &StartWidget::videoToBagRequested, this, [this] {
        setConfigWidget(1);
    });
    connect(startWidget, &StartWidget::bagToImagesRequested, this, [this] {
        setConfigWidget(2);
    });
    setCentralWidget(startWidget);
}


void
MainWindow::setConfigWidget(int mode)
{
    QPointer<BasicConfigWidget> basicConfigWidget;
    switch (mode) {
    case 0:
        basicConfigWidget = new BagToVideoWidget(m_parametersBagToVideo, m_encodingFormat);
        break;
    case 1:
        basicConfigWidget = new VideoToBagWidget(m_parametersVideoToBag);
        break;
    case 2:
        basicConfigWidget = new BagToImagesWidget(m_parametersBagToImages);
        break;
    }
    setCentralWidget(basicConfigWidget);

    connect(basicConfigWidget, &BasicConfigWidget::back, this, &MainWindow::setStartWidget);
    connect(basicConfigWidget, &BasicConfigWidget::okPressed, this, [this, mode] {
        setProgressWidget(mode);
    });
}


void
MainWindow::setProgressWidget(int mode)
{
    QPointer<BasicProgressWidget> basicProgressWidget;
    switch (mode) {
    case 0:
    case 1:
        basicProgressWidget = new VideoProgressWidget(mode == 0 ? m_parametersBagToVideo : m_parametersVideoToBag, mode == 0);
        break;
    case 2:
        basicProgressWidget = new ImagesProgressWidget(m_parametersBagToImages);
        break;
    }
    setCentralWidget(basicProgressWidget);

    connect(basicProgressWidget, &BasicProgressWidget::progressStopped, this, [this, mode] {
        setConfigWidget(mode);
    });
    connect(basicProgressWidget, &BasicProgressWidget::finished, this, &MainWindow::setStartWidget);

    basicProgressWidget->startThread();
}


void
MainWindow::closeEvent(QCloseEvent *event)
{
    std::raise(SIGINT);
    event->accept();
}
