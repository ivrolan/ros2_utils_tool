#include "MainWindow.hpp"

#include "BagInfoWidget.hpp"
#include "BagToImagesWidget.hpp"
#include "BagToVideoWidget.hpp"
#include "DummyBagWidget.hpp"
#include "DummyBagProgressWidget.hpp"
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
    connect(startWidget, &StartWidget::functionRequested, this, &MainWindow::setConfigWidget);
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
        basicConfigWidget = new BagToImagesWidget(m_parametersBagToImages);
        break;
    case 2:
        basicConfigWidget = new VideoToBagWidget(m_parametersVideoToBag);
        break;
    case 3:
        basicConfigWidget = new DummyBagWidget(m_dummyBagParameters);
        break;
    case 4:
        basicConfigWidget = new BagInfoWidget(m_basicInfoParameters);
        break;
    }
    setCentralWidget(basicConfigWidget);

    connect(basicConfigWidget, &BasicConfigWidget::back, this, &MainWindow::setStartWidget);
    connect(basicConfigWidget, &BasicConfigWidget::okPressed, this, [this, mode] {
        if (mode == 4) {
            setStartWidget();
        }
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
    case 3:
        basicProgressWidget = new DummyBagProgressWidget(m_dummyBagParameters);
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
