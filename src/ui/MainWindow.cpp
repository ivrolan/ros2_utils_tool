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

#include <csignal>

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
    QPointer<BasicInputWidget> basicInputWidget;
    switch (mode) {
    case 0:
        basicInputWidget = new BagToVideoWidget(m_parametersBagToVideo, m_encodingFormat);
        break;
    case 1:
        basicInputWidget = new BagToImagesWidget(m_parametersBagToImages);
        break;
    case 2:
        basicInputWidget = new VideoToBagWidget(m_parametersVideoToBag);
        break;
    case 3:
        basicInputWidget = new DummyBagWidget(m_dummyBagParameters);
        break;
    case 4:
        basicInputWidget = new BagInfoWidget(m_basicInfoParameters);
        break;
    }
    setCentralWidget(basicInputWidget);

    connect(basicInputWidget, &BasicInputWidget::back, this, &MainWindow::setStartWidget);
    connect(basicInputWidget, &BasicInputWidget::okPressed, this, [this, mode] {
        setProgressWidget(mode);
    });
}


void
MainWindow::setProgressWidget(int mode)
{
    QPointer<BasicProgressWidget> basicProgressWidget;
    switch (mode) {
    case 0:
    case 2:
        basicProgressWidget = new VideoProgressWidget(mode == 0 ? m_parametersBagToVideo : m_parametersVideoToBag, mode == 0);
        break;
    case 1:
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
