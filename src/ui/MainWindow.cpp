#include "MainWindow.hpp"

#include "BagInfoWidget.hpp"
#include "BagToImagesWidget.hpp"
#include "BagToVideoWidget.hpp"
#include "DummyBagWidget.hpp"
#include "EditBagWidget.hpp"
#include "ProgressWidget.hpp"
#include "StartWidget.hpp"
#include "VideoToBagWidget.hpp"

#include <QCloseEvent>

#include <csignal>

MainWindow::MainWindow()
{
    setWindowTitle("ROS Tools");

    setStartWidget();
}


void
MainWindow::setStartWidget()
{
    auto* const startWidget = new StartWidget;
    resize(DEFAULT_WIDTH, DEFAULT_HEIGHT);
    setCentralWidget(startWidget);
    connect(startWidget, &StartWidget::functionRequested, this, &MainWindow::setConfigWidget);
}


void
MainWindow::setConfigWidget(int mode)
{
    QPointer<BasicInputWidget> basicInputWidget;
    switch (mode) {
    case 0:
        basicInputWidget = new BagToVideoWidget(m_parametersBagToVideo);
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
        basicInputWidget = new EditBagWidget(m_editBagParameters);
        break;
    case 5:
        basicInputWidget = new BagInfoWidget;
        break;
    }
    resize(mode == 4 ? basicInputWidget->width() : DEFAULT_WIDTH,
           mode == 4 ? basicInputWidget->height() : DEFAULT_HEIGHT);
    setCentralWidget(basicInputWidget);

    connect(basicInputWidget, &BasicInputWidget::back, this, &MainWindow::setStartWidget);
    connect(basicInputWidget, &BasicInputWidget::okPressed, this, [this, mode] {
        setProgressWidget(mode);
    });
}


void
MainWindow::setProgressWidget(int mode)
{
    QPointer<ProgressWidget> progressWidget;
    switch (mode) {
    case 0:
        progressWidget = new ProgressWidget(":/icons/bag_to_video_black.svg", ":/icons/bag_to_video_white.svg",
                                            "Encoding Video...", m_parametersBagToVideo, mode);
        break;
    case 1:
        progressWidget = new ProgressWidget(":/icons/bag_to_images_black.svg", ":/icons/bag_to_images_white.svg",
                                            "Writing Images...", m_parametersBagToImages, mode);
        break;
    case 2:
        progressWidget = new ProgressWidget(":/icons/video_to_bag_black.svg", ":/icons/video_to_bag_white.svg",
                                            "Writing to Bag...", m_parametersVideoToBag, mode);
        break;
    case 3:
        progressWidget = new ProgressWidget(":/icons/dummy_bag_black.svg", ":/icons/dummy_bag_white.svg",
                                            "Creating ROSBag...", m_dummyBagParameters, mode);
        break;
    case 4:
        progressWidget = new ProgressWidget(":/icons/edit_bag_black.svg", ":/icons/edit_bag_white.svg",
                                            "Writing to edited ROSBag...", m_editBagParameters, mode);
        break;
    }
    resize(DEFAULT_WIDTH, DEFAULT_HEIGHT);
    setCentralWidget(progressWidget);

    connect(progressWidget, &ProgressWidget::progressStopped, this, [this, mode] {
        setConfigWidget(mode);
    });
    connect(progressWidget, &ProgressWidget::finished, this, &MainWindow::setStartWidget);

    progressWidget->startThread();
}


void
MainWindow::closeEvent(QCloseEvent *event)
{
    std::raise(SIGINT);
    event->accept();
}
