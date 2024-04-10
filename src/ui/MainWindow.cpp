#include "MainWindow.hpp"

#include "BagToVideoWidget.hpp"
#include "ProgressWidget.hpp"
#include "StartWidget.hpp"

#include <QCloseEvent>

#include "rclcpp/rclcpp.hpp"

MainWindow::MainWindow()
{
    setWindowTitle("ROS Tools");

    setStartWidget();

    resize(500, 500);
}


void
MainWindow::setStartWidget()
{
    auto* const startWidget = new StartWidget;
    connect(startWidget, &StartWidget::bagToVideoRequested, this, &MainWindow::setBagToVideoWidget);

    setCentralWidget(startWidget);
}


void
MainWindow::setBagToVideoWidget()
{
    auto* const bagToVideoWidget = new BagToVideoWidget;
    setCentralWidget(bagToVideoWidget);

    connect(bagToVideoWidget, &BagToVideoWidget::back, this, &MainWindow::setStartWidget);
    connect(bagToVideoWidget, &BagToVideoWidget::parametersSet, this, [this] (const QString& bagDirectory, const QString& topicName,
                                                                              const QString& vidDirectory, bool useHardwareAcceleration) {
        setProgressWidget(bagDirectory, topicName, vidDirectory, useHardwareAcceleration);
    });
}


void
MainWindow::setProgressWidget(const QString bagDirectory, const QString topicName, const QString vidDirectory, bool useHardwareAcceleration)
{
    auto* const progressWidget = new ProgressWidget(bagDirectory, topicName, vidDirectory, useHardwareAcceleration);
    setCentralWidget(progressWidget);

    connect(progressWidget, &ProgressWidget::encodingStopped, this, &MainWindow::setBagToVideoWidget);
    connect(progressWidget, &ProgressWidget::finished, this, &MainWindow::setStartWidget);
}


void
MainWindow::closeEvent(QCloseEvent *event)
{
    rclcpp::shutdown();
    event->accept();
}
