#include "ProgressWidget.hpp"

#include "EncodingThread.hpp"
#include "Utils.hpp"

#include <QLabel>
#include <QMessageBox>
#include <QProgressBar>
#include <QPushButton>
#include <QVBoxLayout>

ProgressWidget::ProgressWidget(const QString& bagDirectory, const QString& topicName, const QString& vidDirectory,
                               bool useHardwareAcceleration, QWidget *parent) :
    QWidget(parent)
{
    const auto isDarkMode = Utils::isDarkMode();
    auto* const headerPixmapLabel = new QLabel;
    headerPixmapLabel->setPixmap(QIcon(isDarkMode ? ":/icons/bag_to_video_white.svg" : ":/icons/bag_to_video_black.svg").pixmap(QSize(100, 45)));
    headerPixmapLabel->setAlignment(Qt::AlignHCenter);

    auto* const headerLabel = new QLabel("Encoding Video...");
    Utils::setWidgetHeaderFont(headerLabel);
    headerLabel->setAlignment(Qt::AlignHCenter);

    auto* const progressBar = new QProgressBar;

    auto* const progressLabel = new QLabel;
    progressLabel->setAlignment(Qt::AlignHCenter);

    auto* const cancelButton = new QPushButton("Cancel");
    auto* const finishedButton = new QPushButton("Done");
    finishedButton->setVisible(false);

    auto* const buttonLayout = new QHBoxLayout;
    buttonLayout->addWidget(cancelButton);
    buttonLayout->addStretch();
    buttonLayout->addWidget(finishedButton);

    auto* const uiLayout = new QVBoxLayout;
    uiLayout->addStretch();
    uiLayout->addWidget(headerPixmapLabel);
    uiLayout->addWidget(headerLabel);
    uiLayout->addSpacing(30);
    uiLayout->addWidget(progressBar);
    uiLayout->addWidget(progressLabel);
    uiLayout->addStretch();

    auto* const uiLayoutStretched = new QHBoxLayout;
    uiLayoutStretched->addStretch();
    uiLayoutStretched->addLayout(uiLayout);
    uiLayoutStretched->addStretch();

    auto* const mainLayout = new QVBoxLayout;
    mainLayout->addLayout(uiLayoutStretched);
    mainLayout->addLayout(buttonLayout);

    setLayout(mainLayout);

    m_encodingThread = new EncodingThread(bagDirectory, topicName, vidDirectory, useHardwareAcceleration, this);
    connect(m_encodingThread, &EncodingThread::calculatedTopicMessageCount, this, [this](int messageCount) {
        m_messageCount = messageCount;
    });
    connect(m_encodingThread, &EncodingThread::openingVideoWriterFailed, this, [this] {
        auto* const messageBox = new QMessageBox(QMessageBox::Warning, "Failed writing file!",
                                                 "The video writing failed. Please make sure that all parameters are set correctly "
                                                 "and disable the hardware acceleration, if necessary.");
        messageBox->exec();
        emit encodingStopped();
    });
    connect(m_encodingThread, &EncodingThread::encodingProgressChanged, this, [this, progressLabel, progressBar] (int iteration, int progress) {
        progressLabel->setText("Encoding frame " + QString::number(iteration) + " of " + QString::number(m_messageCount) + "...");
        progressBar->setValue(progress);
    });
    connect(m_encodingThread, &EncodingThread::finished, this, [finishedButton] {
        finishedButton->setVisible(true);
    });
    connect(cancelButton, &QPushButton::clicked, this, [this] {
        if (m_encodingThread->isRunning()) {
            m_encodingThread->requestInterruption();
        }
        emit encodingStopped();
    });
    connect(finishedButton, &QPushButton::clicked, this, [this] {
        emit finished();
    });

    // Start encoding right away
    m_encodingThread->start();
}


ProgressWidget::~ProgressWidget()
{
    m_encodingThread->quit();
    m_encodingThread->wait();
}
