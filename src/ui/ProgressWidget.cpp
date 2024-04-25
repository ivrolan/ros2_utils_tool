#include "ProgressWidget.hpp"

#include "EncodingThread.hpp"
#include "UtilsUI.hpp"
#include "WriteToBagThread.hpp"

#include <QLabel>
#include <QMessageBox>
#include <QProgressBar>
#include <QPushButton>
#include <QShortcut>
#include <QVBoxLayout>

ProgressWidget::ProgressWidget(const QString& bagDirectory, const QString& topicName, const QString& vidDirectory,
                               bool useHardwareAcceleration, bool useEncode, QWidget *parent) :
    QWidget(parent)
{
    const auto isDarkMode = Utils::UI::isDarkMode();
    auto* const headerPixmapLabel = new QLabel;
    if (useEncode) {
        headerPixmapLabel->setPixmap(QIcon(isDarkMode ? ":/icons/bag_to_video_white.svg" : ":/icons/bag_to_video_black.svg").pixmap(QSize(100, 45)));
    } else {
        headerPixmapLabel->setPixmap(QIcon(isDarkMode ? ":/icons/video_to_bag_white.svg" : ":/icons/video_to_bag_black.svg").pixmap(QSize(100, 45)));
    }
    headerPixmapLabel->setAlignment(Qt::AlignHCenter);

    auto* const headerLabel = new QLabel("Encoding Video...");
    Utils::UI::setWidgetHeaderFont(headerLabel);
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

    auto* const doneShortCut = new QShortcut(QKeySequence(Qt::Key_Return), this);

    if (useEncode) {
        m_thread = new EncodingThread(bagDirectory, topicName, vidDirectory, useHardwareAcceleration, this);
    } else {
        m_thread = new WriteToBagThread(bagDirectory, topicName, vidDirectory, useHardwareAcceleration, this);
    }

    connect(m_thread, &BasicThread::calculatedMaximumInstances, this, [this](int count) {
        m_maximumCount = count;
    });
    connect(m_thread, &BasicThread::openingCVInstanceFailed, this, [this] {
        auto* const messageBox = new QMessageBox(QMessageBox::Warning, "Failed writing file!",
                                                 "The video writing failed. Please make sure that all parameters are set correctly "
                                                 "and disable the hardware acceleration, if necessary.");
        messageBox->exec();
        emit encodingStopped();
    });
    connect(m_thread, &BasicThread::progressChanged, this, [this, progressLabel, progressBar, useEncode] (int iteration, int progress) {
        const auto frontString = useEncode ? QString("Encoding") : QString("Writing");
        progressLabel->setText(frontString + " frame " + QString::number(iteration) + " of " + QString::number(m_maximumCount) + "...");
        progressBar->setValue(progress);
    });
    connect(m_thread, &BasicThread::finished, this, [cancelButton, finishedButton] {
        cancelButton->setVisible(false);
        finishedButton->setVisible(true);
    });

    connect(cancelButton, &QPushButton::clicked, this, [this] {
        if (m_thread->isRunning()) {
            m_thread->requestInterruption();
        }
        emit encodingStopped();
    });
    connect(finishedButton, &QPushButton::clicked, this, [this] {
        emit finished();
    });
    connect(doneShortCut, &QShortcut::activated, this, [this, finishedButton] {
        if (!finishedButton->isVisible()) {
            return;
        }
        emit finished();
    });

    // Start encoding right away
    m_thread->start();
}


ProgressWidget::~ProgressWidget()
{
    m_thread->quit();
    m_thread->wait();
}
