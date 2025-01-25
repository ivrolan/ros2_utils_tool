#include "ProgressWidget.hpp"

#include "BasicThread.hpp"
#include "DummyBagThread.hpp"
#include "EditBagThread.hpp"
#include "EncodingThread.hpp"
#include "WriteToBagThread.hpp"
#include "WriteToImageThread.hpp"

#include <QLabel>
#include <QMessageBox>
#include <QProgressBar>
#include <QPushButton>
#include <QShortcut>
#include <QVBoxLayout>

ProgressWidget::ProgressWidget(const QString& headerPixmapLabelTextBlack, const QString& headerPixmapLabelTextWhite,
                               const QString& headerLabelText, Utils::UI::InputParameters& parameters,
                               const int threadTypeId, QWidget *parent) :
    QWidget(parent)
{
    const auto isDarkMode = Utils::UI::isDarkMode();

    auto* const headerPixmapLabel = new QLabel;
    headerPixmapLabel->setPixmap(QIcon(isDarkMode ? headerPixmapLabelTextWhite : headerPixmapLabelTextBlack).pixmap(QSize(100, 45)));
    headerPixmapLabel->setAlignment(Qt::AlignHCenter);

    auto* const headerLabel = new QLabel(headerLabelText);
    Utils::UI::setWidgetFontSize(headerLabel);
    headerLabel->setAlignment(Qt::AlignHCenter);

    auto* const progressBar = new QProgressBar;
    progressBar->setVisible(false);

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

    switch (threadTypeId) {
    case 0:
        m_thread = new EncodingThread(dynamic_cast<Utils::UI::VideoInputParameters&>(parameters), this);
        break;
    case 1:
        m_thread = new WriteToImageThread(dynamic_cast<Utils::UI::ImageInputParameters&>(parameters), this);
        break;
    case 2:
        m_thread = new WriteToBagThread(dynamic_cast<Utils::UI::BagInputParameters&>(parameters), this);
        break;
    case 3:
        m_thread = new DummyBagThread(dynamic_cast<Utils::UI::DummyBagInputParameters&>(parameters), this);
        break;
    case 4:
        m_thread = new EditBagThread(dynamic_cast<Utils::UI::EditBagInputParameters&>(parameters), this);
        break;
    }

    connect(cancelButton, &QPushButton::clicked, this, [this] {
        if (m_thread->isRunning()) {
            m_thread->requestInterruption();
        }
        emit progressStopped();
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

    connect(m_thread, &BasicThread::calculatedMaximumInstances, this, [this](int count) {
        m_maximumCount = count;
    });
    connect(m_thread, &BasicThread::startingDataCollection, this, [progressLabel] {
        progressLabel->setText("Collecting necessary data...");
    });
    connect(m_thread, &BasicThread::openingCVInstanceFailed, this, [this] {
        auto* const messageBox = new QMessageBox(QMessageBox::Warning, "Failed writing file!",
                                                 "The video writing failed. Please make sure that all parameters are set correctly "
                                                 "and disable the hardware acceleration, if necessary.");
        messageBox->exec();
        emit progressStopped();
    });
    connect(m_thread, &BasicThread::progressChanged, this, [this, progressLabel, progressBar] (int iteration, int progress) {
        if (!progressBar->isVisible()) {
            progressBar->setVisible(true);
        }
        progressBar->setValue(progress);
        progressLabel->setText(QString::number(iteration) + " of " + QString::number(m_maximumCount) + "...");
    });
    connect(m_thread, &BasicThread::finished, this, [cancelButton, finishedButton] {
        cancelButton->setVisible(false);
        finishedButton->setVisible(true);
    });
}


ProgressWidget::~ProgressWidget()
{
    m_thread->requestInterruption();
    m_thread->wait();
}


void
ProgressWidget::startThread()
{
    m_thread->start();
}
