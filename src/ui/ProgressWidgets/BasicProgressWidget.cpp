#include "BasicProgressWidget.hpp"

#include "BasicThread.hpp"
#include "UtilsUI.hpp"

#include <QLabel>
#include <QMessageBox>
#include <QProgressBar>
#include <QPushButton>
#include <QShortcut>
#include <QVBoxLayout>

BasicProgressWidget::BasicProgressWidget(QWidget *parent) :
    QWidget(parent)
{
    m_headerPixmapLabel = new QLabel;
    m_headerPixmapLabel->setAlignment(Qt::AlignHCenter);

    m_headerLabel = new QLabel;
    Utils::UI::setWidgetFontSize(m_headerLabel);
    m_headerLabel->setAlignment(Qt::AlignHCenter);

    m_progressBar = new QProgressBar;

    m_progressLabel = new QLabel;
    m_progressLabel->setAlignment(Qt::AlignHCenter);

    m_cancelButton = new QPushButton("Cancel");
    m_finishedButton = new QPushButton("Done");
    m_finishedButton->setVisible(false);

    auto* const buttonLayout = new QHBoxLayout;
    buttonLayout->addWidget(m_cancelButton);
    buttonLayout->addStretch();
    buttonLayout->addWidget(m_finishedButton);

    auto* const uiLayout = new QVBoxLayout;
    uiLayout->addStretch();
    uiLayout->addWidget(m_headerPixmapLabel);
    uiLayout->addWidget(m_headerLabel);
    uiLayout->addSpacing(30);
    uiLayout->addWidget(m_progressBar);
    uiLayout->addWidget(m_progressLabel);
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

    connect(m_cancelButton, &QPushButton::clicked, this, [this] {
        if (m_thread->isRunning()) {
            m_thread->requestInterruption();
        }
        emit progressStopped();
    });
    connect(m_finishedButton, &QPushButton::clicked, this, [this] {
        emit finished();
    });
    connect(doneShortCut, &QShortcut::activated, this, [this] {
        if (!m_finishedButton->isVisible()) {
            return;
        }
        emit finished();
    });
}


BasicProgressWidget::~BasicProgressWidget()
{
    m_thread->quit();
    m_thread->wait();
}


void
BasicProgressWidget::startThread()
{
    m_thread->start();
}


void
BasicProgressWidget::connectThread()
{
    if (!m_thread) {
        return;
    }

    connect(m_thread, &BasicThread::calculatedMaximumInstances, this, [this](int count) {
        m_maximumCount = count;
    });
    connect(m_thread, &BasicThread::openingCVInstanceFailed, this, [this] {
        auto* const messageBox = new QMessageBox(QMessageBox::Warning, "Failed writing file!",
                                                 "The video writing failed. Please make sure that all parameters are set correctly "
                                                 "and disable the hardware acceleration, if necessary.");
        messageBox->exec();
        emit progressStopped();
    });
    connect(m_thread, &BasicThread::progressChanged, this, [this] (int iteration, int progress) {
        m_progressLabel->setText("Frame " + QString::number(iteration) + " of " + QString::number(m_maximumCount) + "...");
        m_progressBar->setValue(progress);
    });
    connect(m_thread, &BasicThread::finished, this, [this] {
        m_cancelButton->setVisible(false);
        m_finishedButton->setVisible(true);
    });
}
