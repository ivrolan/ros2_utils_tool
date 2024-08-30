#include "VideoToBagWidget.hpp"

#include "UtilsROS.hpp"

#include <QCheckBox>
#include <QDialogButtonBox>
#include <QEvent>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QShortcut>
#include <QToolButton>
#include <QVBoxLayout>

#include <filesystem>

VideoToBagWidget::VideoToBagWidget(Utils::UI::VideoParameters& videoParameters, QWidget *parent) :
    QWidget(parent), m_videoParameters(videoParameters)
{
    m_headerPixmapLabel = new QLabel;
    m_headerPixmapLabel->setAlignment(Qt::AlignHCenter);
    setPixmapLabelIcon();

    auto* const headerTextLabel = new QLabel("Write Video to a ROSBag");
    Utils::UI::setWidgetHeaderFont(headerTextLabel);
    headerTextLabel->setAlignment(Qt::AlignHCenter);

    m_videoNameLineEdit = new QLineEdit(m_videoParameters.videoDirectory);
    m_videoNameLineEdit->setToolTip("The directory of the source video file.");
    auto* const searchVideoFileButton = new QToolButton;
    auto* const searchVideoFileLayout = Utils::UI::createLineEditButtonLayout(m_videoNameLineEdit, searchVideoFileButton);

    m_bagNameLineEdit = new QLineEdit(m_videoParameters.bagDirectory);
    m_bagNameLineEdit->setToolTip("The directory where the ROSBag should be stored.");
    auto* const bagLocationButton = new QToolButton;
    auto* const storeBagLayout = Utils::UI::createLineEditButtonLayout(m_bagNameLineEdit, bagLocationButton);

    m_topicNameLineEdit = new QLineEdit(m_videoParameters.topicName);
    m_topicNameLineEdit->setToolTip("The video's topic name inside the ROSBag.");

    m_useHardwareAccCheckBox = new QCheckBox;
    m_useHardwareAccCheckBox->setToolTip("Enable hardware acceleration for faster video decoding and writing.");
    m_useHardwareAccCheckBox->setCheckState(m_videoParameters.useHardwareAcceleration ? Qt::Checked : Qt::Unchecked);

    auto* const formLayout = new QFormLayout;
    formLayout->addRow("Video File:", searchVideoFileLayout);
    formLayout->addRow("Bag Location:", storeBagLayout);
    formLayout->addRow("Topic Name:", m_topicNameLineEdit);
    formLayout->addRow("Use HW Acceleration:", m_useHardwareAccCheckBox);

    auto* const controlsLayout = new QVBoxLayout;
    controlsLayout->addStretch();
    controlsLayout->addWidget(m_headerPixmapLabel);
    controlsLayout->addWidget(headerTextLabel);
    controlsLayout->addSpacing(40);
    controlsLayout->addLayout(formLayout);
    controlsLayout->addStretch();

    auto* const controlsSqueezedLayout = new QHBoxLayout;
    controlsSqueezedLayout->addStretch();
    controlsSqueezedLayout->addLayout(controlsLayout);
    controlsSqueezedLayout->addStretch();

    auto* const backButton = new QPushButton("Back");
    m_okButton = new QPushButton("Ok");
    m_okButton->setEnabled(false);

    auto* const buttonBox = new QDialogButtonBox;
    buttonBox->addButton(m_okButton, QDialogButtonBox::AcceptRole);

    auto* const buttonLayout = new QHBoxLayout;
    buttonLayout->addWidget(backButton);
    buttonLayout->addStretch();
    buttonLayout->addWidget(buttonBox);

    auto* const mainLayout = new QVBoxLayout;
    mainLayout->addLayout(controlsSqueezedLayout);
    mainLayout->addLayout(buttonLayout);
    setLayout(mainLayout);

    auto* const okShortCut = new QShortcut(QKeySequence(Qt::Key_Return), this);
    enableOkButton();

    connect(searchVideoFileButton, &QPushButton::clicked, this, &VideoToBagWidget::searchButtonPressed);
    connect(bagLocationButton, &QPushButton::clicked, this, &VideoToBagWidget::bagLocationButtonPressed);
    connect(m_topicNameLineEdit, &QLineEdit::textChanged, this, [this] {
        m_videoParameters.topicName = m_topicNameLineEdit->text();
        enableOkButton();
    });
    connect(m_useHardwareAccCheckBox, &QCheckBox::stateChanged, this, [this] (int state) {
        m_videoParameters.useHardwareAcceleration = state == Qt::Checked;
    });
    connect(backButton, &QPushButton::clicked, this, [this] {
        emit back();
    });
    connect(buttonBox, &QDialogButtonBox::accepted, this, &VideoToBagWidget::okButtonPressed);
    connect(okShortCut, &QShortcut::activated, this, &VideoToBagWidget::okButtonPressed);
}


void
VideoToBagWidget::searchButtonPressed()
{
    m_okButton->setEnabled(false);

    const auto fileName = QFileDialog::getOpenFileName(this, "Open Directory");
    if (fileName.isEmpty()) {
        return;
    }

    QFileInfo fileInfo(fileName);
    if (fileInfo.suffix().toLower() != "mp4" && fileInfo.suffix().toLower() != "mkv") {
        auto *const msgBox = new QMessageBox(QMessageBox::Critical, "Wrong format!", "The video must be in mp4 or mkv format!", QMessageBox::Ok);
        msgBox->exec();
        return;
    }

    m_videoParameters.videoDirectory = fileName;
    m_videoNameLineEdit->setText(fileName);
    enableOkButton();
}


void
VideoToBagWidget::bagLocationButtonPressed()
{
    const auto fileName = QFileDialog::getSaveFileName(this, "Save Video");
    if (fileName.isEmpty()) {
        return;
    }

    m_videoParameters.bagDirectory = fileName;
    m_bagNameLineEdit->setText(fileName);
    enableOkButton();
}


void
VideoToBagWidget::okButtonPressed()
{
    if (!m_okButton->isEnabled()) {
        return;
    }

    if (!Utils::ROS::doesTopicNameFollowROS2Convention(m_topicNameLineEdit->text())) {
        auto *const msgBox = new QMessageBox(QMessageBox::Critical, "Wrong topic name format!",
                                             "The topic name does not follow the ROS2 naming convention!", QMessageBox::Ok);
        msgBox->exec();
        return;
    }
    if (std::filesystem::exists(m_bagNameLineEdit->text().toStdString())) {
        auto *const msgBox = new QMessageBox(QMessageBox::Warning, "Bag file already exists!",
                                             "A bag file already exists under the specified directory! Are you sure you want to continue? This will overwrite the existing file.",
                                             QMessageBox::Yes | QMessageBox::No);
        const auto ret = msgBox->exec();
        if (ret == QMessageBox::No) {
            return;
        }
    }

    emit okPressed();
}


void
VideoToBagWidget::enableOkButton()
{
    m_okButton->setEnabled(!m_videoNameLineEdit->text().isEmpty() && !m_bagNameLineEdit->text().isEmpty() && !m_topicNameLineEdit->text().isEmpty());
}


void
VideoToBagWidget::setPixmapLabelIcon()
{
    const auto isDarkMode = Utils::UI::isDarkMode();
    m_headerPixmapLabel->setPixmap(QIcon(isDarkMode ? ":/icons/video_to_bag_white.svg" : ":/icons/video_to_bag_black.svg").pixmap(QSize(100, 45)));
}


bool
VideoToBagWidget::event(QEvent *event)
{
    if (event->type() == QEvent::ApplicationPaletteChange || event->type() == QEvent::PaletteChange) {
        setPixmapLabelIcon();
    }
    return QWidget::event(event);
}
