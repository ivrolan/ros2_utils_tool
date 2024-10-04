#include "VideoToBagWidget.hpp"

#include "UtilsROS.hpp"

#include <QCheckBox>
#include <QDialogButtonBox>
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
    BasicInputWidget("Write Video to a ROSBag", ":/icons/video_to_bag_white.svg", ":/icons/video_to_bag_black.svg", parent),
    m_videoParameters(videoParameters)
{
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

    auto* const useHardwareAccCheckBox = new QCheckBox;
    useHardwareAccCheckBox->setToolTip("Enable hardware acceleration for faster video decoding and writing.");
    useHardwareAccCheckBox->setCheckState(m_videoParameters.useHardwareAcceleration ? Qt::Checked : Qt::Unchecked);

    auto* const formLayout = new QFormLayout;
    formLayout->addRow("Video File:", searchVideoFileLayout);
    formLayout->addRow("Bag Location:", storeBagLayout);
    formLayout->addRow("Topic Name:", m_topicNameLineEdit);
    formLayout->addRow("Use HW Acceleration:", useHardwareAccCheckBox);

    auto* const controlsLayout = new QVBoxLayout;
    controlsLayout->addStretch();
    controlsLayout->addWidget(m_headerPixmapLabel);
    controlsLayout->addWidget(m_headerLabel);
    controlsLayout->addSpacing(40);
    controlsLayout->addLayout(formLayout);
    controlsLayout->addStretch();

    auto* const controlsSqueezedLayout = new QHBoxLayout;
    controlsSqueezedLayout->addStretch();
    controlsSqueezedLayout->addLayout(controlsLayout);
    controlsSqueezedLayout->addStretch();

    auto* const mainLayout = new QVBoxLayout;
    mainLayout->addLayout(controlsSqueezedLayout);
    mainLayout->addLayout(m_buttonLayout);
    setLayout(mainLayout);

    auto* const okShortCut = new QShortcut(QKeySequence(Qt::Key_Return), this);
    enableOkButton(!m_videoNameLineEdit->text().isEmpty() && !m_bagNameLineEdit->text().isEmpty() && !m_topicNameLineEdit->text().isEmpty());

    connect(searchVideoFileButton, &QPushButton::clicked, this, &VideoToBagWidget::searchButtonPressed);
    connect(bagLocationButton, &QPushButton::clicked, this, &VideoToBagWidget::bagLocationButtonPressed);
    connect(m_topicNameLineEdit, &QLineEdit::textChanged, this, [this] {
        m_videoParameters.topicName = m_topicNameLineEdit->text();
        enableOkButton(!m_videoNameLineEdit->text().isEmpty() && !m_bagNameLineEdit->text().isEmpty() && !m_topicNameLineEdit->text().isEmpty());
    });
    connect(useHardwareAccCheckBox, &QCheckBox::stateChanged, this, [this] (int state) {
        m_videoParameters.useHardwareAcceleration = state == Qt::Checked;
    });
    connect(m_dialogButtonBox, &QDialogButtonBox::accepted, this, &VideoToBagWidget::okButtonPressed);
    connect(okShortCut, &QShortcut::activated, this, &VideoToBagWidget::okButtonPressed);
}


void
VideoToBagWidget::searchButtonPressed()
{
    enableOkButton(false);

    const auto videoDir = QFileDialog::getOpenFileName(this, "Open Video");
    if (videoDir.isEmpty()) {
        return;
    }

    QFileInfo fileInfo(videoDir);
    if (fileInfo.suffix().toLower() != "mp4" && fileInfo.suffix().toLower() != "mkv") {
        auto *const msgBox = new QMessageBox(QMessageBox::Critical, "Wrong format!", "The video must be in mp4 or mkv format!", QMessageBox::Ok);
        msgBox->exec();
        return;
    }

    m_videoParameters.videoDirectory = videoDir;
    m_videoNameLineEdit->setText(videoDir);

    QDir videoDirectoryDir(videoDir);
    videoDirectoryDir.cdUp();
    if (const auto autoBagDirectory = videoDirectoryDir.path() + "/video_bag"; !std::filesystem::exists(autoBagDirectory.toStdString())) {
        m_bagNameLineEdit->setText(autoBagDirectory);
        m_videoParameters.bagDirectory = autoBagDirectory;
    }

    enableOkButton(!m_videoNameLineEdit->text().isEmpty() && !m_bagNameLineEdit->text().isEmpty() && !m_topicNameLineEdit->text().isEmpty());
}


void
VideoToBagWidget::bagLocationButtonPressed()
{
    const auto fileName = QFileDialog::getSaveFileName(this, "Save ROSBag");
    if (fileName.isEmpty()) {
        return;
    }

    m_videoParameters.bagDirectory = fileName;
    m_bagNameLineEdit->setText(fileName);
    enableOkButton(!m_videoNameLineEdit->text().isEmpty() && !m_bagNameLineEdit->text().isEmpty() && !m_topicNameLineEdit->text().isEmpty());
}


void
VideoToBagWidget::okButtonPressed()
{
    if (!m_okButton->isEnabled()) {
        return;
    }

    if (!Utils::ROS::doesTopicNameFollowROS2Convention(m_topicNameLineEdit->text())) {
        Utils::UI::createCriticalMessageBox("Wrong topic name format!", "The topic name does not follow the ROS2 naming convention!");
        return;
    }
    if (std::filesystem::exists(m_bagNameLineEdit->text().toStdString())) {
        auto *const msgBox = new QMessageBox(QMessageBox::Warning, "Bag file already exists!",
                                             "A bag file already exists under the specified directory! Are you sure you want to continue? This will overwrite the existing file.",
                                             QMessageBox::Yes | QMessageBox::No);
        if (const auto ret = msgBox->exec(); ret == QMessageBox::No) {
            return;
        }
    }

    emit okPressed();
}
