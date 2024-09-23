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
    BasicConfigWidget(":/icons/video_to_bag_white.svg", ":/icons/video_to_bag_black.svg", parent),
    m_videoParameters(videoParameters)
{
    auto* const headerTextLabel = new QLabel("Write Video to a ROSBag");
    Utils::UI::setWidgetFontSize(headerTextLabel);
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
    controlsLayout->addWidget(headerTextLabel);
    controlsLayout->addSpacing(40);
    controlsLayout->addLayout(formLayout);
    controlsLayout->addStretch();

    auto* const controlsSqueezedLayout = new QHBoxLayout;
    controlsSqueezedLayout->addStretch();
    controlsSqueezedLayout->addLayout(controlsLayout);
    controlsSqueezedLayout->addStretch();

    auto* const backButton = new QPushButton("Back");

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
    connect(backButton, &QPushButton::clicked, this, [this] {
        emit back();
    });
    connect(buttonBox, &QDialogButtonBox::accepted, this, &VideoToBagWidget::okButtonPressed);
    connect(okShortCut, &QShortcut::activated, this, &VideoToBagWidget::okButtonPressed);
}


void
VideoToBagWidget::searchButtonPressed()
{
    enableOkButton(false);

    const auto fileName = QFileDialog::getOpenFileName(this, "Open Video");
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
