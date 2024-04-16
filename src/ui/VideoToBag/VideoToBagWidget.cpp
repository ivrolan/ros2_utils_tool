#include "VideoToBagWidget.hpp"

#include "UtilsROS.hpp"
#include "UtilsUI.hpp"

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

VideoToBagWidget::VideoToBagWidget(QWidget *parent) :
    QWidget(parent)
{
    const auto isDarkMode = UtilsUI::isDarkMode();
    auto* const headerPixmapLabel = new QLabel;
    headerPixmapLabel->setPixmap(QIcon(isDarkMode ? ":/icons/video_to_bag_white.svg" : ":/icons/video_to_bag_black.svg").pixmap(QSize(100, 45)));
    headerPixmapLabel->setAlignment(Qt::AlignHCenter);

    auto* const headerTextLabel = new QLabel("Write Video to a ROSBag");
    UtilsUI::setWidgetHeaderFont(headerTextLabel);
    headerTextLabel->setAlignment(Qt::AlignHCenter);

    m_videoNameLineEdit = new QLineEdit;
    m_videoNameLineEdit->setToolTip("The directory of the source video file.");
    auto* const searchVideoFileButton = new QToolButton;
    auto* const searchVideoFileLayout = UtilsUI::createLineEditButtonLayout(m_videoNameLineEdit, searchVideoFileButton);

    m_rosBagNameLineEdit = new QLineEdit;
    m_rosBagNameLineEdit->setToolTip("The directory where the ROSBag should be stored.");
    auto* const bagLocationButton = new QToolButton;
    auto* const storeBagLayout = UtilsUI::createLineEditButtonLayout(m_rosBagNameLineEdit, bagLocationButton);

    m_topicNameLineEdit = new QLineEdit;
    m_topicNameLineEdit->setToolTip("The video's topic name inside the ROSBag.");

    m_useHardwareAccCheckBox = new QCheckBox;
    m_useHardwareAccCheckBox->setToolTip("Enable hardware acceleration for faster video decoding and writing.");

    auto* const formLayout = new QFormLayout;
    formLayout->addRow("Video File:", searchVideoFileLayout);
    formLayout->addRow("Bag Location:", storeBagLayout);
    formLayout->addRow("Topic Name:", m_topicNameLineEdit);
    formLayout->addRow("Use HW Acceleration:", m_useHardwareAccCheckBox);

    auto* const controlsLayout = new QVBoxLayout;
    controlsLayout->addStretch();
    controlsLayout->addWidget(headerPixmapLabel);
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

    connect(searchVideoFileButton, &QPushButton::clicked, this, &VideoToBagWidget::searchButtonPressed);
    connect(bagLocationButton, &QPushButton::clicked, this, &VideoToBagWidget::bagLocationButtonPressed);
    connect(m_topicNameLineEdit, &QLineEdit::textChanged, this, [this] {
        enableOkButton();
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
    m_videoNameLineEdit->clear();
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

    m_videoNameLineEdit->setText(fileName);
    enableOkButton();
}


void
VideoToBagWidget::bagLocationButtonPressed()
{
    const auto fileName = QFileDialog::getSaveFileName(this, "Save Video");

    m_rosBagNameLineEdit->setText(fileName);

    enableOkButton();
}


void
VideoToBagWidget::enableOkButton()
{
    m_okButton->setEnabled(!m_videoNameLineEdit->text().isEmpty() &&
                           !m_rosBagNameLineEdit->text().isEmpty() &&
                           !m_topicNameLineEdit->text().isEmpty());
}


void
VideoToBagWidget::okButtonPressed()
{
    if (!m_okButton->isEnabled()) {
        return;
    }

    if (!UtilsROS::doesTopicNameFollowROS2Convention(m_topicNameLineEdit->text())) {
        auto *const msgBox = new QMessageBox(QMessageBox::Critical, "Wrong topic name format!",
                                             "The topic name does not follow the ROS2 naming convention!", QMessageBox::Ok);
        msgBox->exec();
        return;
    }
    emit parametersSet(m_videoNameLineEdit->text(), m_rosBagNameLineEdit->text(), m_topicNameLineEdit->text(),
                       m_useHardwareAccCheckBox->checkState() == Qt::Checked);
}
