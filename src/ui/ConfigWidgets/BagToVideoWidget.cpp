#include "BagToVideoWidget.hpp"

#include "UtilsROS.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QEvent>
#include <QFileDialog>
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

BagToVideoWidget::BagToVideoWidget(Utils::UI::VideoParameters& videoParameters, QString& encodingFormat, QWidget *parent) :
    BasicConfigWidget(":/icons/bag_to_video_white.svg", ":/icons/bag_to_video_black.svg", parent),
    m_videoParameters(videoParameters), m_encodingFormat(encodingFormat)
{
    auto* const headerTextLabel = new QLabel("Encode Video from ROSBag");
    Utils::UI::setWidgetHeaderFont(headerTextLabel);
    headerTextLabel->setAlignment(Qt::AlignHCenter);

    m_bagNameLineEdit = new QLineEdit(m_videoParameters.bagDirectory);
    m_bagNameLineEdit->setToolTip("The directory of the ROSBag source file.");

    auto* const searchBagButton = new QToolButton;
    auto* const searchBagFileLayout = Utils::UI::createLineEditButtonLayout(m_bagNameLineEdit, searchBagButton);

    m_topicNameComboBox = new QComboBox;
    m_topicNameComboBox->setMinimumWidth(200);
    m_topicNameComboBox->setToolTip("The ROSBag topic of the video file.\n"
                                    "If the Bag contains multiple video topics, you can choose one of them.");
    if (!m_videoParameters.bagDirectory.isEmpty()) {
        Utils::UI::fillComboBoxWithTopics(m_topicNameComboBox, m_videoParameters.bagDirectory);
    }

    m_videoNameLineEdit = new QLineEdit(m_videoParameters.videoDirectory);
    m_videoNameLineEdit->setToolTip("The directory where the video file should be stored.");

    auto* const videoLocationButton = new QToolButton;
    auto* const searchVideoPathLayout = Utils::UI::createLineEditButtonLayout(m_videoNameLineEdit, videoLocationButton);

    m_formatComboBox = new QComboBox;
    m_formatComboBox->addItem("mp4", 0);
    m_formatComboBox->addItem("mkv", 1);
    m_formatComboBox->setToolTip("The video format file.");
    if (!m_videoParameters.videoDirectory.isEmpty()) {
        QFileInfo fileInfo(m_videoParameters.videoDirectory);
        m_formatComboBox->setCurrentText(fileInfo.suffix());
    } else {
        m_formatComboBox->setCurrentText(m_encodingFormat);
    }

    auto* const useHardwareAccCheckBox = new QCheckBox;
    useHardwareAccCheckBox->setToolTip("Enable hardware acceleration for faster encoding.");
    useHardwareAccCheckBox->setCheckState(m_videoParameters.useHardwareAcceleration ? Qt::Checked : Qt::Unchecked);

    auto* const formLayout = new QFormLayout;
    formLayout->addRow("Bag File:", searchBagFileLayout);
    formLayout->addRow("Topic Name:", m_topicNameComboBox);
    formLayout->addRow("Video Location:", searchVideoPathLayout);
    formLayout->addRow("Format:", m_formatComboBox);
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
    enableOkButton(!m_bagNameLineEdit->text().isEmpty() &&
                   !m_topicNameComboBox->currentText().isEmpty() &&
                   !m_videoNameLineEdit->text().isEmpty());


    connect(searchBagButton, &QPushButton::clicked, this, &BagToVideoWidget::searchButtonPressed);
    connect(videoLocationButton, &QPushButton::clicked, this, &BagToVideoWidget::videoLocationButtonPressed);
    connect(m_topicNameComboBox, &QComboBox::currentTextChanged, this, [this] (const QString& text) {
        m_videoParameters.topicName = text;
    });
    connect(m_formatComboBox, &QComboBox::currentTextChanged, this, &BagToVideoWidget::formatComboBoxTextChanged);
    connect(useHardwareAccCheckBox, &QCheckBox::stateChanged, this, [this] (int state) {
        m_videoParameters.useHardwareAcceleration = state == Qt::Checked;
    });
    connect(backButton, &QPushButton::clicked, this, [this] {
        emit back();
    });
    connect(buttonBox, &QDialogButtonBox::accepted, this, &BagToVideoWidget::okButtonPressed);
    connect(okShortCut, &QShortcut::activated, this, &BagToVideoWidget::okButtonPressed);
}


void
BagToVideoWidget::searchButtonPressed()
{
    const auto bagDirectory = QFileDialog::getExistingDirectory(this, "Open Directory", "",
                                                                QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    if (bagDirectory.isEmpty()) {
        return;
    }

    if (const auto containsVideoTopics = Utils::UI::fillComboBoxWithTopics(m_topicNameComboBox, bagDirectory); !containsVideoTopics) {
        auto *const msgBox = new QMessageBox(QMessageBox::Critical, "Topic not found!",
                                             "The bag file does not contain any image/video topics!", QMessageBox::Ok);
        msgBox->exec();
        return;
    }

    m_videoParameters.bagDirectory = bagDirectory;
    m_bagNameLineEdit->setText(bagDirectory);
    // Only enable if both line edits contain text
    enableOkButton(!m_bagNameLineEdit->text().isEmpty() &&
                   !m_topicNameComboBox->currentText().isEmpty() && !m_videoNameLineEdit->text().isEmpty());
}


void
BagToVideoWidget::videoLocationButtonPressed()
{
    const auto fileName = QFileDialog::getSaveFileName(this, "Save Video", "",
                                                       m_formatComboBox->currentText() + " files (*." + m_formatComboBox->currentText() + ")");
    if (fileName.isEmpty()) {
        return;
    }

    // Only enable if both line edits contain text
    m_fileDialogOpened = true;
    m_videoParameters.videoDirectory = fileName;
    m_videoNameLineEdit->setText(fileName);
    enableOkButton(!m_bagNameLineEdit->text().isEmpty() &&
                   !m_topicNameComboBox->currentText().isEmpty() && !m_videoNameLineEdit->text().isEmpty());
}


void
BagToVideoWidget::formatComboBoxTextChanged(const QString& text)
{
    m_encodingFormat = text;
    // If the combo box item changes, apply a different appendix to the text in the video line edit
    if (m_videoNameLineEdit->text().isEmpty()) {
        return;
    }

    auto newLineEditText = m_videoNameLineEdit->text();
    newLineEditText.truncate(newLineEditText.lastIndexOf(QChar('.')));
    newLineEditText += "." + text;

    m_videoParameters.videoDirectory = newLineEditText;
    m_videoNameLineEdit->setText(newLineEditText);
}


void
BagToVideoWidget::okButtonPressed()
{
    if (!m_okButton->isEnabled()) {
        return;
    }

    // Only ask if exists and the file dialog has not been called
    if (std::filesystem::exists(m_videoNameLineEdit->text().toStdString()) && !m_fileDialogOpened) {
        auto *const msgBox = new QMessageBox(QMessageBox::Warning, "Video already exists!",
                                             "A video already exists under the specified directory! Are you sure you want to continue? This will overwrite the existing file.",
                                             QMessageBox::Yes | QMessageBox::No);
        const auto ret = msgBox->exec();
        if (ret == QMessageBox::No) {
            return;
        }
    }

    emit okPressed();
}
