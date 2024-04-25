#include "BagToVideoWidget.hpp"

#include "UtilsROS.hpp"
#include "UtilsUI.hpp"

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

BagToVideoWidget::BagToVideoWidget(QWidget *parent) :
    QWidget(parent)
{
    m_headerPixmapLabel = new QLabel;
    m_headerPixmapLabel->setAlignment(Qt::AlignHCenter);
    setPixmapLabelIcon();

    auto* const headerTextLabel = new QLabel("Encode Video from ROSBag");
    Utils::UI::setWidgetHeaderFont(headerTextLabel);
    headerTextLabel->setAlignment(Qt::AlignHCenter);

    m_fileNameLineEdit = new QLineEdit;
    m_fileNameLineEdit->setToolTip("The directory of the ROSBag source file.");

    auto* const searchBagButton = new QToolButton;
    auto* const searchBagFileLayout = Utils::UI::createLineEditButtonLayout(m_fileNameLineEdit, searchBagButton);

    m_topicNameComboBox = new QComboBox;
    m_topicNameComboBox->setMinimumWidth(200);
    m_topicNameComboBox->setToolTip("The ROSBag topic of the video file.\n"
                                    "If the Bag contains multiple video topics, you can choose one of them.");

    m_videoNameLineEdit = new QLineEdit;
    m_videoNameLineEdit->setToolTip("The directory where the video file should be stored.");

    auto* const videoLocationButton = new QToolButton;
    auto* const searchVideoPathLayout = Utils::UI::createLineEditButtonLayout(m_videoNameLineEdit, videoLocationButton);

    m_formatComboBox = new QComboBox;
    m_formatComboBox->addItem("mp4", 0);
    m_formatComboBox->addItem("mkv", 1);
    m_formatComboBox->setToolTip("The video format file.");

    m_useHardwareAccCheckBox = new QCheckBox;
    m_useHardwareAccCheckBox->setToolTip("Enable hardware acceleration for faster encoding.");

    auto* const formLayout = new QFormLayout;
    formLayout->addRow("Bag File:", searchBagFileLayout);
    formLayout->addRow("Topic Name:", m_topicNameComboBox);
    formLayout->addRow("Video Location:", searchVideoPathLayout);
    formLayout->addRow("Format:", m_formatComboBox);
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

    connect(searchBagButton, &QPushButton::clicked, this, &BagToVideoWidget::searchButtonPressed);
    connect(videoLocationButton, &QPushButton::clicked, this, &BagToVideoWidget::videoLocationButtonPressed);
    connect(m_formatComboBox, &QComboBox::currentTextChanged, this, &BagToVideoWidget::formatComboBoxTextChanged);
    connect(backButton, &QPushButton::clicked, this, [this] {
        emit back();
    });
    connect(buttonBox, &QDialogButtonBox::accepted, this, &BagToVideoWidget::okButtonPressed);
    connect(okShortCut, &QShortcut::activated, this, &BagToVideoWidget::okButtonPressed);
}


void
BagToVideoWidget::searchButtonPressed()
{
    m_fileNameLineEdit->setText(QFileDialog::getExistingDirectory(this, "Open Directory", "",
                                                                  QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks));
    if (m_fileNameLineEdit->text().isEmpty()) {
        return;
    }

    m_topicNameComboBox->clear();
    const auto videoTopics = Utils::ROS::getBagVideoTopics(m_fileNameLineEdit->text().toStdString());
    // Only enable if both line edits contain text
    m_okButton->setEnabled(!videoTopics.empty() && !m_videoNameLineEdit->text().isEmpty());

    if (videoTopics.empty()) {
        auto *const msgBox = new QMessageBox(QMessageBox::Critical, "Topic not found!",
                                             "The bag file does not contain any image/video topics!", QMessageBox::Ok);
        msgBox->exec();
        return;
    }

    for (const auto& videoTopic : videoTopics) {
        m_topicNameComboBox->addItem(QString::fromStdString(videoTopic));
    }
}


void
BagToVideoWidget::videoLocationButtonPressed()
{
    const auto fileName = QFileDialog::getSaveFileName(this, "Save Video", "",
                                                       m_formatComboBox->currentText() + " files (*." + m_formatComboBox->currentText() + ")");
    m_videoNameLineEdit->setText(fileName);
    // Only enable if both line edits contain text
    m_okButton->setEnabled(!m_topicNameComboBox->currentText().isEmpty() && !m_videoNameLineEdit->text().isEmpty());
}


void
BagToVideoWidget::formatComboBoxTextChanged(const QString& text)
{
    // If the combo box item changes, apply a different appendix to the text in the video line edit
    if (m_videoNameLineEdit->text().isEmpty()) {
        return;
    }
    auto newLineEditText = m_videoNameLineEdit->text();
    newLineEditText.truncate(newLineEditText.lastIndexOf(QChar('.')));
    newLineEditText += "." + text;
    m_videoNameLineEdit->setText(newLineEditText);
}


void
BagToVideoWidget::okButtonPressed()
{
    if (!m_okButton->isEnabled()) {
        return;
    }

    emit parametersSet(m_fileNameLineEdit->text(), m_topicNameComboBox->currentText(), m_videoNameLineEdit->text(),
                       m_useHardwareAccCheckBox->checkState() == Qt::Checked);
}


void
BagToVideoWidget::setPixmapLabelIcon()
{
    const auto isDarkMode = Utils::UI::isDarkMode();
    m_headerPixmapLabel->setPixmap(QIcon(isDarkMode ? ":/icons/bag_to_video_white.svg" : ":/icons/bag_to_video_black.svg").pixmap(QSize(100, 45)));
}


bool
BagToVideoWidget::event(QEvent *event)
{
    if (event->type() == QEvent::ApplicationPaletteChange || event->type() == QEvent::PaletteChange) {
        setPixmapLabelIcon();
    }
    return QWidget::event(event);
}
