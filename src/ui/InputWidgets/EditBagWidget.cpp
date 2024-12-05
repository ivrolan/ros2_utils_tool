#include "EditBagWidget.hpp"

#include <QDialogButtonBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QToolButton>
#include <QTreeWidget>
#include <QVBoxLayout>

#include "MessageCountWidget.hpp"
#include "UtilsROS.hpp"

#include <filesystem>

EditBagWidget::EditBagWidget(Utils::UI::EditBagParameters& editBagParameters, QWidget *parent) :
    BasicInputWidget("Edit ROSBag", ":/icons/edit_bag", parent),
    m_editBagParameters(editBagParameters), m_editBagParamSettings(editBagParameters, "edit_bag")
{
    auto* const formLayout = new QFormLayout;
    formLayout->addRow("Bag Location:", m_findSourceLayout);
    m_sourceLineEdit->setText(m_editBagParameters.sourceDirectory);

    m_editLabel = new QLabel("Unselect all items you want to remove.<br>"
                             "Change the message count if you want to drop messages.<br>"
                             "Type a name into the 'Rename' column to rename a topic.");
    m_editLabel->setVisible(false);

    auto labelFont = m_editLabel->font();
    labelFont.setBold(true);
    m_editLabel->setFont(labelFont);

    m_treeWidget = new QTreeWidget;
    m_treeWidget->setVisible(false);
    m_treeWidget->setColumnCount(4);
    m_treeWidget->headerItem()->setText(COL_CHECKBOXES, "");
    m_treeWidget->headerItem()->setText(COL_TOPICS, "Topics:");
    m_treeWidget->headerItem()->setText(COL_MESSAGE_COUNT, "Message Count:");
    m_treeWidget->headerItem()->setText(COL_RENAMING, "Rename:");
    m_treeWidget->setRootIsDecorated(false);

    m_targetLineEdit = new QLineEdit(m_editBagParameters.targetDirectory);
    auto* const targetPushButton = new QToolButton;
    auto* const targetLineEditLayout = Utils::UI::createLineEditButtonLayout(m_targetLineEdit, targetPushButton);

    auto* const targetFormLayout = new QFormLayout;
    targetFormLayout->addRow("Edited Bag Location:", targetLineEditLayout);
    targetFormLayout->setContentsMargins(0, 0, 0, 0);

    m_targetBagNameWidget = new QWidget;
    m_targetBagNameWidget->setLayout(targetFormLayout);
    m_targetBagNameWidget->setVisible(false);

    auto* const controlsLayout = new QVBoxLayout;
    controlsLayout->addStretch();
    controlsLayout->addWidget(m_headerPixmapLabel);
    controlsLayout->addWidget(m_headerLabel);
    controlsLayout->addSpacing(30);
    controlsLayout->addLayout(formLayout);
    controlsLayout->addSpacing(10);
    controlsLayout->addWidget(m_editLabel);
    controlsLayout->addWidget(m_treeWidget);
    controlsLayout->addWidget(m_targetBagNameWidget);
    // Give it a more "squishy" look
    controlsLayout->setContentsMargins(30, 30, 30, 30);
    controlsLayout->addStretch();

    m_okButton->setEnabled(true);
    m_okButton->setVisible(false);

    auto* const mainLayout = new QVBoxLayout;
    mainLayout->addLayout(controlsLayout);
    mainLayout->addLayout(m_buttonLayout);
    setLayout(mainLayout);

    connect(m_findSourceButton, &QPushButton::clicked, this, [this] {
        createTopicTree(true);
    });
    connect(m_treeWidget, &QTreeWidget::itemChanged, this, &EditBagWidget::itemCheckStateChanged);
    connect(targetPushButton, &QPushButton::clicked, this, &EditBagWidget::targetPushButtonPressed);
    connect(m_dialogButtonBox, &QDialogButtonBox::accepted, this, &EditBagWidget::okButtonPressed);

    if (!m_sourceLineEdit->text().isEmpty()) {
        createTopicTree(false);
    }
}


void
EditBagWidget::createTopicTree(bool newTreeRequested)
{
    if (newTreeRequested) {
        const auto bagDirectory = QFileDialog::getExistingDirectory(this, "Open ROSBag", "",
                                                                    QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
        if (bagDirectory.isEmpty() || !Utils::ROS::doesDirectoryContainBagFile(bagDirectory)) {
            return;
        }

        m_sourceLineEdit->setText(bagDirectory);
        writeSettingsParameter(m_editBagParameters.sourceDirectory, bagDirectory, m_editBagParamSettings);
        m_editBagParameters.topics.clear();
        m_editBagParamSettings.write();
    }

    m_treeWidget->clear();
    m_treeWidget->blockSignals(true);

    const auto& bagMetaData = Utils::ROS::getBagMetadata(m_editBagParameters.sourceDirectory);
    // Fill tree widget with topics
    for (size_t i = 0; i < bagMetaData.topics_with_message_count.size(); i++) {
        const auto topicWithMessageCount = bagMetaData.topics_with_message_count.at(i);
        const auto& topicMetaData = topicWithMessageCount.topic_metadata;

        const auto it = std::find_if(m_editBagParameters.topics.begin(), m_editBagParameters.topics.end(), [topicMetaData] (const auto& editBagTopic) {
            return editBagTopic.originalTopicName.toStdString() == topicMetaData.name;
        });
        // If the settings do not contain any topic items, create them
        const auto itemAlreadyExists = it != m_editBagParameters.topics.end();
        if (!itemAlreadyExists) {
            Utils::UI::EditBagTopic editBagTopic;
            editBagTopic.originalTopicName = QString::fromStdString(topicMetaData.name);
            editBagTopic.upperBoundary = topicWithMessageCount.message_count;
            m_editBagParameters.topics.push_back(editBagTopic);
        }

        auto& editBagItem = itemAlreadyExists ? *it : m_editBagParameters.topics.back();

        auto* const item = new QTreeWidgetItem;
        m_treeWidget->addTopLevelItem(item);

        item->setFlags(item->flags() & ~Qt::ItemIsSelectable);
        item->setCheckState(COL_CHECKBOXES, editBagItem.isSelected ? Qt::Checked : Qt::Unchecked);
        // Create item widgets
        auto* const topicLabel = new QLabel(QString::fromStdString(topicMetaData.name));
        topicLabel->setEnabled(editBagItem.isSelected);
        auto* const messageCountWidget = new MessageCountWidget(editBagItem.lowerBoundary, topicWithMessageCount.message_count - 1, editBagItem.upperBoundary);
        messageCountWidget->setEnabled(editBagItem.isSelected);
        auto* const renamingLineEdit = new QLineEdit(editBagItem.renamedTopicName);
        renamingLineEdit->setEnabled(editBagItem.isSelected);
        m_treeWidget->setItemWidget(item, COL_TOPICS, topicLabel);
        m_treeWidget->setItemWidget(item, COL_MESSAGE_COUNT, messageCountWidget);
        m_treeWidget->setItemWidget(item, COL_RENAMING, renamingLineEdit);

        connect(messageCountWidget, &MessageCountWidget::lowerValueChanged, this, [i, this](int value) {
            writeSettingsParameter(m_editBagParameters.topics[i].lowerBoundary, static_cast<size_t>(value), m_editBagParamSettings);
        });
        connect(messageCountWidget, &MessageCountWidget::upperValueChanged, this, [i, this](int value) {
            writeSettingsParameter(m_editBagParameters.topics[i].upperBoundary, static_cast<size_t>(value), m_editBagParamSettings);
        });
        connect(renamingLineEdit, &QLineEdit::textChanged, this, [i, this](const QString& text) {
            writeSettingsParameter(m_editBagParameters.topics[i].renamedTopicName, text, m_editBagParamSettings);
        });
    }

    m_treeWidget->resizeColumnToContents(COL_CHECKBOXES);
    m_treeWidget->resizeColumnToContents(COL_TOPICS);
    m_treeWidget->resizeColumnToContents(COL_MESSAGE_COUNT);
    m_treeWidget->resizeColumnToContents(COL_RENAMING);
    m_treeWidget->blockSignals(false);

    m_editLabel->setVisible(true);
    m_treeWidget->setVisible(true);
    m_targetBagNameWidget->setVisible(true);
    m_okButton->setVisible(true);
}


void
EditBagWidget::itemCheckStateChanged(QTreeWidgetItem* item, int column)
{
    if (column != COL_CHECKBOXES) {
        return;
    }
    // Disable item widgets as well
    m_treeWidget->itemWidget(item, COL_TOPICS)->setEnabled(item->checkState(COL_CHECKBOXES) == Qt::Checked ? true : false);
    m_treeWidget->itemWidget(item, COL_MESSAGE_COUNT)->setEnabled(item->checkState(COL_CHECKBOXES) == Qt::Checked ? true : false);
    m_treeWidget->itemWidget(item, COL_RENAMING)->setEnabled(item->checkState(COL_CHECKBOXES) == Qt::Checked ? true : false);

    const auto rowIndex = m_treeWidget->indexOfTopLevelItem(item);
    writeSettingsParameter(m_editBagParameters.topics[rowIndex].isSelected, item->checkState(COL_CHECKBOXES) == Qt::Checked, m_editBagParamSettings);
}


void
EditBagWidget::targetPushButtonPressed()
{
    const auto fileName = QFileDialog::getSaveFileName(this, "Target Bag File", "", "");
    if (fileName.isEmpty()) {
        return;
    }

    m_targetLineEdit->setText(fileName);
    writeSettingsParameter(m_editBagParameters.targetDirectory, fileName, m_editBagParamSettings);
}


void
EditBagWidget::okButtonPressed()
{
    QTreeWidgetItemIterator it(m_treeWidget);
    while (*it) {
        auto* const messageCountWidget = dynamic_cast<MessageCountWidget*>(m_treeWidget->itemWidget((*it), COL_MESSAGE_COUNT));
        if (messageCountWidget->getHigherValue() <= messageCountWidget->getLowerValue()) {
            auto *const msgBox = new QMessageBox(QMessageBox::Warning, "Message count invalid!",
                                                 "Please make sure that the lower message count is actually lower than the higher message count!",
                                                 QMessageBox::Ok);
            msgBox->exec();
            return;
        }
        auto* const renamingLineEdit = dynamic_cast<QLineEdit*>(m_treeWidget->itemWidget((*it), COL_RENAMING));
        if (!renamingLineEdit->text().isEmpty() && !Utils::ROS::doesTopicNameFollowROS2Convention(renamingLineEdit->text())) {
            auto *const msgBox = new QMessageBox(QMessageBox::Warning, "Renamed topic name(s) invalid!",
                                                 "Please make sure that the new topic names are following the ROS2 naming convention!",
                                                 QMessageBox::Ok);
            msgBox->exec();
            return;
        }

        ++it;
    }
    if (m_targetLineEdit->text().isEmpty()) {
        auto *const msgBox = new QMessageBox(QMessageBox::Warning, "No target specified!", "Please make sure that a target file has been entered!",
                                             QMessageBox::Ok);
        msgBox->exec();
        return;
    }

    if (std::filesystem::exists(m_editBagParameters.targetDirectory.toStdString())) {
        auto *const msgBox = new QMessageBox(QMessageBox::Warning, "Bagfile already exists!",
                                             "A bag file already exists under the specified directory! Are you sure you want to continue? "
                                             "This will overwrite the existing file.",
                                             QMessageBox::Yes | QMessageBox::No);
        if (const auto ret = msgBox->exec(); ret == QMessageBox::No) {
            return;
        }
    }

    emit okPressed();
}
