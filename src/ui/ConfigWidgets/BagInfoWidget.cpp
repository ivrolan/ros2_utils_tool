#include "BagInfoWidget.hpp"

#include <QDialogButtonBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QToolButton>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QVBoxLayout>

#include "UtilsROS.hpp"

BagInfoWidget::BagInfoWidget(Utils::UI::BasicParameters& bagInfoParameters, QWidget *parent) :
    BasicInputWidget("Get Infos from ROSBag", ":/icons/bag_to_video_white.svg", ":/icons/bag_to_video_black.svg", parent),
    m_bagInfoParameters(bagInfoParameters)
{
    auto* const bagLineEdit = new QLineEdit();
    bagLineEdit->setToolTip("The directory of the ROSBag source file.");

    auto* const searchBagButton = new QToolButton;
    auto* const searchBagFileLayout = Utils::UI::createLineEditButtonLayout(bagLineEdit, searchBagButton);

    auto* const formLayout = new QFormLayout;
    formLayout->addRow("Bag File:", searchBagFileLayout);

    auto* const formLayoutSqueezed = new QHBoxLayout;
    formLayoutSqueezed->addStretch();
    formLayoutSqueezed->addLayout(formLayout);
    formLayoutSqueezed->addStretch();

    m_infoTreeWidget = new QTreeWidget;
    m_infoTreeWidget->setColumnCount(2);
    m_infoTreeWidget->headerItem()->setText(COL_DESCRIPTION, "Metadata");
    m_infoTreeWidget->headerItem()->setText(COL_INFORMATION, "Values");
    m_infoTreeWidget->setVisible(false);
    m_infoTreeWidget->setRootIsDecorated(false);

    auto* const controlsLayout = new QVBoxLayout;
    controlsLayout->addStretch();
    controlsLayout->addWidget(m_headerPixmapLabel);
    controlsLayout->addWidget(m_headerLabel);
    controlsLayout->addSpacing(40);
    controlsLayout->addLayout(formLayoutSqueezed);
    controlsLayout->addSpacing(10);
    controlsLayout->addWidget(m_infoTreeWidget);
    controlsLayout->addStretch();

    auto* const backButton = new QPushButton("Back");

    auto* const buttonBox = new QDialogButtonBox;
    buttonBox->addButton(m_okButton, QDialogButtonBox::AcceptRole);
    m_okButton->setEnabled(true);
    m_okButton->setVisible(false);

    auto* const buttonLayout = new QHBoxLayout;
    buttonLayout->addWidget(backButton);
    buttonLayout->addStretch();
    buttonLayout->addWidget(buttonBox);

    auto* const mainLayout = new QVBoxLayout;
    mainLayout->addLayout(controlsLayout);
    mainLayout->addLayout(buttonLayout);
    setLayout(mainLayout);

    connect(searchBagButton, &QPushButton::clicked, this, &BagInfoWidget::displayBagInfo);
    connect(backButton, &QPushButton::clicked, this, [this] {
        emit back();
    });
    connect(m_okButton, &QPushButton::clicked, this, [this] {
        emit back();
    });
}


void
BagInfoWidget::displayBagInfo()
{
    const auto bagDirectory = QFileDialog::getExistingDirectory(this, "Open ROSBag", "",
                                                                QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    if (bagDirectory.isEmpty()) {
        return;
    }

    m_infoTreeWidget->clear();
    m_bagInfoParameters.bagDirectory = bagDirectory;
    const auto& bagMetaData = Utils::ROS::getBagMetadata(bagDirectory.toStdString());

    QList<QTreeWidgetItem*> treeWidgetItems;
    treeWidgetItems.append(new QTreeWidgetItem({ "Duration (Nanoseconds):", QString::number(bagMetaData.duration.count()) }));
    treeWidgetItems.append(new QTreeWidgetItem({ "Duration (Seconds):", QString::number((float) bagMetaData.duration.count() / 1000000000) }));
    treeWidgetItems.append(new QTreeWidgetItem({ "Size:", QString::number((float) bagMetaData.bag_size / 1000000000) + " GB" }));
    treeWidgetItems.append(new QTreeWidgetItem({ "Size (Message Count):", QString::number(bagMetaData.message_count) }));

    auto* const topicMetaDataItem = new QTreeWidgetItem({ "Topic Information:" });

    for (const auto& topicInformation : bagMetaData.topics_with_message_count) {
        const auto& topicMetaData = topicInformation.topic_metadata;

        auto* const topicInformationItem = new QTreeWidgetItem(topicMetaDataItem);
        topicInformationItem->setText(COL_DESCRIPTION, QString::fromStdString(topicMetaData.name));

        auto* const topicTypeItem = new QTreeWidgetItem(topicInformationItem);
        topicTypeItem->setText(COL_DESCRIPTION, "Type:");
        topicTypeItem->setText(COL_INFORMATION, QString::fromStdString(topicMetaData.type));
        auto* const topicMessageCountItem = new QTreeWidgetItem(topicInformationItem);
        topicMessageCountItem->setText(COL_DESCRIPTION, "Message Count:");
        topicMessageCountItem->setText(COL_INFORMATION, QString::number(topicInformation.message_count));
    }

    treeWidgetItems.append(topicMetaDataItem);

    m_infoTreeWidget->addTopLevelItems(treeWidgetItems);
    m_infoTreeWidget->expandAll();
    m_infoTreeWidget->resizeColumnToContents(COL_DESCRIPTION);
    m_infoTreeWidget->setVisible(true);
    m_okButton->setVisible(true);
}
