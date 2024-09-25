#include "DummyBagWidget.hpp"

#include "DummyTopicWidget.hpp"
#include "UtilsROS.hpp"

#include <QDialogButtonBox>
#include <QEvent>
#include <QFileDialog>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QSet>
#include <QSpinBox>
#include <QToolButton>
#include <QVBoxLayout>

DummyBagWidget::DummyBagWidget(Utils::UI::DummyBagParameters& dummyBagParameters, QWidget *parent) :
    BasicInputWidget("Create Dummy ROSBag", ":/icons/dummy_bag_white.svg", ":/icons/dummy_bag_black.svg", parent),
    m_dummyBagParameters(dummyBagParameters)
{
    m_bagNameLineEdit = new QLineEdit(m_dummyBagParameters.bagDirectory);
    m_bagNameLineEdit->setToolTip("The directory where the ROSBag file should be stored.");

    auto* const bagDirectoryButton = new QToolButton;
    auto* const bagDirectoryLayout = Utils::UI::createLineEditButtonLayout(m_bagNameLineEdit, bagDirectoryButton);

    m_messageCountSpinBox = new QSpinBox;
    m_messageCountSpinBox->setRange(1, 1000);
    m_messageCountSpinBox->setToolTip("The number of messages stored in the ROSBag.");
    m_messageCountSpinBox->setValue(m_dummyBagParameters.messageCount);

    m_minusButton = new QToolButton;
    m_minusButton->setToolTip("Remove the topic above.");
    m_plusButton = new QToolButton;
    m_plusButton->setToolTip("Add another topic. Currently, a maximum of 3 topics can be added.");

    auto* const plusMinusButtonLayout = new QHBoxLayout;
    plusMinusButtonLayout->addStretch();
    plusMinusButtonLayout->addWidget(m_minusButton);
    plusMinusButtonLayout->addWidget(m_plusButton);

    m_formLayout = new QFormLayout;
    m_formLayout->addRow("Bag File:", bagDirectoryLayout);
    m_formLayout->addRow("", plusMinusButtonLayout);
    m_formLayout->addRow("Message Count:", m_messageCountSpinBox);

    auto* const controlsLayout = new QVBoxLayout;
    controlsLayout->addStretch();
    controlsLayout->addWidget(m_headerPixmapLabel);
    controlsLayout->addWidget(m_headerLabel);
    controlsLayout->addSpacing(40);
    controlsLayout->addLayout(m_formLayout);
    controlsLayout->addSpacing(20);
    controlsLayout->addStretch();

    auto* const controlsSqueezedLayout = new QHBoxLayout;
    controlsSqueezedLayout->addStretch();
    controlsSqueezedLayout->addLayout(controlsLayout);
    controlsSqueezedLayout->addStretch();

    m_okButton->setEnabled(true);

    auto* const mainLayout = new QVBoxLayout;
    mainLayout->addLayout(controlsSqueezedLayout);
    mainLayout->addLayout(m_buttonLayout);
    setLayout(mainLayout);

    const auto addNewTopic = [this] {
        m_dummyBagParameters.topicTypes.push_back("String");
        m_dummyBagParameters.topicNames.push_back("");
        createNewDummyTopicWidget(m_dummyBagParameters.topicTypes.size() - 1);
    };

    connect(bagDirectoryButton, &QPushButton::clicked, this, &DummyBagWidget::bagDirectoryButtonPressed);
    connect(m_messageCountSpinBox, QOverload<int>::of(&QSpinBox::valueChanged), this, [this] (int value) {
        m_dummyBagParameters.messageCount = value;
    });
    connect(m_minusButton, &QPushButton::clicked, this, &DummyBagWidget::removeDummyTopicWidget);
    connect(m_plusButton, &QPushButton::clicked, this, [addNewTopic] {
        addNewTopic();
    });
    connect(m_okButton, &QPushButton::clicked, this, &DummyBagWidget::okButtonPressed);

    setPixmapLabelIcon();

    for (auto i = 0; i < m_dummyBagParameters.topicTypes.size(); i++) {
        createNewDummyTopicWidget(i, m_dummyBagParameters.topicTypes.at(i), m_dummyBagParameters.topicNames.at(i));
    }
    if (m_dummyBagParameters.topicTypes.empty()) {
        addNewTopic();
    }
}


void
DummyBagWidget::bagDirectoryButtonPressed()
{
    const auto fileName = QFileDialog::getSaveFileName(this, "Save ROSBag");
    if (fileName.isEmpty()) {
        return;
    }

    m_dummyBagParameters.bagDirectory = fileName;
    m_bagNameLineEdit->setText(fileName);
}


void
DummyBagWidget::removeDummyTopicWidget()
{
    m_formLayout->removeRow(m_dummyBagParameters.topicTypes.size());
    m_dummyTopicWidgets.pop_back();

    m_dummyBagParameters.topicTypes.pop_back();
    m_dummyBagParameters.topicNames.pop_back();

    m_plusButton->setEnabled(m_numberOfTopics != MAXIMUM_NUMBER_OF_TOPICS);
    m_minusButton->setEnabled(m_dummyBagParameters.topicTypes.size() != 1);
}


void
DummyBagWidget::createNewDummyTopicWidget(int index, const QString& topicTypeText, const QString& topicNameText)
{
    auto* const dummyTopicWidget = new DummyTopicWidget(topicTypeText, topicNameText);

    connect(dummyTopicWidget, &DummyTopicWidget::topicTypeChanged, this, [this, index] (const QString& text) {
        m_dummyBagParameters.topicTypes[index] = text;
    });
    connect(dummyTopicWidget, &DummyTopicWidget::topicNameChanged, this, [this, index] (const QString& text) {
        m_dummyBagParameters.topicNames[index] = text;
    });

    m_formLayout->insertRow(m_formLayout->rowCount() - 2, "Topic " + QString::number(m_numberOfTopics + 1) + ":", dummyTopicWidget);
    m_dummyTopicWidgets.push_back(dummyTopicWidget);

    m_numberOfTopics++;
    m_plusButton->setEnabled(m_numberOfTopics != MAXIMUM_NUMBER_OF_TOPICS);
    m_minusButton->setEnabled(m_dummyBagParameters.topicTypes.size() != 1);
}


void
DummyBagWidget::okButtonPressed()
{
    if (m_bagNameLineEdit->text().isEmpty()) {
        Utils::UI::createCriticalMessageBox("No bag name specified!", "Please specify a bag name before continuing!");
        return;
    }

    // Sets remove duplicates, so use a set to check if duplicate topic names exist
    QSet<QString> topicNameSet;

    for (QPointer<DummyTopicWidget> dummyTopicWidget : m_dummyTopicWidgets) {
        if (dummyTopicWidget->getTopicName().isEmpty()) {
            Utils::UI::createCriticalMessageBox("Empty topic name!", "Please enter a topic name for every topic!");
            return;
        }
        if (!Utils::ROS::doesTopicNameFollowROS2Convention(dummyTopicWidget->getTopicName())) {
            Utils::UI::createCriticalMessageBox("Wrong topic name format!", "Please make sure that the topic names follow the ROS2 conventioning!");
            return;
        }

        topicNameSet.insert(dummyTopicWidget->getTopicName());
    }

    if (topicNameSet.size() != m_dummyTopicWidgets.size()) {
        Utils::UI::createCriticalMessageBox("Duplicate topic names!", "Please make sure that no duplicate topic names are used!");
        return;
    }

    emit okPressed();
}


void
DummyBagWidget::setPixmapLabelIcon()
{
    const auto isDarkMode = Utils::UI::isDarkMode();
    m_headerPixmapLabel->setPixmap(QIcon(isDarkMode ? m_pathLogoDark : m_pathLogoLight).pixmap(QSize(100, 45)));
    m_minusButton->setIcon(QIcon(isDarkMode ? ":/icons/minus_white.svg" : ":/icons/minus_black.svg"));
    m_plusButton->setIcon(QIcon(isDarkMode ? ":/icons/plus_white.svg" : ":/icons/plus_black.svg"));
}


bool
DummyBagWidget::event(QEvent *event)
{
    if (event->type() == QEvent::ApplicationPaletteChange || event->type() == QEvent::PaletteChange) {
        setPixmapLabelIcon();
    }
    return QWidget::event(event);
}
