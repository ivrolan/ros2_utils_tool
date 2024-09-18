#pragma once

#include "BasicConfigWidget.hpp"
#include "UtilsUI.hpp"

#include <QPointer>
#include <QWidget>

class DummyTopicWidget;

class QFormLayout;
class QLineEdit;
class QPushButton;
class QSpinBox;
class QToolButton;

/**
 * @brief The widget used to manage creating a ROS bag with dummy data
 */
class DummyBagWidget : public BasicConfigWidget
{
    Q_OBJECT

public:
    DummyBagWidget(Utils::UI::DummyBagParameters& dummyBagParameters,
                   QWidget*                       parent = 0);

private slots:
    void
    bagDirectoryButtonPressed();

    void
    removeDummyTopicWidget();

    void
    createNewDummyTopicWidget(int            index,
                              const QString& topicTypeText = "",
                              const QString& topicNameText = "");

    void
    okButtonPressed();

private:
    void
    setPixmapLabelIcon();

    bool
    event(QEvent *event);

private:
    QVector<QPointer<DummyTopicWidget> > m_dummyTopicWidgets;

    QPointer<QFormLayout> m_formLayout;
    QPointer<QLineEdit> m_bagNameLineEdit;
    QPointer<QSpinBox> m_messageCountSpinBox;

    QPointer<QToolButton> m_minusButton;
    QPointer<QToolButton> m_plusButton;

    Utils::UI::DummyBagParameters& m_dummyBagParameters;

    int m_numberOfTopics = 0;

    static constexpr int MAXIMUM_NUMBER_OF_TOPICS = 3;
};
