#pragma once

#include "BasicInputWidget.hpp"
#include "DummyBagInputSettings.hpp"
#include "UtilsUI.hpp"

#include <QPointer>
#include <QWidget>

class DummyTopicWidget;

class QFormLayout;
class QToolButton;

/**
 * @brief The widget used to manage creating a ROS bag with dummy data
 */
class DummyBagWidget : public BasicInputWidget
{
    Q_OBJECT

public:
    DummyBagWidget(Utils::UI::DummyBagInputParameters& parameters,
                   QWidget*                            parent = 0);

private slots:
    void
    bagDirectoryButtonPressed();

    void
    removeDummyTopicWidget();

    void
    createNewDummyTopicWidget(const Utils::UI::DummyBagInputParameters::DummyBagTopic& topics,
                              int                                                      index);

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

    QPointer<QToolButton> m_minusButton;
    QPointer<QToolButton> m_plusButton;

    Utils::UI::DummyBagInputParameters& m_parameters;

    DummyBagInputSettings m_settings;

    int m_numberOfTopics = 0;

    static constexpr int MAXIMUM_NUMBER_OF_TOPICS = 3;
};
