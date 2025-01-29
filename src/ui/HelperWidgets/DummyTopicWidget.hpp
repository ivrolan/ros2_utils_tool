#pragma once

#include <QComboBox>
#include <QLineEdit>
#include <QPointer>
#include <QWidget>

// Small helper widget used to select a topic type for the ROS bag dummy creation
class DummyTopicWidget : public QWidget
{
    Q_OBJECT

public:
    DummyTopicWidget(const QString& topicTypeText = "",
                     const QString& topicNameText = "",
                     QWidget*       parent = 0);

    const QString
    getTopicType() const
    {
        return m_topicTypeComboBox->currentText();
    }

    const QString
    getTopicName() const
    {
        return m_topicNameLineEdit->text();
    }

signals:
    void
    topicTypeChanged(QString type);

    void
    topicNameChanged(QString name);

private:
    QPointer<QComboBox> m_topicTypeComboBox;
    QPointer<QLineEdit> m_topicNameLineEdit;
};
