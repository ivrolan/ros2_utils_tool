#pragma once

#include <QPointer>
#include <QSpinBox>
#include <QWidget>

/**
 * @brief Display the desired final message count of a to be edited ROS bag
 */
class MessageCountWidget : public QWidget
{
    Q_OBJECT

public:
    MessageCountWidget(int      minimum,
                       int      maximum,
                       int      currentMaximumValue,
                       QWidget* parent = 0);

signals:
    void
    lowerValueChanged(int value);

    void
    upperValueChanged(int value);

public:
    int
    getLowerValue() const
    {
        return m_lowerBox->value();
    }

    int
    getHigherValue() const
    {
        return m_upperBox->value();
    }

private:
    QPointer<QSpinBox> m_lowerBox;
    QPointer<QSpinBox> m_upperBox;
};
