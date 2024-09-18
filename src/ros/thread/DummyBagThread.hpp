#pragma once

#include "BasicThread.hpp"
#include "UtilsUI.hpp"

// Dummy bag thread, used to write dummy messages to a ROS bag
class DummyBagThread : public BasicThread {
    Q_OBJECT
public:
    explicit
    DummyBagThread(const Utils::UI::DummyBagParameters& dummyBagParameters,
                   QObject*                             parent = nullptr);

    void
    run() override;

private:
    const QVector<QString>& m_topicTypes;
    const QVector<QString>& m_topicNames;

    const int m_messageCount;
};
