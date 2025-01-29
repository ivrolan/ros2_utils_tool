#pragma once

#include "BasicThread.hpp"
#include "UtilsUI.hpp"

// Thread handling encoding a video out of a ROS bag
class EncodingThread : public BasicThread {
    Q_OBJECT
public:
    explicit
    EncodingThread(const Utils::UI::VideoInputParameters& parameters,
                   QObject*                               parent = nullptr);

    void
    run() override;

private:
    const Utils::UI::VideoInputParameters& m_parameters;
};
