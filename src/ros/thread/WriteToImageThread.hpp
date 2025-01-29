#pragma once

#include "BasicThread.hpp"
#include "UtilsUI.hpp"

// Thread used to write images out of a ROS bag
class WriteToImageThread : public BasicThread {
    Q_OBJECT
public:
    explicit
    WriteToImageThread(const Utils::UI::ImageInputParameters& parameters,
                       QObject*                               parent = nullptr);

    void
    run() override;

private:
    const Utils::UI::ImageInputParameters& m_parameters;
};
