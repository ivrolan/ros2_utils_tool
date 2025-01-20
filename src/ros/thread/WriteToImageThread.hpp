#pragma once

#include "BasicThread.hpp"
#include "UtilsUI.hpp"

// Encoding thread, encoding a video out of a ROSBag
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
