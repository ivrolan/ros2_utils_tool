#pragma once

#include "BasicThread.hpp"
#include "UtilsUI.hpp"

// Encoding thread, encoding a video out of a ROSBag
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
