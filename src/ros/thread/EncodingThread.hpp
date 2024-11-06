#pragma once

#include "BasicThread.hpp"
#include "UtilsUI.hpp"

// Encoding thread, encoding a video out of a ROSBag
class EncodingThread : public BasicThread {
    Q_OBJECT
public:
    explicit
    EncodingThread(const Utils::UI::VideoParameters& videoParameters,
                   QObject*                          parent = nullptr);

    void
    run() override;

private:
    const std::string m_targetDirectory;
    const std::string m_format;

    const int m_fps;

    const bool m_useHardwareAcceleration;
    const bool m_useBWImages;
    const bool m_lossless;
};
