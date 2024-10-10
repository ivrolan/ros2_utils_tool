#pragma once

#include "BasicThread.hpp"
#include "UtilsUI.hpp"

// Encoding thread, encoding a video out of a ROSBag
class WriteToBagThread : public BasicThread {
    Q_OBJECT

public:
    explicit
    WriteToBagThread(const Utils::UI::BagParameters& bagParameters,
                     QObject*                        parent = nullptr);

    void
    run() override;

private:
    const std::string m_videoDirectory;

    const int m_fps;
    const bool m_useHardwareAcceleration;
    const bool m_useCDRForSerialization;
};
