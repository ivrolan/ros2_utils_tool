#pragma once

#include "BasicThread.hpp"
#include "UtilsUI.hpp"

// Encoding thread, encoding a video out of a ROSBag
class WriteToBagThread : public BasicThread {
    Q_OBJECT

public:
    explicit
    WriteToBagThread(const Utils::UI::BagInputParameters& parameters,
                     QObject*                             parent = nullptr);

    void
    run() override;

private:
    const Utils::UI::BagInputParameters& m_parameters;
};
