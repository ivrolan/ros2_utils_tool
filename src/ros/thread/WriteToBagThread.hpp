#pragma once

#include "BasicThread.hpp"
#include "UtilsUI.hpp"

// Thread used to weite a video to a bag
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
