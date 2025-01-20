#pragma once

#include "BasicThread.hpp"
#include "UtilsUI.hpp"

// Thread used to write an edited ROS bag file
class EditBagThread : public BasicThread {
    Q_OBJECT
public:
    explicit
    EditBagThread(const Utils::UI::EditBagInputParameters& parameters,
                  QObject*                                 parent = nullptr);

    void
    run() override;

private:
    const Utils::UI::EditBagInputParameters& m_parameters;
};
