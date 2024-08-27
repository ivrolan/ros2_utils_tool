#pragma once

#include "BasicProgressWidget.hpp"

/**
 * @brief Progress widget variant used to display video encoding or writing a video to a bag
 */
class VideoProgressWidget : public BasicProgressWidget
{
    Q_OBJECT

public:
    VideoProgressWidget(const QString& bagDirectory,
                        const QString& topicName,
                        const QString& vidDirectory,
                        bool           useHardwareAcceleration,
                        bool           isEncoding,
                        QWidget*       parent = 0);
};
