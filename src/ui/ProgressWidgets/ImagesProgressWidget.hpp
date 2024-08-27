#pragma once

#include "BasicProgressWidget.hpp"

/**
 * @brief Progress widget variant used to display writing images out of a ROSBag
 */
class ImagesProgressWidget : public BasicProgressWidget
{
    Q_OBJECT

public:
    ImagesProgressWidget(const QString& bagDirectory,
                         const QString& topicName,
                         const QString& vidDirectory,
                         const QString& format = "",
                         int            compressionLevel = 90,
                         QWidget*       parent = 0);
};
