#include "EncodingThread.hpp"

#include "UtilsROS.hpp"
#include "VideoEncoder.hpp"

#include <cv_bridge/cv_bridge.h>

#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/reader.hpp"

#include "sensor_msgs/msg/image.hpp"

EncodingThread::EncodingThread(const QString& bagDirectory,
                               const QString& topicName,
                               const QString& vidDirectory,
                               bool           useHardwareAcceleration,
                               QObject*       parent) :
    BasicThread(bagDirectory, topicName, vidDirectory, useHardwareAcceleration, parent)
{
}


void
EncodingThread::run()
{
    rosbag2_cpp::Reader reader;
    reader.open(m_bagDirectory.toStdString());

    const auto messageCount = Utils::ROS::getTopicMessageCount(m_bagDirectory.toStdString(), m_topicName.toStdString());
    emit calculatedMaximumInstances(messageCount);

    // Read a very first message to get its width and height value, which is needed for the video encoder
    auto firstMsg = reader.read_next();
    auto rosMsg = std::make_shared<sensor_msgs::msg::Image>();

    rclcpp::Serialization<sensor_msgs::msg::Image> serialization;
    rclcpp::SerializedMessage firstSerializedMessage(*firstMsg->serialized_data);
    serialization.deserialize_message(&firstSerializedMessage, rosMsg.get());
    const auto width = rosMsg->width;
    const auto height = rosMsg->height;

    const auto videoEncoder = std::make_shared<VideoEncoder>(m_vidDirectory.right(3) == "mp4");
    if (!videoEncoder->setVideoWriter(m_vidDirectory.toStdString(), width, height, m_useHardwareAcceleration)) {
        emit openingCVInstanceFailed();
        return;
    }

    // Now the main encoding
    auto iterationCount = 0;
    reader.open(m_bagDirectory.toStdString());

    cv_bridge::CvImagePtr cvPointer;

    while (reader.has_next()) {
        if (isInterruptionRequested()) {
            reader.close();
            return;
        }

        // Read and deserialize the message
        auto msg = reader.read_next();
        rclcpp::SerializedMessage serializedMessage(*msg->serialized_data);
        serialization.deserialize_message(&serializedMessage, rosMsg.get());
        // Convert message to cv and encode
        cvPointer = cv_bridge::toCvCopy(*rosMsg, sensor_msgs::image_encodings::BGR8);
        videoEncoder->writeImageToVideo(cvPointer->image);

        iterationCount++;
        // Inform of progress update
        emit progressChanged(iterationCount, ((float) iterationCount / (float) messageCount) * 100);
    }

    reader.close();
    emit finished();
}
