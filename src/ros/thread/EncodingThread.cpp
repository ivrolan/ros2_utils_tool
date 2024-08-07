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

    // Prepare parameters
    rclcpp::Serialization<sensor_msgs::msg::Image> serialization;
    cv_bridge::CvImagePtr cvPointer;
    const auto videoEncoder = std::make_shared<VideoEncoder>(m_vidDirectory.right(3) == "mp4");
    auto iterationCount = 0;
    reader.open(m_bagDirectory.toStdString());

    // Now the main encoding
    while (reader.has_next()) {
        if (isInterruptionRequested()) {
            reader.close();
            return;
        }

        // Read and deserialize the message
        rosbag2_storage::SerializedBagMessageSharedPtr msg = reader.read_next();
        rclcpp::SerializedMessage serializedMessage(*msg->serialized_data);
        auto rosMsg = std::make_shared<sensor_msgs::msg::Image>();
        serialization.deserialize_message(&serializedMessage, rosMsg.get());

        // Setup the video encoder on the first iteration
        if (iterationCount == 0) {
            const auto width = rosMsg->width;
            const auto height = rosMsg->height;

            if (!videoEncoder->setVideoWriter(m_vidDirectory.toStdString(), width, height, m_useHardwareAcceleration)) {
                emit openingCVInstanceFailed();
                return;
            }
        }

        // Convert message to cv and encode
        cvPointer = cv_bridge::toCvCopy(*rosMsg, rosMsg->encoding);
        videoEncoder->writeImageToVideo(cvPointer->image);

        iterationCount++;
        // Inform of progress update
        emit progressChanged(iterationCount, ((float) iterationCount / (float) messageCount) * 100);
    }

    reader.close();
    emit finished();
}
