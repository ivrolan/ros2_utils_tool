#include "EncodingThread.hpp"

#include "UtilsROS.hpp"
#include "VideoEncoder.hpp"

#include "rosbag2_cpp/reader.hpp"

#include "sensor_msgs/msg/image.hpp"

#include <filesystem>

#ifdef ROS_JAZZY
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

EncodingThread::EncodingThread(const Utils::UI::VideoParameters& videoParameters,
                               QObject*                          parent) :
    BasicThread(videoParameters.bagDirectory, videoParameters.topicName, parent),
    m_videoDirectory(videoParameters.videoDirectory.toStdString()),
    m_useHardwareAcceleration(videoParameters.useHardwareAcceleration)
{
}


void
EncodingThread::run()
{
    rosbag2_cpp::Reader reader;
    reader.open(m_bagDirectory);

    const auto messageCount = Utils::ROS::getTopicMessageCount(m_bagDirectory, m_topicName);
    emit calculatedMaximumInstances(messageCount);

    // Prepare parameters
    rclcpp::Serialization<sensor_msgs::msg::Image> serialization;
    cv_bridge::CvImagePtr cvPointer;
    auto iterationCount = 0;
    const auto topicNameStdString = m_topicName;
    const auto videoEncoder = std::make_shared<VideoEncoder>(std::filesystem::path(m_videoDirectory).extension() == ".mp4");

    // Now the main encoding
    while (reader.has_next()) {
        if (isInterruptionRequested()) {
            reader.close();
            return;
        }

        // Read and deserialize the message
        rosbag2_storage::SerializedBagMessageSharedPtr msg = reader.read_next();
        if (msg->topic_name != topicNameStdString) {
            continue;
        }

        rclcpp::SerializedMessage serializedMessage(*msg->serialized_data);
        auto rosMsg = std::make_shared<sensor_msgs::msg::Image>();
        serialization.deserialize_message(&serializedMessage, rosMsg.get());

        // Setup the video encoder on the first iteration
        if (iterationCount == 0) {
            const auto width = rosMsg->width;
            const auto height = rosMsg->height;

            if (!videoEncoder->setVideoWriter(m_videoDirectory, width, height, m_useHardwareAcceleration)) {
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
