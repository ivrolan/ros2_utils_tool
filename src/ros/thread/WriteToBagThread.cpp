#include "WriteToBagThread.hpp"

#include <opencv2/videoio.hpp>

#include "rosbag2_cpp/writer.hpp"

#include "sensor_msgs/msg/image.hpp"

#include <filesystem>

#ifdef ROS_JAZZY
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

WriteToBagThread::WriteToBagThread(const Utils::UI::BagParameters& bagParameters,
                                   QObject*                        parent) :
    BasicThread(bagParameters.sourceDirectory, bagParameters.topicName, parent),
    m_targetDirectory(bagParameters.targetDirectory.toStdString()),
    m_fps(bagParameters.fps), m_useHardwareAcceleration(bagParameters.useHardwareAcceleration),
    m_useCDRForSerialization(bagParameters.useCDRForSerialization)
{
}


void
WriteToBagThread::run()
{
    auto videoCapture = cv::VideoCapture(m_sourceDirectory, cv::CAP_ANY, {
        cv::CAP_PROP_HW_ACCELERATION, m_useHardwareAcceleration ? cv::VIDEO_ACCELERATION_ANY : cv::VIDEO_ACCELERATION_NONE
    });
    if (!videoCapture.isOpened()) {
        emit openingCVInstanceFailed();
        return;
    }

    if (std::filesystem::exists(m_targetDirectory)) {
        std::filesystem::remove_all(m_targetDirectory);
    }

    const auto frameCount = videoCapture.get(cv::CAP_PROP_FRAME_COUNT);
    emit calculatedMaximumInstances(frameCount);

    rosbag2_cpp::Writer writer;
    writer.open(m_targetDirectory);
    auto iterationCount = 0;

    rosbag2_storage::TopicMetadata topicMetadata;
    topicMetadata.name = m_topicName;
    topicMetadata.type = "sensor_msgs/msg/Image";
    topicMetadata.serialization_format = m_useCDRForSerialization ? "cdr" : "sqlite3";
    writer.create_topic(topicMetadata);

    while (true) {
        if (isInterruptionRequested()) {
            return;
        }

        // Create image
        cv::Mat frame;
        videoCapture >> frame;
        if (frame.empty()) {
            break;
        }

        iterationCount++;

        // Create empty sensor message
        sensor_msgs::msg::Image message;
        std_msgs::msg::Header header;
        const auto seconds = (float) iterationCount / m_fps;
        const auto time = rclcpp::Time(seconds, seconds * 1000000000);
        header.stamp = time;

        // Convert image and write
        const auto cvBridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, frame);
        cvBridge.toImageMsg(message);
        writer.write(message, m_topicName, time);

        emit progressChanged(iterationCount, ((float) iterationCount / (float) frameCount) * 100);
    }

    emit finished();
}
