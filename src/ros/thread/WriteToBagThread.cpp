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
    m_bagParameters(bagParameters)
{
}


void
WriteToBagThread::run()
{
    auto videoCapture = cv::VideoCapture(m_sourceDirectory, cv::CAP_ANY, {
        cv::CAP_PROP_HW_ACCELERATION, m_bagParameters.useHardwareAcceleration ? cv::VIDEO_ACCELERATION_ANY : cv::VIDEO_ACCELERATION_NONE
    });
    if (!videoCapture.isOpened()) {
        emit openingCVInstanceFailed();
        return;
    }

    const auto targetDirectoryStd = m_bagParameters.targetDirectory.toStdString();
    if (std::filesystem::exists(targetDirectoryStd)) {
        std::filesystem::remove_all(targetDirectoryStd);
    }

    const auto frameCount = videoCapture.get(cv::CAP_PROP_FRAME_COUNT);
    emit calculatedMaximumInstances(frameCount);

    const auto finalFPS = m_bagParameters.useCustomFPS ? m_bagParameters.fps : videoCapture.get(cv::CAP_PROP_FPS);

    rosbag2_cpp::Writer writer;
    writer.open(targetDirectoryStd);
    auto iterationCount = 0;

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
        const auto seconds = (float) iterationCount / finalFPS;
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
