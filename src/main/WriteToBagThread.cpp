#include "WriteToBagThread.hpp"

#include <cv_bridge/cv_bridge.h>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/writer.hpp"

#include "sensor_msgs/msg/image.hpp"

#include "Utils.hpp"

WriteToBagThread::WriteToBagThread(const QString& bagDirectory,
                                   const QString& topicName,
                                   const QString& vidDirectory,
                                   bool           useHardwareAcceleration,
                                   QObject*       parent) :
    BasicThread(bagDirectory, topicName, vidDirectory, useHardwareAcceleration, parent)
{
}


void
WriteToBagThread::run()
{
    auto videoCapture = cv::VideoCapture(m_vidDirectory.toStdString(), cv::CAP_ANY, {
        cv::CAP_PROP_HW_ACCELERATION, m_useHardwareAcceleration ? cv::VIDEO_ACCELERATION_VAAPI : cv::VIDEO_ACCELERATION_NONE
    });
    if (!videoCapture.isOpened()) {
        emit openingCVInstanceFailed();
        return;
    }

    const auto frameCount = videoCapture.get(cv::CAP_PROP_FRAME_COUNT);
    emit calculatedMaximumInstances(frameCount);

    rosbag2_cpp::Writer writer;
    writer.open(m_bagDirectory.toStdString());
    auto iterationCount = 0;

    while (1) {
        if (isInterruptionRequested()) {
            return;
        }

        cv::Mat frame;
        videoCapture >> frame;
        if (frame.empty()) {
            break;
        }

        sensor_msgs::msg::Image message;
        std_msgs::msg::Header header;
        const auto time = rclcpp::Clock(RCL_ROS_TIME).now();
        header.stamp = time;

        // Convert back to sensor_msgs and write
        const auto cvBridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, frame);
        cvBridge.toImageMsg(message);
        writer.write(message, m_topicName.toStdString(), time);

        iterationCount++;
        emit progressChanged(iterationCount, ((float) iterationCount / (float) frameCount) * 100);
    }

    emit finished();
}
