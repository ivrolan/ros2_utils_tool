#include "DummyBagThread.hpp"

#include "UtilsROS.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/writer.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

#ifdef ROS_JAZZY
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

DummyBagThread::DummyBagThread(const Utils::UI::DummyBagParameters& dummyBagParameters,
                               QObject*                             parent) :
    BasicThread(dummyBagParameters.bagDirectory, dummyBagParameters.topicName, parent),
    m_topicTypes(dummyBagParameters.topicTypes), m_topicNames(dummyBagParameters.topicNames),
    m_messageCount(dummyBagParameters.messageCount)
{
}


void
DummyBagThread::run()
{
    emit calculatedMaximumInstances(m_messageCount);

    rosbag2_cpp::Writer writer;
    writer.open(m_bagDirectory.toStdString());

    for (auto i = 1; i <= m_messageCount; i++) {
        for (auto j = 0; j < m_topicTypes.size(); j++) {
            const auto timeStamp = rclcpp::Clock().now();

            if (m_topicTypes.at(j) == "String") {
                std_msgs::msg::String message;
                message.data = "Message " + std::to_string(i);

                writer.write(message, m_topicNames.at(j).toStdString(), timeStamp);
            } else if (m_topicTypes.at(j) == "Integer") {
                std_msgs::msg::Int32 message;
                message.data = i;

                writer.write(message, m_topicNames.at(j).toStdString(), timeStamp);
            } else if (m_topicTypes.at(j) == "Image") {
                cv::Mat mat(720, 1280, CV_8UC3, cv::Scalar(255, 0, 0));
                sensor_msgs::msg::Image message;
                std_msgs::msg::Header header;
                header.stamp = timeStamp;

                const auto cvBridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, mat);
                cvBridge.toImageMsg(message);
                writer.write(message, m_topicNames.at(j).toStdString(), timeStamp);
            }
        }

        emit progressChanged(i, ((float) i / (float) m_messageCount) * 100);
    }

    writer.close();
    emit finished();
}
