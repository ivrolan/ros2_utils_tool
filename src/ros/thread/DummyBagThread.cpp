#include "DummyBagThread.hpp"

#include "UtilsROS.hpp"

#include "sensor_msgs/msg/image.hpp"

#ifdef ROS_JAZZY
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

DummyBagThread::DummyBagThread(const Utils::UI::DummyBagParameters& dummyBagParameters,
                               QObject*                             parent) :
    BasicThread(dummyBagParameters.sourceDirectory, dummyBagParameters.topicName, parent),
    m_dummyBagParameters(dummyBagParameters)
{
}


void
DummyBagThread::run()
{
    emit calculatedMaximumInstances(m_dummyBagParameters.messageCount);

    rosbag2_cpp::Writer writer;
    writer.open(m_sourceDirectory);

    for (auto i = 1; i <= m_dummyBagParameters.messageCount; i++) {
        for (auto j = 0; j < m_dummyBagParameters.topics.size(); j++) {
            const auto timeStamp = rclcpp::Clock().now();

            if (m_dummyBagParameters.topics.at(j).type == "String") {
                Utils::ROS::writeMessage(std_msgs::msg::String(), "Message " + std::to_string(i), writer, m_dummyBagParameters.topics.at(j).name, timeStamp);
            } else if (m_dummyBagParameters.topics.at(j).type == "Integer") {
                Utils::ROS::writeMessage(std_msgs::msg::Int32(), i, writer, m_dummyBagParameters.topics.at(j).name, timeStamp);
            } else if (m_dummyBagParameters.topics.at(j).type == "Image") {
                cv::Mat mat(720, 1280, CV_8UC3, cv::Scalar(255, 0, 0));
                sensor_msgs::msg::Image message;
                std_msgs::msg::Header header;
                header.stamp = timeStamp;

                const auto cvBridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, mat);
                cvBridge.toImageMsg(message);
                writer.write(message, m_dummyBagParameters.topics.at(j).name.toStdString(), timeStamp);
            }
        }

        emit progressChanged(i, ((float) i / (float) m_dummyBagParameters.messageCount) * 100);
    }

    writer.close();
    emit finished();
}
