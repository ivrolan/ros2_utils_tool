#include "WriteToImageThread.hpp"

#include "UtilsROS.hpp"

#include <opencv2/imgcodecs.hpp>

#include "rosbag2_cpp/reader.hpp"

#include "sensor_msgs/msg/image.hpp"

#include <filesystem>

#ifdef ROS_JAZZY
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

WriteToImageThread::WriteToImageThread(const Utils::UI::ImageParameters& imageParameters,
                                       QObject*                          parent) :
    BasicThread(imageParameters.bagDirectory, imageParameters.topicName, parent),
    m_imagesDirectory(imageParameters.imagesDirectory.toStdString()),
    m_format(imageParameters.format.toStdString()), m_quality(imageParameters.quality), m_useBWImages(imageParameters.useBWImages),
    m_optimizeJPGOrPNGBinary(imageParameters.format == "jpg" ? imageParameters.jpgOptimize : imageParameters.pngBilevel)
{
}


void
WriteToImageThread::run()
{
    rosbag2_cpp::Reader reader;
    reader.open(m_bagDirectory);

    if (!std::filesystem::exists(m_imagesDirectory)) {
        std::filesystem::create_directory(m_imagesDirectory);
    }
    if (!std::filesystem::is_empty(m_imagesDirectory)) {
        // Remove all images currently present
        for (const auto& entry : std::filesystem::directory_iterator(m_imagesDirectory)) {
            std::filesystem::remove_all(entry.path());
        }
    }

    const auto messageCount = Utils::ROS::getTopicMessageCount(m_bagDirectory, m_topicName);
    emit calculatedMaximumInstances(messageCount);

    // Prepare parameters
    rclcpp::Serialization<sensor_msgs::msg::Image> serialization;
    cv_bridge::CvImagePtr cvPointer;
    auto iterationCount = 0;

    // Adjust the quality value to fit OpenCV param range
    if (m_format == "jpg") {
        m_quality = (m_quality * 10) + 10;
    }

    // Now the main encoding
    while (reader.has_next()) {
        if (isInterruptionRequested()) {
            reader.close();
            return;
        }

        // Read and deserialize the message
        rosbag2_storage::SerializedBagMessageSharedPtr msg = reader.read_next();
        if (msg->topic_name != m_topicName) {
            continue;
        }

        rclcpp::SerializedMessage serializedMessage(*msg->serialized_data);
        auto rosMsg = std::make_shared<sensor_msgs::msg::Image>();
        serialization.deserialize_message(&serializedMessage, rosMsg.get());

        // Convert message to cv and encode
        cvPointer = cv_bridge::toCvCopy(*rosMsg, rosMsg->encoding);

        if (m_useBWImages && (m_format == "jpg" || !m_optimizeJPGOrPNGBinary)) {
            cv::cvtColor(cvPointer->image, cvPointer->image, cv::COLOR_BGR2GRAY);
        } else if (m_format == "png" && m_optimizeJPGOrPNGBinary) {
            // Converting to a different channel seems to be saver then converting
            // to grayscale before calling imwrite
            cv::Mat mat(cvPointer->image.size(), CV_8UC1);
            mat.convertTo(cvPointer->image, CV_8UC1);
        }

        cv::imwrite(m_imagesDirectory + "/" + std::to_string(iterationCount) + "." + m_format, cvPointer->image,
                    { m_format == "jpg" ? cv::IMWRITE_JPEG_QUALITY : cv::IMWRITE_PNG_COMPRESSION, m_quality,
                      m_format == "jpg" ? cv::IMWRITE_JPEG_OPTIMIZE : cv::IMWRITE_PNG_BILEVEL, m_optimizeJPGOrPNGBinary });

        iterationCount++;
        // Inform of progress update
        emit progressChanged(iterationCount, ((float) iterationCount / (float) messageCount) * 100);
    }

    reader.close();
    emit finished();
}
