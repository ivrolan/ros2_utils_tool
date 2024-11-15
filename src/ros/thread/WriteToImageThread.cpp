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
    BasicThread(imageParameters.sourceDirectory, imageParameters.topicName, parent),
    m_imageParameters(imageParameters)
{
}


void
WriteToImageThread::run()
{
    rosbag2_cpp::Reader reader;
    reader.open(m_sourceDirectory);

    const auto targetDirectoryStd = m_imageParameters.targetDirectory.toStdString();
    const auto formatStd = m_imageParameters.format.toStdString();

    if (!std::filesystem::exists(targetDirectoryStd)) {
        std::filesystem::create_directory(targetDirectoryStd);
    }
    if (!std::filesystem::is_empty(targetDirectoryStd)) {
        // Remove all images currently present
        for (const auto& entry : std::filesystem::directory_iterator(targetDirectoryStd)) {
            std::filesystem::remove_all(entry.path());
        }
    }

    const auto messageCount = Utils::ROS::getTopicMessageCount(m_sourceDirectory, m_topicName);
    emit calculatedMaximumInstances(messageCount);

    // Prepare parameters
    rclcpp::Serialization<sensor_msgs::msg::Image> serialization;
    cv_bridge::CvImagePtr cvPointer;
    auto iterationCount = 0;

    // Adjust the quality value to fit OpenCV param range
    const auto finalQuality = formatStd == "jpg" ? (m_imageParameters.quality * 10) + 10 : m_imageParameters.quality;

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

        if (m_imageParameters.useBWImages && (formatStd == "jpg" || !m_imageParameters.jpgOptimize)) {
            cv::cvtColor(cvPointer->image, cvPointer->image, cv::COLOR_BGR2GRAY);
        } else if (formatStd == "png" && m_imageParameters.pngBilevel) {
            // Converting to a different channel seems to be saver then converting
            // to grayscale before calling imwrite
            cv::Mat mat(cvPointer->image.size(), CV_8UC1);
            mat.convertTo(cvPointer->image, CV_8UC1);
        }

        cv::imwrite(targetDirectoryStd + "/" + std::to_string(iterationCount) + "." + formatStd, cvPointer->image,
                    { formatStd == "jpg" ? cv::IMWRITE_JPEG_QUALITY : cv::IMWRITE_PNG_COMPRESSION, finalQuality,
                      formatStd == "jpg" ? cv::IMWRITE_JPEG_OPTIMIZE : cv::IMWRITE_PNG_BILEVEL,
                      formatStd == "jpg" ? m_imageParameters.jpgOptimize : m_imageParameters.pngBilevel });

        iterationCount++;
        // Inform of progress update
        emit progressChanged(iterationCount, ((float) iterationCount / (float) messageCount) * 100);
    }

    reader.close();
    emit finished();
}
