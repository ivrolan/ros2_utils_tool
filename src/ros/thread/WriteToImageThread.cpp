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
    const auto targetDirectoryStd = m_imageParameters.targetDirectory.toStdString();

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
    emit startingDataCollection();

    rosbag2_cpp::Reader reader;
    reader.open(m_sourceDirectory);

    std::deque<rosbag2_storage::SerializedBagMessageSharedPtr> queue;
    while (reader.has_next()) {
        if (isInterruptionRequested()) {
            reader.close();
            return;
        }

        auto message = reader.read_next();
        if (message->topic_name != m_topicName) {
            continue;
        }
        queue.push_front(message);
    }
    reader.close();

    // Prepare parameters
    rclcpp::Serialization<sensor_msgs::msg::Image> serialization;
    auto iterationCount = 0;
    std::mutex mutex;

    // We want to run the image writing in parallel
    const auto writeImageFromQueue = [this, &targetDirectoryStd, &mutex, &iterationCount, &queue, serialization, messageCount] {
        while (true) {
            mutex.lock();

            if (isInterruptionRequested() || queue.empty()) {
                mutex.unlock();
                break;
            }

            // Deserialize
            rclcpp::SerializedMessage serializedMessage(*queue.back()->serialized_data);
            auto rosMsg = std::make_shared<sensor_msgs::msg::Image>();
            serialization.deserialize_message(&serializedMessage, rosMsg.get());
            queue.pop_back();

            // Convert message to cv
            auto cvPointer = cv_bridge::toCvCopy(*rosMsg, rosMsg->encoding);
            // Convert to grayscale
            if (m_imageParameters.format == "png" && m_imageParameters.pngBilevel) {
                // Converting to a different channel seems to be saver then converting
                // to grayscale before calling imwrite
                cv::Mat mat(cvPointer->image.size(), CV_8UC1);
                mat.convertTo(cvPointer->image, CV_8UC1);
            } else if (m_imageParameters.useBWImages) {
                cv::cvtColor(cvPointer->image, cvPointer->image, cv::COLOR_BGR2GRAY);
            }

            // Inform of progress update
            iterationCount++;
            emit progressChanged(iterationCount, ((float) iterationCount / (float) messageCount) * 100);

            // Have to create this as extra string to keep it atomic inside the mutex
            const auto targetString = targetDirectoryStd + "/" + std::to_string(iterationCount) + "." + m_imageParameters.format.toStdString();

            mutex.unlock();
            // The main writing can be done in parallel
            cv::imwrite(targetString, cvPointer->image,
                        { m_imageParameters.format == "jpg" ? cv::IMWRITE_JPEG_QUALITY : cv::IMWRITE_PNG_COMPRESSION,
                          // Adjust the quality value to fit OpenCV param range
                          m_imageParameters.format == "jpg" ? (m_imageParameters.quality * 10) + 10 : m_imageParameters.quality,
                          m_imageParameters.format == "jpg" ? cv::IMWRITE_JPEG_OPTIMIZE : cv::IMWRITE_PNG_BILEVEL,
                          m_imageParameters.format == "jpg" ? m_imageParameters.jpgOptimize : m_imageParameters.pngBilevel });
        }
    };

    std::vector<std::thread> threadPool;
    for (unsigned int i = 0; i < std::thread::hardware_concurrency(); ++i) {
        threadPool.emplace_back(writeImageFromQueue);
    }
    for (auto& thread : threadPool) {
        thread.join();
    }

    emit finished();
}
