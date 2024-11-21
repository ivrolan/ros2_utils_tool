#include "EncodingThread.hpp"

#include "UtilsROS.hpp"
#include "VideoEncoder.hpp"

#include "rosbag2_cpp/reader.hpp"

#include "sensor_msgs/msg/image.hpp"

#ifdef ROS_JAZZY
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

EncodingThread::EncodingThread(const Utils::UI::VideoParameters& videoParameters,
                               QObject*                          parent) :
    BasicThread(videoParameters.sourceDirectory, videoParameters.topicName, parent),
    m_videoParameters(videoParameters)
{
}


void
EncodingThread::run()
{
    rosbag2_cpp::Reader reader;
    reader.open(m_sourceDirectory);

    const auto messageCount = Utils::ROS::getTopicMessageCount(m_sourceDirectory, m_topicName);
    emit calculatedMaximumInstances(messageCount);

    // Prepare parameters
    rclcpp::Serialization<sensor_msgs::msg::Image> serialization;
    cv_bridge::CvImagePtr cvPointer;
    auto iterationCount = 0;
    const auto topicNameStdString = m_topicName;
    const auto videoEncoder = std::make_shared<VideoEncoder>(m_videoParameters.format == "mp4" ? cv::VideoWriter::fourcc('m', 'p', '4', 'v') :
                                                             m_videoParameters.lossless ? cv::VideoWriter::fourcc('F', 'F', 'V', '1') : cv::VideoWriter::fourcc('X', '2', '6', '4'));

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

            if (!videoEncoder->setVideoWriter(m_videoParameters.targetDirectory.toStdString(), m_videoParameters.fps, width, height,
                                              m_videoParameters.useHardwareAcceleration, m_videoParameters.useBWImages)) {
                emit openingCVInstanceFailed();
                return;
            }
        }

        // Convert message to cv and encode
        cvPointer = cv_bridge::toCvCopy(*rosMsg, rosMsg->encoding);

        if (m_videoParameters.useBWImages) {
            // @note
            // It seems that just setting the VIDEOWRITER_PROP_IS_COLOR in the videowriter leads to a broken video,
            // at least if FFMPEG is used. Converting to a gray mat beforehand provides a fix. More information here:
            // https://github.com/opencv/opencv/issues/26276#issuecomment-2406825667
            cv::Mat greyMat;
            cv::cvtColor(cvPointer->image, greyMat, cv::COLOR_BGR2GRAY);
            videoEncoder->writeImageToVideo(greyMat);
        } else {
            videoEncoder->writeImageToVideo(cvPointer->image);
        }

        iterationCount++;
        // Inform of progress update
        emit progressChanged(iterationCount, ((float) iterationCount / (float) messageCount) * 100);
    }

    reader.close();
    emit finished();
}
