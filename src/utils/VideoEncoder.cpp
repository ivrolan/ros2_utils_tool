#include "VideoEncoder.hpp"

VideoEncoder::VideoEncoder(int fourcc) :
    m_fourcc(fourcc)
{
}


bool
VideoEncoder::setVideoWriter(const std::string& directory, int fps, int width, int height, bool useHardwareAcceleration, bool useBWImages)
{
    m_videoWriter = cv::VideoWriter(directory, m_fourcc, fps, cv::Size(width, height), {
        cv::VIDEOWRITER_PROP_HW_ACCELERATION, useHardwareAcceleration ? cv::VIDEO_ACCELERATION_ANY : cv::VIDEO_ACCELERATION_NONE,
        cv::VIDEOWRITER_PROP_IS_COLOR, !useBWImages
    });

    return m_videoWriter.isOpened();
}
