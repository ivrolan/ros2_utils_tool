#include "VideoEncoder.hpp"

VideoEncoder::VideoEncoder(bool isMP4)
{
    m_fourcc = isMP4 ? cv::VideoWriter::fourcc('m', 'p', '4', 'v')
                     : cv::VideoWriter::fourcc('X', '2', '6', '4');
}


bool
VideoEncoder::setVideoWriter(const std::string& directory, int width, int height, bool useHardwareAcceleration)
{
    m_videoWriter = cv::VideoWriter(directory, m_fourcc, 30, cv::Size(width, height), {
        cv::VIDEOWRITER_PROP_HW_ACCELERATION, useHardwareAcceleration ? cv::VIDEO_ACCELERATION_ANY : cv::VIDEO_ACCELERATION_NONE
    });

    return m_videoWriter.isOpened();
}
