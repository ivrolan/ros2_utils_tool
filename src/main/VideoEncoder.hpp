#pragma once

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

// Encoding thread, encoding a video separately
class VideoEncoder {
public:
    explicit
    VideoEncoder(bool isMP4);

    bool
    setVideoWriter(const std::string& directory,
                   int                width,
                   int                height,
                   bool               useHardwareAcceleration);

    inline void
    writeImageToVideo(const cv::Mat& mat)
    {
        m_videoWriter.write(mat);
    }

private:
    int m_fourcc;
    cv::VideoWriter m_videoWriter;
};
