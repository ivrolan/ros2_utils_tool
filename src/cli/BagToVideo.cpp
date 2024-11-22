#include "EncodingThread.hpp"

#include "UtilsCLI.hpp"
#include "UtilsGeneral.hpp"
#include "UtilsROS.hpp"
#include "UtilsUI.hpp"

#include <QCoreApplication>
#include <QObject>

#include <filesystem>
#include <iostream>

void
showHelp()
{
    std::cout << "Usage: ros2 run mediassist4_ros_tools tool_bag_to_video path/to/ROSBag topic_name path/of/stored/video\n" << std::endl;
    std::cout << "Additional parameters:" << std::endl;
    std::cout << "-r or --rate: Framerate for the encoded video. Must be from 10 to 60." << std::endl;
    std::cout << "-a or --accelerate: Use hardware acceleration." << std::endl;
    std::cout << "-c or --colorless: Use colorless images." << std::endl;
    std::cout << "-l or --lossless (mkv only): Use lossless images." << std::endl;
    std::cout << "-h or --help: Show this help." << std::endl;
}


int
main(int argc, char* argv[])
{
    // Create application
    QCoreApplication app(argc, argv);
    const auto arguments = app.arguments();
    if (arguments.size() < 4 || arguments.contains("--help") || arguments.contains("-h")) {
        showHelp();
        return 0;
    }

    Utils::UI::VideoParameters videoParameters;

    // Handle bag directory
    videoParameters.sourceDirectory = arguments.at(1);
    auto dirPath = videoParameters.sourceDirectory;
    if (!std::filesystem::exists(videoParameters.sourceDirectory.toStdString())) {
        std::cerr << "Bag file not found. Make sure that the bag file exists!" << std::endl;
        return 0;
    }
    if (const auto doesDirContainBag = Utils::ROS::doesDirectoryContainBagFile(videoParameters.sourceDirectory); !doesDirContainBag) {
        std::cerr << "The directory does not contain a bag file!" << std::endl;
        return 0;
    }

    // Topic name
    videoParameters.topicName = arguments.at(2);
    if (!Utils::ROS::doesBagContainTopicName(videoParameters.sourceDirectory, videoParameters.topicName)) {
        std::cerr << "Topic has not been found in the bag file!" << std::endl;
        return 0;
    }
    if (Utils::ROS::getTopicType(videoParameters.sourceDirectory, videoParameters.topicName) != "sensor_msgs/msg/Image") {
        std::cerr << "The entered topic is not in sensor message format!" << std::endl;
        return 0;
    }

    // Video directory
    videoParameters.targetDirectory = arguments.at(3);
    dirPath = videoParameters.targetDirectory;
    dirPath.truncate(dirPath.lastIndexOf(QChar('/')));
    if (!std::filesystem::exists(dirPath.toStdString())) {
        std::cerr << "The entered directory for the video file does not exist. Please specify a correct directory!" << std::endl;
        return 0;
    }
    videoParameters.format = videoParameters.targetDirectory.right(3);
    if (videoParameters.format != "mp4" && videoParameters.format != "mkv") {
        std::cerr << "The entered video name is not in correct format. Please make sure that the video file ends in mp4 or mkv!" << std::endl;
        return 0;
    }

    // Check for optional arguments
    if (arguments.size() > 4) {
        // Framerate
        if (!Utils::CLI::checkArgumentValidity(arguments, "-r", "--rate", videoParameters.fps, 10, 60)) {
            std::cerr << "Please enter a framerate in the range of 10 to 60!" << std::endl;
            return 0;
        }

        // Hardware acceleration
        videoParameters.useHardwareAcceleration = Utils::CLI::containsArguments(arguments, "-a", "--accelerate");
        videoParameters.useBWImages = Utils::CLI::containsArguments(arguments, "-c", "--colorless");
        videoParameters.lossless = Utils::CLI::containsArguments(arguments, "-l", "--lossless");
    }

    auto this_messageCount = 0;

    // Create encoding thread and connect to its informations
    auto* const encodingThread = new EncodingThread(videoParameters);

    QObject::connect(encodingThread, &EncodingThread::calculatedMaximumInstances, [&this_messageCount](int count) {
        this_messageCount = count;
    });
    QObject::connect(encodingThread, &EncodingThread::openingCVInstanceFailed, [] {
        std::cerr << "The video writing failed. Please make sure that all parameters are set correctly and disable the hardware acceleration, if necessary." << std::endl;
        return 0;
    });
    QObject::connect(encodingThread, &EncodingThread::progressChanged, [&this_messageCount] (int iteration, int progress) {
        const auto progressString = Utils::General::drawProgressString(progress);
        // Always clear the last line for a nice "progress bar" feeling in the terminal
        std::cout << progressString << " " << progress << "% (Frame " << iteration << " of " << this_messageCount << ")\r";
    });
    QObject::connect(encodingThread, &EncodingThread::finished, [] {
        std::cout << "Encoding finished! \r" << std::endl;
        return EXIT_SUCCESS;
    });
    QObject::connect(encodingThread, &EncodingThread::finished, encodingThread, &QObject::deleteLater);

    std::cout << "Encoding video. Please wait..." << std::endl;
    encodingThread->start();
    // Wait until the thread is finished
    while (!encodingThread->isFinished()) {
    }

    return EXIT_SUCCESS;
}
