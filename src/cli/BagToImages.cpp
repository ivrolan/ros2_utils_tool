#include "WriteToImageThread.hpp"

#include "UtilsCLI.hpp"
#include "UtilsGeneral.hpp"
#include "UtilsROS.hpp"
#include "UtilsUI.hpp"

#include <QCoreApplication>
#include <QObject>

#include "rclcpp/rclcpp.hpp"

#include <filesystem>
#include <iostream>

void
showHelp()
{
    std::cout << "Usage: ros2 run mediassist4_ros_tools tool_bag_to_images path/to/ROSBag topic_name path/to/target/image/dir" << std::endl;
    std::cout << "Additional parameters:" << std::endl;
    std::cout << "-f or --format: Must be jpg, png or bmp (jpg is default).\n" << std::endl;
    std::cout << "-q 0-9 or --quality 0-9 (jpg and png only): Image quality, must be between 0 and 9 (9 is highest, 8 is default)." << std::endl;
    std::cout << "-c or --colorless: Encode images without color." << std::endl;
    std::cout << "-o or --optimize (jpg only): Optimize jpg file size." << std::endl;
    std::cout << "-b or --binary (png only): Write images with only black and white pixels." << std::endl;
    std::cout << "-h or --help: Show this help." << std::endl;
}


int
main(int argc, char* argv[])
{
    // Create application
    QCoreApplication app(argc, argv);
    const auto& arguments = app.arguments();
    if (arguments.size() < 3 || arguments.contains("--help") || arguments.contains("-h")) {
        showHelp();
        return 0;
    }

    Utils::UI::ImageParameters imageParameters;

    // Handle bag directory
    imageParameters.sourceDirectory = arguments.at(1);
    if (!std::filesystem::exists(imageParameters.sourceDirectory.toStdString())) {
        std::cerr << "Bag file not found. Make sure that the bag file exists!" << std::endl;
        return 0;
    }
    if (const auto doesDirContainBag = Utils::ROS::doesDirectoryContainBagFile(imageParameters.sourceDirectory); !doesDirContainBag) {
        std::cerr << "The directory does not contain a bag file!" << std::endl;
        return 0;
    }

    // Topic name
    imageParameters.topicName = arguments.at(2);
    if (!Utils::ROS::doesBagContainTopicName(imageParameters.sourceDirectory, imageParameters.topicName)) {
        std::cerr << "Topic has not been found in the bag file!" << std::endl;
        return 0;
    }
    if (Utils::ROS::getTopicType(imageParameters.sourceDirectory, imageParameters.topicName) != "sensor_msgs/msg/Image") {
        std::cerr << "The entered topic is not in sensor message format!" << std::endl;
        return 0;
    }

    // Images directory
    imageParameters.targetDirectory = arguments.at(3);

    // Check for optional arguments
    if (arguments.size() > 4) {
        if (!Utils::CLI::checkArgumentValidity(arguments, "-q", "--quality", imageParameters.quality, 0, 9)) {
            std::cerr << "Please enter a quality value in the range of 0 to 9!" << std::endl;
            return 0;
        }
        if (Utils::CLI::containsArguments(arguments, "-f", "--format")) {
            const auto qualityFormatIndex = Utils::CLI::getArgumentsIndex(arguments, "-f", "--format");
            if (arguments.at(qualityFormatIndex) == arguments.last() ||
                (arguments.at(qualityFormatIndex + 1) != "jpg" && arguments.at(qualityFormatIndex + 1) != "png" && arguments.at(qualityFormatIndex + 1) != "bmp")) {
                std::cerr << "Please enter either 'jpg', 'png' or 'bmp' for the format!" << std::endl;
                return 0;
            }
            imageParameters.format = arguments.at(qualityFormatIndex + 1);
        }

        imageParameters.useBWImages = Utils::CLI::containsArguments(arguments, "-c", "--colorless");
        imageParameters.jpgOptimize = imageParameters.format == "jpg" && Utils::CLI::containsArguments(arguments, "-o", "--optimize");
        imageParameters.pngBilevel = imageParameters.format == "png" && Utils::CLI::containsArguments(arguments, "-b", "--binary");
    }

    auto thisMessageCount = 0;

    // Create thread and connect to its informations
    auto* const writeToImageThread = new WriteToImageThread(imageParameters);
    QObject::connect(writeToImageThread, &WriteToImageThread::calculatedMaximumInstances, [&thisMessageCount](int count) {
        thisMessageCount = count;
    });
    QObject::connect(writeToImageThread, &WriteToImageThread::progressChanged, [&thisMessageCount] (int iteration, int progress) {
        const auto progressString = Utils::General::drawProgressString(progress);
        // Always clear the last line for a nice "progress bar" feeling in the terminal
        std::cout << progressString << " " << progress << "% (Frame " << iteration << " of " << thisMessageCount << ")\r";
    });
    QObject::connect(writeToImageThread, &WriteToImageThread::finished, [] {
        std::cout << "Writing images finished! \r" << std::endl;
        return EXIT_SUCCESS;
    });
    QObject::connect(writeToImageThread, &WriteToImageThread::finished, writeToImageThread, &QObject::deleteLater);

    std::cout << "Writing images. Please wait..." << std::endl;
    writeToImageThread->start();
    // Wait until the thread is finished
    while (!writeToImageThread->isFinished()) {
    }

    return EXIT_SUCCESS;
}
