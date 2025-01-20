#include "EditBagThread.hpp"

#include "UtilsROS.hpp"

#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/writer.hpp"

#include <filesystem>

#ifdef ROS_JAZZY
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

EditBagThread::EditBagThread(const Utils::UI::EditBagInputParameters& parameters,
                             QObject*                                 parent) :
    BasicThread(parameters.sourceDirectory, parameters.topicName, parent),
    m_parameters(parameters)
{
}


void
EditBagThread::run()
{
    auto totalInstances = 0;
    for (const auto& topic : m_parameters.topics) {
        if (!topic.isSelected) {
            continue;
        }

        totalInstances += topic.upperBoundary - topic.lowerBoundary;
    }
    emit calculatedMaximumInstances(totalInstances);
    emit startingDataCollection();

    const auto targetDirectoryStd = m_parameters.targetDirectory.toStdString();
    if (std::filesystem::exists(targetDirectoryStd)) {
        std::filesystem::remove_all(targetDirectoryStd);
    }

    rosbag2_cpp::Writer writer;
    writer.open(targetDirectoryStd);
    std::mutex mutex;
    std::atomic<int> instanceCount = 1;

    // Move to own lambda for multithreading
    const auto writeTopicToBag = [this, &writer, &instanceCount, &mutex, totalInstances] (const auto& topic) {
        const auto originalTopicNameStd = topic.originalTopicName.toStdString();
        const auto& metadata = Utils::ROS::getBagMetadata(m_parameters.sourceDirectory);
        // Create a new topic using either the original or new name
        for (const auto &topicMetaData : metadata.topics_with_message_count) {
            if (topicMetaData.topic_metadata.name == originalTopicNameStd) {
                auto topicToBeModified = topicMetaData.topic_metadata;

                if (!topic.renamedTopicName.isEmpty()) {
                    topicToBeModified.name = topic.renamedTopicName.toStdString();
                }
                writer.create_topic(topicToBeModified);
                break;
            }
        }

        rosbag2_cpp::Reader reader;
        mutex.lock();
        reader.open(m_sourceDirectory);
        mutex.unlock();
        size_t boundaryCounter = 0;

        while (reader.has_next()) {
            if (isInterruptionRequested()) {
                reader.close();
                return;
            }
            // Read, stay within boundaries
            auto message = reader.read_next();
            if (message->topic_name != originalTopicNameStd) {
                continue;
            }
            if (boundaryCounter < topic.lowerBoundary) {
                boundaryCounter++;
                continue;
            }
            if (boundaryCounter == topic.upperBoundary) {
                break;
            }

            if (!topic.renamedTopicName.isEmpty()) {
                message->topic_name = topic.renamedTopicName.toStdString();
            }
            writer.write(message);

            boundaryCounter++;
            emit progressChanged(instanceCount, ((float) instanceCount / (float) totalInstances) * 100);
            instanceCount++;
        }

        reader.close();
    };

    // Parallelize the topic writing
    std::vector<std::thread> threadPool;
    for (const auto& topic : m_parameters.topics) {
        if (!topic.isSelected) {
            continue;
        }

        threadPool.emplace_back(writeTopicToBag, topic);
    }
    for (auto& thread : threadPool) {
        thread.join();
    }

    writer.close();
    emit finished();
}
