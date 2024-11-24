#include <bug_manager.h>

namespace patterns
{

BugManager::BugManager() : m_bugTracker(std::make_unique<BugTracker>()), m_logger(rclcpp::get_logger("BugManager")) {}

void BugManager::push(const BugDetection &detection)
{
    BugTracker::TrackedBug trackedBug;
    trackedBug.frameNumber = detection.frameNumber;
    trackedBug.lastTimestampMs = detection.timestampMs;
    trackedBug.timestampDetectionMs = detection.timestampMs;
    trackedBug.positionPixel = detection.position;
    m_bugTracker->push(trackedBug);
}

void BugManager::pop()
{
    m_bugTracker->pop();
}

void BugManager::erase(int index)
{
    m_bugTracker->erase(index);
}

int BugManager::size() const
{
    return m_bugTracker->size();
}

bool BugManager::empty() const
{
    return m_bugTracker->size() == 0;
}

cv::Vec3f BugManager::at(int index)
{
    BugTracker::TrackedBug &trackedBug = m_bugTracker->at(index);
    cv::Vec3f bugAtIndex;

    return bugAtIndex;
}

void BugManager::clear()
{
    m_bugTracker->clear();
}

void BugManager::Tick(uint64_t &timeNowMs)
{
    processBugs(timeNowMs);
}

void BugManager::processBugs(uint64_t &timeNowMs)
{
    for (size_t bugIdx = 0; bugIdx < m_bugTracker->size(); bugIdx++) {
        float bugTimeToFireSecs = m_bugTracker->calculateBugIdxTimeToFire(bugIdx);
        std::cout << "\nBug " << bugIdx << " Time to fire: " << bugTimeToFireSecs << " seconds.";
        RCLCPP_INFO_STREAM(m_logger, "\nBug " << bugIdx << " Time to fire: " << bugTimeToFireSecs << " seconds.");
        if (bugTimeToFireSecs < 1.0) {
            RCLCPP_INFO_STREAM(m_logger,
                               "\nSending fire command based on a time-to-fire of " << bugTimeToFireSecs << "seconds.");
        }
    }
}

} // namespace patterns