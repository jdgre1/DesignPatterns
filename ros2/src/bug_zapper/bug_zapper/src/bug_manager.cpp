#include <bug_manager.h>
#include <config.h>

namespace patterns
{

BugManager::BugManager() : m_bugTracker(std::make_unique<BugTracker>()), m_logger(rclcpp::get_logger("BugManager")) {}

void BugManager::push(const bug_zapper_msgs::msg::BugDetection &detection)
{
    BugTracker::TrackedBug trackedBug;
    trackedBug.frameNumber = detection.frame_number;
    trackedBug.lastTimestampMs = detection.timestamp_ms;
    trackedBug.timestampDetectionMs = detection.timestamp_ms;
    trackedBug.positionPixel[0] = detection.position.x; // X-coordinate
    trackedBug.positionPixel[1] = detection.position.y; // Y-coordinate
    trackedBug.positionPixel[2] = detection.position.z; // Z-coordinate
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

void BugManager::clear()
{
    m_bugTracker->clear();
}

void BugManager::processDetections()
{
    for (bug_zapper_msgs::msg::BugDetection bugDet : m_detections) {
        push(bugDet);
    }
    m_detections.clear();
}

void BugManager::Tick(uint64_t &timeNowMs)
{
    processDetections();
    m_bugTracker->Tick(timeNowMs);
    processBugs(timeNowMs);
}

void BugManager::processBugs(uint64_t &timeNowMs)
{
    std::vector<size_t> bugsToRemove;
    m_fireCommandMessages.clear();
    for (size_t bugIdx = 0; bugIdx < m_bugTracker->size(); bugIdx++) {
        BugTracker::TrackedBug &bug = m_bugTracker->at(bugIdx);
        if (abs(bug.velocityPixelPerSec) > 0 && abs(bug.velocityPixelPerSec < 10000)) {
            bug.positionPixel[1] += bug.velocityPixelPerSec * (timeNowMs - bug.lastTimestampMs) / 1000.0;
            bug.lastTimestampMs = timeNowMs;
            bug.numUpdates++;
            float bugTimeToFireSecs = m_bugTracker->calculateBugIdxTimeToFire(bugIdx);
            float minY = bug.positionPixel[1] - bug.positionPixel[2];
            float maxY = bug.positionPixel[1] + bug.positionPixel[2];
            float totalBugLength = maxY - minY;
            float firingDurationMs = 1000 * totalBugLength / bug.velocityPixelPerSec;

            bug_zapper_msgs::msg::FireCommand fireCmdMsg;
            fireCmdMsg.opening_time = timeNowMs + bugTimeToFireSecs * 1000;
            fireCmdMsg.closing_time = fireCmdMsg.opening_time + firingDurationMs;

            float minX = bug.positionPixel[1] - bug.positionPixel[2];
            float maxX = bug.positionPixel[1] + bug.positionPixel[2];

            float ratioCameraFrameStart = minX / config::FIELD_WIDTH_PIXELS;
            float ratioCameraFrameEnd = maxX / config::FIELD_WIDTH_PIXELS;

            // ToDo: etemine which guns to fire based on span of bug across frame:

            m_fireCommandMessages.push_back(fireCmdMsg);


            if (bugTimeToFireSecs < 1.0) {
                RCLCPP_ERROR_STREAM(m_logger, "\nSending fire command based on a time-to-fire of " << bugTimeToFireSecs
                                                                                                  << "seconds.");

                bugsToRemove.push_back(bugIdx);
            }
        }
        else {
            uint64_t timePassedMs = timeNowMs - bug.lastTimestampMs;
           
            if (timePassedMs > 20000 || (timePassedMs > 1000 && bug.velocityPixelPerSec < 1)) {
                RCLCPP_INFO_STREAM(m_logger, " Bug removed"
                                             << bugIdx << " Velocity: " << bug.velocityPixelPerSec << " pixels per sec."
                                             << " m_timeNowMs - lastTimeStampMs: " << timePassedMs << " ms.");
                bugsToRemove.push_back(bugIdx);
            }
        }
    }
    // Sort descending
    std::sort(bugsToRemove.rbegin(), bugsToRemove.rend());

    // Erase elements starting from the largest index
    for (size_t idx : bugsToRemove) {
        m_bugTracker->erase(idx);
    }
}

} // namespace patterns