#include <bug_tracker.h>
#include <config.h>
#include <utils.h>

namespace patterns
{

BugTracker::BugTracker() : m_size(0), m_logger(rclcpp::get_logger("BugTracker")) {}

void BugTracker::push(BugTracker::TrackedBug &bug)
{
    if (!bugExists(bug)) {
        if (m_size >= 30) {
            throw std::overflow_error("Array is full. Cannot push more elements.");
        }
        bug.isClassified = true;
        m_trackedBugs[m_size++] = bug; // Add the element and increment size
    }
}

bool BugTracker::bugExists(BugTracker::TrackedBug &bug)
{
    size_t idx = 0;
    for (BugTracker::TrackedBug &memberBug : m_trackedBugs) {

        if (!memberBug.isClassified) {
            continue;
        }
        cv::Point point1(cvRound(memberBug.positionPixel[0]), cvRound(memberBug.positionPixel[1]));
        cv::Point point2(cvRound(bug.positionPixel[0]), cvRound(bug.positionPixel[1]));

        bool dispWithinRange =
            utils::calculateSignedYDistance(point1, point2) < config::MAX_BUG_DISPLACEMENT_BETWEEN_FRAMES &&
            point1.y != point2.y;
        bool xDisplacementWithinRange = abs(point1.x - point2.x) < config::MAX_BUG_X_DISPLACEMENT_BETWEEN_FRAMES;
        bool similarRadius = utils::areBugShapesSimilar(memberBug.positionPixel, bug.positionPixel);
        bool timePassedSinceLastSample = (m_timeNowMs - memberBug.lastTimestampMs != 0);

        if (dispWithinRange && xDisplacementWithinRange && similarRadius && timePassedSinceLastSample) {
            memberBug.positionPixel = bug.positionPixel;
            // RCLCPP_INFO_STREAM(m_logger, " dispWithinRange "
            //                                  << dispWithinRange << " xDisplacementWithinRange: "
            //                                  << xDisplacementWithinRange << " similarRadius: " << similarRadius
            //                                  << " timePassedSinceLastSample: " << timePassedSinceLastSample);

            float velocityPixelPerSec = 1000.0 * (point2.y - point1.y) / (m_timeNowMs - memberBug.lastTimestampMs);

            if (memberBug.prevVelocityPixelPerSec < 0.001) {
                memberBug.velocityPixelPerSec = velocityPixelPerSec;
            }
            else { // Smoothing
                // memberBug.velocityPixelPerSec = 0.5 * (velocityPixelPerSec + memberBug.prevVelocityPixelPerSec);
                memberBug.velocityPixelPerSec = velocityPixelPerSec;
            }
            if (abs(memberBug.velocityPixelPerSec) > 0.001) {
                RCLCPP_WARN_STREAM(m_logger, "\nBug " << idx++ << " updated velocity: " << memberBug.velocityPixelPerSec
                                                      << " pixels per second.");

                memberBug.prevVelocityPixelPerSec = memberBug.velocityPixelPerSec;
                memberBug.lastTimestampMs = m_timeNowMs;
                memberBug.numUpdates++;

                return true;
            }
        }
        else if (!timePassedSinceLastSample && dispWithinRange && xDisplacementWithinRange && similarRadius) {
            return true;
        }
    }

    return false;
}

void BugTracker::pop()
{
    if (m_size == 0) {
        throw std::underflow_error("Array is empty. Cannot pop elements.");
    }
    m_size--; // Decrement size to effectively "remove" the last element
}

void BugTracker::erase(int index)
{
    if (index < 0 || index >= m_size) {
        throw std::out_of_range("Index is out of range.");
    }
    for (int i = index; i < m_size - 1; ++i) {
        m_trackedBugs[i] = m_trackedBugs[i + 1]; // Shift elements left
        m_trackedBugs[i + 1].isClassified = false;
    }
    m_size--; // Decrement size
}

int BugTracker::size() const
{
    return m_size; // Return current size
}

bool BugTracker::empty() const
{
    return m_size == 0; // Check if empty
}

BugTracker::TrackedBug &BugTracker::at(int index)
{
    if (index < 0 || index >= m_size) {
        throw std::out_of_range("Index is out of range.");
    }
    return m_trackedBugs[index]; // Return element by reference
}

float BugTracker::calculateBugIdxTimeToFire(size_t idx)
{
    BugTracker::TrackedBug bug = at(idx);
    if (bug.numUpdates > 0) {
        float distanceLeftPixels = config::CAMERA_LENGTH_PIXELS * 1.2 - bug.positionPixel[1];
        float timeToFireSecs = distanceLeftPixels / bug.velocityPixelPerSec;
        RCLCPP_INFO_STREAM(m_logger, " Bug: distanceLeftPixels: " << distanceLeftPixels << ".");
        RCLCPP_INFO_STREAM(m_logger, " Bug: bug.velocityPixelPerSec: " << bug.velocityPixelPerSec << ".");
        RCLCPP_INFO_STREAM(m_logger, " Bug: Time to fire: " << timeToFireSecs << " seconds.");
        return timeToFireSecs;
    }
    else {
        return 10.0;
    }
}
void BugTracker::Tick(uint64_t &timeNowMs)
{
    m_timeNowMs = timeNowMs;
}
void BugTracker::clear()
{
    m_size = 0; // Reset size to 0
}
} // namespace patterns