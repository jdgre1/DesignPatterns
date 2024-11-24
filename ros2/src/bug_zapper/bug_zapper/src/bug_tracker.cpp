#include <bug_tracker.h>
#include <config.h>
#include <utils.h>

namespace patterns
{

BugTracker::BugTracker() : m_size(0) {}

void BugTracker::push(BugTracker::TrackedBug &bug)
{
    if (!bugExists(bug)) {
        if (m_size >= 10) {
            throw std::overflow_error("Array is full. Cannot push more elements.");
        }
        m_trackedBugs[m_size++] = bug; // Add the element and increment size
    }
}

bool BugTracker::bugExists(BugTracker::TrackedBug &bug)
{   
    size_t idx = 0;
    for (BugTracker::TrackedBug memberBug : m_trackedBugs) {
        if (idx++ == m_size) {
            return false;
        }
        cv::Point point1(cvRound(memberBug.positionPixel[0]), cvRound(memberBug.positionPixel[1]));
        cv::Point point2(cvRound(bug.positionPixel[0]), cvRound(bug.positionPixel[1]));

        bool dispWithinRange =
            config::MAX_BUG_DISPLACEMENT_BETWEEN_FRAMES < utils::calculateSignedYDistance(point1, point2);
        bool xDisplacementWithinRange = abs(point1.x - point2.x) < config::MAX_BUG_X_DISPLACEMENT_BETWEEN_FRAMES;
        bool similarRadius = utils::areBugShapesSimilar(memberBug.positionPixel, bug.positionPixel);

        if (dispWithinRange && xDisplacementWithinRange && similarRadius) {
            memberBug.positionPixel = bug.positionPixel;
            memberBug.velocityPixelPerSec =
                0.5 * (memberBug.prevVelocityPixelPerSec + 1000.0 * (point2.y - point1.y) / bug.lastTimestampMs -
                       memberBug.lastTimestampMs);
            memberBug.prevVelocityPixelPerSec = memberBug.velocityPixelPerSec;
            memberBug.numUpdates++;
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
        float distanceLeftPixels = config::CAMERA_LENGTH_PIXELS - bug.positionPixel[1];
        float timeToFireSecs = distanceLeftPixels / bug.velocityPixelPerSec;
        return timeToFireSecs;
    }
    else {
        return 10.0;
    }
}
void BugTracker::clear()
{
    m_size = 0; // Reset size to 0
}
} // namespace patterns