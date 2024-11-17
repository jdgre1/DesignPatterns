#include <bug_manager.h>

namespace patterns
{

BugManager::BugManager() : m_bugTracker(std::unique_ptr<BugTracker>(std::make_unique<BugTracker>())) {}

void BugManager::push(const BugDetection &detection)
{   
    BugTracker::TrackedBug trackedBug;
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
    BugTracker::TrackedBug& trackedBug = m_bugTracker->at(index); 
    cv::Vec3f bugAtIndex;
    
    return bugAtIndex; 
}

void BugManager::clear()
{
    m_bugTracker->clear();
}
} // namespace patterns