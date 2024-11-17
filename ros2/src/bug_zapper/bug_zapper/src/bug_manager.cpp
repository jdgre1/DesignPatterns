#include <bug_manager.h>

namespace patterns
{

BugManager::BugManager() : m_bugTracker(std::unique_ptr<BugTracker>(std::make_unique<BugTracker>())) {}

void BugManager::push(const BugDetection &value)
{   
    m_bugTracker->push(33);
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

// cv::Vec3f &BugManager::at(int index)
// {
    // return m_bugTracker->at(index); 
// }

void BugManager::clear()
{
    m_bugTracker->clear();
}
} // namespace patterns