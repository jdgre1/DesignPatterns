#include <bug_manager.h>

namespace patterns
{

BugManager::BugManager() : m_size(0) {}

void BugManager::push(const cv::Vec3f &value)
{
    if (m_size >= 10) {
        throw std::overflow_error("Array is full. Cannot push more elements.");
    }
    m_array[m_size++] = value; // Add the element and increment size
}

void BugManager::pop()
{
    if (m_size == 0) {
        throw std::underflow_error("Array is empty. Cannot pop elements.");
    }
    m_size--; // Decrement size to effectively "remove" the last element
}

void BugManager::erase(int index)
{
    if (index < 0 || index >= m_size) {
        throw std::out_of_range("Index is out of range.");
    }
    for (int i = index; i < m_size - 1; ++i) {
        m_array[i] = m_array[i + 1]; // Shift elements left
    }
    m_size--; // Decrement size
}

int BugManager::size() const
{
    return m_size; // Return current size
}

bool BugManager::empty() const
{
    return m_size == 0; // Check if empty
}

cv::Vec3f &BugManager::at(int index)
{
    if (index < 0 || index >= m_size) {
        throw std::out_of_range("Index is out of range.");
    }
    return m_array[index]; // Return element by reference
}

void BugManager::clear()
{
    m_size = 0; // Reset size to 0
}
} // namespace patterns