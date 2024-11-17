#include <bug_tracker.h>

namespace patterns
{

BugTracker::BugTracker() : m_size(0) {}

void BugTracker::push(const int &value)
{
    if (!bugExists()) {
        if (m_size >= 10) {
            throw std::overflow_error("Array is full. Cannot push more elements.");
        }
        m_array[m_size++] = value; // Add the element and increment size
    }
}

bool BugTracker::bugExists()
{
    return true;
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
        m_array[i] = m_array[i + 1]; // Shift elements left
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

// cv::Vec3f &BugTracker::at(int index)
// {
//     if (index < 0 || index >= m_size) {
//         throw std::out_of_range("Index is out of range.");
//     }
//     return m_array[index]; // Return element by reference
// }

int &BugTracker::at(int index)
{
    if (index < 0 || index >= m_size) {
        throw std::out_of_range("Index is out of range.");
    }
    return m_array[index]; // Return element by reference
}


void BugTracker::clear()
{
    m_size = 0; // Reset size to 0
}
} // namespace patterns