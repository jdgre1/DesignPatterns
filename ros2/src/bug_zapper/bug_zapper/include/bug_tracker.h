#pragma once
#ifndef BUGTRACKER_H
#define BUGTRACKER_H

#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <bug.h>

namespace patterns
{
const uint NUMBER_OF_BUGS = 10;

class BugTracker
{

public:
    BugTracker();
    void push(const int &value); // Add an element
    void pop();                           // Remove the last element
    void erase(int index);                // Remove an element at a specific index
    int size() const;                     // Get current size
    bool empty() const;                   // Check if empty
    // cv::Vec3f &at(int index);             // Access an element
    int &at(int index);
    void clear();                         // Clear all elements

private:
    bool bugExists();
    int m_array[NUMBER_OF_BUGS]; // Fixed-size array of cv::Vec3f
    int m_size;                           // Current size of the array

};

} // namespace patterns
#endif