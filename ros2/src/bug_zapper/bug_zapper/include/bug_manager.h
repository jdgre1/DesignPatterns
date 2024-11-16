#pragma once
#ifndef BUGMANAGER_H
#define BUGMANAGER_H

#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

const uint NUMBER_OF_BUGS = 10;

namespace patterns
{

class BugManager
{
public:
    BugManager();
     // Member functions
    void push(const cv::Vec3f& value); // Add an element
    void pop(); // Remove the last element
    void erase(int index); // Remove an element at a specific index
    int size() const; // Get current size
    bool empty() const; // Check if empty
    cv::Vec3f& at(int index); // Access an element
    void clear(); // Clear all elements

private:
    cv::Vec3f m_array[NUMBER_OF_BUGS]; // Fixed-size array of cv::Vec3f
    int m_size; // Current size of the array

};

} // namespace patterns
#endif