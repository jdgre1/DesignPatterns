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
    struct TrackedBug
    {
        uint64_t frameNumber;
        uint64_t timestampDetection;
        cv::Vec3f positionPixel;
        std::array<cv::Point2f, 4> positionBoundaryMetres;
        cv::Point2f velocity;
    };

    BugTracker();
    void push(const BugTracker::TrackedBug &value); // Add an element
    void pop();                         // Remove the last element
    void erase(int index);              // Remove an element at a specific index
    int size() const;                   // Get current size
    bool empty() const;                 // Check if empty
    // cv::Vec3f &at(int index);             // Access an element
    BugTracker::TrackedBug &at(int index);
    void clear(); // Clear all elements

private:
    bool bugExists();
    TrackedBug m_trackedBugs[NUMBER_OF_BUGS]; // Fixed-size array of cv::Vec3f
    int m_size;                               // Current size of the array
};

} // namespace patterns
#endif