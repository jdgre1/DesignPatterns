#pragma once
#ifndef BUGMANAGER_H
#define BUGMANAGER_H

#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <bug_tracker.h>
#include <bug_detection.h>

namespace patterns
{

class BugManager
{

public:

    BugManager();
    // Member functions
    void push(const BugDetection &detection); // Add an element
    void pop();                           // Remove the last element
    void erase(int index);                // Remove an element at a specific index
    int size() const;                     // Get current size
    bool empty() const;                   // Check if empty
    cv::Vec3f at(int index);             // Access an element
    void clear(); // Clear all elements
    void Tick(uint64_t &timeNowMs);
    void processBugs(uint64_t &timeNowMs);

private:
    std::unique_ptr<BugTracker> m_bugTracker;
    float m_timesToFireAtBugsMs[NUMBER_OF_BUGS]; 

};

} // namespace patterns
#endif