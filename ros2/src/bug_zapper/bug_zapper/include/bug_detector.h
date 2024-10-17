#pragma once
#ifndef BUGDETECTOR_H
#define BUGDETECTOR_H

#include <iostream>
#include <queue>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <cv_bridge/cv_bridge.h>

namespace patterns
{

class BugDetector
{
public:
    BugDetector(uint8_t id);
    void AddImage(cv::Mat image);
    void Tick();

private:
    cv::Mat consumeFifoBuffer();
    void detectBugs(cv::Mat& frame);
    void processImage(cv::Mat& image);

    std::queue<cv::Mat> m_imageBuffer;
    uint8_t m_id;
};

} // namespace patterns
#endif