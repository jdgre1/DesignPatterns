#pragma once
#ifndef BUGDETECTOR_H
#define BUGDETECTOR_H

#include <iostream>
#include <vector>
#include <queue>

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
    void AddImage(const cv::Mat& image);
    void Tick();
    void processImage(const cv::Mat& image);
    void consumeFifoBuffer();

private:
    std::queue<cv::Mat> m_imageBuffer;
    uint8_t m_id;
};

} // namespace patterns
#endif