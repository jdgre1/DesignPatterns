#pragma once
#ifndef BUGDETECTION_H
#define BUGDETECTION_H

#include <iostream>

#include <opencv2/core/core.hpp>



namespace patterns
{
    struct BugDetection
    {
        cv::Vec3f position;
        uint64_t frameNumber;
        uint64_t timestampMs;
    };

};
#endif