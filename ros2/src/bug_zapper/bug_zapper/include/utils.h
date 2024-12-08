#include <iostream>

#include <opencv2/core/core.hpp>

namespace patterns
{
namespace utils
{

float calculateSignedYDistance(const cv::Point2f &p1, const cv::Point2f &p2) {
    float dx = p2.x - p1.x;
    float dy = p2.y - p1.y;

    // Calculate the magnitude of the distance
    float distance = std::hypot(dx, dy);

    // Use the sign of dy to determine the signed distance
    return (dy >= 0) ? distance : -distance;
}

bool areBugShapesSimilar(const cv::Vec3f &shape1, const cv::Vec3f &shape2, float tolerance = 20.2f)
{
    // Extract radii (third element) from the shapes
    float radius1 = shape1[2];
    float radius2 = shape2[2];

    // Check if either shape's radius is within the tolerance of the other's
    return std::abs(radius1 - radius2) <= (tolerance * std::max(radius1, radius2));
}

} // namespace utils
} // namespace patterns