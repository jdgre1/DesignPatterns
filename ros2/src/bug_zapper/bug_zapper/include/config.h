#include <iostream>

#include <opencv2/core/core.hpp>

namespace patterns
{
namespace config
{
    const float MAX_BUG_DISPLACEMENT_BETWEEN_FRAMES = 40.0;
    const float MAX_BUG_X_DISPLACEMENT_BETWEEN_FRAMES = 100.0;

    const int BUG_SPAWN_TICK_INTERVAL = 10;
    const int CAMERA_WIDTH_PIXELS = 800;
    const int CAMERA_LENGTH_PIXELS = 1080;
    const int BUG_OFFSET_FROM_WIDTH_PIXELS = 50;
    const int MAX_BUG_DETECTIONS_PER_FRAME = 10;


    

} // namespace utils
} // namespace patterns