#include <iostream>

#include <opencv2/core/core.hpp>

namespace patterns
{
namespace config
{
    const float MAX_BUG_DISPLACEMENT_BETWEEN_FRAMES = 40.0;
    const float MAX_BUG_X_DISPLACEMENT_BETWEEN_FRAMES = 10.0;

    const int BUG_SPAWN_TICK_INTERVAL = 10;
    const int FIELD_WIDTH_PIXELS = 800;
    const int FIELD_LENGTH_PIXELS = 1080;
    const int CAMERA_WIDTH_PIXELS = FIELD_WIDTH_PIXELS - 4;
    const int CAMERA_LENGTH_PIXELS = 0.3 * FIELD_LENGTH_PIXELS;
    const int BUG_OFFSET_FROM_WIDTH_PIXELS = 50;
    const int MAX_BUG_DETECTIONS_PER_FRAME = 10;
    const uint8_t NUM_GUNS = 10;
    const int GUN_EXPLOSION_RADIUS = 40;

    

} // namespace utils
} // namespace patterns