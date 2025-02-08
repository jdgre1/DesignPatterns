#include <random>

#include <bug_factory.h>
#include <bug_sim.h>
#include <config.h>

using namespace std::chrono_literals;
namespace patterns
{

int GenerateRandomNumberBetween(int min, int max)
{
    // Create a random device and seed the generator
    static std::random_device rd;
    static std::mt19937 gen(rd());

    // Define the range 1 to 3 inclusive
    std::uniform_int_distribution<> dis(min, max);

    // Generate and return the random number
    int randomNum = dis(gen);
    return randomNum;
}

BugSim::BugSim(double bugSpeedMin, double bugSpeedMax, uint8_t bugStrength)
    : rclcpp_lifecycle::LifecycleNode("bug_sim_lifecycle_node"), m_bugSpeedMin(bugSpeedMin), m_bugSpeedMax(bugSpeedMax),
      m_bugStrength(bugStrength)
{
    m_startTime = this->get_clock()->now();
    m_timer = this->create_wall_timer(100ms, std::bind(&BugSim::simTimerCallback, this));
    m_cameraFramePub = this->create_publisher<sensor_msgs::msg::Image>("cameraFrame", 10);
    m_fireCommandSub = this->create_subscription<bug_zapper_msgs::msg::FireCommand>(
        "fire_command", 10, std::bind(&BugSim::fireCommandSubCallback, this, std::placeholders::_1));
}

void BugSim::DrawBug(std::shared_ptr<Bug> bug, cv::Mat &frame)
{
    Components::XYComponent &xyPos = bug->getPos();
    uint8_t speed = bug->getSpeed();
    xyPos.posY += speed * 100.0 / 1000.0;
    int thickness = -1; // filled

    switch (bug->getType()) {
    case BugType::Alien:
    {
        cv::Point center(xyPos.posX, xyPos.posY);
        cv::uint8_t radius = bug->getSize();
        cv::Scalar lineColor(255, 0, 0);
        cv::circle(frame, center, radius, lineColor, thickness);
        break;
    }

    case BugType::Zipper:
    {
        cv::Point center(xyPos.posX, xyPos.posY);
        uint8_t radius = bug->getSize();
        cv::Scalar lineColor(0, 0, 255);
        cv::circle(frame, center, radius, lineColor, thickness);
        break;
    }

    case BugType::BigBertha:
    {
        cv::Point center(xyPos.posX, xyPos.posY);
        uint8_t radius = bug->getSize();
        cv::Scalar lineColor(255, 255, 0);
        cv::circle(frame, center, radius, lineColor, thickness);
        break;
    }

    default:
    {
        std::cout << "\nBug type does not exist!";
        return;
    }
    }
}

void BugSim::processFireCommandQueue(cv::Mat &frame)
{

    rclcpp::Time timeZero(0, 0, rcl_clock_type_t::RCL_ROS_TIME); // Time 0
    rclcpp::Duration timeDiff = m_timeNow - m_startTime;
    rclcpp::Time timeSinceStart = timeZero + timeDiff;
    int64_t timeSinceStartMs = timeSinceStart.nanoseconds() / 1'000'000;

    for (auto it = m_fireCommands.begin(); it != m_fireCommands.end(); /* no increment here */) {
        const auto &item = *it;

        int64_t openingTimeMs = item.opening_time.nanoseconds() / 1'000'000;
        int64_t closingTimeMs = item.closing_time.nanoseconds() / 1'000'000;

        if (timeSinceStartMs >= closingTimeMs) {
            // Erase expired item
            it = m_fireCommands.erase(it);
            RCLCPP_INFO(this->get_logger(), "FireCommand expired and removed.");
            continue;
        }
        else if (timeSinceStartMs >= openingTimeMs) {
            // Process the command (draw explosions)
            for (uint8_t gun : item.gunIds) {
                drawGunTriggers(frame, gun);
            }
            // If you want the explosion to persist as long as the object is in the region,
            // you might leave the item in place. Otherwise, decide if it should be removed.
        }
        else {
            // Log the time values
            RCLCPP_INFO(this->get_logger(), "Current Time (ms): %lu, Opening Time (ms): %lu, Closing Time (ms): %lu",
                        timeSinceStartMs, openingTimeMs, closingTimeMs);
        }
        ++it;
    }
}

void BugSim::drawCameraFrame(cv::Mat &frame)
{
    // and its top left corner...
    int y1 = int(config::FIELD_LENGTH_PIXELS * 0.5);
    int y2 = int(config::FIELD_LENGTH_PIXELS * 0.80);
    cv::Point pt1(2, y1);
    // and its bottom right corner.
    cv::Point pt2(config::FIELD_WIDTH_PIXELS - 2, y2);
    // These two calls...
    cv::rectangle(frame, pt1, pt2, cv::Scalar(255, 0, 255), 3);
    cv::putText(frame, "Camera Frame", cv::Point(10, frame.rows / 2 - 10), cv::FONT_HERSHEY_DUPLEX, 1.0,
                CV_RGB(118, 185, 0), 2);

    cv::Mat cameraFrame = frame(cv::Range(pt1.y, pt2.y), cv::Range(pt1.x, pt2.x));
    std_msgs::msg::Header header;
    header.stamp = m_timeNow; // Add timestamp
    sensor_msgs::msg::Image::SharedPtr imgMsg = cv_bridge::CvImage(header, "bgr8", cameraFrame).toImageMsg();
    m_cameraFramePub->publish(*imgMsg.get());
    uint64_t imageTimestampMs = static_cast<int64_t>(header.stamp.sec) * 1000 + RCL_NS_TO_MS(header.stamp.nanosec);
}

void BugSim::AddRandomBug(BugType &bugtype)
{
    u_int8_t speed = 0;
    u_int8_t strength = 0;
    u_int8_t size = 0;

    switch (bugtype) {
    case BugType::Alien:
    {
        speed = static_cast<u_int8_t>(GenerateRandomNumberBetween(60, 70));
        strength = static_cast<u_int8_t>(GenerateRandomNumberBetween(3, 5));
        size = static_cast<u_int8_t>(GenerateRandomNumberBetween(5, 7));
        break;
    }

    case BugType::Zipper:
    {
        speed = static_cast<u_int8_t>(GenerateRandomNumberBetween(100, 120));
        strength = static_cast<u_int8_t>(GenerateRandomNumberBetween(1, 3));
        size = static_cast<u_int8_t>(GenerateRandomNumberBetween(2, 3));
        break;
    }

    case BugType::BigBertha:
    {
        speed = static_cast<u_int8_t>(GenerateRandomNumberBetween(30, 40));
        strength = static_cast<u_int8_t>(GenerateRandomNumberBetween(5, 8));
        size = static_cast<u_int8_t>(GenerateRandomNumberBetween(20, 30));
        break;
    }

    default:
    {
        std::cout << "\nBug type does not exist!";
        return;
    }
    }

    int32_t xPos = static_cast<int32_t>(GenerateRandomNumberBetween(
        config::BUG_OFFSET_FROM_WIDTH_PIXELS, config::FIELD_WIDTH_PIXELS - config::BUG_OFFSET_FROM_WIDTH_PIXELS));
    int32_t yPos = static_cast<int32_t>(GenerateRandomNumberBetween(0, config::BUG_OFFSET_FROM_WIDTH_PIXELS));

    m_bugs.push_back(m_bugfactory.CreateBug(bugtype, size, speed, strength, xPos, yPos));
}

void BugSim::processBugs(cv::Mat &frame)
{
    if (m_bugs.size() < 5 && m_tickCounter++ % config::BUG_SPAWN_TICK_INTERVAL == 0) {
        BugType randomBugType = static_cast<BugType>(GenerateRandomNumberBetween(0, 2));
        // randomBugType = BugType::BigBertha;
        AddRandomBug(randomBugType);
        m_tickCounter = 0;
        // std::cout << "Added bug!" << std::endl;
    }
    int bugToDelete = -1;
    int counter = 0;
    for (std::shared_ptr<Bug> bug : m_bugs) {
        // ~~~ ToDo ~~~
        // uint8_t bugSize = bug->getSize();
        // uint8_t bugSpeed = bug->getSpeed();
        // uint8_t bugStrength = bug->getStrength();
        Components::XYComponent &posXYBug = bug->getPos();
        if (posXYBug.posY - bug->getSize() > frame.rows) {
            bugToDelete = counter;
            continue;
        }
        DrawBug(bug, frame);
        counter++;
    }
    if (bugToDelete > -1) {
        m_bugs.erase(m_bugs.begin() + bugToDelete);
        // std::cout << "Deleted bug!";
    }
}

void BugSim::fireCommandSubCallback(const bug_zapper_msgs::msg::FireCommand::SharedPtr fireCmdMsg)
{
    // RCLCPP_WARN(this->get_logger(), "Received fireCmdMsg!: ");

    // for (uint8_t gun : fireCmdMsg->gun_id) {
    //     RCLCPP_WARN(this->get_logger(), "Gun fired!: %d", static_cast<int>(gun));
    // }
    if (m_fireCommands.size() <= 20) {
        // Calculate absolute opening and closing times

        rclcpp::Time timeZero(0, 0, rcl_clock_type_t::RCL_ROS_TIME); // Time 0
        rclcpp::Time openingTime = timeZero + rclcpp::Duration::from_seconds(fireCmdMsg->opening_time);
        rclcpp::Time closingTime = timeZero + rclcpp::Duration::from_seconds(fireCmdMsg->closing_time);

        // Create and add the item to the vector
        std::vector<uint8_t> gunIds;
        for (uint8_t gun : fireCmdMsg->gun_id) {
            gunIds.push_back(gun);
        }
        FireCommandItem item{openingTime, closingTime, gunIds};
        m_fireCommands.push_back(item);
        rclcpp::Duration timeDiff = m_timeNow - m_startTime;
        rclcpp::Time timeSinceStart = timeZero + timeDiff;
        float timeSinceStartSecs = RCL_NS_TO_MS(timeSinceStart.nanoseconds()) / 1000.0;
        RCLCPP_INFO(this->get_logger(),
                    "FireCommand received and queued! Current time: %.2f, Opening time: %.2f, Closing time: %.2f",
                    timeSinceStartSecs, fireCmdMsg->opening_time, fireCmdMsg->closing_time);
    }
    else {
        RCLCPP_WARN(this->get_logger(), "Fire command vector is full!!");
    }
}

void BugSim::drawGunTriggers(cv::Mat &frame, uint8_t gunID)
{
    int xPos = gunID * config::FIELD_WIDTH_PIXELS / config::NUM_GUNS;
    int yPos = config::FIELD_LENGTH_PIXELS - config::GUN_EXPLOSION_RADIUS;

    cv::Point center(xPos, yPos);
    // cv::Scalar lineColor(0, 0, 255);
    std::vector<cv::Scalar> gunColors = {
        cv::Scalar(255, 0, 0),    // Red
        cv::Scalar(0, 255, 0),    // Green
        cv::Scalar(0, 0, 255),    // Blue
        cv::Scalar(255, 255, 0),  // Cyan
        cv::Scalar(255, 0, 255),  // Magenta
        cv::Scalar(0, 255, 255),  // Yellow
        cv::Scalar(128, 0, 128),  // Purple
        cv::Scalar(128, 128, 0),  // Olive
        cv::Scalar(0, 128, 128),  // Teal
        cv::Scalar(192, 192, 192) // Silver
    };

    // Ensure gunID is in the range [0, 9]
    if (gunID >= 0 && gunID < 10) {
        cv::Scalar lineColor = gunColors[gunID];
        cv::circle(frame, center, 10, lineColor, -1); // Draw the circle with the selected color
        // Font settings
        int fontFace = cv::FONT_HERSHEY_SIMPLEX;
        double fontScale = 0.6; // Adjust for text size
        int thickness = 1;      // Thickness of the text
        cv::Scalar color(255, 255, 255);
        cv::putText(frame, std::to_string(gunID), center, fontFace, fontScale, color, thickness);
    }
    else {
        RCLCPP_WARN(this->get_logger(), "Invalid gunID: %d", gunID);
    }

    // int thickness = -1; // filled
    // cv::circle(frame, center, config::GUN_EXPLOSION_RADIUS, lineColor, thickness);
}

void BugSim::simTimerCallback()
{
    m_timeNow = this->get_clock()->now();

    rclcpp::Time timeZero(0, 0, rcl_clock_type_t::RCL_ROS_TIME); // Time 0
    rclcpp::Duration timeDiff = m_timeNow - m_startTime;
    rclcpp::Time timeSinceStart = timeZero + timeDiff;
    float timeSinceStartMs = RCL_NS_TO_MS(timeSinceStart.nanoseconds());
    // RCLCPP_INFO(this->get_logger(), "Current Sim time: %.2f", timeSinceStartMs);

    cv::Mat frame(cv::Size(config::FIELD_WIDTH_PIXELS, config::FIELD_LENGTH_PIXELS), CV_8UC3,
                  cv::Scalar(255, 255, 255));
    processBugs(frame);
    processFireCommandQueue(frame);
    drawCameraFrame(frame);

    cv::Mat resized;
    cv::resize(frame, resized, cv::Size(), 0.75, 0.75);
    std::string imageText = "Bug-Frame - Sim time: " + std::to_string(timeSinceStartMs);

    // cv::namedWindow("Bug-Frame");
    cv::Point textOrigin(10, 30); // Start at (10, 30) (pixels from top-left)

    // Font settings
    int fontFace = cv::FONT_HERSHEY_SIMPLEX;
    double fontScale = 0.6; // Adjust for text size
    int thickness = 1;      // Thickness of the text
    cv::Scalar color(0, 0, 0);

    // Add the text to the image
    cv::putText(resized, imageText, textOrigin, fontFace, fontScale, color, thickness);

    cv::imshow("Bug-Frame", resized);
    cv::waitKey(100);
}
} // namespace patterns
