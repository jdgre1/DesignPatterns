#include <random>

#include <bug_factory.h>
#include <bug_sim.h>

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
    : rclcpp_lifecycle::LifecycleNode("bug_sim_lifecycle_node")
    , m_bugSpeedMin(bugSpeedMin)
    , m_bugSpeedMax(bugSpeedMax)
    , m_bugStrength(bugStrength)
{
    m_startTime = this->get_clock()->now();
    m_timer = this->create_wall_timer(100ms, std::bind(&BugSim::simTimerCallback, this));
    m_cameraFramePub = this->create_publisher<sensor_msgs::msg::Image>("cameraFrame", 10);
}

void BugSim::DrawBug(std::shared_ptr<Bug> bug, cv::Mat& frame)
{
    Components::XYComponent& xyPos = bug->getPos();
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
        cv::Scalar lineColor(0, 255, 0);
        cv::circle(frame, center, radius, lineColor, thickness);
        break;
    }

    default:
    {
        std::cout << "\nBug type does not exist2!";
        return;
    }
    }
}

void BugSim::drawCameraFrame(cv::Mat& frame)
{
    // and its top left corner...
    int y1 = int(FIELD_LENGTH_PIXELS * 0.5);
    int y2 = int(FIELD_LENGTH_PIXELS * 0.80);
    cv::Point pt1(2, y1);
    // and its bottom right corner.
    cv::Point pt2(FIELD_WIDTH_PIXELS - 2, y2);
    // These two calls...
    cv::rectangle(frame, pt1, pt2, cv::Scalar(255, 0, 255), 3);
    cv::putText(frame,
                "Camera Frame",
                cv::Point(10, frame.rows / 2 - 10),
                cv::FONT_HERSHEY_DUPLEX,
                1.0,
                CV_RGB(118, 185, 0),
                2);

    cv::Mat cameraFrame = frame(cv::Range(pt1.y, pt2.y), cv::Range(pt1.x, pt2.x));
    std_msgs::msg::Header header;
    header.stamp = this->get_clock()->now(); // Add timestamp
    sensor_msgs::msg::Image::SharedPtr imgMsg = cv_bridge::CvImage(header, "bgr8", cameraFrame).toImageMsg();
    m_cameraFramePub->publish(*imgMsg.get());
    std::cout << "Published!" << std::endl;
}

void BugSim::AddRandomBug(BugType& bugtype)
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
        std::cout << "\nBug type does not exist33!";
        return;
    }
    }

    int32_t xPos = static_cast<int32_t>(
        GenerateRandomNumberBetween(BUG_OFFSET_FROM_WIDTH_PIXELS, FIELD_WIDTH_PIXELS - BUG_OFFSET_FROM_WIDTH_PIXELS));
    int32_t yPos = static_cast<int32_t>(GenerateRandomNumberBetween(0, BUG_OFFSET_FROM_WIDTH_PIXELS));

    m_bugs.push_back(m_bugfactory.CreateBug(bugtype, size, speed, strength, xPos, yPos));
}

void BugSim::processBugs(cv::Mat& frame)
{
    if (m_bugs.size() < 5 && m_tickCounter++ % m_bugSpawnTickInterval == 0) {
        BugType randomBugType = static_cast<BugType>(GenerateRandomNumberBetween(0, 2));
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
        Components::XYComponent& posXYBug = bug->getPos();
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

void BugSim::simTimerCallback()
{
    cv::Mat field(cv::Size(FIELD_WIDTH_PIXELS, FIELD_LENGTH_PIXELS), CV_8UC3, cv::Scalar(255, 255, 255));
    processBugs(field);
    drawCameraFrame(field);

    cv::Mat resized;
    cv::resize(field, resized, cv::Size(), 0.75, 0.75);
    cv::namedWindow("Bug-Frame");
    cv::imshow("Bug-Frame", resized);
    cv::waitKey(100);
}
// std::unique_ptr<Bug> newBug = CreateBug(randomBugType, bugSize, speed,
// strength)
} // namespace patterns
