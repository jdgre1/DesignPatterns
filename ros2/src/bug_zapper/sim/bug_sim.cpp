#include <bug_factory.h>
#include <bug_sim.h>

#include <random>

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

    // Generate and return the random numberII
    return dis(gen);
}

BugSim::BugSim(double bugSpeedMin, double bugSpeedMax, uint8_t bugStrength)
    : rclcpp_lifecycle::LifecycleNode("bug_sim_lifecycle_node"), m_bugSpeedMin(bugSpeedMin), m_bugSpeedMax(bugSpeedMax),
      m_bugStrength(bugStrength)
{
    m_startTime = this->get_clock()->now();
    m_timer = this->create_wall_timer(100ms, std::bind(&BugSim::simTimerCallback, this));
}

void BugSim::DrawBug(std::shared_ptr<Bug> bug, cv::Mat &frame, const int &timeMsPassed)
{
    Components::XYComponent &xyPos = bug->getPos();
    uint8_t speed = bug->getSpeed();
    xyPos.posY += speed * timeMsPassed / 1000.0;

    switch (bug->getType()) {
    case BugType::Alien:
    {
        cv::Point center(xyPos.posX, xyPos.posY);
        cv::uint8_t radius = bug->getSize();
        cv::Scalar line_Color(255, 0, 0);
        int thickness = 2;
        cv::circle(frame, center, radius, line_Color, thickness);
    }

    case BugType::Zipper:
    {
        cv::Point center(xyPos.posX, xyPos.posY);
        uint8_t radius = bug->getSize();
        cv::Scalar line_Color(0, 0, 255);
        int thickness = 2;
        cv::circle(frame, center, radius, line_Color, thickness);
    }

    case BugType::BigBertha:
    {
        cv::Point center(xyPos.posX, xyPos.posY);
        uint8_t radius = bug->getSize();
        cv::Scalar line_Color(0, 255, 0);
        int thickness = 2;
        cv::circle(frame, center, radius, line_Color, thickness);
    }

    default:
    {
        std::cout << "\nBug type does not exist!";
        return;
    }
    }
}

void BugSim::AddRandomBug(BugType &bugtype)
{
    u_int8_t speed = 0;
    u_int8_t strength = 0;
    u_int8_t size = 0;
    int32_t xPos = static_cast<int32_t>(
        GenerateRandomNumberBetween(BUG_OFFSET_FROM_WIDTH_PIXELS, FIELD_WIDTH_PIXELS - BUG_OFFSET_FROM_WIDTH_PIXELS));

    switch (bugtype) {
    case BugType::Alien:
    {
        speed = static_cast<u_int8_t>(GenerateRandomNumberBetween(3, 5));
        strength = static_cast<u_int8_t>(GenerateRandomNumberBetween(3, 5));
        size = static_cast<u_int8_t>(GenerateRandomNumberBetween(5, 7));
    }

    case BugType::Zipper:
    {
        speed = static_cast<u_int8_t>(GenerateRandomNumberBetween(5, 7));
        strength = static_cast<u_int8_t>(GenerateRandomNumberBetween(1, 3));
        size = static_cast<u_int8_t>(GenerateRandomNumberBetween(2, 3));
    }

    case BugType::BigBertha:
    {
        speed = static_cast<u_int8_t>(GenerateRandomNumberBetween(1, 3));
        strength = static_cast<u_int8_t>(GenerateRandomNumberBetween(5, 8));
        size = static_cast<u_int8_t>(GenerateRandomNumberBetween(5, 8));
    }

    default:
    {
        std::cout << "\nBug type does not exist!";
        return;
    }
    }

    m_bugs.push_back(m_bugfactory.CreateBug(bugtype, size, speed, strength));
}

void BugSim::simTimerCallback()
{
    cv::Mat cameraFrame(cv::Size(FIELD_WIDTH_PIXELS, FIELD_LENGTH_PIXELS), CV_8UC3, cv::Scalar(255, 255, 255));
    const int diffMs = ((this->get_clock()->now() - m_startTime).to_chrono<std::chrono::milliseconds>()).count();

    size_t numBugs = m_bugs.size();
    if (numBugs < 3 && diffMs % m_bugSpawnIntervalMs == 0) {
        BugType randomBugType = static_cast<BugType>(GenerateRandomNumberBetween(1, 3));
        AddRandomBug(randomBugType);
    }
    int bugToDelete = -1;
    int counter = 0;
    for (std::shared_ptr<Bug> bug : m_bugs) {
        // ~~~ ToDo ~~~
        // uint8_t bugSize = bug->getSize();
        // uint8_t bugSpeed = bug->getSpeed();
        // uint8_t bugStrength = bug->getStrength();
        Components::XYComponent &posXYBug = bug->getPos();
        if (posXYBug.posY - bug->getSize() > cameraFrame.rows) {
            bugToDelete = counter;
            continue;
        }
        DrawBug(bug, cameraFrame, diffMs);
        counter++;
    }
    if (bugToDelete > -1) {
        m_bugs.erase(m_bugs.begin() + bugToDelete);
    }
    cv::namedWindow("Bug-Frame");

    cv::imshow("Bug-Frame", cameraFrame);
    cv::waitKey(diffMs);
}

// std::unique_ptr<Bug> newBug = CreateBug(randomBugType, bugSize, speed,
// strength)
} // namespace patterns
