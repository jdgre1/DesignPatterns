#include <random> 
#include <opencv2/core/core.hpp>

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
        return dis(gen);

    }

    void BugSim::AddRandomBug(BugType& bugtype)
    {
        u_int8_t speed = 0;
        u_int8_t strength = 0;
        u_int8_t size = 0;
        
        switch(bugtype)
            {
                
            case BugType::Alien:
            {
                speed = static_cast<u_int8_t>(GenerateRandomNumberBetween(3,5));
                strength = static_cast<u_int8_t>(GenerateRandomNumberBetween(3,5));
                size = static_cast<u_int8_t>(GenerateRandomNumberBetween(5,7));
            }

            case BugType::Zipper:
            {
                speed = static_cast<u_int8_t>(GenerateRandomNumberBetween(5,7));
                strength = static_cast<u_int8_t>(GenerateRandomNumberBetween(1,3));
                size = static_cast<u_int8_t>(GenerateRandomNumberBetween(2,3));
            }

            case BugType::BigBertha:
            {
                speed = static_cast<u_int8_t>(GenerateRandomNumberBetween(1,3));
                strength = static_cast<u_int8_t>(GenerateRandomNumberBetween(5,8));
                size = static_cast<u_int8_t>(GenerateRandomNumberBetween(5,8));
            }

            default:
            {
                std::cout << "\nBug type does not exist!";
                return;
            }
            }

        m_bugs.push_back(m_bugfactory.CreateBug(bugtype, size, speed, strength));

    }

    BugSim::BugSim(double bugSpeedMin, double bugSpeedMax, uint8_t bugStrength)
    : rclcpp_lifecycle::LifecycleNode("bug_sim_lifecycle_node")
        , m_bugSpeedMin(bugSpeedMin)
        , m_bugSpeedMax(bugSpeedMax)
        , m_bugStrength(bugStrength)
    {
        m_startTime = this->get_clock()->now();
        m_timer =  this->create_wall_timer(500ms, std::bind(&BugSim::simTimerCallback, this));

    }


    void BugSim::simTimerCallback()
    {
        cv::Mat cameraFrame(cv::Size(1000,800), CV_8UC3, cv::Scalar(255,255,255));

        size_t numBugs = m_bugs.size();
        if (numBugs < 3) {
            BugType randomBugType = static_cast<BugType>(GenerateRandomNumberBetween(1, 3));
            AddRandomBug(randomBugType);
        }

        for (std::shared_ptr<Bug> bug: m_bugs) 
        {
            // ~~~ ToDo ~~~
            // bugSize = bug.getSize();
            // bugSpeed = bug.getSpeed();
            // bugStrength = bug.getStrength();
            // bugPos = bug.getPos();
            // bug.updatePos();
        }
    }



            // std::unique_ptr<Bug> newBug = CreateBug(randomBugType, bugSize, speed, strength) 
    }
    
