#ifndef BUGSIM_H
#define BUGSIM_H

#include <iostream>
namespace patterns
{

class BugSim
{
public:
    BugSim(double bugSpeedMin, double bugSpeedMax, uint8_t m_bugStrength);

    void start();

private:

    double m_bugSpeedMin;
    double m_bugSpeedMax;
    uint8_t m_bugStrength;

};

}  // namespace patterns
#endif