#pragma once

#include <iostream>
#include <chrono>


class Timer
{
    public:
        Timer();


        double getElapsedSeconds(void);
        void printElapsedSeconds(void);


    private:
        std::chrono::time_point<std::chrono::steady_clock> m_startPoint;
        std::chrono::time_point<std::chrono::steady_clock> m_endPoint;
        std::chrono::duration<double> m_elapsedSeconds;

        void calculateElapsedSeconds(void);
};


Timer::Timer()
{ 
    m_startPoint = std::chrono::steady_clock::now();
}


void Timer::calculateElapsedSeconds(void)
{
    m_endPoint = std::chrono::steady_clock::now();
    m_elapsedSeconds = m_endPoint - m_startPoint;
}

double Timer::getElapsedSeconds(void)
{
    calculateElapsedSeconds();
    return m_elapsedSeconds.count();
}

void Timer::printElapsedSeconds(void)
{
    calculateElapsedSeconds();
    std::cout << "Total Execution Time: " << m_elapsedSeconds.count() << " s\n";
}

