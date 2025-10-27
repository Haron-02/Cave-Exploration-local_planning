#ifndef _TIME_UTILS_H_
#define _TIME_UTILS_H_

#include <iostream>
#include <chrono>
#include <string>
#include <memory>

namespace time_utils
{
  class Timer
  {
  private:
    bool is_started_;
    std::string name_;
    std::chrono::time_point<std::chrono::high_resolution_clock> timer_start_;
    std::chrono::time_point<std::chrono::high_resolution_clock> timer_stop_;
    std::chrono::duration<double> duration_;

  public:
    typedef std::shared_ptr<Timer> Ptr;

    Timer(std::string name) : name_(name)
    {
      is_started_ = false;
    }

    Timer()
    {
      is_started_ = false;
      name_ = "~";
    }
    ~Timer() = default;
    void Start();

    void Stop(bool show = false, std::string unit = "ms");

    bool isStart();

    int64_t getStartTime(std::string unit = "ms");
    int64_t getStopTime(std::string unit = "ms");

    template <class U>
    int64_t GetDuration();

    int64_t GetDuration(std::string unit = "ms");

    int64_t GetDurationTemp(std::string unit = "ms");

    static int64_t GetTimeNow(std::string unit = "ms");
  };
}

#endif