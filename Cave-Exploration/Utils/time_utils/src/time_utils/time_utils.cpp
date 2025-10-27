#include "time_utils/time_utils.h"

namespace time_utils
{
  void Timer::Start()
  {
    timer_start_ = std::chrono::high_resolution_clock::now();
    is_started_ = true;
  }

  void Timer::Stop(bool show, std::string unit)
  {
    is_started_ = false;
    timer_stop_ = std::chrono::high_resolution_clock::now();
    duration_ = timer_stop_ - timer_start_;
    if (show)
    {
      if (unit == "ms")
      {
        std::cout << name_ << " takes " << GetDuration<std::chrono::milliseconds>() << " ms" << std::endl;
      }
      else if (unit == "us")
      {
        std::cout << name_ << " takes " << GetDuration<std::chrono::microseconds>() << " us" << std::endl;
      }
      else if (unit == "ns")
      {
        std::cout << name_ << " takes " << GetDuration<std::chrono::nanoseconds>() << " ns" << std::endl;
      }
      else if (unit == "s")
      {
        std::cout << name_ << " takes " << GetDuration<std::chrono::seconds>() << " s" << std::endl;
      }
      else
      {
        std::cout << "timer unit error!" << std::endl;
      }
    }
  }

  bool Timer::isStart()
  {
    return is_started_;
  }

  int64_t Timer::getStartTime(std::string unit)
  {
    if (unit == "s")
    {
      return std::chrono::duration_cast<std::chrono::seconds>(timer_start_.time_since_epoch()).count();
    }
    else if (unit == "ms")
    {
      return std::chrono::duration_cast<std::chrono::milliseconds>(timer_start_.time_since_epoch()).count();
    }
    else if (unit == "us")
    {
      return std::chrono::duration_cast<std::chrono::microseconds>(timer_start_.time_since_epoch()).count();
    }
    else if (unit == "ns")
    {
      return std::chrono::duration_cast<std::chrono::nanoseconds>(timer_start_.time_since_epoch()).count();
    }
    else
    {
      std::cout << "timer unit error!" << std::endl;
      return 0;
    }
  }

  int64_t Timer::getStopTime(std::string unit)
  {
    if (unit == "s")
    {
      return std::chrono::duration_cast<std::chrono::seconds>(timer_stop_.time_since_epoch()).count();
    }
    else if (unit == "ms")
    {
      return std::chrono::duration_cast<std::chrono::milliseconds>(timer_stop_.time_since_epoch()).count();
    }
    else if (unit == "us")
    {
      return std::chrono::duration_cast<std::chrono::microseconds>(timer_stop_.time_since_epoch()).count();
    }
    else if (unit == "ns")
    {
      return std::chrono::duration_cast<std::chrono::nanoseconds>(timer_stop_.time_since_epoch()).count();
    }
    else
    {
      std::cout << "timer unit error!" << std::endl;
      return 0;
    }
  }

  template <class U>
  int64_t Timer::GetDuration()
  {
    return std::chrono::duration_cast<U>(duration_).count();
  }

  int64_t Timer::GetDuration(std::string unit)
  {
    if (unit == "ms")
    {
      return std::chrono::duration_cast<std::chrono::milliseconds>(duration_).count();
    }
    else if (unit == "us")
    {
      return std::chrono::duration_cast<std::chrono::microseconds>(duration_).count();
    }
    else if (unit == "ns")
    {
      return std::chrono::duration_cast<std::chrono::nanoseconds>(duration_).count();
    }
    else if (unit == "s")
    {
      return std::chrono::duration_cast<std::chrono::seconds>(duration_).count();
    }
    else
    {
      std::cout << "timer unit error!" << std::endl;
      return 0;
    }
  }

  int64_t Timer::GetDurationTemp(std::string unit)
  {
    if (unit == "ms")
    {
      return GetTimeNow("ms") - getStartTime("ms");
    }
    else if (unit == "us")
    {
      return GetTimeNow("us") - getStartTime("us");
    }
    else if (unit == "ns")
    {
      return GetTimeNow("ns") - getStartTime("ns");
    }
    else if (unit == "s")
    {
      return GetTimeNow("s") - getStartTime("s");
    }
    else
    {
      std::cout << "timer unit error!" << std::endl;
      return 0;
    }
  }

  int64_t Timer::GetTimeNow(std::string unit)
  {
    if (unit == "ms")
    {
      return std::chrono::duration_cast<std::chrono::milliseconds>(
                 std::chrono::high_resolution_clock::now().time_since_epoch())
          .count();
    }
    else if (unit == "us")
    {
      return std::chrono::duration_cast<std::chrono::microseconds>(
                 std::chrono::high_resolution_clock::now().time_since_epoch())
          .count();
    }
    else if (unit == "ns")
    {
      return std::chrono::duration_cast<std::chrono::nanoseconds>(
                 std::chrono::high_resolution_clock::now().time_since_epoch())
          .count();
    }
    else if (unit == "s")
    {
      return std::chrono::duration_cast<std::chrono::seconds>(
                 std::chrono::high_resolution_clock::now().time_since_epoch())
          .count();
    }
    else
    {
      std::cout << "timer unit error!" << std::endl;
      return 0;
    }
  }
}
