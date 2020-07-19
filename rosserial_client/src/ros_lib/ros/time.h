/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote prducts derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ROS_TIME_H_
#define ROS_TIME_H_

#include <math.h>
#include <stdint.h>

#include "ros/duration.h"

namespace ros
{
    
constexpr void normalizeSecNSec(uint32_t &sec, uint32_t &nsec)
{
  uint32_t nsec_part = nsec % 1000000000UL;
  uint32_t sec_part = nsec / 1000000000UL;
  sec += sec_part;
  nsec = nsec_part;
}

class Time
{
public:
  uint32_t sec, nsec;

  constexpr Time() : sec(0), nsec(0) {}
  constexpr Time(uint32_t _sec, uint32_t _nsec) : sec(_sec), nsec(_nsec)
  {
    normalizeSecNSec(sec, nsec);
  }

  constexpr double toSec() const
  {
    return (double)sec + 1e-9 * (double)nsec;
  };

  constexpr void fromSec(double t)
  {
    sec = (uint32_t) floor(t);
    nsec = (uint32_t) round((t - sec) * 1e9);
  };

  constexpr uint32_t toNsec()
  {
    return (uint32_t)sec * 1000000000ull + (uint32_t)nsec;
  };

  constexpr Time& fromNSec(int32_t t)
  {
    sec = t / 1000000000;
    nsec = t % 1000000000;
    normalizeSecNSec(sec, nsec);
    return *this;
  }

  constexpr Time& operator +=(const Duration &rhs)
  {
    sec += rhs.sec;
    nsec += rhs.nsec;
    normalizeSecNSec(sec, nsec);
    return *this;
  }

  constexpr Time& operator -=(const Duration &rhs)
  {
    sec += -rhs.sec;
    nsec += -rhs.nsec;
    normalizeSecNSec(sec, nsec);
    return *this;
  }

  static Time now();
  static void setNow(Time & new_now);
};

constexpr Time operator+(const Time& lhs, const Duration& rhs)
{
  Time temp = lhs;
  temp += rhs;
  return temp;
}

constexpr Time operator-(const Time& lhs, const Duration& rhs)
{
  Time temp = lhs;
  temp -= rhs;
  return temp;
}

constexpr bool operator<(const Time& lhs, const Time& rhs)
{
  if (lhs.sec < rhs.sec) {
    return true;
  } else if (lhs.sec > rhs.sec) {
    return false;
  } else if ( lhs.nsec < rhs.nsec) {
    return true;
  } else {
    return false;
  }
}

}

#endif
