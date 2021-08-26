#pragma once

#include "network.h"
#include <algorithm>
#include <assert.h>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <unordered_map>

inline double DegToRad(const double &input) { return input * M_PI / 180.0; }

inline bool IsRad(const double &input) {
  return ((input >= -M_PI) && (input <= M_PI));
}

// For debugging, print nice tables
namespace {

void PrintTable(std::ostream &os) { os << std::endl; }

template <typename T, typename... RemainderArgs>
void PrintTable(std::ostream &os, T &obj, RemainderArgs &... args) {
  static const char separator = ' ';
  static const int width = 18;
  os << std::left << std::setw(width) << std::setfill(separator) << obj;
  PrintTable(os, args...);
}

template <typename T, typename... RemainderArgs>
void PrintTable(T &obj, RemainderArgs &... args) {
  PrintTable(std::cout, obj, args...);
}

} // namespace

struct ChargerInfo final {
  double lat;                // in rad
  double lon;                // in rad
  double rate;               // in km/hr
  double dist_to_goal = 0.0; // in km
  double drive_cost = 0.0;   // in hrs
  double charge_cost = 0.0;  // in hrs
  explicit ChargerInfo(double lat, double lon, double rate)
      : lat(lat), lon(lon), rate(rate) {}

  double GetTotalCost() const { return drive_cost + charge_cost; }

  // For debugging, print nice tables
  friend std::ostream &operator<<(std::ostream &os, const ChargerInfo &obj) {
    PrintTable(os, obj.lat, obj.lon, obj.rate, obj.dist_to_goal, obj.drive_cost,
               obj.charge_cost);
    return os;
  }
};

using std::acos;
using std::cos;
using std::sin;
static constexpr double kEarthRad = 6356.752; // km

double GetGreatCircleDist(const ChargerInfo &pt1, const ChargerInfo &pt2) {
  assert(IsRad(pt1.lat));
  assert(IsRad(pt1.lon));
  assert(IsRad(pt2.lat));
  assert(IsRad(pt2.lon));

  const double lon_delta = std::abs(pt1.lon - pt2.lon);
  const double central_ang = acos(sin(pt1.lat) * sin(pt2.lat) +
                                  cos(pt1.lat) * cos(pt2.lat) * cos(lon_delta));

  return central_ang * kEarthRad;
}
