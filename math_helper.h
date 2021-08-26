#pragma once

#include <algorithm>
#include <assert.h>
#include <cmath>
#include <iostream>
#include <unordered_map>

inline double DegToRad(const double &input) { return input * M_PI / 180.0; }

inline bool IsRad(const double &input) {
  return ((input >= -M_PI) && (input <= M_PI));
}

struct ChargerInfo final {
  double lat;          // in rad
  double lon;          // in rad
  double rate;         // in km/hr
  double dist_to_goal; // in km
  ChargerInfo(double lat, double lon, double rate)
      : lat(lat), lon(lon), rate(rate), dist_to_goal(0.0) {}

  friend std::ostream &operator<<(std::ostream &os, const ChargerInfo &obj) {
    os << obj.lat << " " << obj.lon << " " << obj.rate << " "
       << obj.dist_to_goal;
    return os;
  }
};

using NetworkMap = std::unordered_map<std::string, ChargerInfo>;

double GetGreatCircleDist(const ChargerInfo &pt1, const ChargerInfo &pt2) {
  using std::acos;
  using std::cos;
  using std::sin;

  static constexpr double kEarthRad = 6356.752; // km

  const double lon_delta = std::abs(pt1.lon - pt2.lon);
  const double central_ang = acos(sin(pt1.lat) * sin(pt2.lat) +
                                  cos(pt1.lat) * cos(pt2.lat) * cos(lon_delta));

  return central_ang * kEarthRad;
}

double GetDistBetweenTwoChargers(const NetworkMap &network_map,
                                 const std::string &initial_charger_name,
                                 const std::string &goal_charger_name) {
  const auto init_iter = network_map.find(initial_charger_name);
  const auto goal_iter = network_map.find(goal_charger_name);

  return GetGreatCircleDist(init_iter->second, goal_iter->second);
}
