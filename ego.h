#pragma once

#include "math_helper.h"
#include "network.h"

#include <array>
#include <queue>
#include <utility>

namespace ego {

using ChargerLoc = std::string;
using NetworkMap = std::unordered_map<std::string, ChargerInfo>;
using NetworkAdjList = std::unordered_map<std::string, NetworkMap>;

class Ego final {
public:
  template <std::size_t Size>
  explicit Ego(const ChargerLoc &init_loc, const ChargerLoc &goal_loc,
               const std::array<row, Size> &network);
  Ego(const Ego &rhs) = delete;
  Ego(Ego &&rhs) = delete;
  Ego &operator=(const Ego &rhs) = delete;
  Ego &operator=(Ego &&rhs) = delete;

  // Main func of this class, calcultate best path and prints it out
  void CalcPathAndPrint();

private:
  double GetDistBetweenTwoChargers(const ChargerLoc &initial_charger_name,
                                   const ChargerLoc &goal_charger_name);

  void CreateNetworkAdjList();

  // Map of charger name to charger info
  NetworkMap network_map_;
  // Adjacency list, which chargers are reachable from a certain charger
  NetworkAdjList network_adj_list_;
  ChargerLoc init_loc_;
  ChargerLoc goal_loc_;

  static constexpr double kMaxCharge = 320.0; // km
  static constexpr double kEgoSpeed = 105.0;  // km/hr
};

} // namespace ego

#include "ego_impl.h"