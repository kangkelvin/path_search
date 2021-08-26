#include "ego.h"

#include <array>
#include <assert.h>
#include <string>

namespace ego {

template <std::size_t Size>
Ego::Ego(const std::string &init_loc, const std::string &goal_loc,
         const std::array<row, Size> &network)
    : curr_loc_(init_loc), goal_loc_(goal_loc) {
  // Create network map from network array
  for (const auto &node : network) {
    network_map_.emplace(node.name, ChargerInfo(DegToRad(node.lat),
                                                DegToRad(node.lon), node.rate));
  }

  // Calc dist to goal from all nodes
  for (auto &node : network_map_) {
    node.second.dist_to_goal =
        GetDistBetweenTwoChargers(network_map_, node.first, goal_loc_);
  }

  delta_to_goal_ = network_map_.at(curr_loc_).dist_to_goal;
}

void Ego::CalcPathAndPrint() {
  std::cout << curr_loc_ << ", ";

  while (true) {
    const auto search_result = this->FindFurthestReachableCharger(curr_loc_);
    // For last charge optimization
    const auto search_result_lookahead =
        this->FindFurthestReachableCharger(search_result.charger_name);

    // Termination case
    if (search_result.charger_name == goal_loc_) {
      std::cout << goal_loc_;
      return;
    }

    this->GotoChargerAndUpdateState(search_result);

    const double charge_time =
        this->ChargeAndGetChargingTime(search_result_lookahead);

    std::cout << search_result.charger_name << ", " << charge_time << ", ";
  }
}

Ego::ChargerSearchInfo
Ego::FindFurthestReachableCharger(const std::string &search_loc) {
  double final_dist_to_charger = 0.0;
  double min_time_cost = std::numeric_limits<double>::max();
  std::string furthest_charger = search_loc;

  for (const auto &node : network_map_) {
    if (search_loc == node.first) {
      continue;
    }

    if (node.second.dist_to_goal > delta_to_goal_) {
      continue;
    }

    const double dist_to_charger =
        GetDistBetweenTwoChargers(network_map_, search_loc, node.first);

    const double time_cost = node.second.dist_to_goal / kEgoSpeed +
                             (kMaxCharge - dist_to_charger) / node.second.rate;

    if (dist_to_charger < curr_charge_ && time_cost < min_time_cost) {
      final_dist_to_charger = dist_to_charger;
      min_time_cost = time_cost;
      furthest_charger = node.first;
    }
  }
  return ChargerSearchInfo(furthest_charger, final_dist_to_charger);
}

void Ego::GotoChargerAndUpdateState(
    const ChargerSearchInfo &next_charger_info) {
  curr_loc_ = next_charger_info.charger_name;
  curr_charge_ -= next_charger_info.dist_to_charger;
  delta_to_goal_ = network_map_.at(curr_loc_).dist_to_goal;
  assert(curr_charge_ > 0.0);
}

double Ego::ChargeAndGetChargingTime(
    const ChargerSearchInfo &search_result_lookahead) {
  // calculate charging time and update ego's charge
  const auto helper_func = [this](double target_charge) {
    const double charge_time =
        (target_charge - curr_charge_) / network_map_.at(curr_loc_).rate;
    curr_charge_ = target_charge;
    return charge_time;
  };

  // special case just before reaching goal. Charge just enough to reach goal.
  if (search_result_lookahead.charger_name == goal_loc_) {
    return helper_func(search_result_lookahead.dist_to_charger);
  }

  return helper_func(kMaxCharge);
}

} // namespace ego