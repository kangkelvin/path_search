#pragma once

#include "math_helper.h"
#include "network.h"

namespace ego {

class Ego final {
public:
  template <std::size_t Size>
  explicit Ego(const std::string &init_loc, const std::string &goal_loc,
               const std::array<row, Size> &network);
  Ego(const Ego &rhs) = default;
  Ego(Ego &&rhs) = default;
  Ego &operator=(const Ego &rhs) = default;
  Ego &operator=(Ego &&rhs) = default;

  void CalcPathAndPrint();

private:
  struct ChargerSearchInfo {
    std::string charger_name;
    double dist_to_charger;
    ChargerSearchInfo(std::string charger_name, double dist_to_charger)
        : charger_name(charger_name), dist_to_charger(dist_to_charger) {}
  };

  ChargerSearchInfo FindFurthestReachableCharger(const std::string &search_loc);

  void GotoChargerAndUpdateState(const ChargerSearchInfo &next_charger_info);

  double
  ChargeAndGetChargingTime(const ChargerSearchInfo &search_result_lookahead);

  NetworkMap network_map_;
  double curr_charge_ = 320.0;
  std::string curr_loc_;
  std::string goal_loc_;
  double delta_to_goal_;

  static constexpr double kMaxCharge = 320.0; // km
  static constexpr double kEgoSpeed = 105.0;  // km/hr
};

} // namespace ego

#include "ego_impl.h"