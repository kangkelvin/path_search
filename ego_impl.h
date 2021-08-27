#include "ego.h"
#include "math_helper.h"

namespace ego {

template <std::size_t Size>
Ego::Ego(const ChargerLoc &init_loc, const ChargerLoc &goal_loc,
         const std::array<row, Size> &network)
    : init_loc_(init_loc), goal_loc_(goal_loc) {
  // Create network map from network array
  for (const auto &node : network) {
    network_map_.emplace(node.name, ChargerInfo(DegToRad(node.lat),
                                                DegToRad(node.lon), node.rate));
  }

  // Calc dist to goal from all nodes, used for A* heuristics
  for (auto &node : network_map_) {
    node.second.dist_to_goal = GetDistBetweenTwoChargers(node.first, goal_loc_);
  }

  CreateNetworkAdjList();
}

void Ego::CalcPathAndPrint() {
  using ChargerCostPair = std::pair<ChargerLoc, double>;
  const auto cmp = [](const ChargerCostPair &lhs, const ChargerCostPair &rhs) {
    return lhs.second > rhs.second;
  };
  std::priority_queue<ChargerCostPair, std::vector<ChargerCostPair>,
                      decltype(cmp)>
      priority_queue(cmp);

  // Tracks chargers that have been visited
  std::unordered_map<ChargerLoc, bool> visited;
  // Time cost to come to chargers
  std::unordered_map<ChargerLoc, double> cost_to_come;
  // Tracks the parent nodes, to obtain best path at the end
  std::unordered_map<ChargerLoc, ChargerCostPair> parent;

  for (const auto &node : network_map_) {
    visited.emplace(node.first, false);
    cost_to_come.emplace(node.first, std::numeric_limits<double>::max());
  }

  // Init starting conditions for A* search
  priority_queue.emplace(init_loc_, 0);
  visited[init_loc_] = true;
  cost_to_come.at(init_loc_) = 0.0;
  ChargerCostPair dummy = std::make_pair(init_loc_, 0.0);
  parent.emplace(init_loc_, dummy);
  bool solution_found = false;

  while (!priority_queue.empty()) {
    const auto front = priority_queue.top();
    priority_queue.pop();

    if (front.first == goal_loc_) {
      solution_found = true;
      break;
    }

    for (const auto &neighbor : network_adj_list_.at(front.first)) {
      const double cost_to_go = neighbor.second.GetTotalCost();
      if (cost_to_come.at(neighbor.first) >
          cost_to_come.at(front.first) + cost_to_go) {
        cost_to_come.at(neighbor.first) =
            cost_to_come.at(front.first) + cost_to_go;
        parent.emplace(
            neighbor.first,
            std::make_pair(front.first, neighbor.second.charge_cost));
      }
      if (!visited.at(neighbor.first)) {
        // A* heuristics
        const double cost_and_heuristics =
            cost_to_come.at(neighbor.first) +
            network_map_.at(neighbor.first).dist_to_goal / kEgoSpeed;
        priority_queue.emplace(neighbor.first, cost_and_heuristics);
        visited.at(neighbor.first) = true;
      }
    }
  }

  if (solution_found) {
    std::vector<ChargerCostPair> solution;

    {
      ChargerLoc iter = goal_loc_;
      while (iter != init_loc_) {
        solution.push_back(parent.at(iter));
        iter = parent.at(iter).first;
      }
    }

    std::cout << init_loc_ << ", ";

    for (auto it = solution.rbegin() + 1; it != solution.rend(); ++it) {
      // Special case at the last charger, only charge enough to go to goal
      if (it + 1 == solution.rend()) {
        const double charge_rate = network_map_.at(it->first).rate;
        const double remaining_charge =
            kMaxCharge - it->second * network_map_.at(it->first).rate;
        const double opt_cost =
            (network_map_.at(it->first).dist_to_goal - remaining_charge) /
            charge_rate;

        std::cout << it->first << ", " << opt_cost << ", ";
      } else {
        // Default case, charge to full before going to next charger
        std::cout << it->first << ", " << it->second << ", ";
      }
    }

    std::cout << goal_loc_ << std::endl;
  } else {
    std::cout << "Solution Not Found\n";
  }
}

double Ego::GetDistBetweenTwoChargers(const ChargerLoc &initial_charger_name,
                                      const ChargerLoc &goal_charger_name) {
  const auto init_iter = network_map_.find(initial_charger_name);
  const auto goal_iter = network_map_.find(goal_charger_name);

  return GetGreatCircleDist(init_iter->second, goal_iter->second);
}

void Ego::CreateNetworkAdjList() {
  // Create network adjacency list for graph search
  for (const auto &node : network_map_) {
    network_adj_list_.emplace(node.first, NetworkMap());
    for (const auto &neighbor : network_map_) {
      if (node.first == neighbor.first) {
        continue;
      }

      const double dist_to_neighbor =
          GetGreatCircleDist(node.second, neighbor.second);
      if (dist_to_neighbor > kMaxCharge) {
        continue;
      }

      network_adj_list_.at(node.first).emplace(neighbor.first, neighbor.second);
      network_adj_list_.at(node.first).at(neighbor.first).drive_cost =
          dist_to_neighbor / kEgoSpeed;
      network_adj_list_.at(node.first).at(neighbor.first).charge_cost =
          dist_to_neighbor / neighbor.second.rate;
    }
  }
}

} // namespace ego