#include "ego.h"

int main(int argc, char **argv) {
  if (argc != 3) {
    std::cout << "Error: requires initial and final supercharger names"
              << std::endl;
    return -1;
  }

  std::string initial_charger_name = argv[1];
  std::string goal_charger_name = argv[2];

  ego::Ego ego(initial_charger_name, goal_charger_name, network);
  ego.CalcPathAndPrint();

  return 0;
}