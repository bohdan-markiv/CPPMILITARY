#include "ballistics.hpp"
#include <iostream>
#include <fstream>
int read_input(const char *path, BallisticsInput &input)
{
  std::ifstream inputFile{path};
  if (!inputFile) {
    std::cerr << "error: failed to open input file: " << path << '\n';
    throw std::runtime_error("Failed to open input file");
  }

  if (!(inputFile >> input.drone.x >> input.drone.y >> input.drone_z >> input.target.x >> input.target.y >> input.attack_speed >>
        input.acceleration_path >> input.ammo_name)) {
    std::cerr << "Error reading input file." << std::endl;
    throw std::runtime_error("Failed to open input file");
  }
  return 0;  // Success
}

void print_drop_solution(const DropSolution &solution)
{
  std::cout << "flight_time " << solution.flight_time << '\n';
  std::cout << "fire_x " << solution.fire.x << '\n';
  std::cout << "fire_y " << solution.fire.y << '\n';
  std::cout << "maneuver_used " << solution.maneuver_used << '\n';
}