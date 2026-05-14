#include "ballistics.hpp"
#include <iostream>
#include <fstream>
#include <cstdlib>

BallisticsInput read_input(const char* path)
{
  std::ifstream file{path};
  if (!file) {
    throw std::runtime_error(std::string("cannot open input file: ") + path);
  }

  BallisticsInput input{};

  file >> input.drone.x;
  if (file.fail()) {
    throw std::runtime_error("bad or missing value for field: drone_x");
  }

  file >> input.drone.y;
  if (file.fail()) {
    throw std::runtime_error("bad or missing value for field: drone_y");
  }

  file >> input.drone_z;
  if (file.fail()) {
    throw std::runtime_error("bad or missing value for field: drone_z");
  }

  file >> input.target.x;
  if (file.fail()) {
    throw std::runtime_error("bad or missing value for field: target_x");
  }

  file >> input.target.y;
  if (file.fail()) {
    throw std::runtime_error("bad or missing value for field: target_y");
  }

  file >> input.attack_speed;
  if (file.fail()) {
    throw std::runtime_error("bad or missing value for field: attack_speed");
  }

  file >> input.acceleration_path;
  if (file.fail()) {
    throw std::runtime_error("bad or missing value for field: acceleration_path");
  }

  file >> input.ammo_name;
  if (file.fail()) {
    throw std::runtime_error("bad or missing value for field: ammo_name");
  }

  return input;
}

void print_drop_solution(const DropSolution& solution)
{
  std::cout << "flight_time " << solution.flight_time << '\n';
  std::cout << "fire_x " << solution.fire.x << '\n';
  std::cout << "fire_y " << solution.fire.y << '\n';
  std::cout << "maneuver_used " << solution.maneuver_used << '\n';
}