#pragma once

#include <string>  // BallisticsInput::ammo_name needs this — and ONLY this

struct Coord {
  float x;
  float y;
};

struct BallisticsInput {
  Coord drone;
  float drone_z;
  Coord target;
  float attack_speed;
  float acceleration_path;
  std::string ammo_name;
};

struct DropSolution {
  Coord fire;
  float flight_time;
  bool maneuver_used;
};

DropSolution compute_drop_solution(const BallisticsInput& input);
void print_drop_solution(const DropSolution& solution);
BallisticsInput read_input(const char* path);