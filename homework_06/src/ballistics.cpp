
#include "ballistics.hpp"
#include <cmath>
#include <string>
#include <stdexcept>
// #include <cstring>
const float kGravity = 9.81F;
const float kPi = std::acos(-1.0F);
struct AmmoParams {
  std::string name;
  float mass;
  float drag;
  float lift;
};

// float M_PI = std::acos(-1.0f);
inline auto distance_calculation(Coord drone, Coord target) -> float
{
  return sqrt(pow(target.x - drone.x, 2) + pow(target.y - drone.y, 2));
}

auto ammo_parameters_identification(std::string ammo_name, float &m, float &d, float &l) -> float
{
  const int kAmmoCount = 5;

  AmmoParams ammo_params[kAmmoCount] = {{"VOG-17", 0.35F, 0.07F, 0.0F},
                                        {"M67", 0.6F, 0.1F, 0.0F},
                                        {"RKG-3", 1.2F, 0.1F, 0.0F},
                                        {"GLIDING-VOG", 0.45F, 0.1F, 1.0F},
                                        {"GLIDING-RKG", 1.4F, 0.1F, 1.0F}};
  int found = -1;
  for (int i = 0; i < kAmmoCount; i++) {
    if (ammo_name == ammo_params[i].name) {
      found = i;
      break;
    }
  }
  if (found == -1) {
    throw std::invalid_argument("unknown ammo type: " + ammo_name);
  }

  m = ammo_params[found].mass;
  d = ammo_params[found].drag;
  l = ammo_params[found].lift;
  return 0.0F;  // Success
}

auto ammo_fly_distance(float m, float d, float l, float &t, float attack_speed, float zd) -> float
{
  float a = d * kGravity * m - 2.0F * pow(d, 2) * l * attack_speed;
  // b = −3g·m² + 3d·l·m·V₀
  float b = -3.0F * kGravity * pow(m, 2) + 3 * d * l * m * attack_speed;
  // c = 6m²·Z₀
  float c = 6 * pow(m, 2) * zd;
  if (a == 0.0F) {
    throw std::runtime_error("The coefficient a cannot be zero.");
  }
  // p = − b² / (3a²)
  float p = (-1.0F * pow(b, 2)) / (3.0F * pow(a, 2));
  // q = 2b³ / (27a³) + c / a
  float q = ((2.0F * pow(b, 3)) / (27.0F * pow(a, 3)) + c / a);

  // arg = 3q / (2p) · √(−3/p)
  float arg = ((3.0F * q) / (2.0F * p) * sqrt(-3.0F / p));
  if (arg < -1.0F || arg > 1.0F) {
    throw std::runtime_error("The argument for the cubic root is out of range: " + std::to_string(arg));
  }
  float fphi = acos(arg);

  // t = 2√(−p/3) · cos( (φ + 4π) / 3 ) − b / (3a)
  t = (2.0F * sqrt(-p / 3.0F) * cos((fphi + 4.0F * kPi) / 3.0F) - b / (3.0F * a));

  if (t < 1e-3F) {
    throw std::runtime_error("The time of flight cannot be negative: " + std::to_string(t));
  }

  // h = V₀t − t²d·V₀/(2m) + t³(6d·g·l·m − 6d²(l²-1)·V₀)/(36m²) +
  //  + t⁴ (−6d²g·l·(1+l²+l⁴)m + 3d³l²(1+l²)V₀ + 6d³l⁴(1+l²)V₀)  / (36(1+l²)²m³)
  //  + t⁵(3d³g·l³m − 3d⁴l²(1+l²)V₀) / (36(1+l²)m⁴)
  //  Розбиваємо на доданки (t^1, t^2, t^3...)
  float t2 = t * t;
  float t3 = t2 * t;
  float t4 = t3 * t;
  float t5 = t4 * t;
  float l2 = l * l;
  float l4 = l2 * l2;

  // Обчислюємо кожен доданок окремо
  float term1 = attack_speed * t;
  float term2 = ((t2 * d * attack_speed) / (2.0F * m));

  float term3_num = 6.0F * d * kGravity * l * m - 6.0F * (d * d) * (l2 - 1.0F) * attack_speed;
  float term3 = ((t3 * term3_num) / (36.0F * m * m));

  float term4_num = -6.0F * (d * d) * kGravity * l * (1.0F + l2 + l4) * m + 3.0F * std::pow(d, 3) * l2 * (1.0F + l2) * attack_speed +
                    6.0F * std::pow(d, 3) * l4 * (1.0F + l2) * attack_speed;
  float term4_den = 36.0F * pow(1.0F + l2, 2) * pow(m, 3);
  float term4 = ((t4 * term4_num) / term4_den);

  float term5_num = 3.0F * pow(d, 3) * kGravity * (l2 * l) * m - 3.0F * pow(d, 4) * l2 * (1.0F + l2) * attack_speed;
  float term5_den = 36.0F * (1.0F + l2) * pow(m, 4);
  float term5 = ((t5 * term5_num) / term5_den);

  float h = term1 - term2 + term3 + term4 + term5;
  if (h < 0.0F) {
    throw std::runtime_error("The fly distance cannot be negative: " + std::to_string(h));
  }
  return h;
}

auto apply_maneuver(Coord drone, Coord target, float full_distance, float h, float acceleration_path) -> Coord
{
  if (full_distance < 1e-6F) {
    return {target.x - (h + acceleration_path), target.y};
  }
  float k = (h + acceleration_path) / full_distance;
  return {target.x - (target.x - drone.x) * k, target.y - (target.y - drone.y) * k};
}

auto compute_drop_solution(const BallisticsInput &input) -> DropSolution

{
  DropSolution solution{};

  float full_distance = distance_calculation(input.drone, input.target);
  // if (full_distance <= 0.0F)
  // {
  //     std::cout << "Mistake with distance." << std::endl;
  //     continue;
  // }
  float d = 0.0F, m = 0.0F;
  float l = 0.0F;
  if (ammo_parameters_identification(input.ammo_name, m, d, l) < 0.0F) {
    throw std::runtime_error("Failed to identify ammo parameters");
  }

  float t = 0.0F;
  float h = ammo_fly_distance(m, d, l, t, input.attack_speed, input.drone_z);
  if (h < 0.0F) {
    throw std::runtime_error("Failed to compute ammo fly distance");
  }

  Coord drone = input.drone;
  solution.maneuver_used = false;
  if (h + input.acceleration_path > full_distance) {
    drone = apply_maneuver(input.drone, input.target, full_distance, h, input.acceleration_path);
    full_distance = distance_calculation(drone, input.target);
    solution.maneuver_used = true;
  }

  // ratio = (D − h) / D
  float ratio = ((full_distance - h) / full_distance);
  solution.flight_time = t;
  solution.fire.x = drone.x + (input.target.x - drone.x) * ratio;
  solution.fire.y = drone.y + (input.target.y - drone.y) * ratio;
  //   out << solution.fire.x << " " << solution.fire.y << std::endl;

  return solution;
}
