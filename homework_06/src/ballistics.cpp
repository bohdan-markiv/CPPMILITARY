#include "ballistics.hpp"
#include <iostream>
#include <sstream>
#define _USE_MATH_DEFINES
#include <cmath>
#include <fstream>
#include <string>
// #include <cstring>
float g = 9.81f;
struct AmmoParams {
  std::string name;
  float mass;
  float drag;
  float lift;
};

const int AMMO_COUNT = 5;
// float M_PI = std::acos(-1.0f);
inline float distanceCalculation(Coord drone, Coord target)
{
  return sqrt(pow(target.x - drone.x, 2) + pow(target.y - drone.y, 2));
}

float ammoParametersIdentification(std::string ammo_name, float &m, float &d, float &l)
{
  const int AMMO_COUNT = 5;

  AmmoParams ammoParams[AMMO_COUNT] = {{"VOG-17", 0.35f, 0.07f, 0.0f},
                                       {"M67", 0.6f, 0.1f, 0.0f},
                                       {"RKG-3", 1.2f, 0.1f, 0.0f},
                                       {"GLIDING-VOG", 0.45f, 0.1f, 1.0f},
                                       {"GLIDING-RKG", 1.4f, 0.1f, 1.0f}};
  int found = -1;
  for (int i = 0; i < AMMO_COUNT; i++) {
    if (ammo_name == ammoParams[i].name) {
      found = i;
      break;
    }
  }
  if (found == -1) {
    throw std::invalid_argument("unknown ammo type: " + ammo_name);
    return -1.0f;  // Return an error code;
  }

  m = ammoParams[found].mass;
  d = ammoParams[found].drag;
  l = ammoParams[found].lift;
  return 0.0f;  // Success
}

float ammoFlyDistance(float m, float d, float l, float &t, float attackSpeed, float zd)
{
  float a = d * g * m - 2.0f * pow(d, 2) * l * attackSpeed;
  // std::cout << "The acceleration of the ammo is: " << a << std::endl;
  // b = −3g·m² + 3d·l·m·V₀
  float b = -3.0f * g * pow(m, 2) + 3 * d * l * m * attackSpeed;
  // std::cout << "The coefficient b is: " << b << std::endl;
  // c = 6m²·Z₀
  float c = 6 * pow(m, 2) * zd;
  // std::cout << "The coefficient c is: " << c << std::endl;
  if (a == 0.0f) {
    throw std::runtime_error("The coefficient a cannot be zero.");
    return 0.0f;
  }
  // p = − b² / (3a²)
  float p = (-1.0f * pow(b, 2)) / (3.0f * pow(a, 2));
  // std::cout << "The coefficient p is: " << p << std::endl;
  // q = 2b³ / (27a³) + c / a
  float q = ((2.0f * pow(b, 3)) / (27.0f * pow(a, 3)) + c / a);
  // std::cout << "The coefficient q is: " << q << std::endl;

  // arg = 3q / (2p) · √(−3/p)
  float arg = ((3.0f * q) / (2.0f * p) * sqrt(-3.0f / p));
  // std::cout << "The argument for the cubic root is: " << arg << std::endl;
  if (arg < -1.0f || arg > 1.0f) {
    throw std::runtime_error("The argument for the cubic root is out of range: " + std::to_string(arg));
    return 0.0f;
  }
  float fphi = acos(arg);
  // std::cout << "The angle fphi is: " << fphi << std::endl;

  // t = 2√(−p/3) · cos( (φ + 4π) / 3 ) − b / (3a)
  t = (2.0f * sqrt(-p / 3.0f) * cos((fphi + 4.0f * M_PI) / 3.0f) - b / (3.0f * a));

  if (t < 1e-3f) {
    throw std::runtime_error("The time of flight cannot be negative: " + std::to_string(t));
    return 0.0f;
    return -1.0f;
  }
  //   std::cout << "The time of flight t is: " << t << std::endl;

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
  float term1 = attackSpeed * t;
  float term2 = ((t2 * d * attackSpeed) / (2.0f * m));

  float term3_num = 6.0f * d * g * l * m - 6.0f * (d * d) * (l2 - 1.0f) * attackSpeed;
  float term3 = ((t3 * term3_num) / (36.0f * m * m));

  float term4_num = -6.0f * (d * d) * g * l * (1.0f + l2 + l4) * m + 3.0f * std::pow(d, 3) * l2 * (1.0f + l2) * attackSpeed +
                    6.0f * std::pow(d, 3) * l4 * (1.0f + l2) * attackSpeed;
  float term4_den = 36.0f * pow(1.0f + l2, 2) * pow(m, 3);
  float term4 = ((t4 * term4_num) / term4_den);

  float term5_num = 3.0f * pow(d, 3) * g * (l2 * l) * m - 3.0f * pow(d, 4) * l2 * (1.0f + l2) * attackSpeed;
  float term5_den = 36.0f * (1.0f + l2) * pow(m, 4);
  float term5 = ((t5 * term5_num) / term5_den);

  float h = term1 - term2 + term3 + term4 + term5;
  if (h < 0.0f) {
    std::cout << "The distance to target cannot be negative: " << h << std::endl;
    return -1.0f;
  }
  std::cout << "The distance to target is: " << h << std::endl;
  return h;
}

Coord apply_maneuver(Coord drone, Coord target, float full_distance, float h, float acceleration_path)
{
  if (full_distance < 1e-6f) {
    return {target.x - (h + acceleration_path), target.y};
  }
  float k = (h + acceleration_path) / full_distance;
  return {target.x - (target.x - drone.x) * k, target.y - (target.y - drone.y) * k};
}

DropSolution compute_drop_solution(const BallisticsInput &input)

{
  DropSolution solution;
  //   std::ofstream out("output.txt");
  std::cout << "The input values are: " << input.drone.x << ", " << input.drone.y << ", " << input.target.x << ", " << input.target.y
            << std::endl;
  float fullDistance = distanceCalculation(input.drone.x, input.drone.y, input.target.x, input.target.y);
  std::cout << "The full distance to target is: " << fullDistance << std::endl;
  // if (fullDistance <= 0.0f)
  // {
  //     std::cout << "Mistake with distance." << std::endl;
  //     continue;
  // }
  float d = 0.0f, m = 0.0f;
  float l = 0.0f;
  if (ammoParametersIdentification(input.ammo_name, m, d, l) < 0.0f) {
    throw std::runtime_error("Failed to identify ammo parameters");
  }
  std::cout << "Ammo parameters: mass = " << m << " kg, drag coefficient = " << d << ", lift coefficient = " << l << std::endl;

  float t = 0.0f;
  ;
  float h = ammoFlyDistance(m, d, l, t, input.attackSpeed, input.drone_z);
  if (h < 0.0f) {
    throw std::runtime_error("Failed to compute ammo fly distance");
  }

  solution.maneuver_used = false;
  if (h + input.accelerationPath > fullDistance) {
    input.drone = apply_maneuver(input.drone, input.target, fullDistance, h, input.accelerationPath);
    solution.maneuver_used = true;
  }

  // ratio = (D − h) / D
  float ratio = ((fullDistance - h) / fullDistance);
  solution.flight_time = t;
  solution.fire.x = input.drone.x + (input.target.x - input.drone.x) * ratio;
  solution.fire.y = input.drone.y + (input.target.y - input.drone.y) * ratio;
  std::cout << "The coordinates to fire are: (" << solution.fire.x << " " << solution.fire.y << ")" << std::endl;
  //   out << solution.fire.x << " " << solution.fire.y << std::endl;

  return solution;
}

int read_input(const char *path, BallisticsInput &input)
{
  std::ifstream inputFile{path};
  if (!inputFile) {
    std::cerr << "error: failed to open input file: " << path << '\n';
    throw std::runtime_error("Failed to open input file");
  }

  if (!(inputFile >> input.drone.xd >> input.drone.yd >> input.drone_z >> input.target.x >> input.target.y >> input.attack_speed >>
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