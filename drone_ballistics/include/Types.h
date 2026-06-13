#pragma once

#include <vector>
#include "nlohmann/json.hpp"

#include <cmath>
#include <string>

using json = nlohmann::json;
const float kPi = std::acos(-1.0F);
const float g = 9.81f;

const int MAX_STEPS = 10000;
enum DronePhase {
  STOPPED,
  ACCELERATING,
  DECELERATING,
  TURNING,
  MOVING

};

enum class SolverType { ANALYTICAL, TABLE };
enum class ProviderType { JSON };
enum class LoaderType { FILE };
struct Result {
  float t;      // час польоту
  float hDist;  // горизонтальна дистанція
};
struct Coord {
  float x;
  float y;

  Coord operator+(const Coord& other) const
  {
    Coord result;
    result.x = x + other.x;
    result.y = y + other.y;
    return result;
  }

  Coord operator-(const Coord& other) const
  {
    Coord result;
    result.x = x - other.x;
    result.y = y - other.y;
    return result;
  }

  Coord operator*(float scalar) const
  {
    Coord result;
    result.x = x * scalar;
    result.y = y * scalar;
    return result;
  }

  Coord operator/(float scalar) const
  {
    Coord result;
    result.x = x / scalar;
    result.y = y / scalar;
    return result;
  }

  bool operator==(const Coord& other) const
  {
    bool result;
    result = (x == other.x) && (y == other.y);
    return result;
  }
};

struct AmmoParams {
  std::string name;
  float mass;
  float drag;
  float lift;
};
struct Interp {
  int lo;      // нижній індекс в осі
  float frac;  // коефіцієнт [0..1]
};

struct DroneConfig {
  Coord startPos;        // початкова позиція (x, y)
  float altitude;        // висота
  float initialDir;      // початковий напрямок (рад)
  float attackSpeed;     // швидкість атаки (м/с)
  float accelPath;       // шлях розгону (м)
  std::string ammoName;  // обрані боєприпаси
  float arrayTimeStep;   // крок часу масиву цілей
  float simTimeStep;     // крок симуляції
  float hitRadius;       // радіус влучення
  float angularSpeed;    // кутова швидкість (рад/с)
  float turnThreshold;   // поріг повороту (рад)
};

struct SimStep {
  Coord pos;              // позиція дрона
  float direction;        // напрямок (рад)
  DronePhase state;       // стан автомата (0-4)
  int targetIdx;          // індекс поточної цілі
  Coord dropPoint;        // точка скиду (куди летить дрон)
  Coord aimPoint;         // куди впаде бомба (якщо скинути зараз)
  Coord predictedTarget;  // прогнозована позиція цілі
};

extern std::vector<SimStep> simLog;
