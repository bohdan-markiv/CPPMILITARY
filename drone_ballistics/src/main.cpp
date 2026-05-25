#define ENABLE_LOG 1
#define ENABLE_DEBUG 0

#if ENABLE_LOG
#define LOG(msg) std::cout << msg << std::endl
#else
#define LOG(msg)
#endif

#if ENABLE_DEBUG
#define DEBUG(msg) std::cout << msg << std::endl
#else
#define DEBUG(msg)
#endif

const float g = 9.81f;

const int MAX_STEPS = 10000;

#include <limits>
#include <cstring>
#include <iostream>
#include "nlohmann/json.hpp"
// #define _USE_MATH_DEFINES
#include <cmath>
#include <fstream>
#include <string>
const float kPi = std::acos(-1.0F);
using json = nlohmann::json;

enum DronePhase {
  STOPPED,
  ACCELERATING,
  DECELERATING,
  TURNING,
  MOVING

};
enum class SolverType { ANALYTICAL };
enum class ProviderType { JSON };
enum class LoaderType { FILE };

struct Velocity {
  float velocityx;
  float velocityy;
};

struct Coord {
  float x;
  float y;

  Coord operator+(const Coord &other) const
  {
    Coord result;
    result.x = x + other.x;
    result.y = y + other.y;
    return result;
  }

  Coord operator-(const Coord &other) const
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

  bool operator==(const Coord &other) const
  {
    bool result;
    result = (x == other.x) && (y == other.y);
    return result;
  }
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Coord, x, y)
struct AmmoParams {
  char name[32];
  float mass;
  float drag;
  float lift;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(AmmoParams, name, mass, drag, lift)

struct DroneConfig {
  Coord startPos;       // початкова позиція (x, y)
  float altitude;       // висота
  float initialDir;     // початковий напрямок (рад)
  float attackSpeed;    // швидкість атаки (м/с)
  float accelPath;      // шлях розгону (м)
  char ammoName[32];    // обрані боєприпаси
  float arrayTimeStep;  // крок часу масиву цілей
  float simTimeStep;    // крок симуляції
  float hitRadius;      // радіус влучення
  float angularSpeed;   // кутова швидкість (рад/с)
  float turnThreshold;  // поріг повороту (рад)
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(DroneConfig,
                                   startPos,
                                   altitude,
                                   initialDir,
                                   attackSpeed,
                                   accelPath,
                                   ammoName,
                                   arrayTimeStep,
                                   simTimeStep,
                                   hitRadius,
                                   angularSpeed,
                                   turnThreshold)

struct SimStep {
  Coord pos;              // позиція дрона
  float direction;        // напрямок (рад)
  DronePhase state;       // стан автомата (0-4)
  int targetIdx;          // індекс поточної цілі
  Coord dropPoint;        // точка скиду (куди летить дрон)
  Coord aimPoint;         // куди впаде бомба (якщо скинути зараз)
  Coord predictedTarget;  // прогнозована позиція цілі
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(SimStep, pos, direction, state, targetIdx, dropPoint, aimPoint, predictedTarget)

struct SimulationLog {
  int totalSteps;
  SimStep *steps;
};

struct TargetsConfig {
  int targetCount;
  int timeSteps;
  Coord **positions;  // 2D array: positions[targetIdx][timeStep]
};

inline float distanceCalculation(Coord currentCoord, Coord targetCoord)
{
  return sqrt(pow(targetCoord.x - currentCoord.x, 2) + pow(targetCoord.y - currentCoord.y, 2));
}

float timeToTarget(float m, float d, float l, float attackSpeed, float zd)
{
  float a = d * g * m - 2.0f * pow(d, 2) * l * attackSpeed;
  // DEBUG("The acceleration of the ammo is: " << a);
  // b = −3g·m² + 3d·l·m·V₀
  float b = -3.0f * g * pow(m, 2) + 3 * d * l * m * attackSpeed;
  // std::cout << "The coefficient b is: " << b << std::endl;
  // c = 6m²·Z₀
  float c = 6 * pow(m, 2) * zd;
  // std::cout << "The coefficient c is: " << c << std::endl;
  if (a == 0.0f) {
    LOG("The coefficient a cannot be zero.");
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
    LOG("The argument for the cubic root is out of range: " << arg);
    return 0.0f;
  }
  float fphi = acos(arg);
  // std::cout << "The angle fphi is: " << fphi << std::endl;

  // t = 2√(−p/3) · cos( (φ + 4π) / 3 ) − b / (3a)
  float t = (2.0f * sqrt(-p / 3.0f) * cos((fphi + 4.0f * kPi) / 3.0f) - b / (3.0f * a));
  if (t < 1e-3f) {
    LOG("The time of flight cannot be negative: " << t);
    return -1.0f;
  }
  // std::cout << "The time of flight t is: " << t << std::endl;
  return t;
}
float ammoFlyDistance(float t, float d, float l, float attackSpeed, float m)
{
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
    LOG("The distance to target cannot be negative: " << h);
    return -1.0f;
  }
  // std::cout << "The distance to target is: " << h << std::endl;
  return h;
}

void interpolateTarget(float t, float arrayTimeStep, Coord *trajectory, int timeSteps, Coord &interpolatedCoord)
{
  int idx = (int)floor(t / arrayTimeStep) % timeSteps;
  int next = (idx + 1) % timeSteps;
  float frac = (t - idx * arrayTimeStep) / arrayTimeStep;
  interpolatedCoord = trajectory[idx] + (trajectory[next] - trajectory[idx]) * frac;
}

float calculateTimeToStop(float currentSpeed, float attackSpeed, bool targetChanged, DronePhase phase, float remainingTurnTime, float a)
{
  float fullAccelTime = attackSpeed / a;
  float timeToStop = 0.0f;

  if (targetChanged) {
    switch (phase) {
      case STOPPED:
        timeToStop = 0.0f;
        break;
      case ACCELERATING:
        timeToStop = currentSpeed / a;
        break;  // stop from current speed
      case MOVING:
        timeToStop = fullAccelTime;
        break;
      case DECELERATING:
        timeToStop = currentSpeed / a;
        break;
      case TURNING:
        timeToStop = remainingTurnTime;
        break;
    }
  }
  return timeToStop;
}
inline float angleDifference(float from, float to)
{
  float diff = fmod(to - from + kPi, 2.0f * kPi);
  if (diff < 0)
    diff += 2.0f * kPi;
  return diff - kPi;
}
class ITargetProvider {
public:
  virtual ~ITargetProvider() {}
  virtual void load() = 0;
  virtual int getTargetCount() = 0;
  virtual int getTimeSteps() = 0;
  virtual Coord *getTarget(int idx) = 0;
};

class IBallisticSolver {
public:
  virtual ~IBallisticSolver(){};
  virtual Coord solver(Coord currentCoord, float zd, Coord targetCoord, float attackSpeed, double m, double d, double l) = 0;
};

class IConfigLoader {
public:
  virtual ~IConfigLoader(){};
  virtual void load() = 0;
  virtual DroneConfig getConfig() = 0;
  virtual AmmoParams getAmmoParams() = 0;
};

class JsonTargetProvider : public ITargetProvider {
private:
  TargetsConfig targets{0, 0, nullptr};

public:
  JsonTargetProvider() = default;
  JsonTargetProvider(const JsonTargetProvider &) = delete;
  JsonTargetProvider &operator=(const JsonTargetProvider &) = delete;
  void load() override
  {
    std::ifstream targetsFile("drone_ballistics/data/targets.json");
    if (!targetsFile.is_open()) {
      std::cerr << "Cannot open targets.json" << std::endl;
      throw std::runtime_error("Cannot open targets.json");
    }
    json targetsRaw = json::parse(targetsFile);

    this->targets.targetCount = targetsRaw["targetCount"];
    this->targets.timeSteps = targetsRaw["timeSteps"];

    this->targets.positions = new Coord *[this->targets.targetCount];
    for (int i = 0; i < this->targets.targetCount; i++) {
      // Allocate each row
      this->targets.positions[i] = new Coord[this->targets.timeSteps];

      // Get a reference to the specific target's positions array
      const auto &jsonPositions = targetsRaw["targets"][i]["positions"];

      for (int j = 0; j < this->targets.timeSteps; j++) {
        // Access the specific position object once
        const auto &pos = jsonPositions[j];

        this->targets.positions[i][j].x = pos["x"].get<float>();
        this->targets.positions[i][j].y = pos["y"].get<float>();
      }
    };
  }
  int getTargetCount() override { return this->targets.targetCount; }
  int getTimeSteps() override { return this->targets.timeSteps; }
  Coord *getTarget(int idx) override
  {  // Return initial position for simplicity
    return this->targets.positions[idx];
  }
  ~JsonTargetProvider()
  {
    // Clean up allocated memory
    for (int i = 0; i < this->targets.targetCount; i++) {
      delete[] this->targets.positions[i];
    }
    delete[] this->targets.positions;
  }
};

class AnalyticalSolver : public IBallisticSolver {
public:
  Coord solver(Coord currentCoord, float zd, Coord targetCoord, float attackSpeed, double m, double d, double l) override
  {
    Coord fireCoord;
    float fullDistance = distanceCalculation(currentCoord, targetCoord);

    float t = timeToTarget(m, d, l, attackSpeed, zd);
    if (t < 0.0f) {
      LOG("Error calculating time to target.");
      throw std::runtime_error("Error calculating time to target.");
    }
    float h = ammoFlyDistance(t, d, l, attackSpeed, m);
    if (h < 0.0f) {
      LOG("Error calculating ammo fly distance.");
      throw std::runtime_error("Error calculating ammo fly distance.");
    }

    // if (h + accelerationPath > fullDistance)
    // {
    //     manouvreDistance(fullDistance, xd, yd, targetX, targetY, h, accelerationPath);
    // }

    // ratio = (D − h) / D, clamped to [0,1]: if target is within bomb range, drop from current position
    float ratio = (fullDistance > 0.0f) ? std::max(0.0f, (fullDistance - h) / fullDistance) : 0.0f;
    fireCoord.x = currentCoord.x + (targetCoord.x - currentCoord.x) * ratio;
    fireCoord.y = currentCoord.y + (targetCoord.y - currentCoord.y) * ratio;
    return fireCoord;
  }
};

class FileConfigLoader : public IConfigLoader {
  DroneConfig config;
  AmmoParams *ammoParams = nullptr;
  int AMMO_COUNT = 0;
  AmmoParams selectedAmmo;

public:
  FileConfigLoader() = default;
  FileConfigLoader(const FileConfigLoader &) = delete;
  FileConfigLoader &operator=(const FileConfigLoader &) = delete;
  ~FileConfigLoader() override { delete[] ammoParams; }
  void load() override
  {
    std::ifstream ammoFile("drone_ballistics/data/ammo.json");
    if (!ammoFile.is_open()) {
      throw std::runtime_error("Cannot open ammo.json");
    }
    json ammo = json::parse(ammoFile);
    // Put the read ammos in dynamic array
    this->AMMO_COUNT = ammo.size();
    this->ammoParams = new AmmoParams[this->AMMO_COUNT];
    for (int i = 0; i < this->AMMO_COUNT; i++) {
      std::strncpy(ammoParams[i].name, ammo[i]["name"].get<std::string>().c_str(), sizeof(ammoParams[i].name) - 1);
      ammoParams[i].mass = ammo[i]["mass"].get<double>();
      ammoParams[i].drag = ammo[i]["drag"].get<double>();
      ammoParams[i].lift = ammo[i]["lift"].get<double>();
    }
    std::ifstream configFile("drone_ballistics/data/config.json");

    if (!configFile.is_open()) {
      throw std::runtime_error("Cannot open config.json");
    }
    json data = json::parse(configFile);

    this->config.startPos = Coord{data["drone"]["position"]["x"], data["drone"]["position"]["y"]};
    this->config.altitude = data["drone"]["altitude"];
    this->config.initialDir = data["drone"]["initialDirection"];
    this->config.attackSpeed = data["drone"]["attackSpeed"];
    this->config.accelPath = data["drone"]["accelerationPath"];
    strncpy(this->config.ammoName, data["ammo"].get<std::string>().c_str(), sizeof(this->config.ammoName) - 1);
    this->config.ammoName[sizeof(this->config.ammoName) - 1] = '\0';
    this->config.arrayTimeStep = data["targetArrayTimeStep"];
    this->config.simTimeStep = data["simulation"]["timeStep"];
    this->config.hitRadius = data["simulation"]["hitRadius"];
    this->config.angularSpeed = data["drone"]["angularSpeed"];
    this->config.turnThreshold = data["drone"]["turnThreshold"];

    int found = -1;
    for (int i = 0; i < this->AMMO_COUNT; i++) {
      if (strcmp(this->config.ammoName, ammoParams[i].name) == 0) {
        selectedAmmo = ammoParams[i];
        found = 1;
      }
    }
    if (found == -1) {
      LOG("Unknown ammo type: " << this->config.ammoName);
      throw std::runtime_error("Unknown ammo type in config.json");
    }
    else {
      DEBUG("Ammo parameters: mass = " << selectedAmmo.mass << " kg, drag coefficient = " << selectedAmmo.drag
                                       << ", lift coefficient = " << selectedAmmo.lift);
    }
  }
  DroneConfig getConfig() override
  {
    // Return loaded DroneConfig
    return this->config;
  }
  AmmoParams getAmmoParams() override
  {
    return this->selectedAmmo;  // Return the selected ammo parameters
  }
  int getAmmoCount() const { return this->AMMO_COUNT; }
};

IBallisticSolver *createSolver(SolverType type)
{
  switch (type) {
    case SolverType::ANALYTICAL:
      return new AnalyticalSolver();
  }
  throw std::runtime_error("Unknown solver type");
};
ITargetProvider *createProvider(ProviderType type)
{
  switch (type) {
    case ProviderType::JSON:
      return new JsonTargetProvider();
  }
  throw std::runtime_error("Unknown provider type");
};
IConfigLoader *createLoader(LoaderType type)
{
  switch (type) {
    case LoaderType::FILE:
      return new FileConfigLoader();
  }
  throw std::runtime_error("Unknown loader type");
};

class Mission {
  IBallisticSolver *solver;
  ITargetProvider *targets;
  IConfigLoader *configs;

  DroneConfig config;
  AmmoParams ammo;
  int currentIteration = 0;
  int currentIdx = 0;
  int targetCount = 0;

  // SimStep *steps = new SimStep[MAX_STEPS];
  SimStep simulation;

  float t = 0.0f;

  // OUTPUT  arrays
  int N = 0;
  // float droneCoordinates[MAX_STEPS * 2];
  // float directions[MAX_STEPS];
  // float phases[MAX_STEPS];
  // float indexes[MAX_STEPS];
  bool TARGET_HIT = false;
  // int CURRENT_TARGET = -1;
  int PREVIOUS_TARGET = -1;
  bool currentDirInitialized = false;
  float currentDir = -1.0f;  // Undefined direction at the start
  float currentSpeed = 0.0f;
  float remainingTurnTime = 0.0f;

  float t_ammo = 0.0f;
  float h_ammo = 0.0f;
  float a = 0.0f;
  int timeSteps = 0;

public:
  Mission(IBallisticSolver *solver, ITargetProvider *targets, IConfigLoader *configs)
    : solver(solver)
    , targets(targets)
    , configs(configs)
  {
  }

  void init()
  {
    configs->load();
    targets->load();

    this->config = configs->getConfig();
    this->ammo = configs->getAmmoParams();
    this->targetCount = targets->getTargetCount();
    this->timeSteps = targets->getTimeSteps();

    simulation.pos = config.startPos;
    simulation.direction = -1.0f;
    simulation.state = STOPPED;
    simulation.targetIdx = -1;
    simulation.dropPoint = {0.0f, 0.0f};
    simulation.aimPoint = {0.0f, 0.0f};
    simulation.predictedTarget = {0.0f, 0.0f};

    currentSpeed = 0.0f;
    remainingTurnTime = 0.0f;
    t = 0.0f;
    N = 0;
    TARGET_HIT = false;

    a = config.attackSpeed * config.attackSpeed / (2.0f * config.accelPath);
    t_ammo = timeToTarget(ammo.mass, ammo.drag, ammo.lift, config.attackSpeed, config.altitude);
    h_ammo = ammoFlyDistance(t_ammo, ammo.drag, ammo.lift, config.attackSpeed, ammo.mass);

    currentIteration = 0;
    currentIdx = 0;
  }
  bool hasNext() { return !TARGET_HIT && N < MAX_STEPS; }
  int getCurrentTargetIdx() { return this->currentIdx; }

  void reset() { init(); }

  void changeSolver(IBallisticSolver *newSolver) { this->solver = newSolver; }

  SimStep step()
  {
    float bestTime = std::numeric_limits<float>::max();
    int chosenIdx = -1;
    Coord chosenDrop{0.0f, 0.0f};
    Coord chosenTargetPos{0.0f, 0.0f};

    for (int i = 0; i < targetCount; i++) {
      bool targetChanged = (i != simulation.targetIdx);

      Coord targetCoord, nextCoord;
      interpolateTarget(t, config.arrayTimeStep, targets->getTarget(i), timeSteps, targetCoord);
      interpolateTarget(t, config.arrayTimeStep, targets->getTarget(i), timeSteps, nextCoord);
      Coord dCoord = nextCoord - targetCoord;
      float targetVx = dCoord.x / config.simTimeStep;
      float targetVy = dCoord.y / config.simTimeStep;

      Coord dropPoint = solver->solver(simulation.pos, config.altitude, targetCoord, config.attackSpeed, ammo.mass, ammo.drag, ammo.lift);

      float desiredDir = atan2(dropPoint.y - simulation.pos.y, dropPoint.x - simulation.pos.x);
      float angleDiff = angleDifference(simulation.direction, desiredDir);

      float startSpeed = currentSpeed;

      if (targetChanged) {
        switch (simulation.state) {
          case STOPPED:
            startSpeed = 0.0f;
            break;
          case ACCELERATING:
            startSpeed = currentSpeed;
            break;  // continue accelerating from current speed
          case MOVING:
            startSpeed = config.attackSpeed;
            break;  // already at max speed
          case DECELERATING:
            startSpeed = currentSpeed;
            break;
          case TURNING:
            startSpeed = currentSpeed;
            break;  // maintain speed during turn
        }
      };

      float penaltyTime = 0.0f;

      if (fabs(angleDiff) > config.turnThreshold) {
        float turningTime = fabs(angleDiff) / config.angularSpeed;
        penaltyTime += turningTime;
        startSpeed = 0.0f;  // Assume we stop to turn
      }

      float timeToStop = calculateTimeToStop(currentSpeed, config.attackSpeed, targetChanged, simulation.state, remainingTurnTime, a);
      penaltyTime += timeToStop;

      float timeToAccel = 0.0f;
      float distanceDuringAccel = 0.0f;
      if (startSpeed < config.attackSpeed) {
        timeToAccel = (config.attackSpeed - startSpeed) / a;
        distanceDuringAccel = startSpeed * timeToAccel + 0.5f * a * timeToAccel * timeToAccel;
      }

      penaltyTime += timeToAccel;

      float droneTravel = distanceCalculation(simulation.pos, dropPoint);
      float cruiseDistance = droneTravel - distanceDuringAccel;
      float totalTimeToTarget = penaltyTime + cruiseDistance / config.attackSpeed;

      Coord predictedCoord;
      for (int iter = 0; iter < 2; iter++) {
        predictedCoord.x = targetCoord.x + targetVx * (totalTimeToTarget + t_ammo);
        predictedCoord.y = targetCoord.y + targetVy * (totalTimeToTarget + t_ammo);
        dropPoint = solver->solver(simulation.pos, config.altitude, predictedCoord, config.attackSpeed, ammo.mass, ammo.drag, ammo.lift);
        droneTravel = distanceCalculation(simulation.pos, dropPoint);
        cruiseDistance = droneTravel - distanceDuringAccel;
        totalTimeToTarget = penaltyTime + cruiseDistance / config.attackSpeed;
      }

      if (totalTimeToTarget < 0.0f) {
        LOG("Calculated negative time to target, which is invalid.");
        throw std::runtime_error("Calculated negative time to target, which is invalid.");
      }
      if (totalTimeToTarget < bestTime) {
        bestTime = totalTimeToTarget;
        chosenIdx = i;
        chosenDrop = dropPoint;
        chosenTargetPos = targetCoord;
      }
    }

    simulation.targetIdx = chosenIdx;
    simulation.dropPoint = chosenDrop;
    simulation.predictedTarget = chosenTargetPos;

    interpolateTarget(t + t_ammo, config.arrayTimeStep, targets->getTarget(chosenIdx), timeSteps, simulation.aimPoint);

    N++;
    t += config.simTimeStep;
    return simulation;
  }
};

int main()
{
  IConfigLoader *configLoader = createLoader(LoaderType::FILE);
  ITargetProvider *targetProvider = createProvider(ProviderType::JSON);
  IBallisticSolver *ballisticSolver = createSolver(SolverType::ANALYTICAL);

  Mission mission(ballisticSolver, targetProvider, configLoader);

  mission.init();

  DroneConfig config = configLoader->getConfig();
  AmmoParams ammoParams = configLoader->getAmmoParams();
  while (mission.hasNext()) {
    SimStep step = mission.step();
    std::cout << "Step " << /*step counter*/ ": pos (" << step.pos.x << ", " << step.pos.y << ")  target=" << step.targetIdx << "  drop=("
              << step.dropPoint.x << ", " << step.dropPoint.y << ")\n";
  }
  delete configLoader;
  delete targetProvider;
  delete ballisticSolver;

  return 0;
}

//   while (N < MAX_STEPS && !TARGET_HIT) {
//     bool targetChanged;
//     float timeToCurrentTarget = 999999.0f;

//     // float a = (startingValues.attackSpeed * startingValues.attackSpeed) / (2.0f * startingValues.accelPath);
//     // const float t_ammo = timeToTarget(m, d, l, startingValues.attackSpeed, startingValues.altitude);
//     // const float h_ammo = ammoFlyDistance(t_ammo, d, l, startingValues.attackSpeed, m);

//     while (mission.hasNext()) {
//       Coord dropPoint = mission.step(simulation.pos, t);
//       std::cout << "Step " << mission.getCurrentTargetIdx() << ": Drop point at (" << dropPoint.x << ", " << dropPoint.y << ")"
//                 << std::endl;
//       targetChanged = (mission.getCurrentTargetIdx() != simulation.targetIdx);
//       if (!currentDirInitialized) {
//         simulation.direction = config.initialDir;
//         currentDirInitialized = true;
//       }
//       float totalTimeToTarget = 0.0f;
//       float penaltyTime = 0.0f;
//       float startSpeed;
//       float turningTime = 0.0f;

//       Velocity targetVelocity = mission.getVelocity();

//       float desiredDir = atan2(dropPoint.y - simulation.pos.y, dropPoint.x - simulation.pos.x);
//       float angleDiff = angleDifference(simulation.direction, desiredDir);

//       if (targetChanged) {
//         switch (simulation.state) {
//           case STOPPED:
//             startSpeed = 0.0f;
//             break;
//           case ACCELERATING:
//             startSpeed = currentSpeed;
//             break;  // continue accelerating from current speed
//           case MOVING:
//             startSpeed = config.attackSpeed;
//             break;  // already at max speed
//           case DECELERATING:
//             startSpeed = currentSpeed;
//             break;  // continue decelerating from current speed
//           case TURNING:
//             startSpeed = currentSpeed;
//             break;  // maintain speed during turn
//         }
//       }
//       else {
//         startSpeed = currentSpeed;
//       }

//       if (fabs(angleDiff) > config.turnThreshold) {
//         turningTime = fabs(angleDiff) / config.angularSpeed;
//         penaltyTime += turningTime;
//         startSpeed = 0.0f;  // Assume we stop to turn
//       }

//       float timeToStop =
//         calculateTimeToStop(currentSpeed, config.attackSpeed, config.accelPath, targetChanged, simulation.state, remainingTurnTime, a);

//       penaltyTime += timeToStop;

//       float timeToAccel = 0.0f;
//       float distanceDuringAccel = 0.0f;
//       if (startSpeed < config.attackSpeed) {
//         timeToAccel = (config.attackSpeed - startSpeed) / a;
//         distanceDuringAccel = startSpeed * timeToAccel + 0.5f * a * timeToAccel * timeToAccel;
//       }
//       penaltyTime += timeToAccel;
//       float droneTravel = distanceCalculation(simulation.pos, dropPoint);

//       float cruiseDistance = droneTravel - distanceDuringAccel;
//       totalTimeToTarget = penaltyTime + cruiseDistance / config.attackSpeed;
//       Coord predictedCoord;
//       for (int iter = 0; iter < 2; iter++) {
//         predictedCoord.x = targetCoord.x + targetVx * (totalTimeToTarget + t_ammo);
//         predictedCoord.y = targetCoord.y + targetVy * (totalTimeToTarget + t_ammo);
//         calculateDropOffPoint(simulation.pos,
//                               startingValues.altitude,
//                               predictedCoord,
//                               startingValues.attackSpeed,
//                               startingValues.accelPath,
//                               AMMO_COUNT,
//                               ammoParams,
//                               startingValues.ammoName,
//                               fireCoord);
//         droneTravel = distanceCalculation(simulation.pos, fireCoord);
//         cruiseDistance = droneTravel - distanceDuringAccel;
//         totalTimeToTarget = penaltyTime + cruiseDistance / startingValues.attackSpeed;
//       }

//       if (totalTimeToTarget < 0.0f) {
//         throw std::runtime_error("Calculated negative time to target, which is invalid.");
//       }
//       if (totalTimeToTarget < timeToCurrentTarget) {
//         timeToCurrentTarget = totalTimeToTarget;
//         simulation.targetIdx = i;
//         simulation.dropPoint.x = fireCoord.x;
//         simulation.dropPoint.y = fireCoord.y;

//         interpolateTarget(
//           simulation.targetIdx, t + t_ammo, startingValues.arrayTimeStep, NUMBER_OF_TARGETS, MAX_TIME_STEPS, targets,
//           simulation.aimPoint);
//         simulation.predictedTarget = predictedCoord;
//       }
//     }

//     Coord diffCoord = simulation.dropPoint - simulation.pos;
//     float desiredDir = atan2(diffCoord.y, diffCoord.x);
//     float angleDiff = angleDifference(simulation.direction, desiredDir);
//     bool needBigTurn = fabs(angleDiff) > startingValues.turnThreshold;

//     switch (simulation.state) {
//       case STOPPED:
//         if (needBigTurn) {
//           simulation.state = TURNING;
//           remainingTurnTime = fabs(angleDiff) / startingValues.angularSpeed;
//         }
//         else {
//           simulation.state = ACCELERATING;
//         }
//         break;

//       case ACCELERATING:
//         if (needBigTurn) {
//           simulation.state = DECELERATING;
//         }
//         else if (currentSpeed >= startingValues.attackSpeed) {
//           simulation.state = MOVING;
//           currentSpeed = startingValues.attackSpeed;  // Ensure we don't exceed max speed
//         }
//         break;

//       case MOVING:
//         if (needBigTurn) {
//           simulation.state = DECELERATING;
//         }
//         break;

//       case DECELERATING:
//         if (currentSpeed <= 0.0f) {
//           currentSpeed = 0.0f;
//           simulation.state = STOPPED;  // next step will decide TURNING or ACCELERATING
//         }
//         break;

//       case TURNING:
//         if (fabs(angleDiff) < 1e-3f) {
//           // Angle reached — exit TURNING gracefully (no snap needed,
//           // currentDir is already at desiredDir within tolerance)
//           simulation.state = ACCELERATING;
//           remainingTurnTime = 0.0f;
//         }
//         else if (remainingTurnTime <= 0.0f) {
//           // Timer ran out but desired direction has changed
//           // (e.g. selected target switched mid-turn).
//           // Recompute remaining turn time so the gradual turn can finish.
//           // DO NOT snap currentDir — that creates a discontinuity that violates
//           // the "no large turns while moving" rule.
//           remainingTurnTime = fabs(angleDiff) / startingValues.angularSpeed;
//         }
//         break;
//     }

//     t += config.simTimeStep;
//   }

//   for (int i = 0; i < mission.config.targetCount; i++) {
//     Coord targetCoord, nextCoord;
//     interpolateTarget(i, t, startingValues.arrayTimeStep, NUMBER_OF_TARGETS, MAX_TIME_STEPS, targets, targetCoord);

//     //   interpolateTarget(
//     //     i, t + startingValues.simTimeStep, startingValues.arrayTimeStep, NUMBER_OF_TARGETS, MAX_TIME_STEPS, targets, nextCoord);

//     Coord dCoord = nextCoord - targetCoord;
//     float targetVx = dCoord.x / startingValues.simTimeStep;
//     float targetVy = dCoord.y / startingValues.simTimeStep;
//     Coord fireCoord;

//     calculateDropOffPoint(simulation.pos,
//                           startingValues.altitude,
//                           targetCoord,
//                           startingValues.attackSpeed,
//                           startingValues.accelPath,
//                           AMMO_COUNT,
//                           ammoParams,
//                           startingValues.ammoName,
//                           fireCoord);
//     float desiredDir = atan2(fireCoord.y - simulation.pos.y, fireCoord.x - simulation.pos.x);
//     float angleDiff = angleDifference(simulation.direction, desiredDir);

//     float startSpeed;

//     if (targetChanged) {
//       switch (simulation.state) {
//         case STOPPED:
//           startSpeed = 0.0f;
//           break;
//         case ACCELERATING:
//           startSpeed = currentSpeed;
//           break;  // continue accelerating from current speed
//         case MOVING:
//           startSpeed = startingValues.attackSpeed;
//           break;  // already at max speed
//         case DECELERATING:
//           startSpeed = currentSpeed;
//           break;  // continue decelerating from current speed
//         case TURNING:
//           startSpeed = currentSpeed;
//           break;  // maintain speed during turn
//       }
//     }
//     else {
//       startSpeed = currentSpeed;
//     }
//     float turningTime = 0.0f;
//     if (fabs(angleDiff) > startingValues.turnThreshold) {
//       turningTime = fabs(angleDiff) / startingValues.angularSpeed;
//       penaltyTime += turningTime;
//       startSpeed = 0.0f;  // Assume we stop to turn
//     }

//     float timeToStop = calculateTimeToStop(
//       currentSpeed, startingValues.attackSpeed, startingValues.accelPath, targetChanged, simulation.state, remainingTurnTime, a);

//     penaltyTime += timeToStop;

//     // Time to acceleration
//     float timeToAccel = 0.0f;
//     float distanceDuringAccel = 0.0f;
//     if (startSpeed < startingValues.attackSpeed) {
//       timeToAccel = (startingValues.attackSpeed - startSpeed) / a;
//       distanceDuringAccel = startSpeed * timeToAccel + 0.5f * a * timeToAccel * timeToAccel;
//     }
//     penaltyTime += timeToAccel;

//     float droneTravel = distanceCalculation(simulation.pos, fireCoord);
//     float cruiseDistance = droneTravel - distanceDuringAccel;
//     totalTimeToTarget = penaltyTime + cruiseDistance / startingValues.attackSpeed;
//     Coord predictedCoord;
//     for (int iter = 0; iter < 2; iter++) {
//       predictedCoord.x = targetCoord.x + targetVx * (totalTimeToTarget + t_ammo);
//       predictedCoord.y = targetCoord.y + targetVy * (totalTimeToTarget + t_ammo);
//       calculateDropOffPoint(simulation.pos,
//                             startingValues.altitude,
//                             predictedCoord,
//                             startingValues.attackSpeed,
//                             startingValues.accelPath,
//                             AMMO_COUNT,
//                             ammoParams,
//                             startingValues.ammoName,
//                             fireCoord);
//       droneTravel = distanceCalculation(simulation.pos, fireCoord);
//       cruiseDistance = droneTravel - distanceDuringAccel;
//       totalTimeToTarget = penaltyTime + cruiseDistance / startingValues.attackSpeed;
//     }

//     if (totalTimeToTarget < 0.0f) {
//       throw std::runtime_error("Calculated negative time to target, which is invalid.");
//     }
//     if (totalTimeToTarget < timeToCurrentTarget) {
//       timeToCurrentTarget = totalTimeToTarget;
//       simulation.targetIdx = i;
//       simulation.dropPoint.x = fireCoord.x;
//       simulation.dropPoint.y = fireCoord.y;

//       interpolateTarget(
//         simulation.targetIdx, t + t_ammo, startingValues.arrayTimeStep, NUMBER_OF_TARGETS, MAX_TIME_STEPS, targets, simulation.aimPoint);
//       simulation.predictedTarget = predictedCoord;
//     }
//   }

//   Coord diffCoord = simulation.dropPoint - simulation.pos;
//   float desiredDir = atan2(diffCoord.y, diffCoord.x);
//   float angleDiff = angleDifference(simulation.direction, desiredDir);
//   bool needBigTurn = fabs(angleDiff) > startingValues.turnThreshold;

//   switch (simulation.state) {
//     case STOPPED:
//       if (needBigTurn) {
//         simulation.state = TURNING;
//         remainingTurnTime = fabs(angleDiff) / startingValues.angularSpeed;
//       }
//       else {
//         simulation.state = ACCELERATING;
//       }
//       break;

//     case ACCELERATING:
//       if (needBigTurn) {
//         simulation.state = DECELERATING;
//       }
//       else if (currentSpeed >= startingValues.attackSpeed) {
//         simulation.state = MOVING;
//         currentSpeed = startingValues.attackSpeed;  // Ensure we don't exceed max speed
//       }
//       break;

//     case MOVING:
//       if (needBigTurn) {
//         simulation.state = DECELERATING;
//       }
//       break;

//     case DECELERATING:
//       if (currentSpeed <= 0.0f) {
//         currentSpeed = 0.0f;
//         simulation.state = STOPPED;  // next step will decide TURNING or ACCELERATING
//       }
//       break;

//     case TURNING:
//       if (fabs(angleDiff) < 1e-3f) {
//         // Angle reached — exit TURNING gracefully (no snap needed,
//         // currentDir is already at desiredDir within tolerance)
//         simulation.state = ACCELERATING;
//         remainingTurnTime = 0.0f;
//       }
//       else if (remainingTurnTime <= 0.0f) {
//         // Timer ran out but desired direction has changed
//         // (e.g. selected target switched mid-turn).
//         // Recompute remaining turn time so the gradual turn can finish.
//         // DO NOT snap currentDir — that creates a discontinuity that violates
//         // the "no large turns while moving" rule.
//         remainingTurnTime = fabs(angleDiff) / startingValues.angularSpeed;
//       }
//       break;
//   }

//   switch (simulation.state) {
//     case STOPPED:
//       // no motion
//       break;

//     case ACCELERATING:
//       currentSpeed += a * startingValues.simTimeStep;
//       if (currentSpeed > startingValues.attackSpeed)
//         currentSpeed = startingValues.attackSpeed;
//       // For small turns, adjust heading smoothly while moving
//       if (!needBigTurn && fabs(angleDiff) > 1e-4f) {
//         float turnStep = startingValues.angularSpeed * startingValues.simTimeStep;
//         if (turnStep > fabs(angleDiff))
//           turnStep = fabs(angleDiff);
//         simulation.direction += (angleDiff > 0 ? 1.0f : -1.0f) * turnStep;
//       }
//       // Update position
//       simulation.pos.x += currentSpeed * cos(simulation.direction) * startingValues.simTimeStep;
//       simulation.pos.y += currentSpeed * sin(simulation.direction) * startingValues.simTimeStep;
//       break;

//     case MOVING:
//       if (!needBigTurn && fabs(angleDiff) > 1e-4f) {
//         float turnStep = startingValues.angularSpeed * startingValues.simTimeStep;
//         if (turnStep > fabs(angleDiff))
//           turnStep = fabs(angleDiff);
//         simulation.direction += (angleDiff > 0 ? 1.0f : -1.0f) * turnStep;
//       }
//       simulation.pos.x += currentSpeed * cos(simulation.direction) * startingValues.simTimeStep;
//       simulation.pos.y += currentSpeed * sin(simulation.direction) * startingValues.simTimeStep;
//       break;

//     case DECELERATING:
//       currentSpeed -= a * startingValues.simTimeStep;
//       if (currentSpeed < 0.0f)
//         currentSpeed = 0.0f;
//       simulation.pos.x += currentSpeed * cos(simulation.direction) * startingValues.simTimeStep;
//       simulation.pos.y += currentSpeed * sin(simulation.direction) * startingValues.simTimeStep;
//       break;

//     case TURNING: {
//       // Turn in place — no position change
//       float turnStep = startingValues.angularSpeed * startingValues.simTimeStep;
//       if (turnStep > fabs(angleDiff))
//         turnStep = fabs(angleDiff);
//       simulation.direction += (angleDiff > 0 ? 1.0f : -1.0f) * turnStep;
//       remainingTurnTime -= startingValues.simTimeStep;
//       if (remainingTurnTime < 0.0f)
//         remainingTurnTime = 0.0f;
//     } break;
//   }
//   PREVIOUS_TARGET = simulation.targetIdx;

//   DEBUG("Step " << N << ": Drone at (" << simulation.pos.x << ", " << simulation.pos.y << ")  dir=" << simulation.direction
//                 << "  speed=" << currentSpeed << "  phase=" << simulation.state << "  target=" << simulation.targetIdx);

//   droneCoordinates[2 * N] = simulation.pos.x;  // note: index 2*N, not N
//   droneCoordinates[2 * N + 1] = simulation.pos.y;
//   directions[N] = simulation.direction;
//   phases[N] = simulation.state;
//   indexes[N] = simulation.targetIdx;
//   N++;
//   steps[N - 1] = simulation;

//   if (simulation.state == MOVING) {
//     Coord bombLandPoint;

//     bombLandPoint.x = simulation.pos.x + cos(simulation.direction) * h_ammo;
//     bombLandPoint.y = simulation.pos.y + sin(simulation.direction) * h_ammo;
//     // Where will current target be at impact time?
//     Coord impactCoord;
//     interpolateTarget(
//       simulation.targetIdx, t + t_ammo, startingValues.arrayTimeStep, NUMBER_OF_TARGETS, MAX_TIME_STEPS, targets, impactCoord);
//     float missDistance = distanceCalculation(bombLandPoint, impactCoord);

//     if (missDistance <= startingValues.hitRadius) {
//       TARGET_HIT = true;
//       DEBUG("Target " << simulation.targetIdx << " HIT at sim time t=" << t << "s (step " << N << ")");
//       DEBUG("  Drone:  (" << simulation.pos.x << ", " << simulation.pos.y << ")  dir=" << simulation.direction << "  speed=" <<
//       currentSpeed
//                           << std::endl);
//       DEBUG("  Bomb lands at:   (" << bombLandPoint.x << ", " << bombLandPoint.y << ")" << std::endl);
//       DEBUG("  Target at impact:(" << impactCoord.x << ", " << impactCoord.y << ")" << std::endl);
//       DEBUG("  Miss distance:   " << missDistance << " m  (hitRadius=" << startingValues.hitRadius << ")" << std::endl);
//       break;
//     }
//   }
//   // Store the current step's state
//   t += startingValues.simTimeStep;
// }
// LOG("Simulation ended after " << N << " steps.");

// json out_json;
// out_json["totalSteps"] = N;
// out_json["steps"] = json::array();
// for (int i = 0; i < N; i++) {
//   json step;
//   step["position"] = {{"x", steps[i].pos.x}, {"y", steps[i].pos.y}};
//   step["direction"] = steps[i].direction;
//   step["state"] = (int)steps[i].state;
//   step["targetIndex"] = steps[i].targetIdx;
//   step["dropPoint"] = {{"x", steps[i].dropPoint.x}, {"y", steps[i].dropPoint.y}};
//   step["aimPoint"] = {{"x", steps[i].aimPoint.x}, {"y", steps[i].aimPoint.y}};
//   step["predictedTarget"] = {{"x", steps[i].predictedTarget.x}, {"y", steps[i].predictedTarget.y}};
//   out_json["steps"].push_back(step);
// }
// std::ofstream o("simulation.json");
// o << out_json.dump(2);
// // 1. Delete each row first
// for (int i = 0; i < NUMBER_OF_TARGETS; i++) {
//   delete[] targets[i];
// }

// // 2. Delete the array of pointers
// delete[] targets;

// delete[] ammoParams;
// delete[] steps;

// }