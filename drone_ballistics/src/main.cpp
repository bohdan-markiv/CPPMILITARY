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
std::vector<SimStep> simLog;
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
  virtual Coord solve(Coord currentCoord, float zd, Coord targetCoord, float attackSpeed, double m, double d, double l) = 0;
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
  ~JsonTargetProvider() override
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
  Coord solve(Coord currentCoord, float zd, Coord targetCoord, float attackSpeed, double m, double d, double l) override
  {
    Coord fireCoord;
    double fullDistance = distanceCalculation(currentCoord, targetCoord);

    double t = timeToTarget(m, d, l, attackSpeed, zd);
    if (t < 0.0) {
      LOG("Error calculating time to target.");
      throw std::runtime_error("Error calculating time to target.");
    }
    double h = ammoFlyDistance(t, d, l, attackSpeed, m);
    if (h < 0.0) {
      LOG("Error calculating ammo fly distance.");
      throw std::runtime_error("Error calculating ammo fly distance.");
    }

    // if (h + accelerationPath > fullDistance)
    // {
    //     manouvreDistance(fullDistance, xd, yd, targetX, targetY, h, accelerationPath);
    // }

    // ratio = (D − h) / D, clamped to [0,1]: if target is within bomb range, drop from current position
    double ratio = (fullDistance > 0.0) ? std::max(0.0, (fullDistance - h) / fullDistance) : 0.0;
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

  SimStep simulation;

  double t = 0.0;

  // OUTPUT  arrays
  int N = 0;
  bool TARGET_HIT = false;

  double currentSpeed = 0.0;
  double remainingTurnTime = 0.0;

  double t_ammo = 0.0;
  double h_ammo = 0.0;
  double a = 0.0;
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
    simulation.direction = config.initialDir;
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

      Coord targetCoord{0.0f, 0.0f}, nextCoord{0.0f, 0.0f};
      interpolateTarget(t, config.arrayTimeStep, targets->getTarget(i), timeSteps, targetCoord);
      interpolateTarget(t + config.simTimeStep, config.arrayTimeStep, targets->getTarget(i), timeSteps, nextCoord);
      Coord dCoord = nextCoord - targetCoord;
      double targetVx = dCoord.x / config.simTimeStep;
      double targetVy = dCoord.y / config.simTimeStep;

      Coord dropPoint{.x = 0.0f, .y = 0.0f};
      dropPoint = solver->solve(simulation.pos, config.altitude, targetCoord, config.attackSpeed, ammo.mass, ammo.drag, ammo.lift);

      double desiredDir = atan2(dropPoint.y - simulation.pos.y, dropPoint.x - simulation.pos.x);
      double angleDiff = angleDifference(simulation.direction, desiredDir);

      double startSpeed = currentSpeed;

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

      double penaltyTime = 0.0;

      if (fabs(angleDiff) > config.turnThreshold) {
        double turningTime = fabs(angleDiff) / config.angularSpeed;
        penaltyTime += turningTime;
        startSpeed = 0.0;  // Assume we stop to turn
      }

      double timeToStop = calculateTimeToStop(currentSpeed, config.attackSpeed, targetChanged, simulation.state, remainingTurnTime, a);
      penaltyTime += timeToStop;

      double timeToAccel = 0.0;
      double distanceDuringAccel = 0.0;
      if (startSpeed < config.attackSpeed) {
        timeToAccel = (config.attackSpeed - startSpeed) / a;
        distanceDuringAccel = startSpeed * timeToAccel + 0.5 * a * timeToAccel * timeToAccel;
      }

      penaltyTime += timeToAccel;

      double droneTravel = distanceCalculation(simulation.pos, dropPoint);
      double cruiseDistance = droneTravel - distanceDuringAccel;
      double totalTimeToTarget = penaltyTime + cruiseDistance / config.attackSpeed;

      Coord predictedCoord{.x = 0.0, .y = 0.0};
      for (int iter = 0; iter < 2; iter++) {
        predictedCoord.x = targetCoord.x + targetVx * (totalTimeToTarget + t_ammo);
        predictedCoord.y = targetCoord.y + targetVy * (totalTimeToTarget + t_ammo);
        dropPoint = solver->solve(simulation.pos, config.altitude, predictedCoord, config.attackSpeed, ammo.mass, ammo.drag, ammo.lift);
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

    Coord diffCoord{.x = simulation.dropPoint.x - simulation.pos.x, .y = simulation.dropPoint.y - simulation.pos.y};
    double desiredDir = atan2(diffCoord.y, diffCoord.x);
    double angleDiff = angleDifference(simulation.direction, desiredDir);
    bool needBigTurn = fabs(angleDiff) > config.turnThreshold;

    switch (simulation.state) {
      case STOPPED:
        if (needBigTurn) {
          simulation.state = TURNING;
          remainingTurnTime = fabs(angleDiff) / config.angularSpeed;
        }
        else {
          simulation.state = ACCELERATING;
        }
        break;

      case ACCELERATING:
        if (needBigTurn) {
          simulation.state = DECELERATING;
        }
        else if (currentSpeed >= config.attackSpeed) {
          simulation.state = MOVING;
          currentSpeed = config.attackSpeed;  // Ensure we don't exceed max speed
        }
        break;

      case MOVING:
        if (needBigTurn) {
          simulation.state = DECELERATING;
        }
        break;

      case DECELERATING:
        if (currentSpeed <= 0.0f) {
          currentSpeed = 0.0f;
          simulation.state = STOPPED;  // next step will decide TURNING or ACCELERATING
        }
        else if (!needBigTurn) {
          // Turn requirement gone, re-accelerate
          simulation.state = ACCELERATING;
        }
        break;

      case TURNING:
        if (fabs(angleDiff) < 1e-2f) {
          simulation.state = ACCELERATING;
          remainingTurnTime = 0.0f;
        }
        else if (remainingTurnTime <= 0.0f) {
          remainingTurnTime = fabs(angleDiff) / config.angularSpeed;
        }
        break;
    }

    switch (simulation.state) {
      case STOPPED:
        // no motion
        break;

      case ACCELERATING:
        currentSpeed += a * config.simTimeStep;
        if (currentSpeed > config.attackSpeed)
          currentSpeed = config.attackSpeed;
        // For small turns, adjust heading smoothly while moving
        if (!needBigTurn && fabs(angleDiff) > 1e-4f) {
          double turnStep = config.angularSpeed * config.simTimeStep;
          if (turnStep > fabs(angleDiff))
            turnStep = fabs(angleDiff);
          simulation.direction += (angleDiff > 0 ? 1.0 : -1.0) * turnStep;
        }
        // Update position
        simulation.pos.x += currentSpeed * cos(simulation.direction) * config.simTimeStep;
        simulation.pos.y += currentSpeed * sin(simulation.direction) * config.simTimeStep;
        break;

      case MOVING:
        if (!needBigTurn && fabs(angleDiff) > 1e-4f) {
          double turnStep = config.angularSpeed * config.simTimeStep;
          if (turnStep > fabs(angleDiff))
            turnStep = fabs(angleDiff);
          simulation.direction += (angleDiff > 0 ? 1.0 : -1.0) * turnStep;
        }
        simulation.pos.x += currentSpeed * cos(simulation.direction) * config.simTimeStep;
        simulation.pos.y += currentSpeed * sin(simulation.direction) * config.simTimeStep;
        break;

      case DECELERATING:
        currentSpeed -= a * config.simTimeStep;
        if (currentSpeed < 0.0)
          currentSpeed = 0.0;
        simulation.pos.x += currentSpeed * cos(simulation.direction) * config.simTimeStep;
        simulation.pos.y += currentSpeed * sin(simulation.direction) * config.simTimeStep;
        break;

      case TURNING: {
        // Turn in place — no position change
        double turnStep = config.angularSpeed * config.simTimeStep;
        if (turnStep > fabs(angleDiff))
          turnStep = fabs(angleDiff);
        simulation.direction += (angleDiff > 0 ? 1.0 : -1.0) * turnStep;
        remainingTurnTime -= config.simTimeStep;
        if (remainingTurnTime < 0.0)
          remainingTurnTime = 0.0;
      } break;
    }

    if (simulation.state == MOVING) {
      Coord bombLand{.x = 0.0, .y = 0.0};
      bombLand.x = simulation.pos.x + h_ammo * cos(simulation.direction);
      bombLand.y = simulation.pos.y + h_ammo * sin(simulation.direction);

      Coord hitCoord{.x = 0.0, .y = 0.0};
      interpolateTarget(t + t_ammo, config.arrayTimeStep, targets->getTarget(simulation.targetIdx), timeSteps, hitCoord);

      double missDistance = distanceCalculation(bombLand, hitCoord);

      if (missDistance <= config.hitRadius) {
        TARGET_HIT = true;
        LOG("Target " << simulation.targetIdx << " HIT at t=" << t << "s (step " << N << ")");
        LOG("  Drone:  (" << simulation.pos.x << ", " << simulation.pos.y << ")  dir=" << simulation.direction);
        LOG("  Bomb lands at:   (" << bombLand.x << ", " << bombLand.y << ")");
        LOG("  Target at impact:(" << hitCoord.x << ", " << hitCoord.y << ")");
        LOG("  Miss distance:   " << missDistance << " m");
      }
    }

    N++;
    t += config.simTimeStep;
    return simulation;
  }
  [[nodiscard]] int getN() const { return N; }
};

auto main() -> int
{
  IConfigLoader *configLoader = createLoader(LoaderType::FILE);
  ITargetProvider *targetProvider = createProvider(ProviderType::JSON);
  IBallisticSolver *ballisticSolver = createSolver(SolverType::ANALYTICAL);

  Mission mission(ballisticSolver, targetProvider, configLoader);

  mission.init();

  while (mission.hasNext()) {
    SimStep step = mission.step();
    std::cout << "Step " << mission.getN() << " pos=(" << step.pos.x << "," << step.pos.y << ")" << " dir=" << step.direction
              << " state=" << step.state << " target=" << step.targetIdx << "\n";
    simLog.push_back(step);
  }

  json out;
  out["totalSteps"] = simLog.size();
  out["steps"] = json::array();
  for (const SimStep &s : simLog) {
    json stepJson;
    stepJson["position"] = {{"x", s.pos.x}, {"y", s.pos.y}};
    stepJson["direction"] = s.direction;
    stepJson["state"] = (int)s.state;
    stepJson["targetIndex"] = s.targetIdx;
    stepJson["dropPoint"] = {{"x", s.dropPoint.x}, {"y", s.dropPoint.y}};
    stepJson["aimPoint"] = {{"x", s.aimPoint.x}, {"y", s.aimPoint.y}};
    stepJson["predictedTarget"] = {{"x", s.predictedTarget.x}, {"y", s.predictedTarget.y}};
    out["steps"].push_back(stepJson);
  }

  std::ofstream outFile("drone_ballistics/output/simulation.json");
  if (!outFile.is_open()) {
    throw std::runtime_error("Cannot open output/simulation.json — does the folder exist?");
  }
  outFile << out.dump(2);
  outFile.close();

  delete configLoader;
  delete targetProvider;
  delete ballisticSolver;

  return 0;
}
