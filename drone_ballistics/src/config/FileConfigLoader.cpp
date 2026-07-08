#include "Types.h"
#include "config/FileConfigLoader.h"
#include <cstddef>
#include <unordered_map>
#include <fstream>

void FileConfigLoader::load()
{
  std::ifstream ammoFile("drone_ballistics/data/ammo.json");
  if (!ammoFile.is_open()) {
    throw std::runtime_error("Cannot open ammo.json");
  }
  json ammo = json::parse(ammoFile);
  std::unordered_map<std::string, AmmoParams> ammoByName;
  // Put the read ammos in dynamic array
  for (size_t i = 0; i < ammo.size(); ++i) {
    AmmoParams params;
    params.name = ammo[i]["name"].get<std::string>();
    params.mass = ammo[i]["mass"].get<double>();
    params.drag = ammo[i]["drag"].get<double>();
    params.lift = ammo[i]["lift"].get<double>();
    ammoByName[params.name] = params;
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
  this->config.ammoName = data["ammo"].get<std::string>();
  this->config.arrayTimeStep = data["targetArrayTimeStep"];
  this->config.simTimeStep = data["simulation"]["timeStep"];
  this->config.hitRadius = data["simulation"]["hitRadius"];
  this->config.angularSpeed = data["drone"]["angularSpeed"];
  this->config.turnThreshold = data["drone"]["turnThreshold"];
  this->config.physicsTimeStep = data["simulation"]["physicsTimeStep"];
  this->config.timeScale = data["simulation"]["timeScale"];

  auto it = ammoByName.find(this->config.ammoName);
  if (it == ammoByName.end()) {
    throw std::runtime_error("Unknown ammo type in config.json");
  }
  this->selectedAmmo = it->second;
}

DroneConfig FileConfigLoader::getConfig()
{
  // Return loaded DroneConfig
  return this->config;
}

AmmoParams FileConfigLoader::getAmmoParams()
{
  return this->selectedAmmo;
}

int FileConfigLoader::getAmmoCount() const
{
  return static_cast<int>(ammoParams_.size());
}
