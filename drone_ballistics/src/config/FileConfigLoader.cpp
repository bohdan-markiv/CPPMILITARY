#include "Types.h"
#include "config/FileConfigLoader.h"

void FileConfigLoader::load()
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
    throw std::runtime_error("Unknown ammo type in config.json");
  }
  else {
    std::cout << "Ammo parameters: mass = " << selectedAmmo.mass << " kg, drag coefficient = " << selectedAmmo.drag
              << ", lift coefficient = " << selectedAmmo.lift << std::endl;
  }
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
  return this->AMMO_COUNT;
}

FileConfigLoader::~FileConfigLoader()
{
  delete[] ammoParams;
}