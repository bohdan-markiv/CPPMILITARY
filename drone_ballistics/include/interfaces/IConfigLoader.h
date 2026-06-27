#pragma once
#include "Types.h"

class IConfigLoader {
public:
  virtual ~IConfigLoader(){};
  virtual void load() = 0;
  virtual DroneConfig getConfig() = 0;
  virtual AmmoParams getAmmoParams() = 0;
};