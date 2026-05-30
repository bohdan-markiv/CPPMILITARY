#include "Types.h"
#include "interfaces/IConfigLoader.h"

class FileConfigLoader : public IConfigLoader {
  DroneConfig config;
  AmmoParams *ammoParams = nullptr;
  int AMMO_COUNT = 0;
  AmmoParams selectedAmmo;

public:
  FileConfigLoader() = default;
  FileConfigLoader(const FileConfigLoader &) = delete;
  FileConfigLoader &operator=(const FileConfigLoader &) = delete;
  ~FileConfigLoader() override;
  void load() override;
  DroneConfig getConfig() override;
  AmmoParams getAmmoParams() override;
  int getAmmoCount() const;
};