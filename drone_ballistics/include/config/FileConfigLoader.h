#include <vector>
#include "Types.h"
#include "interfaces/IConfigLoader.h"

class FileConfigLoader : public IConfigLoader {
  DroneConfig config;
  std::vector<AmmoParams> ammoParams_;
  AmmoParams selectedAmmo;

public:
  FileConfigLoader() = default;
  FileConfigLoader(const FileConfigLoader &) = delete;
  FileConfigLoader &operator=(const FileConfigLoader &) = delete;
  void load() override;
  DroneConfig getConfig() override;
  AmmoParams getAmmoParams() override;
  int getAmmoCount() const;
};