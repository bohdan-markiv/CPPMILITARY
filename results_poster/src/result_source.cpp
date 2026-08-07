#include "result_source.hpp"
#include <filesystem>
#include <fstream>

namespace poster {
using json = nlohmann::json;
FileResultSource::FileResultSource(std::string rootDir)
  : rootDir_(std::move(rootDir))
{
}

std::optional<json> FileResultSource::load(const std::string& testId) const
{
  const std::filesystem::path p = std::filesystem::path(rootDir_) / testId / "simulation.json";
  if (!std::filesystem::exists(p)) {
    return std::nullopt;
  }

  std::ifstream file(p);
  json parsed;
  file >> parsed;
  return parsed;
}

json makeEnvelope(const std::string& studentId, const std::string& testId, const json& simulation)
{
  return json{{"studentId", studentId}, {"testId", testId}, {"simulation", simulation}};
}

}  // namespace poster