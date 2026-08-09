#pragma once
#include <nlohmann/json.hpp>
#include <optional>
#include <string>

namespace poster {
using json = nlohmann::json;
json makeEnvelope(const std::string& studentId, const std::string& testId, const json& simulation);

class IResultSource {
public:
  virtual ~IResultSource() = default;
  // std::nullopt if the file doesn't exist — a missing test is skipped, not an error
  virtual std::optional<json> load(const std::string& testId) const = 0;
};

class FileResultSource : public IResultSource {
public:
  explicit FileResultSource(std::string rootDir);
  std::optional<json> load(const std::string& testId) const override;

private:
  std::string rootDir_;
};

}  // namespace poster