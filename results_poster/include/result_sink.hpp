#pragma once
#include <nlohmann/json.hpp>
#include <optional>
#include <string>

namespace poster {

class IResultSink {
public:
  virtual ~IResultSink() = default;
  virtual std::optional<int> post(const nlohmann::json& envelope) = 0;
  virtual bool verify(const std::string& testId, const std::string& studentId) = 0;
};

class HttpResultSink : public IResultSink {
public:
  HttpResultSink(std::string host, std::string apiKey);
  std::optional<int> post(const nlohmann::json& envelope) override;
  bool verify(const std::string& testId, const std::string& studentId) override;

private:
  std::string host_;
  std::string apiKey_;
};

}  // namespace poster