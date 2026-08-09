#include "result_sink.hpp"
#include <httplib/httplib.h>

namespace poster {

HttpResultSink::HttpResultSink(std::string host, std::string apiKey)
  : host_(std::move(host))
  , apiKey_(std::move(apiKey))
{
}

std::optional<int> HttpResultSink::post(const nlohmann::json& envelope)
{
  httplib::Client client(host_);
  client.set_connection_timeout(2, 0);
  client.set_read_timeout(2, 0);
  httplib::Headers headers = {{"x-api-key", apiKey_}};
  auto res = client.Post("/api/dz12/results", headers, envelope.dump(), "application/json");
  if (!res) {
    return std::nullopt;
  }
  return res->status;
}

bool HttpResultSink::verify(const std::string& testId, const std::string& studentId)
{
  httplib::Client client(host_);
  client.set_connection_timeout(2, 0);
  client.set_read_timeout(2, 0);
  httplib::Headers headers = {{"x-api-key", apiKey_}};

  std::string path = "/api/dz12/results/" + testId + "/" + studentId;
  httplib::Result res = client.Get(path.c_str(), headers);
  return res && res->status == 200;
}

}  // namespace poster