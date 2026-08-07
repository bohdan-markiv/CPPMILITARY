#pragma once
#include "posting_policy.hpp"
#include "result_sink.hpp"
#include "result_source.hpp"
#include <string>
#include <vector>
#include <chrono>

namespace poster {

enum class Outcome { Posted, Verified, Rejected, GaveUp, Skipped };
class ISleeper {
public:
  virtual ~ISleeper() = default;
  virtual void sleep(std::chrono::milliseconds d) = 0;
};

class RealSleeper : public ISleeper {
public:
  void sleep(std::chrono::milliseconds d) override;
};

struct TestReport {
  std::string testId;
  Outcome outcome;
  int attempts;
};
std::string toString(Outcome o);
void printReport(const std::vector<TestReport>& reports, std::ostream& out);
class PostRunner {
public:
  PostRunner(IResultSource& source, IResultSink& sink, std::string studentId, ISleeper& sleeper);

  TestReport runOne(const std::string& testId);

  std::vector<TestReport> runAll(const std::vector<std::string>& testIds);

private:
  IResultSource& source_;
  IResultSink& sink_;
  std::string studentId_;
  ISleeper& sleeper_;
};

}  // namespace poster