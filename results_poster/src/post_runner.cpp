#include "post_runner.hpp"
#include <chrono>
#include <thread>
#include <ostream>

namespace poster {

PostRunner::PostRunner(IResultSource& source, IResultSink& sink, std::string studentId, ISleeper& sleeper)
  : source_(source)
  , sink_(sink)
  , studentId_(std::move(studentId))
  , sleeper_(sleeper)
{
}
void RealSleeper::sleep(std::chrono::milliseconds d)
{
  std::this_thread::sleep_for(d);
}

TestReport PostRunner::runOne(const std::string& testId)
{
  std::optional<nlohmann::json> simulation = source_.load(testId);
  if (!simulation) {
    return TestReport{testId, Outcome::Skipped, 0};
  }
  nlohmann::json envelope = makeEnvelope(studentId_, testId, *simulation);
  for (int attempt = 1; attempt <= kMaxAttempts; ++attempt) {
    std::optional<int> status = sink_.post(envelope);
    Decision decision = decide(status, attempt);
    switch (decision) {
      case Decision::Success: {
        bool verified = sink_.verify(testId, studentId_);
        return TestReport{testId, verified ? Outcome::Verified : Outcome::Posted, attempt};
      }
      case Decision::Fatal:
        return TestReport{testId, Outcome::Rejected, attempt};
      case Decision::GaveUp:
        return TestReport{testId, Outcome::GaveUp, attempt};
      case Decision::Retry:
        sleeper_.sleep(std::chrono::seconds(1));
        break;
    }
  }
  return TestReport{testId, Outcome::GaveUp, kMaxAttempts};
}

std::vector<TestReport> PostRunner::runAll(const std::vector<std::string>& testIds)
{
  std::vector<TestReport> reports;
  for (const std::string& testId : testIds) {
    TestReport report = runOne(testId);
    reports.push_back(report);
  }
  return reports;
}

std::string toString(Outcome o)
{
  switch (o) {
    case Outcome::Posted:
      return "POSTED";
    case Outcome::Verified:
      return "VERIFIED";
    case Outcome::Rejected:
      return "REJECTED";
    case Outcome::GaveUp:
      return "GAVEUP";
    case Outcome::Skipped:
      return "SKIPPED";
  }
  return "Unknown";
}
void printReport(const std::vector<TestReport>& reports, std::ostream& out)
{
  for (const TestReport& report : reports) {
    std::string outcomeStr = toString(report.outcome);
    out << report.testId << " " << outcomeStr << " after " << report.attempts << " attempts\n";
  }
}
}  // namespace poster