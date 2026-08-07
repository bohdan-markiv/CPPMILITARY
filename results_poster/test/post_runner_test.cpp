#include <gtest/gtest.h>
#include "post_runner.hpp"

using namespace poster;

class FakeSource : public IResultSource {
public:
  std::optional<nlohmann::json> result;
  std::optional<nlohmann::json> load(const std::string&) const override { return result; }
};

class FakeSleeper : public ISleeper {
public:
  std::vector<std::chrono::milliseconds> calls;
  void sleep(std::chrono::milliseconds d) override { calls.push_back(d); }
};
class FakeSink : public IResultSink {
public:
  std::vector<std::optional<int>> responses;  // scripted, one per call
  int calls = 0;
  bool verifyResult = true;

  std::optional<int> post(const nlohmann::json&) override
  {
    return responses.at(calls++);  // .at() → loud failure if a test under-scripts
  }
  bool verify(const std::string&, const std::string&) override { return verifyResult; }
};

TEST(PostRunnerTest, PostsSuccessfullyOnFirstAttempt)
{
  FakeSource source;
  source.result = nlohmann::json{{"totalSteps", 0}};
  FakeSink sink;
  sink.responses = {201};
  FakeSleeper sleeper;

  PostRunner runner(source, sink, "6666", sleeper);
  TestReport report = runner.runOne("T01");

  EXPECT_EQ(report.outcome, Outcome::Verified);
  EXPECT_EQ(report.attempts, 1);
  EXPECT_EQ(sink.calls, 1);
  EXPECT_TRUE(sleeper.calls.empty());
}
TEST(PostRunnerTest, RetryThanSucceeds)
{
  FakeSource source;
  source.result = nlohmann::json{{"totalSteps", 0}};
  FakeSink sink;
  sink.responses = {503, 503, 201};
  FakeSleeper sleeper;

  PostRunner runner(source, sink, "6666", sleeper);
  TestReport report = runner.runOne("T01");

  EXPECT_EQ(report.outcome, Outcome::Verified);
  EXPECT_EQ(report.attempts, 3);
  EXPECT_EQ(sink.calls, 3);
  EXPECT_EQ(sleeper.calls.size(), 2);
}

TEST(PostRunnerTest, FatalNoRetry)
{
  FakeSource source;
  source.result = nlohmann::json{{"totalSteps", 0}};
  FakeSink sink;
  sink.responses = {400};
  FakeSleeper sleeper;

  PostRunner runner(source, sink, "6666", sleeper);
  TestReport report = runner.runOne("T01");

  EXPECT_EQ(report.outcome, Outcome::Rejected);
  EXPECT_EQ(report.attempts, 1);
  EXPECT_EQ(sink.calls, 1);
  EXPECT_TRUE(sleeper.calls.empty());
}

TEST(PostRunnerTest, ExhaustsRetries)
{
  FakeSource source;
  source.result = nlohmann::json{{"totalSteps", 0}};
  FakeSink sink;
  sink.responses = {503, 503, 503, 503, 503};
  FakeSleeper sleeper;

  PostRunner runner(source, sink, "6666", sleeper);
  TestReport report = runner.runOne("T01");

  EXPECT_EQ(report.outcome, Outcome::GaveUp);
  EXPECT_EQ(report.attempts, 5);
  EXPECT_EQ(sink.calls, 5);
  EXPECT_EQ(sleeper.calls.size(), 4);
}
TEST(PostRunnerTest, MissingFile)
{
  FakeSource source;
  //   source.result = nlohmann::json{{"totalSteps", 0}};
  FakeSink sink;
  sink.responses = {};
  FakeSleeper sleeper;

  PostRunner runner(source, sink, "6666", sleeper);
  TestReport report = runner.runOne("T01");

  EXPECT_EQ(report.outcome, Outcome::Skipped);
  EXPECT_EQ(report.attempts, 0);
  EXPECT_EQ(sink.calls, 0);
  EXPECT_TRUE(sleeper.calls.empty());
}
TEST(PostRunnerTest, PostedButNotVerified)
{
  FakeSource source;
  source.result = nlohmann::json{{"totalSteps", 0}};
  FakeSink sink;
  sink.responses = {201};
  sink.verifyResult = false;
  FakeSleeper sleeper;

  PostRunner runner(source, sink, "6666", sleeper);
  TestReport report = runner.runOne("T01");

  EXPECT_EQ(report.outcome, Outcome::Posted);
  EXPECT_EQ(report.attempts, 1);
  EXPECT_EQ(sink.calls, 1);
}