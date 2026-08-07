#include <gtest/gtest.h>
#include "posting_policy.hpp"

using namespace poster;

TEST(ClassifyTest, SavedFirstTime)
{
    EXPECT_EQ(classify(201), Decision::Success);
}
TEST(ClassifyTest, OverwriteFirstTime)
{
    EXPECT_EQ(classify(200), Decision::Success);
}
TEST(ClassifyTest, BadData)
{
    EXPECT_EQ(classify(400), Decision::Fatal);
}
TEST(ClassifyTest, BadApiKey)
{
    EXPECT_EQ(classify(401), Decision::Fatal);
}

TEST(ClassifyTest, Timeout)
{
    EXPECT_EQ(classify(std::nullopt), Decision::Retry);
}
TEST(ClassifyTest, NotAvailable)
{
    EXPECT_EQ(classify(503), Decision::Retry);
}

TEST(DecideTest, RetryableOnFourthAttempt)
{
    EXPECT_EQ(decide(503, 4), Decision::Retry);
}
TEST(DecideTest, GivesUpOnUnavailableOnFifthAttempt)
{
    EXPECT_EQ(decide(503, 5), Decision::GaveUp);
}
TEST(DecideTest, FatalIgnoresAttemptCap)
{
    EXPECT_EQ(decide(400, 5), Decision::Fatal);
}
TEST(DecideTest, SuccessIgnoresAttemptCap)
{
    EXPECT_EQ(decide(201, 5), Decision::Success);
}
TEST(DecideTest, GivesUpOnTimeoutAtCap) { EXPECT_EQ(decide(std::nullopt, 5), Decision::GaveUp); }
