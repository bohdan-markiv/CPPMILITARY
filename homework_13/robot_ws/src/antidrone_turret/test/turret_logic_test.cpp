#include <gtest/gtest.h>

#include "antidrone_turret/turret_logic.hpp"

namespace {

using namespace antidrone_turret::logic;

// --- 1. Target evaluation (WORKED EXAMPLE — study this one) ---

TEST(TurretLogicTest, LowConfidenceTargetIsNotLocked)
{
  // confidence 0.50 is below the 0.80 threshold -> low confidence
  const auto state = evaluateTarget(true, 0.50F, 0.80F);
  EXPECT_EQ(state, TargetState::kLowConfidence);
}

TEST(TurretLogicTest, ConfidenceAtThresholdIsLocked)
{
  // exactly 0.80 must be on the LOCKED side (>= threshold)
  const auto state = evaluateTarget(true, 0.80F, 0.80F);
  EXPECT_EQ(state, TargetState::kLocked);
}

TEST(TurretLogicTest, InvisibleTargetIsNone)
{
  const auto state = evaluateTarget(false, 0.99F, 0.80F);
  EXPECT_EQ(state, TargetState::kNone);
}

// --- 2. Servo command: x > 320 -> RIGHT (kPositive), error_x > 0 ---

TEST(TurretLogicTest, ServoTargetRightOfCenterTurnsRight)
{
  const auto cmd = servoCommand(420.0F);

  EXPECT_EQ(cmd.direction, Direction::kPositive);
  EXPECT_FLOAT_EQ(cmd.error_x, 100.0F);
  EXPECT_FLOAT_EQ(cmd.target_x, 420.0F);
}

// --- 3. Gimbal command: y < 240 -> UP (kPositive), error_y > 0 ---

TEST(TurretLogicTest, GimbalTargetAboveCenterAimsUp)
{
  const auto cmd = gimbalCommand(180.0F);

  EXPECT_EQ(cmd.direction, Direction::kPositive);
  EXPECT_FLOAT_EQ(cmd.error_y, 60.0F);
  EXPECT_FLOAT_EQ(cmd.target_y, 180.0F);
}

// --- 4. Trigger decision: close + READY -> REQUESTED ---

TEST(TurretLogicTest, CloseTargetWithReadyActuatorRequestsFire)
{
  const auto trig = decideTrigger(25.0F, 30.0F, ActuatorState::kReady);
  EXPECT_EQ(trig, TriggerState::kRequested);
}

// --- 4b. Trigger decision: close + RELOADING -> RELOADING ---

TEST(TurretLogicTest, CloseTargetWithReloadingActuatorHoldsFire)
{

  const auto trig = decideTrigger(25.0F, 30.0F, ActuatorState::kReloading);
  EXPECT_EQ(trig, TriggerState::kReloading);
}

// --- 4c. Trigger decision: far target -> SKIP (even if ready) ---

TEST(TurretLogicTest, FarTargetSkipsEvenWhenReady)
{
  const auto trig = decideTrigger(45.0F, 30.0F, ActuatorState::kReady);
  EXPECT_EQ(trig, TriggerState::kSkip);
}

// --- 5. Status assembly: far but valid target -> LOCKED, TRACK, SKIP ---

TEST(TurretLogicTest, FarValidTargetLocksTracksButSkips)
{
  // A far-but-visible-and-confident target is LOCKED and TRACKED,
  // but the trigger was SKIP because it's out of range.
  const auto status = buildTurretStatus(TargetState::kLocked, TriggerState::kSkip, 0.90F, 45.0F);

  EXPECT_EQ(status.target_state, TargetState::kLocked);
  EXPECT_EQ(status.action, ActionState::kTrack);
  EXPECT_EQ(status.trigger_state, TriggerState::kSkip);
}

}  // namespace