#include "ballistics.hpp"
#include <gtest/gtest.h>

TEST(Ballistics, ComputesKnownDropPointBasic1)
{
  const BallisticsInput input{.drone = {180.0f, 180.0f},
                              .drone_z = 100.0f,
                              .target = {200.0f, 200.0f},
                              .attack_speed = 10.0f,
                              .acceleration_path = 10.0f,
                              .ammo_name = "VOG-17"};
  const DropSolution solution = compute_drop_solution(input);
  EXPECT_NEAR(solution.fire.x, 173.759f, 0.01f);
  EXPECT_NEAR(solution.fire.y, 173.759f, 0.01f);
  // Test case implementation
}
TEST(Ballistics, ComputesKnownDropPointBasic2)
{
  const BallisticsInput input{.drone = {0.0f, 0.0f},
                              .drone_z = 100.0f,
                              .target = {300.0f, 300.0f},
                              .attack_speed = 20.0f,
                              .acceleration_path = 50.0f,
                              .ammo_name = "GLIDING-VOG"};
  const DropSolution solution = compute_drop_solution(input);
  EXPECT_NEAR(solution.fire.x, 242.711f, 0.01f);
  EXPECT_NEAR(solution.fire.y, 242.711f, 0.01f);
  // Test case implementation
}
TEST(Ballistics, ComputesKnownDropPointTest1)
{
  const BallisticsInput input{.drone = {543.0f, 232.0f},
                              .drone_z = 120.0f,
                              .target = {1034.0f, 432.0f},
                              .attack_speed = 13.0f,
                              .acceleration_path = 12.0f,
                              .ammo_name = "GLIDING-RKG"};
  const DropSolution solution = compute_drop_solution(input);
  EXPECT_NEAR(solution.fire.x, 966.534f, 0.01f);
  EXPECT_NEAR(solution.fire.y, 404.519f, 0.01f);
  // Test case implementation
}

TEST(Ballistics, ComputesKnownDropPointTest2)
{
  const BallisticsInput input{.drone = {543.0f, 232.0f},
                              .drone_z = 120.0f,
                              .target = {553.0f, 242.0f},
                              .attack_speed = 13.0f,
                              .acceleration_path = 12.0f,
                              .ammo_name = "RKG-3"};
  const DropSolution solution = compute_drop_solution(input);
  EXPECT_NEAR(solution.fire.x, 513.085f, 0.01f);
  EXPECT_NEAR(solution.fire.y, 202.085f, 0.01f);
  EXPECT_NEAR(solution.maneuver_used, true, 0.01f);
  // Test case implementation
}
TEST(Ballistics, ComputesKnownDropPointTest3)
{
  const BallisticsInput input{.drone = {543.0f, 232.0f},
                              .drone_z = 120.0f,
                              .target = {543.0f, 232.0f},
                              .attack_speed = 13.0f,
                              .acceleration_path = 12.0f,
                              .ammo_name = "M67"};
  const DropSolution solution = compute_drop_solution(input);
  EXPECT_NEAR(solution.fire.x, 490.496f, 0.01f);
  EXPECT_NEAR(solution.fire.y, 232.0f, 0.01f);
  EXPECT_NEAR(solution.maneuver_used, true, 0.01f);
  // Test case implementation
}