#include "ballistics.hpp"
#include <gtest/gtest.h>
#include <stdexcept>

TEST(Ballistics, ComputesKnownDropPointBasic1)
{
  const BallisticsInput input{.drone = {180.0F, 180.0F},
                              .drone_z = 100.0F,
                              .target = {200.0F, 200.0F},
                              .attack_speed = 10.0F,
                              .acceleration_path = 10.0F,
                              .ammo_name = "VOG-17"};
  const DropSolution solution = compute_drop_solution(input);
  EXPECT_NEAR(solution.fire.x, 173.759F, 0.01F);
  EXPECT_NEAR(solution.fire.y, 173.759F, 0.01F);
  // Test case implementation
}
TEST(Ballistics, ComputesKnownDropPointBasic2)
{
  const BallisticsInput input{.drone = {0.0F, 0.0F},
                              .drone_z = 100.0F,
                              .target = {300.0F, 300.0F},
                              .attack_speed = 20.0F,
                              .acceleration_path = 50.0F,
                              .ammo_name = "GLIDING-VOG"};
  const DropSolution solution = compute_drop_solution(input);
  EXPECT_NEAR(solution.fire.x, 242.711F, 0.01F);
  EXPECT_NEAR(solution.fire.y, 242.711F, 0.01F);
  // Test case implementation
}
TEST(Ballistics, ComputesKnownDropPointTest1)
{
  const BallisticsInput input{.drone = {543.0F, 232.0F},
                              .drone_z = 120.0F,
                              .target = {1034.0F, 432.0F},
                              .attack_speed = 13.0F,
                              .acceleration_path = 12.0F,
                              .ammo_name = "GLIDING-RKG"};
  const DropSolution solution = compute_drop_solution(input);
  EXPECT_NEAR(solution.fire.x, 966.534F, 0.01F);
  EXPECT_NEAR(solution.fire.y, 404.519F, 0.01F);
  // Test case implementation
}

TEST(Ballistics, ComputesKnownDropPointTest2)
{
  const BallisticsInput input{.drone = {543.0F, 232.0F},
                              .drone_z = 120.0F,
                              .target = {553.0F, 242.0F},
                              .attack_speed = 13.0F,
                              .acceleration_path = 12.0F,
                              .ammo_name = "RKG-3"};
  const DropSolution solution = compute_drop_solution(input);
  EXPECT_NEAR(solution.fire.x, 513.085F, 0.01F);
  EXPECT_NEAR(solution.fire.y, 202.085F, 0.01F);
  EXPECT_NEAR(solution.maneuver_used, true, 0.01F);
  // Test case implementation
}
TEST(Ballistics, ComputesKnownDropPointTest3)
{
  const BallisticsInput input{.drone = {543.0F, 232.0F},
                              .drone_z = 120.0F,
                              .target = {543.0F, 232.0F},
                              .attack_speed = 13.0F,
                              .acceleration_path = 12.0F,
                              .ammo_name = "M67"};
  const DropSolution solution = compute_drop_solution(input);
  EXPECT_NEAR(solution.fire.x, 490.496F, 0.01F);
  EXPECT_NEAR(solution.fire.y, 232.0F, 0.01F);
  EXPECT_NEAR(solution.maneuver_used, true, 0.01F);
  // Test case implementation
}

TEST(Ballistics, WrongAmmoName)
{
  const BallisticsInput input{.drone = {180.0F, 180.0F},
                              .drone_z = 100.0F,
                              .target = {200.0F, 200.0F},
                              .attack_speed = 10.0F,
                              .acceleration_path = 10.0F,
                              .ammo_name = "UnknownAmmo"};
  EXPECT_THROW(compute_drop_solution(input), std::invalid_argument);
  // Test case implementation
}
