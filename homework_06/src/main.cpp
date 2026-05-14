#include "ballistics.hpp"

#include <iostream>

int main(int argc, char** argv)
{
  // The executable expects exactly one telemetry log path.
  if (argc != 2) {
    std::cerr << "usage: ballistics <input_path>\n";
    return 1;
  }

  try {
    BallisticsInput input;
    read_input(argv[1], input);
    const DropSolution solution = compute_drop_solution(input);
    print_drop_solution(solution);
  }
  catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << '\n';
    return 1;
  }
  return 0;
}
