#define ENABLE_LOG 1
#define ENABLE_DEBUG 1

#include <iostream>
#include <sstream>
// #define _USE_MATH_DEFINES
#include <cmath>
#include <fstream>
#include <string>
#if ENABLE_LOG
#define LOG(msg) std::cout << msg << std::endl
#else
#define LOG(msg)
#endif

#if ENABLE_DEBUG
#define DEBUG(msg) std::cout << msg << std::endl
#else
#define DEBUG(msg)
#endif

const int TICKS_PER_REVOLUTION = 1024;
const double WHEEL_RADIUS_M = 0.3;
const double WHEELBASE_M = 1.0;

struct DataInput {
  float timestamp_ms;
  int fl_ticks;
  int fr_ticks;
  int bl_ticks;
  int br_ticks;
};

struct Output {
  float timestamp_ms;
  double x;
  double y;
  double theta;
};

int main(int argc, char** argv)
{
  // The program expects exactly one argument: a path to telemetry samples.
  if (argc != 2) {
    std::cerr << "usage: ugv_odometry <input_path>\n";
    return 1;
  }

  std::ifstream file(argv[1]);
  if (!file.is_open()) {
    std::cerr << "error: could not open input file\n";
    return 1;
  }
  int line_count = 0;
  std::string line;
  while (std::getline(file, line)) {
    if (!line.empty()) {
      DataInput data_input;
      std::istringstream iss(line);
      if (!(iss >> data_input.timestamp_ms >> data_input.fl_ticks >> data_input.fr_ticks >> data_input.bl_ticks >> data_input.br_ticks)) {
        std::cerr << "error: failed to parse line " << line_count << "\n";
        continue;
      }
      DEBUG("Parsed values: " << data_input.timestamp_ms << " " << data_input.fl_ticks << " " << data_input.fr_ticks << " "
                              << data_input.bl_ticks << " " << data_input.br_ticks);
    }

    line_count++;
  }

  // TODO: implement wheel odometry for a 4-wheel differential-drive UGV.
  //

  //
  // Input: a text file with 5 whitespace-separated values per line:
  //         timestamp_ms fl_ticks fr_ticks bl_ticks br_ticks
  // Output: a table on stdout, starting from the second sample:
  //         timestamp_ms x y theta

  return 0;
}
