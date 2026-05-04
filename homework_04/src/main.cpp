#define ENABLE_LOG 1
#define ENABLE_DEBUG 0

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
  DataInput previousOutput;
  Output currentPosition;

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

      if (line_count != 0) {
        // Крок 1. Delta iмпульсiв по кожному колесу:
        int delta_fl = data_input.fl_ticks - previousOutput.fl_ticks;
        int delta_fr = data_input.fr_ticks - previousOutput.fr_ticks;
        int delta_bl = data_input.bl_ticks - previousOutput.bl_ticks;
        int delta_br = data_input.br_ticks - previousOutput.br_ticks;
        // Крок 2. Усереднити борти (передне i заднє колесо одного боку обертаються синхронно):
        double distance_left = (delta_fl + delta_bl) / 2.0;
        double distance_right = (delta_fr + delta_br) / 2.0;

        // Крок 3. Перевести iмпульси у метри:
        double distance_per_tick = 2 * M_PI * WHEEL_RADIUS_M / static_cast<double>(TICKS_PER_REVOLUTION);

        double dL = distance_left * distance_per_tick;
        double dR = distance_right * distance_per_tick;

        // Крок 4. Скiльки пройшов центр робота i на скiльки повернувся:
        double d = (dL + dR) / 2.0;
        double dTheta = (dR - dL) / WHEELBASE_M;

        // Крок 5. Оновити позицiю (midpoint integration - усереднений напрямок на кроцi):
        currentPosition.x += d * cos(currentPosition.theta + dTheta / 2.0);
        currentPosition.y += d * sin(currentPosition.theta + dTheta / 2.0);
        currentPosition.theta += dTheta;
        currentPosition.timestamp_ms = data_input.timestamp_ms;

        LOG(currentPosition.timestamp_ms << " " << std::round(currentPosition.x * 1000.0) / 1000.0 << " "
                                         << std::round(currentPosition.y * 1000.0) / 1000.0 << " "
                                         << std::round(currentPosition.theta * 1000.0) / 1000.0);
      }
      else {
        currentPosition.timestamp_ms = data_input.timestamp_ms;
        currentPosition.x = 0.0;
        currentPosition.y = 0.0;
        currentPosition.theta = 0.0;
      }

      previousOutput = data_input;
    }
    else {
      LOG("Warning: Skipping empty line at line number " << line_count);
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
