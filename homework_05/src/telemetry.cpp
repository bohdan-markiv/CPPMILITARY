#include "telemetry.hpp"

#include <cstdlib>
#include <fstream>
#include <iostream>

// Debugging exercise notes:
// this file intentionally contains four runtime defects.
// The defects are related to malformed input shape, invalid numeric values,
// unsafe time deltas, and empty logs. Exact locations are not marked on purpose.

const int EXPECTED_FIELD_COUNT = 7;
const int MAX_LINE_LENGTH = 256;

int split_line(char line[], char* fields[], int max_fields)
{
  int count = 0;
  char* cursor = line;

  while (*cursor != '\0' && count < max_fields) {
    while (*cursor == ' ' || *cursor == '\t' || *cursor == '\n' || *cursor == '\r') {
      *cursor = '\0';
      ++cursor;
    }

    if (*cursor == '\0') {
      break;
    }

    fields[count] = cursor;
    ++count;

    while (*cursor != '\0' && *cursor != ' ' && *cursor != '\t' && *cursor != '\n' && *cursor != '\r') {
      ++cursor;
    }
  }

  return count;
}

long parse_long(const char* text)
{
  char* end = nullptr;
  const long value = std::strtol(text, &end, 10);

  if (end == text || *end != '\0') {
    throw std::runtime_error("Error with parsing into long");
  }

  return value;
}

int parse_int(const char* text)
{
  char* end = nullptr;
  const int value = std::strtol(text, &end, 10);
  if (end == text || *end != '\0') {
    throw std::runtime_error("Error with parsing into int");
  }
  return value;
}
double parse_double(const char* text)
{
  char* end = nullptr;
  const double value = std::strtod(text, &end);

  if (end == text || *end != '\0') {
    throw std::runtime_error("Error with parsing into double");
  }

  return value;
}

Frame parse_frame(char line[])
{
  char* fields[EXPECTED_FIELD_COUNT] = {};
  const int field_count = split_line(line, fields, EXPECTED_FIELD_COUNT);

  if (field_count != EXPECTED_FIELD_COUNT) {
    throw std::runtime_error("Invalid frame format: expected " + std::to_string(EXPECTED_FIELD_COUNT) + " fields, got " +
                             std::to_string(field_count));
  };
  (void)field_count;

  Frame frame{};
  frame.timestamp_ms = parse_long(fields[0]);
  frame.seq = parse_int(fields[1]);
  frame.voltage_v = parse_double(fields[2]);
  if (frame.voltage_v <= 0.0) {
    throw std::runtime_error("voltage has to be positive");
  }
  frame.current_a = parse_double(fields[3]);
  if (frame.current_a < 0.0) {
    throw std::runtime_error("current can not be negative");
  }
  frame.temperature_c = parse_double(fields[4]);
  if (frame.temperature_c < -40.0 || frame.temperature_c > 120.0) {
    throw std::runtime_error("temperature is out of valid range");
  }
  frame.gps_fix = parse_int(fields[5]);
  if (frame.gps_fix != 0 && frame.gps_fix != 1) {
    throw std::runtime_error("GPS fix is out of valid range");
  }
  frame.satellites = parse_int(fields[6]);
  if (frame.satellites < 0) {
    throw std::runtime_error("number of satellites is out of valid range");
  }
  return frame;
}

double compute_frame_rate_hz(const Frame frames[], int frame_count)
{
  const long elapsed_ms = frames[frame_count - 1].timestamp_ms - frames[0].timestamp_ms;

  if (elapsed_ms <= 0) {
    throw std::runtime_error("The time difference between the first and last frame must be positive to compute frame rate.");
  }
  return static_cast<double>((frame_count - 1) * 1000 / static_cast<double>(elapsed_ms));
}

int read_frames(const char* path, Frame frames[], int max_frames)
{
  std::ifstream input{path};
  if (!input) {
    std::cerr << "error: failed to open input file: " << path << '\n';
    return 0;
  }

  int frame_count = 0;
  char line[MAX_LINE_LENGTH];

  while (input.getline(line, MAX_LINE_LENGTH)) {
    if (line[0] == '\0') {
      continue;
    }

    if (frame_count < max_frames) {
      frames[frame_count] = parse_frame(line);
      ++frame_count;
    }
  }
  if (frame_count == 0) {
    throw std::runtime_error("File is empty or contains only empty lines, at least one valid frame is required.");
  }

  return frame_count;
}

Summary summarize(const Frame frames[], int frame_count)
{
  for (int i = 1; i < frame_count; ++i) {
    if (frames[i].seq != frames[i - 1].seq + 1) {
      throw std::runtime_error("Frame sequence numbers must be strictly increasing by 1 to compute frame rate.");
    }
    if (frames[i].timestamp_ms <= frames[i - 1].timestamp_ms) {
      throw std::runtime_error("Frame timestamps must be strictly increasing to compute frame rate.");
    }
  }
  Summary summary{};
  summary.frames_total = frame_count;
  summary.frames_valid = frame_count;
  summary.voltage_min = frames[0].voltage_v;
  summary.voltage_max = frames[0].voltage_v;
  summary.low_voltage_frames = 0;

  double temperature_sum = 0.0;

  for (int i = 0; i < frame_count; ++i) {
    if (frames[i].voltage_v < summary.voltage_min) {
      summary.voltage_min = frames[i].voltage_v;
    }

    if (frames[i].voltage_v > summary.voltage_max) {
      summary.voltage_max = frames[i].voltage_v;
    }

    temperature_sum += frames[i].temperature_c;

    if (frames[i].voltage_v < 22.0) {
      ++summary.low_voltage_frames;
    }
  }

  const int temperature_tenths = static_cast<int>(temperature_sum * 10.0) / frame_count;
  summary.temperature_avg = static_cast<double>(temperature_tenths) / 10.0;
  summary.frame_rate_hz = compute_frame_rate_hz(frames, frame_count);
  return summary;
}

void print_summary(const Summary& summary)
{
  std::cout << "frames_total " << summary.frames_total << '\n';
  std::cout << "frames_valid " << summary.frames_valid << '\n';
  std::cout << "voltage_min " << summary.voltage_min << '\n';
  std::cout << "voltage_max " << summary.voltage_max << '\n';
  std::cout << "temperature_avg " << summary.temperature_avg << '\n';
  std::cout << "low_voltage_frames " << summary.low_voltage_frames << '\n';
  std::cout << "frame_rate_hz " << summary.frame_rate_hz << '\n';
}
