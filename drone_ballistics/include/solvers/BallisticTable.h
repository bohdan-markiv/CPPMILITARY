#pragma once

#include <vector>
#include <cmath>
#include "Types.h"

struct BallisticTable {
  // 5 осей — кожна зі своїм набором вузлів (нерівномірний крок)
  std::vector<float> axisZ0;  // висота
  std::vector<float> axisV0;  // швидкість
  std::vector<float> axisM;   // маса
  std::vector<float> axisD;   // опір
  std::vector<float> axisL;   // підйомна сила

  // Результат в кожному вузлі сітки

  // Плоский масив розміром |Z0| * |V0| * |M| * |D| * |L|
  std::vector<Result> data;

  // Індекс у плоскому масиві: [iZ0][iV0][iM][iD][iL]
  size_t index(int iz, int iv, int im, int id, int il) const;

  const Result& at(int iz, int iv, int im, int id, int il) const;

  // Завантаження з текстового файлу
  bool load(const char* path);

  Result lerp(const Result& a, const Result& b, float t) const { return {a.t + (b.t - a.t) * t, a.hDist + (b.hDist - a.hDist) * t}; }

  Interp findInterp(float val, const std::vector<float>& axis) const;

  Result lookup(float Z0, float V0, float m, float d, float l) const;
};
