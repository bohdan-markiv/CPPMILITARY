#include "Types.h"
#include "helpers.h"

float distanceCalculation(Coord currentCoord, Coord targetCoord)
{
  return sqrt(pow(targetCoord.x - currentCoord.x, 2) + pow(targetCoord.y - currentCoord.y, 2));
}

float timeToTarget(float m, float d, float l, float attackSpeed, float zd)
{
  float a = d * g * m - 2.0f * pow(d, 2) * l * attackSpeed;
  // DEBUG("The acceleration of the ammo is: " << a);
  // b = −3g·m² + 3d·l·m·V₀
  float b = -3.0f * g * pow(m, 2) + 3 * d * l * m * attackSpeed;
  // std::cout << "The coefficient b is: " << b << std::endl;
  // c = 6m²·Z₀
  float c = 6 * pow(m, 2) * zd;
  // std::cout << "The coefficient c is: " << c << std::endl;
  if (a == 0.0f) {
    throw std::runtime_error("The coefficient a cannot be zero.");
  }
  // p = − b² / (3a²)
  float p = (-1.0f * pow(b, 2)) / (3.0f * pow(a, 2));
  // std::cout << "The coefficient p is: " << p << std::endl;
  // q = 2b³ / (27a³) + c / a
  float q = ((2.0f * pow(b, 3)) / (27.0f * pow(a, 3)) + c / a);
  // std::cout << "The coefficient q is: " << q << std::endl;

  // arg = 3q / (2p) · √(−3/p)
  float arg = ((3.0f * q) / (2.0f * p) * sqrt(-3.0f / p));
  // std::cout << "The argument for the cubic root is: " << arg << std::endl;
  if (arg < -1.0f || arg > 1.0f) {
    throw std::runtime_error("The argument for the cubic root is out of range for acos: " + std::to_string(arg));
  }
  float fphi = acos(arg);
  // std::cout << "The angle fphi is: " << fphi << std::endl;

  // t = 2√(−p/3) · cos( (φ + 4π) / 3 ) − b / (3a)
  float t = (2.0f * sqrt(-p / 3.0f) * cos((fphi + 4.0f * kPi) / 3.0f) - b / (3.0f * a));
  if (t < 1e-3f) {
    throw std::runtime_error("The time of flight cannot be negative: " + std::to_string(t));
  }
  // std::cout << "The time of flight t is: " << t << std::endl;
  return t;
}
float ammoFlyDistance(float t, float d, float l, float attackSpeed, float m)
{
  // h = V₀t − t²d·V₀/(2m) + t³(6d·g·l·m − 6d²(l²-1)·V₀)/(36m²) +
  //  + t⁴ (−6d²g·l·(1+l²+l⁴)m + 3d³l²(1+l²)V₀ + 6d³l⁴(1+l²)V₀)  / (36(1+l²)²m³)
  //  + t⁵(3d³g·l³m − 3d⁴l²(1+l²)V₀) / (36(1+l²)m⁴)
  //  Розбиваємо на доданки (t^1, t^2, t^3...)
  float t2 = t * t;
  float t3 = t2 * t;
  float t4 = t3 * t;
  float t5 = t4 * t;
  float l2 = l * l;
  float l4 = l2 * l2;

  // Обчислюємо кожен доданок окремо
  float term1 = attackSpeed * t;
  float term2 = ((t2 * d * attackSpeed) / (2.0f * m));

  float term3_num = 6.0f * d * g * l * m - 6.0f * (d * d) * (l2 - 1.0f) * attackSpeed;
  float term3 = ((t3 * term3_num) / (36.0f * m * m));

  float term4_num = -6.0f * (d * d) * g * l * (1.0f + l2 + l4) * m + 3.0f * std::pow(d, 3) * l2 * (1.0f + l2) * attackSpeed +
                    6.0f * std::pow(d, 3) * l4 * (1.0f + l2) * attackSpeed;
  float term4_den = 36.0f * pow(1.0f + l2, 2) * pow(m, 3);
  float term4 = ((t4 * term4_num) / term4_den);

  float term5_num = 3.0f * pow(d, 3) * g * (l2 * l) * m - 3.0f * pow(d, 4) * l2 * (1.0f + l2) * attackSpeed;
  float term5_den = 36.0f * (1.0f + l2) * pow(m, 4);
  float term5 = ((t5 * term5_num) / term5_den);

  float h = term1 - term2 + term3 + term4 + term5;
  if (h < 0.0f) {
    throw std::runtime_error("The distance to target cannot be negative: " + std::to_string(h));
  }
  // std::cout << "The distance to target is: " << h << std::endl;
  return h;
}

void interpolateTarget(float t, float arrayTimeStep, Coord* trajectory, int timeSteps, Coord& interpolatedCoord)
{
  float period = timeSteps * arrayTimeStep;
  float wrappedT = fmodf(t, period);
  if (wrappedT < 0.0f)
    wrappedT += period;

  int idx = (int)floorf(wrappedT / arrayTimeStep);
  if (idx >= timeSteps)
    idx = timeSteps - 1;  // guard against fp rounding at the seam

  int next = (idx + 1) % timeSteps;
  float frac = (wrappedT - idx * arrayTimeStep) / arrayTimeStep;

  interpolatedCoord = trajectory[idx] + (trajectory[next] - trajectory[idx]) * frac;
}
