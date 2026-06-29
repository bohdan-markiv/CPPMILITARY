// guidance.cpp
#include "guidance.h"
#include <cmath>

void Guidance::setAmmo(const dlink::AmmoCfg &a) { ammo_ = a; }

void Guidance::updateTarget(const dlink::TargetPos &t)
{
    tgtX_ = t.x;
    tgtY_ = t.y;
}

Command Guidance::steer(const dlink::Telemetry &t)
{
    // vector from drone to target
    float dx = tgtX_ - t.x;
    float dy = tgtY_ - t.y;
    float desiredDir = std::atan2(dy, dx);

    // heading error, wrapped to [-pi, pi]
    float err = desiredDir - t.dir;
    while (err > M_PI)
        err -= 2 * M_PI;
    while (err < -M_PI)
        err += 2 * M_PI;

    // proportional steering: + err = turn left = +turnRate
    float turn = err * 2.0f; // gain — TUNE THIS
    if (turn > 1.0f)
        turn = 1.0f;
    if (turn < -1.0f)
        turn = -1.0f;

    // throttle: full when far and roughly aligned, ease when very near
    float dist = std::sqrt(dx * dx + dy * dy);
    float accel = (dist > 50.0f) ? 1.0f : 0.3f; // TUNE

    return {accel, turn};
}

bool Guidance::shouldDrop(const dlink::Telemetry &t)
{
    float dx = tgtX_ - t.x;
    float dy = tgtY_ - t.y;
    float dist = std::sqrt(dx * dx + dy * dy);

    // naive: drop when within hitRadius and pointed at target.
    // WRONG on purpose — ignores forward throw of the projectile.
    // You'll refine: release EARLY, so the projectile flies the
    // remaining distance during its fall. That's your ДЗ9/ДЗ10 math.
    return dist < ammo_.hitRadius;
}