#include "guidance.h"
#include <cmath>

void Guidance::setAmmo(const dlink::AmmoCfg &a)
{
    ammo_ = a;
    haveAmmo_ = true;
}

void Guidance::updateTarget(const dlink::TargetPos &t)
{
    tgtX_ = t.x;
    tgtY_ = t.y;
    haveTarget_ = true;
}

// Wrap an angle to [-pi, pi]
static float wrapPi(float a)
{
    while (a > M_PI)
        a -= 2.0f * (float)M_PI;
    while (a < -M_PI)
        a += 2.0f * (float)M_PI;
    return a;
}

Command Guidance::steer(const dlink::Telemetry &t)
{
    float dx = tgtX_ - t.x;
    float dy = tgtY_ - t.y;

    float desiredDir = std::atan2(dy, dx);
    float err = wrapPi(desiredDir - t.dir);

    // Proportional steering. +err = target is to the left = turn left (+turnRate).
    float turn = err * kTurnGain_;
    if (turn > 1.0f)
        turn = 1.0f;
    if (turn < -1.0f)
        turn = -1.0f;

    // Throttle: full when far or badly misaligned; ease off only when very close.
    float dist = std::sqrt(dx * dx + dy * dy);
    float accel;
    if (dist > kNearDist_)
        accel = 1.0f; // far: full speed
    else
        accel = 0.2f; // close: coast, let drop happen
    // If pointing badly wrong, don't accelerate into the wrong direction.
    if (std::fabs(err) > 1.2f)
        accel = 0.0f;

    return {accel, turn};
}

// Estimate how far ahead the projectile travels during its fall.
// Simple model: time to fall from altitude z under gravity, times horizontal speed.
// drag/lift from ammo shift the effective throw; kThrowScale_ tunes the whole thing.
float Guidance::forwardThrow(const dlink::Telemetry &t) const
{
    const float g = 9.81f;
    float z = t.z > 0.0f ? t.z : 1.0f;
    float tFall = std::sqrt(2.0f * z / g); // free-fall time, seconds
    float throw_m = t.speed * tFall;       // horizontal distance covered
    return throw_m * kThrowScale_;
}

bool Guidance::shouldDrop(const dlink::Telemetry &t)
{
    if (!haveAmmo_ || !haveTarget_)
        return false;

    float dx = tgtX_ - t.x;
    float dy = tgtY_ - t.y;
    float dist = std::sqrt(dx * dx + dy * dy);

    // How far the munition will fly forward after release.
    float throw_m = forwardThrow(t);

    // We want to release when our REMAINING distance equals the throw:
    // release short of the target by exactly throw_m.
    // Also require we're roughly pointed at the target (else the throw goes sideways).
    float desiredDir = std::atan2(dy, dx);
    float err = std::fabs(wrapPi(desiredDir - t.dir));

    bool aligned = err < kAlignTol_; // pointed at target
    bool inWindow = dist <= (throw_m + ammo_.hitRadius) &&
                    dist >= (throw_m - ammo_.hitRadius); // right distance

    return aligned && inWindow;
}