#include "drone_link.h"
struct Command
{
    float accel;
    float turnRate;
};

class Guidance
{
public:
    void setAmmo(const dlink::AmmoCfg &a);
    void updateTarget(const dlink::TargetPos &t);

    Command steer(const dlink::Telemetry &t);
    bool shouldDrop(const dlink::Telemetry &t);

private:
    float forwardThrow(const dlink::Telemetry &t) const;

    dlink::AmmoCfg ammo_{};
    float tgtX_ = 0.0f, tgtY_ = 0.0f;
    bool haveAmmo_ = false;
    bool haveTarget_ = false;

    // --- Tuning knobs (adjust these against checker miss distances) ---
    float kTurnGain_ = 2.0f;   // steering aggressiveness
    float kNearDist_ = 40.0f;  // m: below this, coast instead of full throttle
    float kThrowScale_ = 1.0f; // scales the drop lead distance (start at 1.0)
    float kAlignTol_ = 0.03f;  // rad: how aligned before we allow a drop (~8.6 deg)
};
