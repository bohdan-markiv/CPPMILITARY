#include "drone_link.h"
struct Command
{
    float accel;
    float turnRate;
};

class Guidance
{
public:
    void setAmmo(const dlink::AmmoCfg &a); // store hitRadius, ballistics
    void updateTarget(const dlink::TargetPos &t);
    Command steer(const dlink::Telemetry &t);   // heading error -> accel/turnRate
    bool shouldDrop(const dlink::Telemetry &t); // geometry right to release?
private:
    dlink::AmmoCfg ammo_{};
    float tgtX_ = 0, tgtY_ = 0;
};
