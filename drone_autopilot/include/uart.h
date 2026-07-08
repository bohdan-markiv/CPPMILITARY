#pragma once
#include "drone_link.h"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <cstdio>
using namespace dlink;

class Uart
{
public:
    bool open(const char *dev);
    bool poll(dlink::Telemetry &t, bool &gotTelem,
              dlink::TargetPos &tgt, bool &gotTgt,
              dlink::AmmoCfg &ammo, bool &gotAmmo);
    void sendControl(float accel, float turnRate);

private:
    int fd_ = -1;
    dlink::Parser parser_;
};