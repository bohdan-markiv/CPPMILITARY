#include "uart.h"

bool Uart::open(const char *dev)
{
    fd_ = ::open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK); // ::open = POSIX, not this method
    if (fd_ < 0)
    {
        perror("open");
        return false;
    }
    termios tio{};
    tcgetattr(fd_, &tio);
    cfmakeraw(&tio); // 8N1, no char processing
    cfsetispeed(&tio, B115200);
    cfsetospeed(&tio, B115200);
    tio.c_cflag |= (CLOCAL | CREAD);
    tcsetattr(fd_, TCSANOW, &tio);
    return true;
}

bool Uart::poll(dlink::Telemetry &t, bool &gotTelem,
                dlink::TargetPos &tgt, bool &gotTgt,
                dlink::AmmoCfg &ammo, bool &gotAmmo)
{
    gotTelem = gotTgt = gotAmmo = false;
    uint8_t buf[256];
    int n = ::read(fd_, buf, sizeof(buf));
    if (n <= 0)
        return false;

    uint8_t type, len, payload[260];
    for (int i = 0; i < n; ++i)
    {
        if (parser_.feed(buf[i], type, payload, len))
        {
            switch (type)
            {
            case PKT_TELEMETRY:
                std::memcpy(&t, payload, sizeof t);
                gotTelem = true;
                break;
            case PKT_TARGET:
                std::memcpy(&tgt, payload, sizeof tgt);
                gotTgt = true;
                break;
            case PKT_AMMO:
                std::memcpy(&ammo, payload, sizeof ammo);
                gotAmmo = true;
                break;
            }
        }
    }
    return gotTelem || gotTgt || gotAmmo;
}

void Uart::sendControl(float accel, float turnRate)
{
    Control c{accel, turnRate};
    uint8_t out[64];
    size_t m = encode(PKT_CONTROL, &c, sizeof c, out);
    ::write(fd_, out, m);
}