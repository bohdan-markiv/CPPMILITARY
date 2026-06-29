#include <gpiod.h>
#include <cstdio>
#include <unistd.h>
class Gpio
{
public:
    bool open(const char *chip, int startLine, int dropLine);
    void raiseStart(); // set START=1, hold
    void pulseDrop();  // DROP=1, usleep(80000), DROP=0; latch once
    void close();

private:
    gpiod_chip *chip_ = nullptr;
    gpiod_line *start_ = nullptr;
    gpiod_line *drop_ = nullptr;
    bool dropped_ = false; // ensures single pulse
};