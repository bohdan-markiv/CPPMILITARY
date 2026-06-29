#include "gpio.h"

bool Gpio::open(const char *chip, int startLine, int dropLine)
{
    chip_ = gpiod_chip_open_by_name(chip);
    if (!chip_)
    {
        perror("gpiod_chip_open_by_name");
        return false;
    }

    start_ = gpiod_chip_get_line(chip_, startLine);
    drop_ = gpiod_chip_get_line(chip_, dropLine);
    if (!start_ || !drop_)
    {
        perror("gpiod_chip_get_line");
        return false;
    }

    if (gpiod_line_request_output(start_, "drone", 0) < 0)
    {
        perror("req start");
        return false;
    }
    if (gpiod_line_request_output(drop_, "drone", 0) < 0)
    {
        perror("req drop");
        return false;
    }
    return true;
};

void Gpio::raiseStart()
{
    gpiod_line_set_value(start_, 1);
};

void Gpio::pulseDrop()
{
    if (dropped_)
        return;
    dropped_ = true;
    gpiod_line_set_value(drop_, 1);
    usleep(80000);
    gpiod_line_set_value(drop_, 0);
};

void Gpio::close()
{
    if (chip_)
        gpiod_chip_close(chip_);
    chip_ = nullptr;
}