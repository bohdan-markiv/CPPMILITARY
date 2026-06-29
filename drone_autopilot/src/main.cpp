#include "uart.h"
#include "gpio.h"
#include "guidance.h"
#include "drone_link.h"

int main(int argc, char **argv)
{
    const char *dev = "/tmp/ttyA";
    const char *chip = "gpiochip1";
    int startLine = 24;
    int dropLine = 23;

    for (int i = 1; i < argc; ++i)
    {
        if (!strcmp(argv[i], "--uart") && i + 1 < argc)
            dev = argv[++i];
        else if (!strcmp(argv[i], "--gpiochip") && i + 1 < argc)
            chip = argv[++i];
        else if (!strcmp(argv[i], "--start-line") && i + 1 < argc)
            startLine = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--drop-line") && i + 1 < argc)
            dropLine = atoi(argv[++i]);
    }
    Uart uart;
    uart.open(dev);
    Gpio gpio;
    gpio.open(chip, startLine, dropLine);
    Guidance g;

    gpio.raiseStart(); // tell checker: go

    while (true)
    {
        Telemetry t;
        TargetPos tgt;
        AmmoCfg ammo;
        bool gt = false, gtg = false, ga = false;
        uart.poll(t, gt, tgt, gtg, ammo, ga);

        if (ga)
            g.setAmmo(ammo);
        if (gtg)
            g.updateTarget(tgt);
        if (gt)
        {
            Command c = g.steer(t);
            uart.sendControl(c.accel, c.turnRate);
            if (g.shouldDrop(t))
                gpio.pulseDrop();
        }
        usleep(2000); // don't busy-spin
    }
}