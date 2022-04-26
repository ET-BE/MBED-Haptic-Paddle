#include "mbed.h"
#include "states/control.h"

const float Fs = 500.0f; // Hz

Timer t;

int main()
{
    printf("Getting started\n");

    t.start();

    Control machine(Fs);

    machine.setState(Control::RUN);

    while (true) {

        const std::chrono::duration<float> loop_time(1.0f / Fs);
        auto next = t.elapsed_time() + loop_time;

        machine.run();

        // Busy wait untill the looptime (including execution) has passed
        while (t.elapsed_time() < next) {}
    }
}
