#include "mbed.h"
#include "states/control.h"

const float Fs = 500.0f; // Hz

Timer t;

/**
 * Implement default method to point to local console instance.
 *
 * This is needed to combine e.g. `serial.read` and `printf`.
 */
FileHandle *mbed::mbed_override_console(int) {
    return &Control::console;
}

int main()
{
    printf("Getting started\n");

    t.start();

    Control machine(Fs);

    while (true) {

        const std::chrono::duration<float> loop_time(1.0f / Fs);
        auto next = t.elapsed_time() + loop_time;

        machine.run();

        // Busy wait until the looptime (including execution) has passed
        while (t.elapsed_time() < next) {}
    }
}
