#include <cstdio>
#include <pico/stdlib.h>

#include "aps6408.hpp"

using namespace pimoroni;

APS6408 sram;

int main() {
    stdio_init_all();

    while (!stdio_usb_connected());

    sram.init();

    uint32_t data = 0x12345678;
    sram.write(0, &data, 1);

    uint32_t read_data = 0;
    sram.read(0, &read_data, 1);

    printf("Read back %08x\n", read_data);

    return 0;
}