#include <cstdio>
#include <random>
#include <pico/stdlib.h>
#include <hardware/clocks.h>
#include <hardware/vreg.h>

#include "aps6408.hpp"

extern "C" {
#include "logic_analyser.h"
}

using namespace pimoroni;

const uint32_t data_array[8] = {
    0x12345678,
    0x23456789,
    0x3456789A,
    0x456789AB,
    0x56789ABC,
    0x6789ABCD,
    0x789ABCDE,
    0x89ABCDEF
};

#define XFER_LEN_WORDS 256

int main() {
    vreg_set_voltage(VREG_VOLTAGE_1_15);
    sleep_us(100);

    set_sys_clock_hz(264000000, true);

    stdio_init_all();

    APS6408 sram;

    while (!stdio_usb_connected());

    logic_analyser_init(pio1, 4, 12, 1000, 1.f);
    sram.init();

    uint32_t data = 0x92345678;
    sram.write(0, &data, 1);

    uint32_t read_data = 0;
    sram.read_blocking(0, &read_data, 1);
    logic_analyser_arm(13, false);
    sram.read_blocking(0, &read_data, 1);
    logic_analyser_print_capture_buf(0);

    printf("Read back %08x\n", read_data);

    for (int i = 0; i < 8 * 1024 * 1024; i += 32) {
        sram.write(i, data_array, 8);
    }

    uint32_t data_buf[XFER_LEN_WORDS];
    int fail_count = 0;
    for (int i = 0; i < 8 * 1024 * 1024; i += 32) {
        sram.read_blocking(i, data_buf, 8);
        for (int j = 0; j < 8; ++j) {
            if (data_buf[j] != data_array[j]) {
                printf("Fail at addr %x: %x != %x, previous word %x\n", i+j*4, data_buf[j], data_array[j], data_buf[j-1]);
                if (++fail_count > 10)
                    while (1);
            }
        }
    }

    if (fail_count == 0)
        printf("All OK\n");
    else
        printf("Done with %d fails\n", fail_count);

    std::mt19937 mt;
    int seed = 0;

    while (++seed) {
        mt.seed(seed);
        for (int i = 0; i < 8 * 1024 * 1024; i += XFER_LEN_WORDS*4) {
            for (int j = 0; j < XFER_LEN_WORDS; ++j) {
                data_buf[j] = mt();
            }
            sram.write_blocking(i, data_buf, XFER_LEN_WORDS);
        }


        mt.seed(seed);
        fail_count = 0;
        for (int i = 0; i < 8 * 1024 * 1024; i += XFER_LEN_WORDS*4) {
            sram.read_blocking(i, data_buf, XFER_LEN_WORDS);
            for (int j = 0; j < XFER_LEN_WORDS; ++j) {
                uint32_t val = mt();
                if (data_buf[j] != val) {
                    printf("Fail at addr %x: %x != %x, previous word %x\n", i+j*4, data_buf[j], val, data_buf[j-1]);
                    if (++fail_count > 10)
                        while (1);
                }
            }
        }

        if (fail_count == 0)
            printf("Seed %d OK\n", seed);
        else
            printf("Seed %d had %d fails\n", seed, fail_count);
    }

    while (1);

    return 0;
}