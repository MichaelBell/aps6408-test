#include <stdlib.h>
#include "hardware/pio.h"

void logic_analyser_arm(uint trigger_pin, bool trigger_level);
void logic_analyser_print_capture_buf(int min_sample);
void logic_analyser_init(PIO pio_, int pin_base_, int pin_count_, int n_samples_, float div);
bool logic_analyser_is_capture_done();