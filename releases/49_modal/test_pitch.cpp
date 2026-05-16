#include <stdio.h>
#include <stdint.h>
#include "resources_q15.cpp"
#include "dsp_q15.h"

int main() {
    printf("Inc 20480: %u\n", MidiToIncrementU32(20480));
    printf("Inc 20480+12st: %u\n", MidiToIncrementU32(20480+3072));
    printf("Inc 20480-12st: %u\n", MidiToIncrementU32(20480-3072));
    return 0;
}
