/* gpif_config.c -- GPIF II configuration (the form GPIF II Designer generates, cyfxgpif2config.h).
 * Data taken from the vendor image. It must be the first application object: its const config is
 * the first thing in the vendor's .rodata and its tables the first thing in .data. */
#include "app.h"

/* Transition function values used in the state machine. */
uint16_t CyFxGpifTransition[] = {
    0x0000, 0x8080, 0x2222, 0x5555, 0x7F7F, 0x1F1F, 0x8888
};

/* Table containing the transition information for various states. */
CyU3PGpifWaveData CyFxGpifWavedata[] = {
    {{0x1E086001,0x000302C4,0x80000000},{0x00000000,0x00000000,0x00000000}},
    {{0x4E080302,0x00000300,0x80000000},{0x1E086006,0x000302C4,0x80000000}},
    {{0x1E086001,0x000302C4,0x80000000},{0x4E040704,0x20000300,0xC0100000}},
    {{0x4E080302,0x00000300,0x80000000},{0x1E086001,0x000302C4,0x80000000}},
    {{0x00000000,0x00000000,0x00000000},{0x00000000,0x00000000,0x00000000}},
    {{0x00000000,0x00000000,0x00000000},{0x3E738705,0x00000300,0xC0100000}},
    {{0x00000000,0x00000000,0x00000000},{0x5E002703,0x2003030C,0x80000000}},
    {{0x00000000,0x00000000,0x00000000},{0x4E040704,0x20000300,0xC0100000}}
};

/* Table that maps state indices to the descriptor table indices. */
uint8_t CyFxGpifWavedataPosition[] = {
    0, 1, 0, 2, 0, 0, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4,
    4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
    4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
    4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
    0, 5, 0, 2, 0, 0, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4,
    4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
    4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
    4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
    0, 6, 0, 2, 0, 0, 6, 4, 4, 4, 4, 4, 4, 4, 4, 4,
    4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
    4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
    4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
    0, 7, 0, 2, 0, 0, 7
};

/* GPIF II configuration register values. */
uint32_t CyFxGpifRegValue[] = {
    0x80000380, 0x000010A7, 0x01070002, 0x00000044,
    0x00000000, 0x00000000, 0x00000000, 0x00000082,
    0x00000782, 0x00011500, 0x0000FE8F, 0x000001FF,
    0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0x00000000, 0x00000010, 0x00000014, 0x00000013,
    0x00000000, 0x00000017, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0x00000000, 0x00000006, 0x00000000, 0x0000FFFF,
    0x0000010A, 0x00000000, 0x0000FFFF, 0x00000000,
    0x0000FFFF, 0x0000010A, 0x00000000, 0x0000FFFF,
    0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0x00000000, 0x80010400,
    0x80010401, 0x80010402, 0x80010403, 0x00000000,
    0x00000000, 0x00000000, 0x000C0000, 0x00000000,
    0x00000000, 0x00000000, 0x00000000, 0xFFFFFFC1
};

/* This structure holds all the configuration inputs for the GPIF II. */
const CyU3PGpifConfig_t CyFxGpifConfig = {
    (uint16_t)(sizeof(CyFxGpifWavedataPosition) / sizeof(uint8_t)),
    CyFxGpifWavedata,
    CyFxGpifWavedataPosition,
    (uint16_t)(sizeof(CyFxGpifTransition) / sizeof(uint16_t)),
    CyFxGpifTransition,
    (uint16_t)(sizeof(CyFxGpifRegValue) / sizeof(uint32_t)),
    CyFxGpifRegValue
};
