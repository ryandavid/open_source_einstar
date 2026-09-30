/* mkpackage: build the update package EXStar sends (bulk 00/06) from the FX3 boot image and the FPGA
 * bitstream. The package is one flash slot -- the FX3 image, 0xFF up to 0x40000, the bitstream, 0xFF
 * up to the slot size -- as 4096-byte pages, each followed by the 8-bit sum of the page.
 *
 *     mkpackage <fx3.img> <fpga.bin> <out.img>
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define FPGA_OFFSET 0x40000u
#define SLOT_SIZE   0x140000u        /* 320 pages */
#define PAGE_SIZE   4096u

static size_t load(const char *path, unsigned char *dst, size_t max)
{
    FILE *f = fopen(path, "rb");
    size_t n;
    if (!f) {
        perror(path);
        exit(1);
    }
    n = fread(dst, 1, max + 1, f);
    fclose(f);
    if (n > max) {
        fprintf(stderr, "%s: larger than its %zu-byte region\n", path, max);
        exit(1);
    }
    return n;
}

int main(int argc, char **argv)
{
    static unsigned char slot[SLOT_SIZE];
    FILE *out;
    size_t i, j;

    if (argc != 4) {
        fprintf(stderr, "usage: %s <fx3.img> <fpga.bin> <out.img>\n", argv[0]);
        return 2;
    }
    memset(slot, 0xff, sizeof slot);
    load(argv[1], slot, FPGA_OFFSET);
    load(argv[2], slot + FPGA_OFFSET, SLOT_SIZE - FPGA_OFFSET);

    out = fopen(argv[3], "wb");
    if (!out) {
        perror(argv[3]);
        return 1;
    }
    for (i = 0; i < SLOT_SIZE; i += PAGE_SIZE) {
        unsigned char sum = 0;
        for (j = 0; j < PAGE_SIZE; j++)
            sum += slot[i + j];
        fwrite(slot + i, 1, PAGE_SIZE, out);
        fputc(sum, out);
    }
    return fclose(out) ? 1 : 0;
}
