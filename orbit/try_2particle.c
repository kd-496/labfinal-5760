#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/mman.h>

#define HW_REGS_BASE       0xff200000
#define HW_REGS_SPAN       0x00005000
#define ORBITAL_BASE       0x00000000  // Adjust based on the memory map of your FPGA design
#define ORBITAL_SPAN       0x00000010  // Size for 4 registers (4 * 4 bytes)

#define SDRAM_BASE         0xC0000000
#define SDRAM_SPAN         0x04000000
#define VGA_PIXEL(x,y,color) do {\
    int *pixel_ptr;\
    pixel_ptr = (int *)((char *)vga_pixel_ptr + (((y) * 640 + (x)) << 1));\
    *(short *)pixel_ptr = (color);\
} while (0)

#define yellow             (0+(63<<5)+(31<<11))
#define cyan               (31+(63<<5)+(0<<11))
#define black              (0x0000)

void *vga_pixel_virtual_base = NULL;
volatile unsigned int *vga_pixel_ptr = NULL;

void draw_particles(int px1, int py1, int px2, int py2) {
    VGA_PIXEL(px1, py1, yellow);
    VGA_PIXEL(px2, py2, cyan);
}

int main() {
    int fd;
    void *h2p_lw_virtual_base;
    volatile unsigned int *orbital_base_ptr;
    unsigned int px1, py1, px2, py2;

    // Open /dev/mem
    if ((fd = open("/dev/mem", (O_RDWR | O_SYNC))) == -1) {
        printf("ERROR: could not open \"/dev/mem\"...\n");
        return 1;
    }

    // Map FPGA memory
    h2p_lw_virtual_base = mmap(NULL, HW_REGS_SPAN, (PROT_READ | PROT_WRITE), MAP_SHARED, fd, HW_REGS_BASE);
    if (h2p_lw_virtual_base == MAP_FAILED) {
        printf("ERROR: mmap1() failed...\n");
        close(fd);
        return 1;
    }
    orbital_base_ptr = (unsigned int *)((char *)h2p_lw_virtual_base + ORBITAL_BASE);

    // Map SDRAM memory
    vga_pixel_virtual_base = mmap(NULL, SDRAM_SPAN, (PROT_READ | PROT_WRITE), MAP_SHARED, fd, SDRAM_BASE);
    if (vga_pixel_virtual_base == MAP_FAILED) {
        printf("ERROR: mmap2() failed...\n");
        close(fd);
        return 1;
    }
    vga_pixel_ptr = (unsigned int *)(vga_pixel_virtual_base);

    while (1) {
        // Read particle positions from the FPGA
        px1 = *(orbital_base_ptr);
        py1 = *(orbital_base_ptr + 1);
        px2 = *(orbital_base_ptr + 2);
        py2 = *(orbital_base_ptr + 3);

        // Clear the previous frame
        for (int y = 0; y < 480; y++) {
            for (int x = 0; x < 640; x++) {
                VGA_PIXEL(x, y, black);
            }
        }

        // Draw the particles
        draw_particles(px1, py1, px2, py2);

        usleep(50000);  // 50 ms delay
    }

    close(fd);
    return 0;
}

