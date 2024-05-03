#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <math.h>

// Constants for the orbital simulation
#define G 6.67430e-11  // Gravitational constant in m^3 kg^-1 s^-2
#define M 5.972e24     // Mass of Earth in kg
#define dt 10          // Time step in seconds

// VGA display settings
#define SDRAM_BASE 0xC0000000
#define FPGA_CHAR_BASE 0xC9000000
#define HW_REGS_BASE 0xff200000
#define HW_REGS_SPAN 0x00005000
#define VGA_BASE 0xC0000000  // Change this to your actual framebuffer base address
#define VGA_WIDTH 640
#define VGA_HEIGHT 480

// Address mappings
void *h2p_lw_virtual_base;
volatile unsigned int *vga_pixel_ptr = NULL;
int fd;

// Function prototypes for graphics
void VGA_pixel(int x, int y, short color);
void clear_screen();

int main(void) {
    // Open /dev/mem
    if ((fd = open("/dev/mem", (O_RDWR | O_SYNC))) == -1) {
        printf("ERROR: could not open \"/dev/mem\"...\n");
        return (1);
    }

    // Map FPGA space
    h2p_lw_virtual_base = mmap(NULL, HW_REGS_SPAN, (PROT_READ | PROT_WRITE), MAP_SHARED, fd, HW_REGS_BASE);
    if (h2p_lw_virtual_base == MAP_FAILED) {
        printf("ERROR: mmap() failed...\n");
        close(fd);
        return (1);
    }

    // Initialize pointers to VGA pixel buffer
    vga_pixel_ptr = (unsigned int *)(h2p_lw_virtual_base);

    // Clear the screen
    clear_screen();

    // Initial conditions for the orbit
    double x = 6.371e6 + 400000;  // Earth radius + altitude of 400 km
    double y = 0;
    double vx = 0;
    double vy = sqrt(G * M / x);  // Initial velocity for circular orbit
    double ax, ay, r;

    // Convert meters to pixels (scaling factor for visualization)
    double scale = 0.00005;  // Adjust scale for your display resolution and desired orbit size

    int px, py;  // Screen coordinates

    while (1) {
        // Compute the radial distance
        r = sqrt(x * x + y * y);

        // Compute acceleration components
        ax = -G * M * x / (r * r * r);
        ay = -G * M * y / (r * r * r);

        // Update velocity
        vx += ax * dt;
        vy += ay * dt;

        // Update position
        x += vx * dt;
        y += vy * dt;

        // Convert to screen coordinates
        px = (int)(x * scale) + 320;  // Center on the screen horizontally
        py = (int)(y * scale) + 240;  // Center on the screen vertically

        // Draw the new position
        clear_screen();  // Clear the previous position
        VGA_pixel(px, py, 0xffff);  // Draw new position

        usleep(50000);  // Delay for visibility
    }

    close(fd);
    return 0;
}



void VGA_pixel(int x, int y, short color) {
    if (x >= 0 && x < VGA_WIDTH && y >= 0 && y < VGA_HEIGHT) {
        volatile short *pixel_addr = (volatile short *)(VGA_BASE + (y * VGA_WIDTH + x) * 2);
        *pixel_addr = color;
    }
}


void clear_screen() {
    for (int y = 0; y < VGA_HEIGHT; y++) {
        for (int x = 0; x < VGA_WIDTH; x++) {
            VGA_pixel(x, y, 0x0000);  // Black color
        }
    }
}
