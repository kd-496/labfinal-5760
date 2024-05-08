///////////////////////////////////////
/// 640x480 version! 16-bit color
/// This code will segfault the original
/// DE1 computer
/// compile with
/// gcc graphics_video_16bit.c -o gr -O2 -lm
///
///////////////////////////////////////
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h> 
#include <sys/ipc.h> 
#include <sys/shm.h> 
#include <sys/mman.h> 
#include <sys/time.h> 
#include <math.h>
//#include "address_map_arm_brl4.h"

// video display
#define SDRAM_BASE            0xC0000000
#define SDRAM_END             0xC3FFFFFF
#define SDRAM_SPAN            0x04000000
// characters
#define FPGA_CHAR_BASE        0xC9000000 
#define FPGA_CHAR_END         0xC9001FFF
#define FPGA_CHAR_SPAN        0x00002000
/* Cyclone V FPGA devices */
#define HW_REGS_BASE          0xff200000
#define HW_REGS_SPAN          0x00005000 

// Constants for the orbital simulation
#define G 6.67430e-11  // Gravitational constant in m^3 kg^-1 s^-2
#define M 5.972e24     // Mass of Earth in kg
#define dt 10          // Time step in seconds

// graphics primitives
void VGA_text(int, int, char *);
void VGA_text_clear();
void VGA_box(int, int, int, int, short);
void VGA_rect(int, int, int, int, short);
void VGA_line(int, int, int, int, short);
void VGA_Vline(int, int, int, short);
void VGA_Hline(int, int, int, short);
void VGA_disc(int, int, int, short);
void VGA_circle(int, int, int, int);
// 16-bit primary colors
#define red         (0+(0<<5)+(31<<11))
#define yellow      (0+(63<<5)+(31<<11))
#define cyan        (31+(63<<5)+(0<<11))
#define black       (0x0000)
#define white       (0xffff)

#define VGA_PIXEL(x,y,color) do {\
    int *pixel_ptr;\
    pixel_ptr = (int *)((char *)vga_pixel_ptr + (((y) * 640 + (x)) << 1));\
    *(short *)pixel_ptr = (color);\
} while (0)

// the light weight bus base
void *h2p_lw_virtual_base;

// pixel buffer
volatile unsigned int *vga_pixel_ptr = NULL;
void *vga_pixel_virtual_base;

// character buffer
volatile unsigned int *vga_char_ptr = NULL;
void *vga_char_virtual_base;

// /dev/mem file id
int fd;

// Struct to store particle state information
typedef struct {
    double x, y;   // Position (meters)
    double vx, vy; // Velocity (m/s)
    double ax, ay; // Acceleration (m/s^2)
    int px, py;    // Screen coordinates
    int color;     // Particle color
} Particle;

// Initialize a particle's state
void init_particle(Particle *p, double altitude, double scale, int color) {
    p->x = 6.371e6 + altitude;  // Earth's radius + altitude
    p->y = 0;
    p->vx = 0;
    p->vy = sqrt(G * M / p->x);  // Initial velocity for circular orbit
    p->ax = 0;
    p->ay = 0;
    p->px = (int)(p->x * scale) + 320;
    p->py = (int)(p->y * scale) + 240;
    p->color = color;
}

// Update particle's position
void update_particle(Particle *p, double scale) {
    // Compute the radial distance
    double r = sqrt(p->x * p->x + p->y * p->y);

    // Compute acceleration components
    p->ax = -G * M * p->x / (r * r * r);
    p->ay = -G * M * p->y / (r * r * r);

    // Update velocity
    p->vx += p->ax * dt;
    p->vy += p->ay * dt;

    // Clear previous pixel
    VGA_PIXEL(p->px, p->py, black);

    // Update position
    p->x += p->vx * dt;
    p->y += p->vy * dt;

    // Convert to screen coordinates
    p->px = (int)(p->x * scale) + 320;
    p->py = (int)(p->y * scale) + 240;

    // Draw the new position
    VGA_PIXEL(p->px, p->py, p->color);
}

int main(void) {
    // === Need to mmap: =======================
    // FPGA_CHAR_BASE
    // FPGA_ONCHIP_BASE      
    // HW_REGS_BASE        

    // === Get FPGA addresses ==================
    // Open /dev/mem
    if ((fd = open("/dev/mem", (O_RDWR | O_SYNC))) == -1) {
        printf("ERROR: could not open \"/dev/mem\"...\n");
        return 1;
    }

    // Get virtual address that maps to physical
    h2p_lw_virtual_base = mmap(NULL, HW_REGS_SPAN, (PROT_READ | PROT_WRITE), MAP_SHARED, fd, HW_REGS_BASE);
    if (h2p_lw_virtual_base == MAP_FAILED) {
        printf("ERROR: mmap1() failed...\n");
        close(fd);
        return 1;
    }

    // === Get VGA char addr =====================
    // Get virtual address that maps to physical
    vga_char_virtual_base = mmap(NULL, FPGA_CHAR_SPAN, (PROT_READ | PROT_WRITE), MAP_SHARED, fd, FPGA_CHAR_BASE);
    if (vga_char_virtual_base == MAP_FAILED) {
        printf("ERROR: mmap2() failed...\n");
        close(fd);
        return 1;
    }

    // Get the address that maps to the FPGA LED control 
    vga_char_ptr = (unsigned int *)(vga_char_virtual_base);

    // === Get VGA pixel addr ====================
    // Get virtual address that maps to physical
    vga_pixel_virtual_base = mmap(NULL, SDRAM_SPAN, (PROT_READ | PROT_WRITE), MAP_SHARED, fd, SDRAM_BASE);
    if (vga_pixel_virtual_base == MAP_FAILED) {
        printf("ERROR: mmap3() failed...\n");
        close(fd);
        return 1;
    }

    // Get the address that maps to the FPGA pixel buffer
    vga_pixel_ptr = (unsigned int *)(vga_pixel_virtual_base);

    // ===========================================

    /* Create a message to be displayed on the VGA
       and LCD displays */
    char text_top_row[40] = "DE1-SoC ARM/FPGA\0";
    char text_bottom_row[40] = "Cornell ece5760\0";
    char text_next[40] = "Graphics primitives\0";

    // Clear the screen
    VGA_box(0, 0, 639, 479, black);
    // Clear the text
    VGA_text_clear();
    // Write text
    VGA_text(10, 1, text_top_row);
    VGA_text(10, 2, text_bottom_row);
    VGA_text(10, 3, text_next);

    // Initialize conditions for two particles
    Particle p1, p2;
    double scale = 0.000001;  // Adjust scale for your display resolution and desired orbit size

    init_particle(&p1, 400000, scale, yellow);
    init_particle(&p2, 800000, scale, cyan);

    while (1) {
        update_particle(&p1, scale);
        update_particle(&p2, scale);
        usleep(5000);  // Delay for visibility
    }

    close(fd);
    return 0;
}

/****************************************************************************************
 * Subroutine to send a string of text to the VGA monitor
****************************************************************************************/
void VGA_text(int x, int y, char *text_ptr) {
    volatile char *character_buffer = (char *)vga_char_ptr;  // VGA character buffer
    int offset;
    /* Assume that the text string fits on one line */
    offset = (y << 7) + x;
    while (*text_ptr) {
        // Write to the character buffer
        *(character_buffer + offset) = *(text_ptr);
        ++text_ptr;
        ++offset;
    }
}

/****************************************************************************************
 * Subroutine to clear text to the VGA monitor
****************************************************************************************/
void VGA_text_clear() {
    volatile char *character_buffer = (char *)vga_char_ptr;  // VGA character buffer
    int offset, x, y;
    for (x = 0; x < 79; x++) {
        for (y = 0; y < 59; y++) {
            /* Assume that the text string fits on one line */
            offset = (y << 7) + x;
            // Write to the character buffer
            *(character_buffer + offset) = ' ';
        }
    }
}

/****************************************************************************************
 * Draw a filled rectangle on the VGA monitor
****************************************************************************************/
#define SWAP(X,Y) do{int temp=X; X=Y; Y=temp;}while(0)

void VGA_box(int x1, int y1, int x2, int y2, short pixel_color) {
    int row, col;

    /* Check and fix box coordinates to be valid */
    if (x1 > 639) x1 = 639;
    if (y1 > 479) y1 = 479;
    if (x2 > 639) x2 = 639;
    if (y2 > 479) y2 = 479;
    if (x1 < 0) x1 = 0;
    if (y1 < 0) y1 = 0;
    if (x2 < 0) x2 = 0;
    if (y2 < 0) y2 = 0;
    if (x1 > x2) SWAP(x1, x2);
    if (y1 > y2) SWAP(y1, y2);
    for (row = y1; row <= y2; row++)
        for (col = x1; col <= x2; ++col) {
            VGA_PIXEL(col, row, pixel_color);
        }
}

/****************************************************************************************
 * Draw a line on the VGA monitor
****************************************************************************************/
void VGA_line(int x1, int y1, int x2, int y2, short c) {
    int e;
    signed int dx, dy, j, temp;
    signed int s1, s2, xchange;
    signed int x, y;

    /* Check and fix line coordinates to be valid */
    if (x1 > 639) x1 = 639;
    if (y1 > 479) y1 = 479;
    if (x2 > 639) x2 = 639;
    if (y2 > 479) y2 = 479;
    if (x1 < 0) x1 = 0;
    if (y1 < 0) y1 = 0;
    if (x2 < 0) x2 = 0;
    if (y2 < 0) y2 = 0;

    x = x1;
    y = y1;

    // Take absolute value
    if (x2 < x1) {
        dx = x1 - x2;
        s1 = -1;
    } else if (x2 == x1) {
        dx = 0;
        s1 = 0;
    } else {
        dx = x2 - x1;
        s1 = 1;
    }

    if (y2 < y1) {
        dy = y1 - y2;
        s2 = -1;
    } else if (y2 == y1) {
        dy = 0;
        s2 = 0;
    } else {
        dy = y2 - y1;
        s2 = 1;
    }

    xchange = 0;

    if (dy > dx) {
        temp = dx;
        dx = dy;
        dy = temp;
        xchange = 1;
    }

    e = ((int)dy << 1) - dx;

    for (j = 0; j <= dx; j++) {
        VGA_PIXEL(x, y, c);

        if (e >= 0) {
            if (xchange == 1) x = x + s1;
            else y = y + s2;
            e = e - ((int)dx << 1);
        }

        if (xchange == 1) y = y + s2;
        else x = x + s1;

        e = e + ((int)dy << 1);
    }
}
