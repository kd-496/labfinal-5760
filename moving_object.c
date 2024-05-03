#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "system.h"  // Assuming a system header file with FPGA base addresses

#define G 6.67430e-11
#define M 5.972e24
#define dt 1

// Assuming memory-mapped I/O addresses for the VGA controller
#define VGA_CTRL_BASE 0xFF200000  // Base address for VGA controller
#define VGA_PIXEL_X_REG (VGA_CTRL_BASE + 0x00)  // X-coordinate
#define VGA_PIXEL_Y_REG (VGA_CTRL_BASE + 0x04)  // Y-coordinate
#define VGA_PIXEL_COLOR_REG (VGA_CTRL_BASE + 0x08)  // Color

void write_pixel(int x, int y, unsigned int color) {
    *(volatile unsigned int *)(VGA_PIXEL_X_REG) = x;
    *(volatile unsigned int *)(VGA_PIXEL_Y_REG) = y;
    *(volatile unsigned int *)(VGA_PIXEL_COLOR_REG) = color;
}

int main() {
    double x = 6.371e6 + 400000;  // Initial x position
    double y = 0;                 // Initial y position
    double eccentricity = 0.1;
    double perigee_velocity = sqrt((G * M * (1 + eccentricity)) / (x * (1 - eccentricity)));
    double vx = 0;
    double vy = perigee_velocity;

    int screen_x, screen_y;
    int center_x = 320; // Assuming a 640x480 screen
    int center_y = 240;
    double scale = 0.00001; // Scale to fit the orbit on the screen

    while (1) {
        double r = sqrt(x * x + y * y);
        double ax = -G * M * x / (r * r * r);
        double ay = -G * M * y / (r * r * r);
        vx += ax * dt;
        vy += ay * dt;
        x += vx * dt;
        y += vy * dt;

        // Convert simulation coordinates to VGA screen coordinates
        screen_x = center_x + (int)(x * scale);
        screen_y = center_y - (int)(y * scale); // Inverting y for VGA screen

        // Write the new position to the VGA
        write_pixel(screen_x, screen_y, 0xFFFFFF); // Drawing in white

        // Insert a delay or synchronization mechanism here if necessary
    }
    return 0;
}
