#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <termios.h>
#include <pthread.h>

#define SDRAM_BASE            0xC0000000
#define SDRAM_SPAN            0x04000000
#define FPGA_CHAR_BASE        0xC9000000
#define FPGA_CHAR_SPAN        0x00002000
#define HW_REGS_BASE          0xff200000
#define HW_REGS_SPAN          0x00005000

#define x_size                640
#define y_size                480
#define black                 (0x0000)
#define white                 (0xffff)

#define VGA_PIXEL(x, y, color) do { \
    int *pixel_ptr; \
    pixel_ptr = (int *)((char *)vga_pixel_ptr + (((y) * x_size + (x)) << 1)); \
    *(short *)pixel_ptr = (color); \
} while (0)

void *h2p_lw_virtual_base;
volatile unsigned int *vga_pixel_ptr = NULL;
void *vga_pixel_virtual_base;
volatile unsigned int *vga_char_ptr = NULL;
void *vga_char_virtual_base;

int fd;
struct termios old_tio, new_tio;
char key_pressed = '\0';

void VGA_text(int, int, char *);
void VGA_text_clear();
void VGA_box(int, int, int, int, short);
void VGA_draw_char(int, int, char, short);
void VGA_draw_string(int, int, char *, short);
void VGA_draw_space_shooter(int, int, short);

void VGA_text(int x, int y, char *text_ptr) {
    int offset = (y * 80 + x) << 1;
    volatile char *char_ptr = (char *)vga_char_ptr + offset;
    while (*text_ptr) {
        *char_ptr++ = *text_ptr++;
    }
}

void VGA_text_clear() {
    int i, j;
    volatile char *char_ptr = (char *)vga_char_ptr;
    for (i = 0; i < 60; i++) {
        for (j = 0; j < 80; j++) {
            *char_ptr++ = ' ';
        }
    }
}

void VGA_box(int x1, int y1, int x2, int y2, short pixel_color) {
    int row, col;
    for (row = y1; row <= y2; row++) {
        for (col = x1; col <= x2; ++col) {
            VGA_PIXEL(col, row, pixel_color);
        }
    }
}

void VGA_draw_char(int x, int y, char ch, short pixel_color) {
    static const char font[26][5] = {
        {0x7E, 0x11, 0x11, 0x11, 0x7E}, // A
        {0x7F, 0x49, 0x49, 0x49, 0x36}, // B
        {0x3E, 0x41, 0x41, 0x41, 0x22}, // C
        {0x7F, 0x41, 0x41, 0x22, 0x1C}, // D
        {0x7F, 0x49, 0x49, 0x49, 0x41}, // E
        {0x7F, 0x09, 0x09, 0x09, 0x01}, // F
        {0x3E, 0x41, 0x49, 0x49, 0x7A}, // G
        {0x7F, 0x08, 0x08, 0x08, 0x7F}, // H
        {0x00, 0x41, 0x7F, 0x41, 0x00}, // I
        {0x20, 0x40, 0x41, 0x3F, 0x01}, // J
        {0x7F, 0x08, 0x14, 0x22, 0x41}, // K
        {0x7F, 0x40, 0x40, 0x40, 0x40}, // L
        {0x7F, 0x02, 0x0C, 0x02, 0x7F}, // M
        {0x7F, 0x04, 0x08, 0x10, 0x7F}, // N
        {0x3E, 0x41, 0x41, 0x41, 0x3E}, // O
        {0x7F, 0x09, 0x09, 0x09, 0x06}, // P
        {0x3E, 0x41, 0x51, 0x21, 0x5E}, // Q
        {0x7F, 0x09, 0x19, 0x29, 0x46}, // R
        {0x46, 0x49, 0x49, 0x49, 0x31}, // S
        {0x01, 0x01, 0x7F, 0x01, 0x01}, // T
        {0x3F, 0x40, 0x40, 0x40, 0x3F}, // U
        {0x1F, 0x20, 0x40, 0x20, 0x1F}, // V
        {0x7F, 0x20, 0x18, 0x20, 0x7F}, // W
        {0x63, 0x14, 0x08, 0x14, 0x63}, // X
        {0x07, 0x08, 0x70, 0x08, 0x07}, // Y
        {0x61, 0x51, 0x49, 0x45, 0x43}  // Z
    };

    if (ch >= 'A' && ch <= 'Z') {
        const char *bitmap = font[ch - 'A'];
        for (int i = 0; i < 5; ++i) {
            for (int j = 0; j < 8; ++j) {
                if (bitmap[i] & (1 << j)) {
                    VGA_PIXEL(x + i, y + j, pixel_color);
                } else {
                    VGA_PIXEL(x + i, y + j, black);
                }
            }
        }
    }
}

void VGA_draw_string(int x, int y, char *str, short pixel_color) {
    while (*str) {
        VGA_draw_char(x, y, *str++, pixel_color);
        x += 6;
    }
}

void VGA_draw_space_shooter(int x, int y, short pixel_color) {
    VGA_draw_string(x, y, "SPACE", pixel_color);
    VGA_draw_string(x, y + 16, "SHOOTER", pixel_color);
}

void *keyboard_thread(void *arg) {
    struct termios new_settings;
    tcgetattr(0, &old_tio);
    new_settings = old_tio;
    new_settings.c_lflag &= ~(ICANON | ECHO);
    new_settings.c_cc[VMIN] = 1;
    new_settings.c_cc[VTIME] = 0;
    tcsetattr(0, TCSANOW, &new_settings);

    while (1) {
        key_pressed = getchar();
    }

    tcsetattr(0, TCSANOW, &old_tio);
    return NULL;
}

void show_home_screen() {
    VGA_text_clear();
    VGA_box(0, 0, 639, 479, black);
    VGA_draw_space_shooter(80, 100, white);
    VGA_text(15, 20, "Press Spacebar to start");
}

int main(void) {
    fd = open("/dev/mem", (O_RDWR | O_SYNC));
    if (fd == -1) {
        printf("ERROR: could not open \"/dev/mem\"...\n");
        return 1;
    }

    // Map SDRAM and VGA buffer
    vga_pixel_virtual_base = mmap(NULL, SDRAM_SPAN, PROT_READ | PROT_WRITE, MAP_SHARED, fd, SDRAM_BASE);
    if (vga_pixel_virtual_base == MAP_FAILED) {
        printf("ERROR: mmap() failed...\n");
        close(fd);
        return 1;
    }
    vga_pixel_ptr = (unsigned int *)(vga_pixel_virtual_base);

    // Map VGA character buffer
    vga_char_virtual_base = mmap(NULL, FPGA_CHAR_SPAN, PROT_READ | PROT_WRITE, MAP_SHARED, fd, FPGA_CHAR_BASE);
    if (vga_char_virtual_base == MAP_FAILED) {
        printf("ERROR: mmap() failed...\n");
        close(fd);
        return 1;
    }
    vga_char_ptr = (unsigned int *)(vga_char_virtual_base);

    // Show the home screen
    show_home_screen();

    // Wait for spacebar to start the game
    do {
        key_pressed = getchar();
    } while (key_pressed != ' ');

    close(fd);
    return 0;
}
