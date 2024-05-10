#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <math.h>
#include <termios.h>
#include <pthread.h>
#include <sys/time.h>

#define SDRAM_BASE            0xC0000000
#define SDRAM_SPAN            0x04000000
#define FPGA_CHAR_BASE        0xC9000000
#define FPGA_CHAR_SPAN        0x00002000
#define HW_REGS_BASE          0xff200000
#define HW_REGS_SPAN          0x00005000

// Define colors and dimensions
#define red                   (0+(0<<5)+(31<<11))
#define yellow                (0+(63<<5)+(31<<11))
#define cyan                  (31+(63<<5)+(0<<11))
#define black                 (0x0000)
#define white                 (0xffff)
#define gray                  (15+(31<<5)+(51<<11))
#define green                 (0+(63<<5)+(0<<11))
#define blue                  (31+(0<<5)+(0<<11))
#define magenta               (31+(0<<5)+(31<<11))

#define PLAYER_SIZE           10
#define BULLET_SIZE           4
#define BULLET_SPEED          15

#define x_size                640
#define y_size                480

// Define macros for pixel operations
#define VGA_PIXEL(x, y, color) do { \
    int *pixel_ptr; \
    pixel_ptr = (int *)((char *)vga_pixel_ptr + (((y) * x_size + (x)) << 1)); \
    *(short *)pixel_ptr = (color); \
} while (0)

typedef struct {
    double x, y;
    int px, py;
    int size;
    int active;
    int color;
} Particle;

typedef struct {
    int x, y;
    int active;
    int color;
} Bullet;

// Function prototypes
void VGA_text(int, int, char *);
void VGA_text_clear();
void VGA_box(int, int, int, int, short);
void VGA_disc(int, int, int, short);
void draw_start_screen();
void wait_for_spacebar();
void init_game();
void game_loop();
void close_game();

// Global variables for direct access to hardware
void *h2p_lw_virtual_base;
volatile unsigned int *vga_pixel_ptr = NULL;
void *vga_pixel_virtual_base;
volatile unsigned int *vga_char_ptr = NULL;
void *vga_char_virtual_base;

int fd;
struct termios old_tio, new_tio;
char key_pressed = '\0';

Particle player;
Particle enemies[7];
Bullet bullets[10];
int score = 0;
char score_text[50];

void VGA_text(int x, int y, char *text) {
    volatile char *character_buffer = (char *)vga_char_ptr + (y * 64 + x);
    while (*text) {
        *character_buffer++ = *text++;
    }
}

void VGA_text_clear() {
    volatile char *character_buffer = (char *)vga_char_ptr;
    for (int i = 0; i < 64 * 48; ++i) {
        *character_buffer++ = ' ';
    }
}

void draw_start_screen() {
    VGA_text_clear();
    VGA_text(28, 15, "Space-Shooter");
    VGA_text(20, 20, "Press Spacebar to start");
}
void init_particle(Particle *p, int color, int size) {
    p->x = 0;
    p->y = 0;
    p->px = 0;
    p->py = 0;
    p->size = size;
    p->active = 1;
    p->color = color;
}

void wait_for_spacebar() {
    struct termios info;
    tcgetattr(0, &info);
    info.c_lflag &= ~ICANON;
    info.c_cc[VMIN] = 1;
    tcsetattr(0, TCSANOW, &info);
    while (getchar() != ' ') { usleep(10000); }
    tcsetattr(0, TCSANOW, &old_tio);
}

void init_game() {
    // Initialize the player and enemies
    init_particle(&player, green, PLAYER_SIZE);
    player.px = 320;
    player.py = 440; // Place player at the bottom center

    for (int i = 0; i < 7; i++) {
        init_particle(&enemies[i], (i % 2 == 0) ? magenta : blue, 10 + i); // Different enemies
        enemies[i].px = 50 * i + 100;
        enemies[i].py = 40 * (i + 1);
    }

    for (int i = 0; i < 10; i++) {
        bullets[i].active = 0;
    }
}

void game_loop() {
    // Main game loop logic here
}

void close_game() {
    // Clean up resources
    tcsetattr(0, TCSANOW, &old_tio); // Restore terminal settings
    munmap(vga_pixel_virtual_base, SDRAM_SPAN);
    munmap(vga_char_virtual_base, FPGA_CHAR_SPAN);
    munmap(h2p_lw_virtual_base, HW_REGS_SPAN);
    close(fd);
}

int main(void) {
    // Open device memory and map I/O regions
    if ((fd = open("/dev/mem", (O_RDWR | O_SYNC))) == -1) {
        printf("ERROR: could not open \"/dev/mem\"...\n");
        return 1;
    }
    init_hardware();
    draw_start_screen();
    wait_for_spacebar();
    init_game();
    game_loop();
    close_game();
    return 0;
}
