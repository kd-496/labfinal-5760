///////////////////////////////////////
/// 640x480 version! 16-bit color
/// This code will segfault the original
/// DE1 computer
/// compile with
/// gcc graphics_video_16bit_shooting.c -o gr -O2 -lm -pthread
///
///////////////////////////////////////
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <math.h>
#include <pthread.h>
#include <termios.h>

#define SDRAM_BASE            0xC0000000
#define SDRAM_SPAN            0x04000000
#define FPGA_CHAR_BASE        0xC9000000
#define FPGA_CHAR_SPAN        0x00002000
#define HW_REGS_BASE          0xff200000
#define HW_REGS_SPAN          0x00005000

#define G 6.67430e-11
#define M 5.972e24
#define dt 10

#define red (0+(0<<5)+(31<<11))
#define yellow (0+(63<<5)+(31<<11))
#define cyan (31+(63<<5)+(0<<11))
#define black (0x0000)
#define white (0xffff)
#define green (0+(63<<5)+(0<<11))

#define PLAYER_SIZE 10
#define BULLET_SIZE 4
#define BULLET_SPEED 15
#define NUM_BULLETS 10
#define NUM_TARGETS 5

#define VGA_PIXEL(x,y,color) do {\
    int *pixel_ptr;\
    pixel_ptr = (int *)((char *)vga_pixel_ptr + (((y) * 640 + (x)) << 1));\
    *(short *)pixel_ptr = (color);\
} while (0)

typedef struct {
    double x, y;   // Position (meters)
    double vx, vy; // Velocity (m/s)
    int px, py;    // Screen coordinates
    int size;
    int active;
    int color;
} Particle;

typedef struct {
    int x, y;
    int active;
    int color;
} Bullet;

void VGA_text(int, int, char *);
void VGA_text_clear();
void VGA_box(int, int, int, int, short);
void VGA_disc(int, int, int, short);

void *h2p_lw_virtual_base;
volatile unsigned int *vga_pixel_ptr = NULL;
void *vga_pixel_virtual_base;
volatile unsigned int *vga_char_ptr = NULL;
void *vga_char_virtual_base;

int fd;
struct termios old_tio, new_tio;
char key_pressed = '\0';

Particle player, targets[NUM_TARGETS];
Bullet bullets[NUM_BULLETS];

void init_bullet(Bullet *b, int x, int y, int color) {
    b->x = x;
    b->y = y;
    b->active = 1;
    b->color = color;
}

void fire_bullet() {
    for (int i = 0; i < NUM_BULLETS; i++) {
        if (!bullets[i].active) {
            init_bullet(&bullets[i], player.px, player.py - PLAYER_SIZE, white);
            break;
        }
    }
}

void move_bullets() {
    for (int i = 0; i < NUM_BULLETS; i++) {
        if (bullets[i].active) {
            VGA_box(bullets[i].x, bullets[i].y, bullets[i].x + BULLET_SIZE, bullets[i].y + BULLET_SIZE, black);
            bullets[i].y -= BULLET_SPEED;

            if (bullets[i].y < 0) {
                bullets[i].active = 0;
            } else {
                VGA_box(bullets[i].x, bullets[i].y, bullets[i].x + BULLET_SIZE, bullets[i].y + BULLET_SIZE, bullets[i].color);
            }
        }
    }
}

void detect_collision() {
    for (int i = 0; i < NUM_BULLETS; i++) {
        if (bullets[i].active) {
            for (int j = 0; j < NUM_TARGETS; j++) {
                if (targets[j].active &&
                    (abs(bullets[i].x - targets[j].px) < targets[j].size) &&
                    (abs(bullets[i].y - targets[j].py) < targets[j].size)) {
                    // Collision detected
                    targets[j].active = 0;
                    bullets[i].active = 0;
                    VGA_disc(targets[j].px, targets[j].py, targets[j].size, black); // Clear target
                }
            }
        }
    }
}

void init_particle(Particle *p, double altitude, double scale, int color, int size) {
    p->x = 6.371e6 + altitude;
    p->y = 0;
    p->vx = 0;
    p->vy = sqrt(G * M / p->x);
    p->px = (int)(p->x * scale) + 320;
    p->py = (int)(p->y * scale) + 240;
    p->size = size;
    p->active = 1;
    p->color = color;
}

void update_target(Particle *p, double center_x, double center_y) {
    if (p->active) {
        double angle = atan2(p->vy, p->vx) + 0.05; // Increment the angle
        p->vx = cos(angle) * 0.05;  // Circular motion
        p->vy = sin(angle) * 0.05;

        // Update position in circular orbit
        p->x += p->vx;
        p->y += p->vy;

        // Clear previous position
        VGA_disc(p->px, p->py, p->size, black);

        // Convert to screen coordinates relative to the orbit center
        p->px = (int)(p->x) + center_x;
        p->py = (int)(p->y) + center_y;

        // Draw new position
        VGA_disc(p->px, p->py, p->size, p->color);
    }
}

void update_game() {
    // Player controls and firing bullets
    VGA_disc(player.px, player.py, player.size, black);
    if (key_pressed == 'w') player.py -= 10;
    if (key_pressed == 's') player.py += 10;
    if (key_pressed == 'a') player.px -= 10;
    if (key_pressed == 'd') player.px += 10;
    if (key_pressed == ' ') {
        fire_bullet();
        key_pressed = '\0'; // Reset key press
    }

    // Redraw player
    VGA_disc(player.px, player.py, player.size, player.color);

    // Move and redraw bullets
    move_bullets();

    // Update all targets
    update_target(&targets[0], 200, 200);
    update_target(&targets[1], 400, 100);
    update_target(&targets[2], 500, 350);
    update_target(&targets[3], 100, 400);
    update_target(&targets[4], 300, 300);

    // Detect bullet collisions with targets
    detect_collision();
}

void *keyboard_thread(void *arg) {
    while (1) {
        key_pressed = getchar();
    }
    return NULL;
}

void VGA_text(int x, int y, char *text_ptr) {
    volatile char *character_buffer = (char *)vga_char_ptr;
    int offset = (y << 7) + x;
    while (*text_ptr) {
        *(character_buffer + offset) = *(text_ptr);
        ++text_ptr;
        ++offset;
    }
}

void VGA_text_clear() {
    volatile char *character_buffer = (char *)vga_char_ptr;
    int offset, x, y;
    for (x = 0; x < 79; x++) {
        for (y = 0; y < 59; y++) {
            offset = (y << 7) + x;
            *(character_buffer + offset) = ' ';
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

void VGA_disc(int x, int y, int r, short pixel_color) {
    int col, row;
    for (row = -r; row <= r; row++) {
        for (col = -r; col <= r; col++) {
            if (col * col + row * row <= r * r) {
                VGA_PIXEL(x + col, y + row, pixel_color);
            }
        }
    }
}

void init_game() {
    // Initialize keyboard settings
    tcgetattr(0, &old_tio);
    new_tio = old_tio;
    new_tio.c_lflag &= ~ICANON & ~ECHO;
    tcsetattr(0, TCSANOW, &new_tio);

    // Initialize player
    player.px = 320;
    player.py = 400;
    player.size = PLAYER_SIZE;
    player.color = green;
    player.active = 1;

    // Initialize targets
    for (int i = 0; i < NUM_TARGETS; i++) {
        targets[i].x = 100 + i * 100;  // Different centers
        targets[i].y = 100 + i * 50;
        targets[i].vx = 0.5 - i * 0.1;  // Different orbital velocities
        targets[i].vy = 0.5 - i * 0.1;
        targets[i].px = targets[i].x;
        targets[i].py = targets[i].y;
        targets[i].color = yellow;
        targets[i].size = 10;
        targets[i].active = 1;
    }

    // Initialize bullets
    for (int i = 0; i < NUM_BULLETS; i++) {
        bullets[i].x = 0;
        bullets[i].y = 0;
        bullets[i].active = 0;
        bullets[i].color = white;
    }
}

int main(void) {
    // Open /dev/mem
    if ((fd = open("/dev/mem", (O_RDWR | O_SYNC))) == -1) {
        printf("ERROR: could not open \"/dev/mem\"...\n");
        return 1;
    }

    // Map the lightweight bus base
    h2p_lw_virtual_base = mmap(NULL, HW_REGS_SPAN, PROT_READ | PROT_WRITE, MAP_SHARED, fd, HW_REGS_BASE);
    if (h2p_lw_virtual_base == MAP_FAILED) {
        printf("ERROR: mmap1() failed...\n");
        close(fd);
        return 1;
    }

    // Map the VGA character buffer base
    vga_char_virtual_base = mmap(NULL, FPGA_CHAR_SPAN, (PROT_READ | PROT_WRITE), MAP_SHARED, fd, FPGA_CHAR_BASE);
    if (vga_char_virtual_base == MAP_FAILED) {
        printf("ERROR: mmap2() failed...\n");
        close(fd);
        return 1;
    }

    vga_char_ptr = (unsigned int *)(vga_char_virtual_base);

    // Map the VGA pixel buffer base
    vga_pixel_virtual_base = mmap(NULL, SDRAM_SPAN, (PROT_READ | PROT WRITE), MAP_SHARED, fd, SDRAM_BASE);
    if (vga_pixel_virtual_base == MAP_FAILED) {
        printf("ERROR: mmap3() failed...\n");
        close(fd);
        return 1;
    }

    vga_pixel_ptr = (unsigned int *)(vga_pixel_virtual_base);

    // Initialize the game
    init_game();

    // Start the keyboard thread
    pthread_t keyboard_tid;
    pthread_create(&keyboard_tid, NULL, keyboard_thread, NULL);

    // Main game loop
    while (1) {
        update_game();
        usleep(50000);  // 20 FPS (50ms delay)
    }

    // Restore terminal settings
    tcsetattr(0, TCSANOW, &old_tio);

    // Clean up memory mapping
    close(fd);

    return 0;
}
