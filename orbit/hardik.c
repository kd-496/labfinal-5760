#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <math.h>
#include <termios.h>
#include <pthread.h>

#define SDRAM_BASE            0xC0000000
#define SDRAM_SPAN            0x04000000
#define FPGA_CHAR_BASE        0xC9000000
#define FPGA_CHAR_SPAN        0x00002000
#define HW_REGS_BASE          0xff200000
#define HW_REGS_SPAN          0x00005000

#define G 6.67430e-11
#define M 5.972e24
#define dt 10

#define red         (0+(0<<5)+(31<<11))
#define yellow      (0+(63<<5)+(31<<11))
#define cyan        (31+(63<<5)+(0<<11))
#define black       (0x0000)
#define white       (0xffff)
#define gray        (15+(31<<5)+(51<<11))
#define green       (0+(63<<5)+(0<<11))
#define blue        (31+(0<<5)+(0<<11))
#define magenta     (31+(0<<5)+(31<<11))

#define PLAYER_SIZE 10
#define BULLET_SIZE 4
#define BULLET_SPEED 15

#define VGA_PIXEL(x,y,color) do {\
    int *pixel_ptr;\
    pixel_ptr = (int *)((char *)vga_pixel_ptr + (((y) * 640 + (x)) << 1));\
    *(short *)pixel_ptr = (color);\
} while (0)

typedef struct {
    double x, y;
    double vx, vy;
    int px, py;
    int size;
    int active;
    int color;
    double angle;  // Additional attribute for orbital motion
    double radius; // Radius for orbital motion
    int center_x, center_y; // Center for orbital motion
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
Particle player, enemies[4]; // Array to hold multiple enemies
Bullet bullets[10];
int score = 0; // Player score
char score_text[50];

void init_bullet(Bullet *b, int x, int y, int color) {
    b->x = x;
    b->y = y;
    b->active = 1;
    b->color = color;
}

void fire_bullet() {
    for (int i = 0; i < 10; i++) {
        if (!bullets[i].active) {
            init_bullet(&bullets[i], player.px, player.py - PLAYER_SIZE, white);
            break;
        }
    }
}

void move_bullets() {
    for (int i = 0; i < 10; i++) {
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
    for (int b = 0; b < 10; b++) {
        if (bullets[b].active) {
            for (int e = 0; e < 4; e++) {
                if (enemies[e].active && bullets[b].x >= enemies[e].px - enemies[e].size && bullets[b].x <= enemies[e].px + enemies[e].size &&
                    bullets[b].y >= enemies[e].py - enemies[e].size && bullets[b].y <= enemies[e].py + enemies[e].size) {
                    enemies[e].active = 0;
                    bullets[b].active = 0;
                    score += 100; // Increase score
                    VGA_disc(enemies[e].px, enemies[e].py, enemies[e].size, black); // Clear the enemy from the screen
                }
            }
        }
    }
}

void display_score() {
    sprintf(score_text, "Score: %d", score);
    VGA_text(10, 4, score_text);
}

void check_game_over() {
    int all_inactive = 1;
    for (int i = 0; i < 4; i++) {
        if (enemies[i].active) {
            all_inactive = 0;
            break;
        }
    }
    if (all_inactive) {
        VGA_text_clear();
        VGA_text(10, 15, "Game Over! Final Score: ");
        VGA_text(32, 15, score_text);
    }
}

void init_particle(Particle *p, double altitude, double scale, int color, int size, int cx, int cy, double rad) {
    p->x = cx;
    p->y = cy;
    p->vx = 0;
    p->vy = 0;
    p->px = cx;
    p->py = cy;
    p->size = size;
    p->active = 1;
    p->color = color;
    p->angle = 0;  // Initial angle
    p->radius = rad;  // Orbit radius
    p->center_x = cx; // Center X
    p->center_y = cy; // Center Y
}

void update_particle_orbit(Particle *p, double interval) {
    if (p->active) {
        p->angle += interval; // Update angle for simplicity, assumes circular motion
        p->x = p->center_x + p->radius * cos(p->angle);
        p->y = p->center_y + p->radius * sin(p->angle);
        VGA_disc(p->px, p->py, p->size, black); // Clear old position
        p->px = (int)(p->x);
        p->py = (int)(p->y);
        VGA_disc(p->px, p->py, p->size, p->color); // Draw new position
    }
}

void update_player_position() {
    VGA_disc(player.px, player.py, player.size, black);
    if (key_pressed == 'a' && player.px > 0 + PLAYER_SIZE) player.px -= 10;
    if (key_pressed == 'd' && player.px < 640 - PLAYER_SIZE) player.px += 10;
    if (key_pressed == 'w' && player.py > 0 + PLAYER_SIZE) player.py -= 10;
    if (key_pressed == 's' && player.py < 480 - PLAYER_SIZE) player.py += 10;
    VGA_disc(player.px, player.py, player.size, player.color);
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

void *keyboard_thread(void *arg) {
    while (1) {
        key_pressed = getchar();
        if (key_pressed == 'f') fire_bullet();
    }
    return NULL;
}

int main(void) {
    if ((fd = open("/dev/mem", (O_RDWR | O_SYNC))) == -1) {
        printf("ERROR: could not open \"/dev/mem\"...\n");
        return 1;
    }

    h2p_lw_virtual_base = mmap(NULL, HW_REGS_SPAN, (PROT_READ | PROT_WRITE), MAP_SHARED, fd, HW_REGS_BASE);
    if (h2p_lw_virtual_base == MAP_FAILED) {
        printf("ERROR: mmap1() failed...\n");
        close(fd);
        return 1;
    }

    vga_char_virtual_base = mmap(NULL, FPGA_CHAR_SPAN, (PROT_READ | PROT_WRITE), MAP_SHARED, fd, FPGA_CHAR_BASE);
    if (vga_char_virtual_base == MAP_FAILED) {
        printf("ERROR: mmap2() failed...\n");
        close(fd);
        return 1;
    }

    vga_char_ptr = (unsigned int *)(vga_char_virtual_base);

    vga_pixel_virtual_base = mmap(NULL, SDRAM_SPAN, (PROT_READ | PROT_WRITE), MAP_SHARED, fd, SDRAM_BASE);
    if (vga_pixel_virtual_base == MAP_FAILED) {
        printf("ERROR: mmap3() failed...\n");
        close(fd);
        return 1;
    }

    vga_pixel_ptr = (unsigned int *)(vga_pixel_virtual_base);

    char text_top_row[40] = "DE1-SoC ARM/FPGA\0";
    char text_bottom_row[40] = "Cornell ece5760\0";
    char text_next[40] = "Shooting Game\0";

    VGA_box(0, 0, 639, 479, black);
    VGA_text_clear();
    VGA_text(10, 1, text_top_row);
    VGA_text(10, 2, text_bottom_row);
    VGA_text(10, 3, text_next);

    double scale = 0.000001;
    init_particle(&player, 0, scale, green, PLAYER_SIZE, 320, 440, 0); // Place player at the bottom center
    init_particle(&enemies[0], 400000, scale, yellow, 12, 200, 240, 50); // Enemy1
    init_particle(&enemies[1], 800000, scale, cyan, 12, 400, 240, 100); // Enemy2
    init_particle(&enemies[2], 600000, scale, blue, 15, 100, 120, 75);  // Enemy3
    init_particle(&enemies[3], 700000, scale, magenta, 15, 500, 360, 60); // Enemy4

    for (int i = 0; i < 10; i++) bullets[i].active = 0;

    // Initialize keyboard settings
    tcgetattr(0, &old_tio);
    new_tio = old_tio;
    new_tio.c_lflag &= ~ICANON;
    new_tio.c_lflag &= ~ECHO;
    new_tio.c_cc[VMIN] = 1;
    new_tio.c_cc[VTIME] = 0;
    tcsetattr(0, TCSANOW, &new_tio);

    pthread_t tid;
    pthread_create(&tid, NULL, keyboard_thread, NULL);

    while (1) {
        update_player_position();
        for (int i = 0; i < 4; i++) {
            update_particle_orbit(&enemies[i], 0.05);  // Adjust dt for orbital motion
        }
        move_bullets();
        detect_collision();
        display_score();
        check_game_over();
        usleep(50000);  // 50 ms delay
    }

    tcsetattr(0, TCSANOW, &old_tio);
    close(fd);
    return 0;
}
