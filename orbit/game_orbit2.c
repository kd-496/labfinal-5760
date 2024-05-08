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
#include <sys/mman.h>
#include <sys/time.h>
#include <math.h>
#include <termios.h>
#include <pthread.h>
#include <time.h>

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

#define PLAYER_SIZE 10
#define BULLET_SIZE 4
#define BULLET_SPEED 15
#define ENEMY_COUNT 5
#define BULLET_COUNT 10

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
    double center_x, center_y; // Orbital center
    double angular_speed; // Angular speed for orbital movement
    double angle; // Current angle
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
Particle player;
Particle enemies[ENEMY_COUNT];
Bullet bullets[BULLET_COUNT];

void init_bullet(Bullet *b, int x, int y, int color) {
    b->x = x;
    b->y = y;
    b->active = 1;
    b->color = color;
}

void fire_bullet() {
    for (int i = 0; i < BULLET_COUNT; i++) {
        if (!bullets[i].active) {
            init_bullet(&bullets[i], player.px, player.py - PLAYER_SIZE, white);
            break;
        }
    }
}

void move_bullets() {
    for (int i = 0; i < BULLET_COUNT; i++) {
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
    for (int i = 0; i < BULLET_COUNT; i++) {
        if (bullets[i].active) {
            for (int j = 0; j < ENEMY_COUNT; j++) {
                if (enemies[j].active && bullets[i].x >= enemies[j].px - enemies[j].size && bullets[i].x <= enemies[j].px + enemies[j].size &&
                    bullets[i].y >= enemies[j].py - enemies[j].size && bullets[i].y <= enemies[j].py + enemies[j].size) {
                    enemies[j].active = 0;
                    bullets[i].active = 0;
                    VGA_disc(enemies[j].px, enemies[j].py, enemies[j].size, black);
                }
            }
        }
    }
}

void init_orbital_enemy(Particle *p, double scale, int color, int size, double center_x, double center_y, double radius, double angular_speed) {
    p->x = center_x + radius;
    p->y = center_y;
    p->vx = 0;
    p->vy = 0;
    p->px = (int)(p->x * scale);
    p->py = (int)(p->y * scale);
    p->size = size;
    p->active = 1;
    p->color = color;
    p->center_x = center_x;
    p->center_y = center_y;
    p->angular_speed = angular_speed;
    p->angle = 0;
}

void update_orbital_enemy(Particle *p, double scale) {
    if (p->active) {
        VGA_disc(p->px, p->py, p->size, black);
        p->angle += p->angular_speed * dt / 1000.0; // Increase angle
        p->x = p->center_x + cos(p->angle) * (640 / 2 - p->size);
        p->y = p->center_y + sin(p->angle) * (480 / 2 - p->size);
        p->px = (int)(p->x * scale);
        p->py = (int)(p->y * scale);
        VGA_disc(p->px, p->py, p->size, p->color);
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

    double scale = 1.0;
    player.px = 320;
    player.py = 400;
    player.size = PLAYER_SIZE;
    player.color = green;

    srand(time(NULL));
    for (int i = 0; i < ENEMY_COUNT; i++) {
        double center_x = rand() % (640 - 100) + 50;
        double center_y = rand() % (480 - 100) + 50;
        double radius = rand() % 50 + 30;
        double angular_speed = (rand() % 10 + 10) / 1000.0; // Increased speed
        int color = (i % 2 == 0) ? yellow : cyan;
        init_orbital_enemy(&enemies[i], scale, color, 8, center_x, center_y, radius, angular_speed); // Reduced size
    }

    for (int i = 0; i < BULLET_COUNT; i++) bullets[i].active = 0;

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
        for (int i = 0; i < ENEMY_COUNT; i++) {
            update_orbital_enemy(&enemies[i], scale);
        }
        move_bullets();
        detect_collision();
        usleep(50000);  // 50 ms delay
    }

    tcsetattr(0, TCSANOW, &old_tio);
    close(fd);
    return 0;
}
