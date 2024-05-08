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
Particle player, enemy1, enemy2;
Bullet bullets[10];

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
    for (int i = 0; i < 10; i++) {
        if (bullets[i].active) {
            if (enemy1.active && bullets[i].x >= enemy1.px - enemy1.size && bullets[i].x <= enemy1.px + enemy1.size &&
                bullets[i].y >= enemy1.py - enemy1.size && bullets[i].y <= enemy1.py + enemy1.size) {
                enemy1.active = 0;
                bullets[i].active = 0;
                VGA_disc(enemy1.px, enemy1.py, enemy1.size, black);
            }

            if (enemy2.active && bullets[i].x >= enemy2.px - enemy2.size && bullets[i].x <= enemy2.px + enemy2.size &&
                bullets[i].y >= enemy2.py - enemy2.size && bullets[i].y <= enemy2.py + enemy2.size) {
                enemy2.active = 0;
                bullets[i].active = 0;
                VGA_disc(enemy2.px, enemy2.py, enemy2.size, black);
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

void update_particle(Particle *p, double scale) {
    double r = sqrt(p->x * p->x + p->y * p->y);
    p->vx += (-G * M * p->x / (r * r * r)) * dt;
    p->vy += (-G * M * p->y / (r * r * r)) * dt;

    if (p->active) {
        VGA_disc(p->px, p->py, p->size, black);
        p->x += p->vx * dt;
        p->y += p->vy * dt;
        p->px = (int)(p->x * scale) + 320;
        p->py = (int)(p->y * scale) + 240;
        VGA_disc(p->px, p->py, p->size, p->color);
    }
}

void update_player_position() {
    VGA_disc(player.px, player.py, player.size, black);
    if (key_pressed == 'a' && player.px > 0 + PLAYER_SIZE) player.px -= 10;
    if (key_pressed == 'd' && player.px < 640 - PLAYER_SIZE) player.px += 10;
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
        if (key_pressed == 's') fire_bullet();
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
    init_particle(&player, 0, scale, green, PLAYER_SIZE);
    player.px = 320;
    player.py = 400;

    init_particle(&enemy1, 400000, scale, yellow, 12);
    init_particle(&enemy2, 800000, scale, cyan, 12);

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
        update_particle(&enemy1, scale);
        update_particle(&enemy2, scale);
        move_bullets();
        detect_collision();
        usleep(50000);  // 50 ms delay
    }

    tcsetattr(0, TCSANOW, &old_tio);
    close(fd);
    return 0;
}
