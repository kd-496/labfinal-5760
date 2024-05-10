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

void VGA_text(int, int, char *);
void VGA_text_clear();
void VGA_box(int, int, int, int, short);
void VGA_disc(int, int, int, short);

void *h2p_lw_virtual_base;
volatile unsigned int *vga_pixel_ptr = NULL;
void *vga_pixel_virtual_base;
volatile unsigned int *vga_char_ptr = NULL;
void *vga_char_virtual_base;
volatile unsigned int * enemy_pos_x_ptr = NULL;
volatile unsigned int * enemy_pos_y_ptr = NULL;

int fd;
struct termios old_tio, new_tio;
char key_pressed = '\0';
Particle player;
Particle enemies[7];
Bullet bullets[10];

int score = 0;
char score_text[50];
char time_text[50];

struct timeval start, end;
long time_used;

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
            for (int e = 0; e < 7; e++) {
                if (enemies[e].active && bullets[b].x >= enemies[e].px - enemies[e].size &&
                    bullets[b].x <= enemies[e].px + enemies[e].size &&
                    bullets[b].y >= enemies[e].py - enemies[e].size &&
                    bullets[b].y <= enemies[e].py + enemies[e].size) {
                    enemies[e].active = 0;
                    bullets[b].active = 0;
                    score += 100;
                    VGA_disc(enemies[e].px, enemies[e].py, enemies[e].size, black);
                }
            }
        }
    }
}

void display_score() {
    sprintf(score_text, "Score: %d", score);
    VGA_text(10, 4, score_text);
}

int check_game_over() {
    int all_inactive = 1;
    for (int i = 0; i < 7; i++) {
        if (enemies[i].active) {
            all_inactive = 0;
            break;
        }
    }
    if (all_inactive) {
        VGA_text_clear();
        VGA_text(10, 15, "Game Over! Final Score: ");
        VGA_text(32, 15, score_text);
        return 1;
    }
    return 0;
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

void update_enemies() {
    if (enemy_pos_x_ptr == NULL || enemy_pos_y_ptr == NULL) {
        printf("Enemy position pointer is not initialized.\n");
        return;
    }

    for (int i = 0; i < 7; i++) {
        if (enemies[i].active) {
            VGA_disc(enemies[i].px, enemies[i].py, enemies[i].size, enemies[i].color);
            enemies[i].px = *(enemy_pos_x_ptr) + 20 * i;
            enemies[i].py = *(enemy_pos_y_ptr) + 5 * i;
        }
    }
}

void update_player_position() {
    VGA_disc(player.px, player.py, player.size, black);
    if (key_pressed == 'a' && player.px > 0 + PLAYER_SIZE) player.px -= 10;
    if (key_pressed == 'd' && player.px < x_size - PLAYER_SIZE) player.px += 10;
    if (key_pressed == 'w' && player.py > 0 + PLAYER_SIZE) player.py -= 10;
    if (key_pressed == 's' && player.py < y_size - PLAYER_SIZE) player.py += 10;
    VGA_disc(player.px, player.py, player.size, player.color);
}

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
    struct termios new_settings;
    tcgetattr(0, &old_tio);
    new_settings = old_tio;
    new_settings.c_lflag &= ~(ICANON | ECHO);
    new_settings.c_cc[VMIN] = 1;
    new_settings.c_cc[VTIME] = 0;
    tcsetattr(0, TCSANOW, &new_settings);

    while (1) {
        key_pressed = getchar();
        if (key_pressed == 'f') {
            fire_bullet();
        }
    }

    tcsetattr(0, TCSANOW, &old_tio);
    return NULL;
}

int main(void) {
    fd = open("/dev/mem", (O_RDWR | O_SYNC));
    if (fd == -1) {
        printf("ERROR: could not open \"/dev/mem\"...\n");
        return 1;
    }

    // Map the Lightweight Bridge
    h2p_lw_virtual_base = mmap(NULL, HW_REGS_SPAN, (PROT_READ | PROT_WRITE), MAP_SHARED, fd, HW_REGS_BASE);
    if (h2p_lw_virtual_base == MAP_FAILED) {
        printf("ERROR: mmap1() failed...\n");
        close(fd);
        return 1;
    }

    // Map VGA character buffer
    vga_char_virtual_base = mmap(NULL, FPGA_CHAR_SPAN, (PROT_READ | PROT_WRITE), MAP_SHARED, fd, FPGA_CHAR_BASE);
    if (vga_char_virtual_base == MAP_FAILED) {
        printf("ERROR: mmap2() failed...\n");
        close(fd);
        return 1;
    }
    vga_char_ptr = (unsigned int *)(vga_char_virtual_base);

    // Map VGA pixel buffer
    vga_pixel_virtual_base = mmap(NULL, SDRAM_SPAN, (PROT_READ | PROT_WRITE), MAP_SHARED, fd, SDRAM_BASE);
    if (vga_pixel_virtual_base == MAP_FAILED) {
        printf("ERROR: mmap3() failed...\n");
        close(fd);
        return 1;
    }
    vga_pixel_ptr = (unsigned int *)(vga_pixel_virtual_base);

    // Initial VGA screen clear
    VGA_text_clear();
    VGA_box(0, 0, 639, 479, black);

    // Draw title and prompt
    VGA_text(20, 10, "Space-Shooter");
    VGA_text(15, 20, "Press Spacebar to start");

    // Wait for spacebar to start the game
    do {
        key_pressed = getchar();
    } while (key_pressed != ' ');

    // Clear text and setup game
    VGA_text_clear();
    init_particle(&player, green, PLAYER_SIZE);
    player.px = 320;
    player.py = 440;

    for (int i = 0; i < 7; i++) {
        init_particle(&enemies[i], red + i, 10 + i);
        enemies[i].px = 50 * i + 70;
        enemies[i].py = 60;
    }

    pthread_t tid;
    pthread_create(&tid, NULL, keyboard_thread, NULL);

    // Game main loop
    while (1) {
        update_player_position();
        update_enemies();
        move_bullets();
        detect_collision();
        display_score();
        if (check_game_over()) break;
        usleep(50000);  // 50 ms
    }

    close(fd);
    return 0;
}
