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

#define ENEMY_POS_X           0x10  // Offset for X position
#define ENEMY_POS_Y           0x20  // Offset for Y position
#define ENEMY_NUM             7

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

void VGA_box(int, int, int, int, short);
void VGA_disc(int, int, int, short);
void *keyboard_thread(void *);

int fd;
volatile unsigned int *vga_pixel_ptr = NULL;
volatile unsigned int *vga_char_ptr = NULL;
volatile unsigned int *enemy_pos_x_ptr = NULL;
volatile unsigned int *enemy_pos_y_ptr = NULL;

Particle player; 
Particle enemies[ENEMY_NUM];
Bullet bullets[10];

char key_pressed = '\0';
int game_started = 0; // Added flag to indicate game start

void init_particle(Particle *p, int color, int size) {
    p->x = 0;
    p->y = 0;
    p->px = 0;
    p->py = 0;
    p->size = size;
    p->active = 1;
    p->color = color;
}

void init_bullet(Bullet *b, int x, int y, int color) {
    b->x = x;
    b->y = y;
    b->active = 1;
    b->color = color;
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
        if (key_pressed == ' ') { // Start the game when space bar is pressed
            game_started = 1;
        }
        if (key_pressed == 'f') fire_bullet();
    }
    return NULL;
}

int main(void) {
    // Memory mapping
    if ((fd = open("/dev/mem", (O_RDWR | O_SYNC))) == -1) {
        printf("ERROR: could not open \"/dev/mem\"...\n");
        return 1;
    }

    void *h2p_lw_virtual_base;
    void *vga_pixel_virtual_base;
    void *vga_char_virtual_base;

    // Map the Lightweight Bridge (LW) region
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

    enemy_pos_x_ptr = (unsigned int *)(h2p_lw_virtual_base + ENEMY_POS_X); 
    enemy_pos_y_ptr = (unsigned int *)(h2p_lw_virtual_base + ENEMY_POS_Y);

    // Initialize the text on the VGA screen
    VGA_box(0, 0, x_size - 1, y_size - 1, black);

    // Display "Start" message in a big cool font
    VGA_disc(180, 200, 8, white);
    VGA_disc(180, 201, 8, white);
    VGA_disc(180, 202, 8, white);
    VGA_disc(180, 203, 8, white);
    VGA_disc(180, 204, 8, white);
    VGA_disc(180, 205, 8, white);
    VGA_disc(240, 210, 5, white); // Adjust size of space

    VGA_text(180, 200, "██████╗░███████╗███╗░░░███╗███████╗██╗░░██╗");
    VGA_text(180, 201, "██╔══██╗██╔════╝████╗░████║██╔════╝██║░░██║");
    VGA_text(180, 202, "██████╔╝█████╗░░██╔████╔██║█████╗░░███████║");
    VGA_text(180, 203, "██╔═══╝░██╔══╝░░██║╚██╔╝██║██╔══╝░░██╔══██║");
    VGA_text(180, 204, "██║░░░░░███████╗██║░╚═╝░██║███████╗██║░░██║");
    VGA_text(180, 205, "╚═╝░░░░░╚══════╝╚═╝░░░░░╚═╝╚══════╝╚═╝░░╚═╝");
    VGA_text(240, 210, "Press space to start");

    // Initialize the player and enemies
    init_particle(&player, green, PLAYER_SIZE);
    player.px = 320;
    player.py = 440; // Place player at the bottom center

    // Initialize enemies
    for (int i = 0; i < ENEMY_NUM; i++) {
        init_particle(&enemies[i], red, 8);
    }

    for (int i = 0; i < 10; i++) {
        bullets[i].active = 0;
    }

    // Initialize keyboard settings
    struct termios old_tio, new_tio;
    tcgetattr(0, &old_tio);
    new_tio = old_tio;
    new_tio.c_lflag &= ~ICANON;
    new_tio.c_lflag &= ~ECHO;
    new_tio.c_cc[VMIN] = 1;
    new_tio.c_cc[VTIME] = 0;
    tcsetattr(0, TCSANOW, &new_tio);

    // Create keyboard thread
    pthread_t tid;
    pthread_create(&tid, NULL, keyboard_thread, NULL);

    // Wait for the space bar to be pressed to start the game
    while (!game_started) {
        usleep(100000); // 100 ms delay
    }

    // Main game loop
    while (1) {
        update_player_position();
        update_enemies();
        move_bullets();
        detect_collision();
        display_score();
        if (check_game_over()) break;
        usleep(50000);  // 50 ms delay
    }

    // Cleanup and exit
    close(fd);
    return 0;
}
