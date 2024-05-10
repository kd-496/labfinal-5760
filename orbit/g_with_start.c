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

//#define ENEMY_POS_BASE        0xC9001000  // Base address for enemy positions
//#define ENEMY_POS_SPAN        0x00000100  // Span to cover enough registers for multiple enemies
#define ENEMY_POS_X           0x10  // Offset for X position
#define ENEMY_POS_Y           0x20  // Offset for Y position
#define ENEMY_NUM             7
//#define ENEMY_POS_SIZE        0x00000010  // Size difference between subsequent enemy positions

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
#define grey                  gray

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
//volatile float * enemy_pos_x_external_connection = NULL;
//volatile float * enemy_pos_y_external_connection = NULL;
//volatile float * enemyPosX = NULL;
//volatile float * enemyPosY = NULL;
//void *enemy_pos_virtual_base;

//enemy_pos_x_ptr = (unsigned int *)(ENEMY_POS_X + enemy_pos_x_external_connection);
//enemy_pos_y_ptr = (unsigned int *)(ENEMY_POS_Y + enemy_pos_y_external_connection);

int fd;
struct termios old_tio, new_tio;
char key_pressed = '\0';

Particle player; 
Particle enemies[ENEMY_NUM]; // Array to hold multiple enemies
Bullet bullets[10];

int score = 0; // Player score
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
            for (int e = 0; e < ENEMY_NUM; e++) {
                if (enemies[e].active && bullets[b].x >= enemies[e].px - enemies[e].size &&
                    bullets[b].x <= enemies[e].px + enemies[e].size &&
                    bullets[b].y >= enemies[e].py - enemies[e].size &&
                    bullets[b].y <= enemies[e].py + enemies[e].size) {
                    enemies[e].active = 0;
                    bullets[b].active = 0;
                    score += 100; // Increase score
                    VGA_disc(enemies[e].px, enemies[e].py, enemies[e].size, black); // Clear the enemy from the screen
                }
            }
        }
    }
}

void display_score() { // score & time
    sprintf(score_text, "Score: %d", score);
    VGA_text(10, 6, score_text);
    sprintf(time_text, "Time used: %d seconds", time_used);
    VGA_text(10, 7, time_text);
}

int check_game_over() {
    int all_inactive = 1;
    for (int i = 0; i < ENEMY_NUM; i++) {
        if (enemies[i].active) {
            all_inactive = 0;
            break;
        }
    }
    if (all_inactive) {
        VGA_text_clear();
        VGA_text(10, 12, "Game Over! Final ");
        VGA_text(32, 15, score_text);
        VGA_text(10, 16, " ");
        VGA_text(32, 16, time_text);
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

    float past_coord_x [ENEMY_NUM] = {0.0, 0.0, 0.0, 0.0};
    float past_coord_y [ENEMY_NUM] = {0.0, 0.0, 0.0, 0.0};

void update_enemies() {
    if (enemy_pos_x_ptr == NULL || enemy_pos_y_ptr == NULL) {
        printf("Enemy position pointer is not initialized.\n");
        return;
    }

    for (int i = 0; i < ENEMY_NUM; i++) {
        // unsigned int x_index = (ENEMY_POS_X / sizeof(unsigned int)) + i * (ENEMY_POS_SIZE / sizeof(unsigned int));
        // unsigned int y_index = (ENEMY_POS_Y / sizeof(unsigned int)) + i * (ENEMY_POS_SIZE / sizeof(unsigned int));

        // if (x_index >= (ENEMY_POS_SPAN / sizeof(unsigned int)) || y_index >= (ENEMY_POS_SPAN / sizeof(unsigned int))) {
        //     printf("Index out of bounds error.\n");
        //     continue;
        // }
        VGA_disc(past_coord_x[i], past_coord_y[i], enemies[i].size, black);

        enemies[i].px =  *(enemy_pos_x_ptr) + 20 * i;
        enemies[i].py =  *(enemy_pos_y_ptr) + 5 * i;

        if (i == 0 || i == 2) { enemies[i].px = x_size - enemies[i].px/2; }  //making it seem a little different
        if (i == 1 || i == 2) { enemies[i].py = y_size - enemies[i].py/2; }
        if (i == 4 || i == 6) { enemies[i].px = x_size/2 - enemies[i].px/3; }
        if (i == 5 || i == 6) { enemies[i].py = y_size/4 - enemies[i].px/3; }

        enemies[i].py = enemies[i].py * 0.8;

        past_coord_x[i] = enemies[i].px;
        past_coord_y[i] = enemies[i].py;
        //*(enemy_pos_x_ptr) = enemies[i].px;// + x_index);
        //*(enemy_pos_y_ptr) = enemies[i].py;// + y_index);                                    // is the left and right values in opposite place?
        if (enemies[i].active) {
            VGA_disc(enemies[i].px, enemies[i].py, enemies[i].size, enemies[i].color); // Draw enemy at new position
        }
    }
}

void update_player_position() {
    VGA_disc(player.px, player.py, player.size, black);
    if (key_pressed == 'a' && player.px > 0 + PLAYER_SIZE) player.px -= 10;
    if (key_pressed == 'd' && player.px < x_size - PLAYER_SIZE) player.px += 10;
    if (key_pressed == 'w' && player.py > 0 + PLAYER_SIZE) player.py -= 10;
    if (key_pressed == 's' && player.py < y_size - PLAYER_SIZE) player.py += 10;
    //if (key_pressed == 'x') player.px = 320, player.py = 440;  // reset player place
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

void *keyboard_thread_1(void *arg) {
    while (1) {
        key_pressed = getchar();
        if (key_pressed == ' ') break; //fire_bullet();
    }
    return NULL;
}

int main(void) {
    if ((fd = open("/dev/mem", (O_RDWR | O_SYNC))) == -1) {
        printf("ERROR: could not open \"/dev/mem\"...\n");
        return 1;
    }

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

    // // Map enemy position registers
    // enemy_pos_virtual_base = mmap(NULL, ENEMY_POS_SPAN, (PROT_READ | PROT_WRITE), MAP_SHARED, fd, ENEMY_POS_BASE);
    // if (enemy_pos_virtual_base == MAP_FAILED) {
    //     printf("ERROR: mmap4() failed...\n");
    //     close(fd);
    //     return 1;
    // }
    
    //enemyPosX = (unsigned int *)(h2p_lw_virtual_base + ENEMY_POS_X); 
    //enemyPosY = (unsigned int *)(h2p_lw_virtual_base + ENEMY_POS_Y);
    
    // Initialize keyboard settings
    tcgetattr(0, &old_tio);
    new_tio = old_tio;
    new_tio.c_lflag &= ~ICANON;
    new_tio.c_lflag &= ~ECHO;
    new_tio.c_cc[VMIN] = 1;
    new_tio.c_cc[VTIME] = 0;
    tcsetattr(0, TCSANOW, &new_tio);

    enemy_pos_x_ptr = (unsigned int *)(h2p_lw_virtual_base + ENEMY_POS_X); 
    enemy_pos_y_ptr = (unsigned int *)(h2p_lw_virtual_base + ENEMY_POS_Y);

    //enemy_pos_x_external_connection = (unsigned int *)(h2p_lw_virtual_base + ENEMY_POS_X); 
    //enemy_pos_y_external_connection = (unsigned int *)(h2p_lw_virtual_base + ENEMY_POS_Y);
    char text_one[40] = "Space shooting game\0";
    char text_two[40] = "press any key to start\0";

    VGA_box(0, 0, x_size - 1, y_size - 1, black);
    VGA_text_clear();

    usleep(100000);

    VGA_text(25, 20, text_one);
    VGA_text(25, 30, text_two);

    usleep(1000000);

    pthread_t tid_1;
    pthread_create(&tid_1, NULL, keyboard_thread_1, NULL);
    //while (key_pressed == ' ') {
    //    usleep(100000);
    //}
    
    VGA_text_clear();

    // Initialize the text on the VGA screen
    char text_top_row[40] = "DE1-SoC ARM/FPGA\0";
    char text_bottom_row[40] = "Cornell ece5760\0";
    char text_next[40] = "Shooting Game\0";
    char text_four[80] = "move with 'w' 'a' 's' 'd'\0";
    char text_five[80] = "stop & fire upward with 'f'\0";

    VGA_box(0, 0, x_size - 1, y_size - 1, black);
    VGA_text_clear();

    VGA_text(10, 1, text_top_row);
    VGA_text(10, 2, text_bottom_row);
    VGA_text(10, 3, text_next);
    VGA_text(10, 4, text_four);
    VGA_text(10, 5, text_five);

    // Initialize the player and enemies
    init_particle(&player, green, PLAYER_SIZE);
    player.px = 320;
    player.py = 440; // Place player at the bottom center

    init_particle(&enemies[0], yellow, 8); // Enemy1
    init_particle(&enemies[1], cyan, 8);   // Enemy2
    init_particle(&enemies[2], blue, 7);   // Enemy3
    init_particle(&enemies[3], magenta, 6); // Enemy4
    init_particle(&enemies[4], red, 8); 
    init_particle(&enemies[5], white, 8);       
    init_particle(&enemies[6], grey, 7);       

    for (int i = 0; i < 10; i++) bullets[i].active = 0;

    pthread_t tid;
    pthread_create(&tid, NULL, keyboard_thread, NULL);

    gettimeofday(&start, NULL);
    gettimeofday(&end, NULL);

    int sign_ = 0;
    // Main game loop
    while (1) {
        update_player_position();
        update_enemies(); // Update enemy positions based on FPGA computations
        move_bullets();
        if (sign_  == 1) { break; }
        detect_collision();
        display_score();
        if (check_game_over() == 1) { sign_ = 1; }
        gettimeofday(&end, NULL);
        time_used = (end.tv_sec - start.tv_sec);
        usleep(100000);  // 50 ms delay
    }

    close(fd);
    return 0;
}
