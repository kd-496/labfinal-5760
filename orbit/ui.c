#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/time.h>

#define MAX_BULLETS 10
#define MAX_ENEMIES 5
#define PLAYER_X 40
#define PLAYER_Y 20

typedef struct {
    int x, y, active;
} Bullet;

typedef struct {
    int x, y, active;
} Enemy;

// Global game variables
Bullet bullets[MAX_BULLETS];
Enemy enemies[MAX_ENEMIES];
int game_running = 1;
int score = 0;

// Function declarations
void initialize_game();
void run_game();
void cleanup_game();
void *keyboard_thread(void *arg);
void update_bullets();
void update_enemies();
void draw_game();
void check_collisions();
void spawn_enemy();
void fire_bullet();

int main() {
    pthread_t tid;

    // Initialize the game
    initialize_game();

    // Start the keyboard input thread
    pthread_create(&tid, NULL, keyboard_thread, NULL);

    // Game main loop
    run_game();

    // Wait for the keyboard thread to finish
    pthread_join(tid, NULL);

    // Cleanup and exit
    cleanup_game();
    return 0;
}

void initialize_game() {
    system("clear");
    for (int i = 0; i < MAX_BULLETS; i++) {
        bullets[i].active = 0;
    }
    for (int i = 0; i < MAX_ENEMIES; i++) {
        enemies[i].active = 0;
    }
    spawn_enemy();
}

void run_game() {
    while (game_running) {
        update_bullets();
        update_enemies();
        check_collisions();
        draw_game();
        usleep(50000); // Slow down the game loop
    }
}

void cleanup_game() {
    printf("Game Over! Final score: %d\nPress any key to exit...\n", score);
    getchar(); // Wait for a key press
}

void *keyboard_thread(void *arg) {
    char ch;
    system("/bin/stty raw");
    while ((ch = getchar()) != 'q') {
        switch(ch) {
            case 'a': // Move left
                if (PLAYER_X > 1) PLAYER_X--;
                break;
            case 'd': // Move right
                if (PLAYER_X < 80 - 2) PLAYER_X++;
                break;
            case ' ': // Fire a bullet
                fire_bullet();
                break;
        }
    }
    game_running = 0;
    system("/bin/stty cooked");
    return NULL;
}

void update_bullets() {
    for (int i = 0; i < MAX_BULLETS; i++) {
        if (bullets[i].active) {
            bullets[i].y--;
            if (bullets[i].y < 1) {
                bullets[i].active = 0;
            }
        }
    }
}

void update_enemies() {
    for (int i = 0; i < MAX_ENEMIES; i++) {
        if (enemies[i].active) {
            enemies[i].y++;
            if (enemies[i].y > 20 - 2) {
                enemies[i].active = 0;
                spawn_enemy();
            }
        }
    }
}

void check_collisions() {
    for (int i = 0; i < MAX_BULLETS; i++) {
        if (bullets[i].active) {
            for (int j = 0; j < MAX_ENEMIES; j++) {
                if (enemies[j].active && bullets[i].x == enemies[j].x && bullets[i].y == enemies[j].y) {
                    bullets[i].active = 0;
                    enemies[j].active = 0;
                    score += 100;
                    spawn_enemy();
                }
            }
        }
    }
}

void draw_game() {
    system("clear");
    for (int i = 0; i < MAX_BULLETS; i++) {
        if (bullets[i].active) {
            printf("\033[%d;%dH|", bullets[i].y, bullets[i].x);
        }
    }
    for (int i = 0; i < MAX_ENEMIES; i++) {
        if (enemies[i].active) {
            printf("\033[%d;%dHX", enemies[i].y, enemies[i].x);
        }
    }
    printf("\033[%d;%dH^", PLAYER_Y, PLAYER_X);
    printf("\033[1;1HScore: %d", score);
    fflush(stdout);
}

void spawn_enemy() {
    for (int i = 0; i < MAX_ENEMIES; i++) {
        if (!enemies[i].active) {
            enemies[i].active = 1;
            enemies[i].x = rand() % (80 - 2) + 1;
            enemies[i].y = 1;
            break;
        }
    }
}

void fire_bullet() {
    for (int i = 0; i < MAX_BULLETS; i++) {
        if (!bullets[i].active) {
            bullets[i].active = 1;
            bullets[i].x = PLAYER_X;
            bullets[i].y = PLAYER_Y - 1;
            break;
        }
    }
}
