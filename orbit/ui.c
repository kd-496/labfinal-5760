#include <ncurses.h>
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

    // Initialize ncurses
    initscr();
    noecho();
    cbreak();
    curs_set(0); // Hide cursor
    keypad(stdscr, TRUE); // Enable keyboard input
    start_color(); // Enable colors

    // Define color pairs
    init_pair(1, COLOR_YELLOW, COLOR_BLACK);
    init_pair(2, COLOR_RED, COLOR_BLACK);
    init_pair(3, COLOR_GREEN, COLOR_BLACK);
    init_pair(4, COLOR_CYAN, COLOR_BLACK);

    // Display the start screen
    attron(COLOR_PAIR(2));
    printw("Space-Shooter\n\n");
    attron(COLOR_PAIR(4));
    printw("       /\\ \n");
    printw("      /__\\ \n");
    printw("     /_/_\\ \n");
    printw("    /_/__\\ \n");
    printw("   /_/_/_\\ \n");
    printw("      ||\n");
    printw("      ||\n");
    printw("      ||\n\n");
    attroff(COLOR_PAIR(4));
    attron(COLOR_PAIR(1));
    printw("Press Spacebar to start\n");
    attroff(COLOR_PAIR(1));
    refresh();

    while(getch() != ' '); // Wait for spacebar
    clear();

    // Initialize and run the game
    initialize_game();
    pthread_create(&tid, NULL, keyboard_thread, NULL);
    run_game();
    pthread_join(tid, NULL);

    // Cleanup and exit
    cleanup_game();
    endwin();
    return 0;
}

void initialize_game() {
    clear();
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
        refresh();
        usleep(50000); // Slow down the game loop
    }
}

void cleanup_game() {
    printw("Game Over! Final score: %d\nPress any key to exit...", score);
    refresh();
    getch();
}

void *keyboard_thread(void *arg) {
    int ch;
    while ((ch = getch()) != 'q') {
        switch(ch) {
            case KEY_LEFT:
                if (PLAYER_X > 1) PLAYER_X--;
                break;
            case KEY_RIGHT:
                if (PLAYER_X < COLS - 2) PLAYER_X++;
                break;
            case ' ': // Fire a bullet
                fire_bullet();
                break;
        }
    }
    game_running = 0;
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
            if (enemies[i].y > LINES - 2) {
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
    clear();
    for (int i = 0; i < MAX_BULLETS; i++) {
        if (bullets[i].active) {
            mvprintw(bullets[i].y, bullets[i].x, "|");
        }
    }
    for (int i = 0; i < MAX_ENEMIES; i++) {
        if (enemies[i].active) {
            mvprintw(enemies[i].y, enemies[i].x, "X");
        }
    }
    mvprintw(PLAYER_Y, PLAYER_X, "^");
    mvprintw(0, 0, "Score: %d", score);
}

void spawn_enemy() {
    for (int i = 0; i < MAX_ENEMIES; i++) {
        if (!enemies[i].active) {
            enemies[i].active = 1;
            enemies[i].x = rand() % (COLS - 2) + 1;
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
