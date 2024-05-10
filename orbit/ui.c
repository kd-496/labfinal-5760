#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>

#define MAX_ENEMIES 5
#define MAX_BULLETS 10
#define GAME_AREA_HEIGHT 10
#define GAME_AREA_WIDTH 30

typedef struct {
    int x, y;
    int isActive;
} GameObject;

GameObject player;
GameObject enemies[MAX_ENEMIES];
GameObject bullets[MAX_BULLETS];
int gameRunning = 1;

// Function declarations
void initializeGame();
void drawGameArea();
void *inputThread(void *param);
void updateGame();
void fireBullet();
void moveBullets();
void checkCollisions();

int main() {
    pthread_t threadId;

    // Initialize game objects
    initializeGame();

    // Start input thread
    pthread_create(&threadId, NULL, inputThread, NULL);

    // Game loop
    while (gameRunning) {
        system("clear");
        drawGameArea();
        updateGame();
        usleep(200000); // Sleep to control game speed
    }

    printf("Game Over!\n");
    pthread_cancel(threadId); // Terminate input thread
    return 0;
}

void initializeGame() {
    // Initialize player
    player.x = GAME_AREA_WIDTH / 2;
    player.y = GAME_AREA_HEIGHT - 1;
    player.isActive = 1;

    // Initialize enemies
    for (int i = 0; i < MAX_ENEMIES; i++) {
        enemies[i].x = rand() % GAME_AREA_WIDTH;
        enemies[i].y = rand() % (GAME_AREA_HEIGHT / 2);
        enemies[i].isActive = 1;
    }

    // Initialize bullets
    for (int i = 0; i < MAX_BULLETS; i++) {
        bullets[i].isActive = 0;
    }
}

void drawGameArea() {
    for (int y = 0; y < GAME_AREA_HEIGHT; y++) {
        for (int x = 0; x < GAME_AREA_WIDTH; x++) {
            int printed = 0;
            if (player.x == x && player.y == y) {
                printf("P");
                printed = 1;
            }
            for (int i = 0; i < MAX_ENEMIES; i++) {
                if (enemies[i].x == x && enemies[i].y == y && enemies[i].isActive) {
                    printf("E");
                    printed = 1;
                }
            }
            for (int i = 0; i < MAX_BULLETS; i++) {
                if (bullets[i].x == x && bullets[i].y == y && bullets[i].isActive) {
                    printf("*");
                    printed = 1;
                }
            }
            if (!printed) {
                printf(" ");
            }
        }
        printf("\n");
    }
}

void *inputThread(void *param) {
    char input;
    while (1) {
        input = getchar();
        switch (input) {
            case 'a': // Move left
                if (player.x > 0) player.x--;
                break;
            case 'd': // Move right
                if (player.x < GAME_AREA_WIDTH - 1) player.x++;
                break;
            case 's': // Shoot
                fireBullet();
                break;
            case 'q': // Quit game
                gameRunning = 0;
                return NULL;
        }
    }
}

void updateGame() {
    moveBullets();
    checkCollisions();
}

void fireBullet() {
    for (int i = 0; i < MAX_BULLETS; i++) {
        if (!bullets[i].isActive) {
            bullets[i].x = player.x;
            bullets[i].y = player.y - 1;
            bullets[i].isActive = 1;
            break;
        }
    }
}

void moveBullets() {
    for (int i = 0; i < MAX_BULLETS; i++) {
        if (bullets[i].isActive) {
            bullets[i].y--;
            if (bullets[i].y < 0) {
                bullets[i].isActive = 0;
            }
        }
    }
}

void checkCollisions() {
    // Check bullet collisions with enemies
    for (int i = 0; i < MAX_BULLETS; i++) {
        if (bullets[i].isActive) {
            for (int j = 0; j < MAX_ENEMIES; j++) {
                if (enemies[j].isActive && bullets[i].x == enemies[j].x && bullets[i].y == enemies[j].y) {
                    enemies[j].isActive = 0;
                    bullets[i].isActive = 0;
                    break;
                }
            }
        }
    }
    // Check if all enemies are defeated
    int allDefeated = 1;
    for (int i = 0; i < MAX_ENEMIES; i++) {
        if (enemies[i].isActive) {
            allDefeated = 0;
            break;
        }
    }
    if (allDefeated) {
        gameRunning = 0;
    }
}
