#include <fcntl.h>
#include <math.h>
#include <pthread.h>
#include <semaphore.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ipc.h>
#include <sys/mman.h>
#include <sys/shm.h>
#include <sys/time.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#define particle_number 72
#define dt 10
#define scale 100

typedef struct object {
    unsigned int number;
    float mass;
    float x_coord;
    float y_coord;
    float x_vel;
    float y_vel;
    unsigned int color;
    unsigned int radius;
    unsigned int old_display_x;
    unsigned int old_display_y;
} particle;

// video display
#define SDRAM_BASE 0xC0000000
#define SDRAM_END 0xC3FFFFFF
#define SDRAM_SPAN 0x04000000
// characters
#define FPGA_CHAR_BASE 0xC9000000
#define FPGA_CHAR_END 0xC9001FFF
#define FPGA_CHAR_SPAN 0x00002000
/* Cyclone V FPGA devices */
#define HW_REGS_BASE 0xff200000
#define HW_REGS_SPAN 0x00005000

// graphics primitives
void VGA_text(int, int, char *);
void VGA_text_clear();
void VGA_box(int, int, int, int, short);
void VGA_rect(int, int, int, int, short);
void VGA_line(int, int, int, int, short);
void VGA_Vline(int, int, int, short);
void VGA_Hline(int, int, int, short);
void VGA_disc(int, int, int, short);
void VGA_circle(int, int, int, int);

// 16-bit primary colors
#define red (0 + (0 << 5) + (31 << 11))
#define dark_red (0 + (0 << 5) + (15 << 11))
#define green (0 + (63 << 5) + (0 << 11))
#define dark_green (0 + (31 << 5) + (0 << 11))
#define blue (31 + (0 << 5) + (0 << 11))
#define dark_blue (15 + (0 << 5) + (0 << 11))
#define yellow (0 + (63 << 5) + (31 << 11))
#define cyan (31 + (63 << 5) + (0 << 11))
#define magenta (31 + (0 << 5) + (31 << 11))
#define black (0x0000)
#define gray (15 + (31 << 5) + (51 << 11))
#define white (0xffff)
int colors[] = {red, dark_red, green, dark_green, blue, dark_blue, yellow, cyan, magenta, gray, black, white};

// pixel macro
#define VGA_PIXEL(x, y, color) \
    do { \
        int *pixel_ptr; \
        pixel_ptr = (int *)((char *)vga_pixel_ptr + (((y)*640 + (x)) << 1)); \
        *(short *)pixel_ptr = (color); \
    } while (0)

// the light weight buss base
void *h2p_lw_virtual_base;

// pixel buffer
volatile unsigned int *vga_pixel_ptr = NULL;
void *vga_pixel_virtual_base;

// character buffer
volatile unsigned int *vga_char_ptr = NULL;
void *vga_char_virtual_base;

// /dev/mem file id
int fd;

// measure time
struct timeval t1, t2;
double elapsedTime;

// main bus; PIO
#define FPGA_AXI_BASE 0xC0000000
#define FPGA_AXI_SPAN 0x00001000
// main axi bus base
void *h2p_virtual_base;
volatile unsigned int *axi_pio_ptr = NULL;
volatile unsigned int *axi_pio_read_ptr = NULL;

// lw bus; PIO
#define FPGA_LW_BASE 0xff200000
#define FPGA_LW_SPAN 0x00001000

// HPS_to_FPGA FIFO status address = 0
volatile unsigned int *lw_pio_ptr = NULL;
volatile unsigned int *lw_pio_read_ptr = NULL;

unsigned int *obj_1_num_a = NULL;
float *obj_1_mass_a = NULL;
float *obj_1_x_coord_a = NULL;
float *obj_1_y_coord_a = NULL;

unsigned int *obj_2_num_a0 = NULL;
float *obj_2_mass_a0 = NULL;
float *obj_2_x_coord_a0 = NULL;
float *obj_2_y_coord_a0 = NULL;
volatile float *obj_1_delta_x_acc_b0 = NULL;
volatile float *obj_1_delta_y_acc_b0 = NULL;
volatile bool *complete0 = NULL;

unsigned int *obj_2_num_a1 = NULL;
float *obj_2_mass_a1 = NULL;
float *obj_2_x_coord_a1 = NULL;
float *obj_2_y_coord_a1 = NULL;
volatile float *obj_1_delta_x_acc_b1 = NULL;
volatile float *obj_1_delta_y_acc_b1 = NULL;
volatile bool *complete1 = NULL;

unsigned int *obj_2_num_a2 = NULL;
float *obj_2_mass_a2 = NULL;
float *obj_2_x_coord_a2 = NULL;
float *obj_2_y_coord_a2 = NULL;
volatile float *obj_1_delta_x_acc_b2 = NULL;
volatile float *obj_1_delta_y_acc_b2 = NULL;
volatile bool *complete2 = NULL;

unsigned int *obj_2_num_a3 = NULL;
float *obj_2_mass_a3 = NULL;
float *obj_2_x_coord_a3 = NULL;
float *obj_2_y_coord_a3 = NULL;
volatile float *obj_1_delta_x_acc_b3 = NULL;
volatile float *obj_1_delta_y_acc_b3 = NULL;
volatile bool *complete3 = NULL;

unsigned int *obj_2_num_a4 = NULL;
float *obj_2_mass_a4 = NULL;
float *obj_2_x_coord_a4 = NULL;
float *obj_2_y_coord_a4 = NULL;
volatile float *obj_1_delta_x_acc_b4 = NULL;
volatile float *obj_1_delta_y_acc_b4 = NULL;
volatile bool *complete4 = NULL;

unsigned int *obj_2_num_a5 = NULL;
float *obj_2_mass_a5 = NULL;
float *obj_2_x_coord_a5 = NULL;
float *obj_2_y_coord_a5 = NULL;
volatile float *obj_1_delta_x_acc_b5 = NULL;
volatile float *obj_1_delta_y_acc_b5 = NULL;
volatile bool *complete5 = NULL;

unsigned int *obj_2_num_a6 = NULL;
float *obj_2_mass_a6 = NULL;
float *obj_2_x_coord_a6 = NULL;
float *obj_2_y_coord_a6 = NULL;
volatile float *obj_1_delta_x_acc_b6 = NULL;
volatile float *obj_1_delta_y_acc_b6 = NULL;
volatile bool *complete6 = NULL;

unsigned int *obj_2_num_a7 = NULL;
float *obj_2_mass_a7 = NULL;
float *obj_2_x_coord_a7 = NULL;
float *obj_2_y_coord_a7 = NULL;
volatile float *obj_1_delta_x_acc_b7 = NULL;
volatile float *obj_1_delta_y_acc_b7 = NULL;
volatile bool *complete7 = NULL;

bool *my_reset_ptr = NULL;

bool *data_sent_ptr = NULL;

// read offset is 0x10 for both busses
// remember that eaxh axi master bus needs unique address
#define OBJ1_NUM_A 0x00
#define OBJ1_MASS_A 0x10
#define OBJ1_X_COORD_A 0x20
#define OBJ1_Y_COORD_A 0x30
#define COMPLETE0 0xa0
#define RESET 0xc0

#define OBJ2_NUM_A0 0x40
#define OBJ2_MASS_A0 0x50
#define OBJ2_X_COORD_A0 0x60
#define OBJ2_Y_COORD_A0 0x70
#define OBJ1_DELTA_X_ACC0 0x80
#define OBJ1_DELTA_Y_ACC0 0x90

#define OBJ2_NUM_A1 0x100
#define OBJ2_MASS_A1 0x110
#define OBJ2_X_COORD_A1 0x120
#define OBJ2_Y_COORD_A1 0x130
#define COMPLETE1 0x140
#define OBJ1_DELTA_X_ACC1 0x150
#define OBJ1_DELTA_Y_ACC1 0x160

#define OBJ2_NUM_A2 0x200
#define OBJ2_MASS_A2 0x210
#define OBJ2_X_COORD_A2 0x220
#define OBJ2_Y_COORD_A2 0x230
#define COMPLETE2 0x240
#define OBJ1_DELTA_X_ACC2 0x250
#define OBJ1_DELTA_Y_ACC2 0x260

#define OBJ2_NUM_A3 0x300
#define OBJ2_MASS_A3 0x310
#define OBJ2_X_COORD_A3 0x320
#define OBJ2_Y_COORD_A3 0x330
#define COMPLETE3 0x340
#define OBJ1_DELTA_X_ACC3 0x350
#define OBJ1_DELTA_Y_ACC3 0x360

#define OBJ2_NUM_A4 0x400
#define OBJ2_MASS_A4 0x410
#define OBJ2_X_COORD_A4 0x420
#define OBJ2_Y_COORD_A4 0x430
#define COMPLETE4 0x440
#define OBJ1_DELTA_X_ACC4 0x450
#define OBJ1_DELTA_Y_ACC4 0x460

#define OBJ2_NUM_A5 0x500
#define OBJ2_MASS_A5 0x510
#define OBJ2_X_COORD_A5 0x520
#define OBJ2_Y_COORD_A5 0x530
#define COMPLETE5 0x540
#define OBJ1_DELTA_X_ACC5 0x550
#define OBJ1_DELTA_Y_ACC5 0x560

#define OBJ2_NUM_A6 0x600
#define OBJ2_MASS_A6 0x610
#define OBJ2_X_COORD_A6 0x620
#define OBJ2_Y_COORD_A6 0x630
#define COMPLETE6 0x640
#define OBJ1_DELTA_X_ACC6 0x650
#define OBJ1_DELTA_Y_ACC6 0x660

#define OBJ2_NUM_A7 0x700
#define OBJ2_MASS_A7 0x710
#define OBJ2_X_COORD_A7 0x720
#define OBJ2_Y_COORD_A7 0x730
#define COMPLETE7 0x740
#define OBJ1_DELTA_X_ACC7 0x750
#define OBJ1_DELTA_Y_ACC7 0x760

#define DATA_SENT 0x800

int delta_t = dt;
int choice = -1;
int go = -1;
int particle_1 = -1;
int particle_2 = -1;
float x_rel_vel = 0;
float y_rel_vel = 0;
float x_rel_pos = 0;
float y_rel_pos = 0;

float particle_mass = 0;
float x_coord = 0;
float y_coord = 0;
float x_vel = 0;
float y_vel = 0;

particle *p_old;
particle *p_reset;

void *read1() {
    while (1) {
        printf("Enter 1 to change speed, enter 2 for relative speed, enter 3 for particle status:");
        scanf("%d", &choice);
        int delta_t_old = 0;
        switch (choice) {
            case 1:
                printf("Enter a value between 5 - 100 to change speed: ");
                scanf("%d", &delta_t);
                break;
            case 2:
                delta_t_old = delta_t;
                delta_t = 0;
                printf("Enter two particle number you want to compare (between 0 and %d)\n", particle_number - 1);
                printf("First particle:\n");
                scanf("%d", &particle_1);
                printf("Second particle:\n");
                scanf("%d", &particle_2);
                x_rel_pos = p_old[particle_1].x_coord - p_old[particle_2].x_coord;
                y_rel_pos = p_old[particle_1].y_coord - p_old[particle_2].y_coord;
                x_rel_vel = p_old[particle_1].x_vel - p_old[particle_2].x_vel;
                y_rel_vel = p_old[particle_1].y_vel - p_old[particle_2].y_vel;
                printf("X relative position is: %fm\n", x_rel_pos);
                printf("Y relative position is: %fm\n", y_rel_pos);
                printf("X relative velocity is: %fm/s\n", x_rel_vel);
                printf("Y relative velocity is: %fm/s\n", y_rel_vel);
                printf("Continue? (1 for yes, 2 for no)\n");
                VGA_circle(p_old[particle_1].old_display_x, p_old[particle_1].old_display_y, 10, green);
                while (1) {
                    scanf("%d", &go);
                    if (go == 1) {
                        break;
                    }
                }
                VGA_circle(p_old[particle_1].old_display_x, p_old[particle_1].old_display_y, 10, black);
                VGA_circle(p_old[particle_2].old_display_x, p_old[particle_2].old_display_y, 10, black);
                delta_t = delta_t_old;
                break;
            case 3:
                delta_t_old = delta_t;
                delta_t = 0;
                printf("Enter the particle number you want to show details (between 0 and %d):\n", particle_number - 1);
                scanf("%d", &particle_1);
                x_coord = p_old[particle_1].x_coord;
                y_coord = p_old[particle_1].y_coord;
                x_vel = p_old[particle_1].x_vel;
                y_vel = p_old[particle_1].y_vel;
                particle_mass = p_old[particle_1].mass;
                printf("mass is: %fkg\n", particle_mass);
                printf("X position is: %fm\n", x_coord);
                printf("Y position is: %fm\n", y_coord);
                printf("X velocity is: %fm/s\n", x_vel);
                printf("Y velocity is: %fm/s\n", y_vel);
                printf("Continue? (1 for yes, 2 for no)\n");
                VGA_circle(p_old[particle_1].old_display_x, p_old[particle_1].old_display_y, 10, yellow);
                while (1) {
                    scanf("%d", &go);
                    if (go == 1) {
                        break;
                    }
                }
                VGA_circle(p_old[particle_1].old_display_x, p_old[particle_1].old_display_y, 10, black);
                delta_t = delta_t_old;
                break;
            default:
                break;
        }
    }
}

int main(void) {
    pthread_t thread_read;

    // For portability, explicitly create threads in a joinable state
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

    // thread
    pthread_create(&thread_read, NULL, read1, NULL);

    // === need to mmap: =======================
    // FPGA_CHAR_BASE
    // FPGA_ONCHIP_BASE
    // HW_REGS_BASE
    // === get FPGA addresses ==================
    // Open /dev/mem
    if ((fd = open("/dev/mem", (O_RDWR | O_SYNC))) == -1) {
        printf("ERROR: could not open \"/dev/mem\"...\n");
        return (1);
    }

    // get virtual addr that maps to physical
    h2p_lw_virtual_base = mmap(NULL, HW_REGS_SPAN, (PROT_READ | PROT_WRITE), MAP_SHARED, fd, HW_REGS_BASE);
    if (h2p_lw_virtual_base == MAP_FAILED) {
        printf("ERROR: mmap1() failed...\n");
        close(fd);
        return (1);
    }

    // === get VGA char addr =====================
    // get virtual addr that maps to physical
    vga_char_virtual_base = mmap(NULL, FPGA_CHAR_SPAN, (PROT_READ | PROT_WRITE), MAP_SHARED, fd, FPGA_CHAR_BASE);
    if (vga_char_virtual_base == MAP_FAILED) {
        printf("ERROR: mmap2() failed...\n");
        close(fd);
        return (1);
    }

    // Get the address that maps to the FPGA LED control
    vga_char_ptr = (unsigned int *)(vga_char_virtual_base);

    // === get VGA pixel addr ====================
    // get virtual addr that maps to physical
    vga_pixel_virtual_base = mmap(NULL, SDRAM_SPAN, (PROT_READ | PROT_WRITE), MAP_SHARED, fd, SDRAM_BASE);
    if (vga_pixel_virtual_base == MAP_FAILED) {
        printf("ERROR: mmap3() failed...\n");
        close(fd);
        return (1);
    }

    // Get the address that maps to the FPGA pixel buffer
    vga_pixel_ptr = (unsigned int *)(vga_pixel_virtual_base);

    // ===========================================
    // get virtual address for
    // AXI bus addr
    h2p_virtual_base = mmap(NULL, FPGA_AXI_SPAN, (PROT_READ | PROT_WRITE), MAP_SHARED, fd, FPGA_AXI_BASE);
    if (h2p_virtual_base == MAP_FAILED) {
        printf("ERROR: mmap3() failed...\n");
        close(fd);
        return (1);
    }

    //============================================
    // get virtual addr that maps to physical
    // for light weight AXI bus
    h2p_lw_virtual_base = mmap(NULL, FPGA_LW_SPAN, (PROT_READ | PROT_WRITE), MAP_SHARED, fd, FPGA_LW_BASE);
    if (h2p_lw_virtual_base == MAP_FAILED) {
        printf("ERROR: mmap1() failed...\n");
        close(fd);
        return (1);
    }

    //============================================

    p_old = (particle *)calloc(particle_number, sizeof(particle));
    p_reset = (particle *)calloc(particle_number, sizeof(particle));

    obj_1_num_a = (unsigned int *)(h2p_lw_virtual_base + OBJ1_NUM_A);
    obj_1_mass_a = (float *)(h2p_lw_virtual_base + OBJ1_MASS_A);
    obj_1_x_coord_a = (float *)(h2p_lw_virtual_base + OBJ1_X_COORD_A);
    obj_1_y_coord_a = (float *)(h2p_lw_virtual_base + OBJ1_Y_COORD_A);

    obj_2_num_a0 = (unsigned int *)(h2p_lw_virtual_base + OBJ2_NUM_A0);
    obj_2_mass_a0 = (float *)(h2p_lw_virtual_base + OBJ2_MASS_A0);
    obj_2_x_coord_a0 = (float *)(h2p_lw_virtual_base + OBJ2_X_COORD_A0);
    obj_2_y_coord_a0 = (float *)(h2p_lw_virtual_base + OBJ2_Y_COORD_A0);
    obj_1_delta_x_acc_b0 = (float *)(h2p_lw_virtual_base + OBJ1_DELTA_X_ACC0);
    obj_1_delta_y_acc_b0 = (float *)(h2p_lw_virtual_base + OBJ1_DELTA_Y_ACC0);
    complete0 = (bool *)(h2p_lw_virtual_base + COMPLETE0);

    obj_2_num_a1 = (unsigned int *)(h2p_lw_virtual_base + OBJ2_NUM_A1);
    obj_2_mass_a1 = (float *)(h2p_lw_virtual_base + OBJ2_MASS_A1);
    obj_2_x_coord_a1 = (float *)(h2p_lw_virtual_base + OBJ2_X_COORD_A1);
    obj_2_y_coord_a1 = (float *)(h2p_lw_virtual_base + OBJ2_Y_COORD_A1);
    obj_1_delta_x_acc_b1 = (float *)(h2p_lw_virtual_base + OBJ1_DELTA_X_ACC1);
    obj_1_delta_y_acc_b1 = (float *)(h2p_lw_virtual_base + OBJ1_DELTA_Y_ACC1);
    complete1 = (bool *)(h2p_lw_virtual_base + COMPLETE1);

    obj_2_num_a2 = (unsigned int *)(h2p_lw_virtual_base + OBJ2_NUM_A2);
    obj_2_mass_a2 = (float *)(h2p_lw_virtual_base + OBJ2_MASS_A2);
    obj_2_x_coord_a2 = (float *)(h2p_lw_virtual_base + OBJ2_X_COORD_A2);
    obj_2_y_coord_a2 = (float *)(h2p_lw_virtual_base + OBJ2_Y_COORD_A2);
    obj_1_delta_x_acc_b2 = (float *)(h2p_lw_virtual_base + OBJ1_DELTA_X_ACC2);
    obj_1_delta_y_acc_b2 = (float *)(h2p_lw_virtual_base + OBJ1_DELTA_Y_ACC2);
    complete2 = (bool *)(h2p_lw_virtual_base + COMPLETE2);

    obj_2_num_a3 = (unsigned int *)(h2p_lw_virtual_base + OBJ2_NUM_A3);
    obj_2_mass_a3 = (float *)(h2p_lw_virtual_base + OBJ2_MASS_A3);
    obj_2_x_coord_a3 = (float *)(h2p_lw_virtual_base + OBJ2_X_COORD_A3);
    obj_2_y_coord_a3 = (float *)(h2p_lw_virtual_base + OBJ2_Y_COORD_A3);
    obj_1_delta_x_acc_b3 = (float *)(h2p_lw_virtual_base + OBJ1_DELTA_X_ACC3);
    obj_1_delta_y_acc_b3 = (float *)(h2p_lw_virtual_base + OBJ1_DELTA_Y_ACC3);
    complete3 = (bool *)(h2p_lw_virtual_base + COMPLETE3);

    obj_2_num_a4 = (unsigned int *)(h2p_lw_virtual_base + OBJ2_NUM_A4);
    obj_2_mass_a4 = (float *)(h2p_lw_virtual_base + OBJ2_MASS_A4);
    obj_2_x_coord_a4 = (float *)(h2p_lw_virtual_base + OBJ2_X_COORD_A4);
    obj_2_y_coord_a4 = (float *)(h2p_lw_virtual_base + OBJ2_Y_COORD_A4);
    obj_1_delta_x_acc_b4 = (float *)(h2p_lw_virtual_base + OBJ1_DELTA_X_ACC4);
    obj_1_delta_y_acc_b4 = (float *)(h2p_lw_virtual_base + OBJ1_DELTA_Y_ACC4);
    complete4 = (bool *)(h2p_lw_virtual_base + COMPLETE4);

    obj_2_num_a5 = (unsigned int *)(h2p_lw_virtual_base + OBJ2_NUM_A5);
    obj_2_mass_a5 = (float *)(h2p_lw_virtual_base + OBJ2_MASS_A5);
    obj_2_x_coord_a5 = (float *)(h2p_lw_virtual_base + OBJ2_X_COORD_A5);
    obj_2_y_coord_a5 = (float *)(h2p_lw_virtual_base + OBJ2_Y_COORD_A5);
    obj_1_delta_x_acc_b5 = (float *)(h2p_lw_virtual_base + OBJ1_DELTA_X_ACC5);
    obj_1_delta_y_acc_b5 = (float *)(h2p_lw_virtual_base + OBJ1_DELTA_Y_ACC5);
    complete5 = (bool *)(h2p_lw_virtual_base + COMPLETE5);

    obj_2_num_a6 = (unsigned int *)(h2p_lw_virtual_base + OBJ2_NUM_A6);
    obj_2_mass_a6 = (float *)(h2p_lw_virtual_base + OBJ2_MASS_A6);
    obj_2_x_coord_a6 = (float *)(h2p_lw_virtual_base + OBJ2_X_COORD_A6);
    obj_2_y_coord_a6 = (float *)(h2p_lw_virtual_base + OBJ2_Y_COORD_A6);
    obj_1_delta_x_acc_b6 = (float *)(h2p_lw_virtual_base + OBJ1_DELTA_X_ACC6);
    obj_1_delta_y_acc_b6 = (float *)(h2p_lw_virtual_base + OBJ1_DELTA_Y_ACC6);
    complete6 = (bool *)(h2p_lw_virtual_base + COMPLETE6);

    obj_2_num_a7 = (unsigned int *)(h2p_lw_virtual_base + OBJ2_NUM_A7);
    obj_2_mass_a7 = (float *)(h2p_lw_virtual_base + OBJ2_MASS_A7);
    obj_2_x_coord_a7 = (float *)(h2p_lw_virtual_base + OBJ2_X_COORD_A7);
    obj_2_y_coord_a7 = (float *)(h2p_lw_virtual_base + OBJ2_Y_COORD_A7);
    obj_1_delta_x_acc_b7 = (float *)(h2p_lw_virtual_base + OBJ1_DELTA_X_ACC7);
    obj_1_delta_y_acc_b7 = (float *)(h2p_lw_virtual_base + OBJ1_DELTA_Y_ACC7);
    complete7 = (bool *)(h2p_lw_virtual_base + COMPLETE7);

    my_reset_ptr = (bool *)(h2p_lw_virtual_base + RESET);

    data_sent_ptr = (bool *)(h2p_lw_virtual_base + DATA_SENT);

    struct timespec req, rem;
    req.tv_sec = 0;
    req.tv_nsec = 1;

    int i = 0;
    int j = 0;
    struct timeval t1, t2;
    double elapsedTime;

    int k = 0;
    // init
    p_old[0].number = 0;
    p_old[0].mass = 8000000000000000;
    p_old[0].x_coord = 30000;
    p_old[0].y_coord = 20000;
    p_old[0].x_vel = 0;
    p_old[0].y_vel = 0;
    p_old[0].color = yellow;
    p_old[0].radius = 4;
    p_old[0].old_display_x = p_old[0].x_coord / scale;
    p_old[0].old_display_y = p_old[0].y_coord / scale;

    p_old[1].number = 1;
    p_old[1].mass = 2000000000000000;
    p_old[1].x_coord = 30000 - 7000;
    p_old[1].y_coord = 20000;
    p_old[1].x_vel = -3.25;
    p_old[1].y_vel = 6;
    p_old[1].color = red;
    p_old[1].radius = 4;
    p_old[1].old_display_x = p_old[1].x_coord / scale;
    p_old[1].old_display_y = p_old[1].y_coord / scale;

    p_old[2].number = 2;
    p_old[2].mass = 90000000000000;
    p_old[2].x_coord = 30000 + 10000;
    p_old[2].y_coord = 20000;
    p_old[2].x_vel = 3.25;
    p_old[2].y_vel = 4;
    p_old[2].color = red;
    p_old[2].radius = 4;
    p_old[2].old_display_x = p_old[2].x_coord / scale;
    p_old[2].old_display_y = p_old[2].y_coord / scale;

    for (k = 3; k < particle_number; k++) {
        p_old[k].number = k;
        p_old[k].mass = random_range(100000, 1000000000);
        p_old[k].x_coord = random_range(50 * 100, 589 * 100);
        p_old[k].y_coord = random_range(50 * 100, 429 * 100);
        p_old[k].x_vel = random_range(0, 6) - 3;
        p_old[k].y_vel = random_range(0, 4) - 2;
        p_old[k].color = white;
        p_old[k].radius = 1;
        p_old[k].old_display_x = p_old[k].x_coord / scale;
        p_old[k].old_display_y = p_old[k].y_coord / scale;
    }

    memcpy(p_reset, p_old, particle_number * sizeof(particle));

    particle *p_new;
    VGA_box(0, 0, 639, 479, black);
    char screen_text[32];

    while (1) {
        p_new = (particle *)calloc(particle_number, sizeof(particle));
        memcpy(p_new, p_old, particle_number * sizeof(particle));
        gettimeofday(&t1, NULL);
        for (i = 0; i < particle_number; i++) {
            *obj_1_num_a = p_old[i].number;
            *obj_1_mass_a = p_old[i].mass;
            *obj_1_x_coord_a = p_old[i].x_coord;
            *obj_1_y_coord_a = p_old[i].y_coord;

            int j = 0;
            float obj_1_sum_x_acc = 0;
            float obj_1_sum_y_acc = 0;
            while (j < particle_number) {
                *obj_2_num_a0 = p_old[j].number;
                *obj_2_mass_a0 = p_old[j].mass;
                *obj_2_x_coord_a0 = p_old[j].x_coord;
                *obj_2_y_coord_a0 = p_old[j].y_coord;

                j++;

                *obj_2_num_a1 = p_old[j].number;
                *obj_2_mass_a1 = p_old[j].mass;
                *obj_2_x_coord_a1 = p_old[j].x_coord;
                *obj_2_y_coord_a1 = p_old[j].y_coord;

                j++;

                *obj_2_num_a2 = p_old[j].number;
                *obj_2_mass_a2 = p_old[j].mass;
                *obj_2_x_coord_a2 = p_old[j].x_coord;
                *obj_2_y_coord_a2 = p_old[j].y_coord;

                j++;

                *obj_2_num_a3 = p_old[j].number;
                *obj_2_mass_a3 = p_old[j].mass;
                *obj_2_x_coord_a3 = p_old[j].x_coord;
                *obj_2_y_coord_a3 = p_old[j].y_coord;

                j++;

                *obj_2_num_a4 = p_old[j].number;
                *obj_2_mass_a4 = p_old[j].mass;
                *obj_2_x_coord_a4 = p_old[j].x_coord;
                *obj_2_y_coord_a4 = p_old[j].y_coord;

                j++;

                *obj_2_num_a5 = p_old[j].number;
                *obj_2_mass_a5 = p_old[j].mass;
                *obj_2_x_coord_a5 = p_old[j].x_coord;
                *obj_2_y_coord_a5 = p_old[j].y_coord;

                j++;

                *obj_2_num_a6 = p_old[j].number;
                *obj_2_mass_a6 = p_old[j].mass;
                *obj_2_x_coord_a6 = p_old[j].x_coord;
                *obj_2_y_coord_a6 = p_old[j].y_coord;

                j++;

                *obj_2_num_a7 = p_old[j].number;
                *obj_2_mass_a7 = p_old[j].mass;
                *obj_2_x_coord_a7 = p_old[j].x_coord;
                *obj_2_y_coord_a7 = p_old[j].y_coord;

                j++;

                *my_reset_ptr = false;
                usleep(2);

                *my_reset_ptr = true;
                usleep(2);

                *my_reset_ptr = false;
                usleep(2);

                while (!(*complete0))
                    ;

                // calculate net acceleration
                obj_1_sum_x_acc += +*obj_1_delta_x_acc_b0 + *obj_1_delta_x_acc_b1 + *obj_1_delta_x_acc_b2 + *obj_1_delta_x_acc_b3 + *obj_1_delta_x_acc_b4 + *obj_1_delta_x_acc_b5 + *obj_1_delta_x_acc_b6 + *obj_1_delta_x_acc_b7;
                obj_1_sum_y_acc += *obj_1_delta_y_acc_b0 + *obj_1_delta_y_acc_b1 + *obj_1_delta_y_acc_b2 + *obj_1_delta_y_acc_b3 + *obj_1_delta_y_acc_b4 + *obj_1_delta_y_acc_b5 + *obj_1_delta_y_acc_b6 + *obj_1_delta_y_acc_b7;
            }

            float obj_1_new_x_vel = p_old[i].x_vel + obj_1_sum_x_acc * delta_t;
            float obj_1_new_y_vel = p_old[i].y_vel + obj_1_sum_y_acc * delta_t;

            float obj_1_new_x_coord = p_old[i].x_coord + obj_1_new_x_vel * delta_t;
            float obj_1_new_y_coord = p_old[i].y_coord + obj_1_new_y_vel * delta_t;

            int obj_1_new_display_x = (obj_1_new_x_coord < 0) ? 0 : (obj_1_new_x_coord > 639) ? 639
                                                                                              : (int)obj_1_new_x_coord;
            int obj_1_new_display_y = (obj_1_new_y_coord < 0) ? 0 : (obj_1_new_y_coord > 479) ? 479
                                                                                              : (int)obj_1_new_y_coord;

            int obj_1_old_display_x = (p_old[i].x_coord < 0) ? 0 : (p_old[i].x_coord > 639) ? 639
                                                                                             : (int)p_old[i].x_coord;
            int obj_1_old_display_y = (p_old[i].y_coord < 0) ? 0 : (p_old[i].y_coord > 479) ? 479
                                                                                             : (int)p_old[i].y_coord;

            p_new[i].x_coord = obj_1_new_x_coord;
            p_new[i].y_coord = obj_1_new_y_coord;

            p_new[i].x_vel = obj_1_new_x_vel;
            p_new[i].y_vel = obj_1_new_y_vel;

            p_new[i].old_display_x = p_new[i].x_coord / scale;
            p_new[i].old_display_y = p_new[i].y_coord / scale;

            VGA_disc(p_old[i].old_display_x, p_old[i].old_display_y, p_old[i].radius, black); // remove old

            VGA_disc(p_new[i].old_display_x, p_new[i].old_display_y, p_old[i].radius, p_old[i].color); // add new
        }
        gettimeofday(&t2, NULL);
        elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0; // sec to ms
        elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0; // us to ms

        VGA_text_clear();

        sprintf(screen_text, "%f ms.", elapsedTime);

        VGA_text(5, 52, screen_text);

        sprintf(screen_text, "%d particles.", particle_number);
        VGA_text(5, 53, screen_text);
        // delete old memory
        free(p_old);
        p_old = p_new;
    }

} // end main

/**************************************************************************************
* Random number within range (upper,lower)
**************************************************************************************/
int random_range(int lower, int upper) {
    int num = (rand() %
                   (upper - lower + 1)) +
              lower;
    return num;
}

/****************************************************************************************
* Subroutine to send a string of text to the VGA monitor
****************************************************************************************/
void VGA_text(int x, int y, char *text_ptr) {
    volatile char *character_buffer = (char *)vga_char_ptr; // VGA character buffer
    int offset;
    /* assume that the text string fits on one line */
    offset = (y << 7) + x;
    while (*(text_ptr)) {
        // write to the character buffer
        *(character_buffer + offset) = *(text_ptr);
        ++text_ptr;
        ++offset;
    }
}

/****************************************************************************************
* Subroutine to clear text to the VGA monitor
****************************************************************************************/
void VGA_text_clear() {
    volatile char *character_buffer = (char *)vga_char_ptr; // VGA character buffer
    int offset, x, y;
    for (x = 0; x < 79; x++) {
        for (y = 0; y < 59; y++) {
            /* assume that the text string fits on one line */
            offset = (y << 7) + x;
            // write to the character buffer
            *(character_buffer + offset) = ' ';
        }
    }
}

/****************************************************************************************
* Draw a filled rectangle on the VGA monitor
****************************************************************************************/
#define SWAP(X, Y) \
    do {           \
        int temp = X; \
        X = Y;        \
        Y = temp;     \
    } while (0)

void VGA_box(int x1, int y1, int x2, int y2, short pixel_color) {
    char *pixel_ptr;
    int row, col;

    /* check and fix box coordinates to be valid */
    if (x1 > 639)
        x1 = 639;
    if (y1 > 479)
        y1 = 479;
    if (x2 > 639)
        x2 = 639;
    if (y2 > 479)
        y2 = 479;
    if (x1 < 0)
        x1 = 0;
    if (y1 < 0)
        y1 = 0;
    if (x2 < 0)
        x2 = 0;
    if (y2 < 0)
        y2 = 0;
    if (x1 > x2)
        SWAP(x1, x2);
    if (y1 > y2)
        SWAP(y1, y2);
    for (row = y1; row <= y2; row++)
        for (col = x1; col <= x2; ++col) {
            // 640x480
            // pixel_ptr = (char *)vga_pixel_ptr + (row<<10) + col ;
            // set pixel color
            //*(char *)pixel_ptr = pixel_color;
            VGA_PIXEL(col, row, pixel_color);
        }
}

/****************************************************************************************
* Draw a outline rectangle on the VGA monitor
****************************************************************************************/
#define SWAP(X, Y) \
    do {           \
        int temp = X; \
        X = Y;        \
        Y = temp;     \
    } while (0)

void VGA_rect(int x1, int y1, int x2, int y2, short pixel_color) {
    char *pixel_ptr;
    int row, col;

    /* check and fix box coordinates to be valid */
    if (x1 > 639)
        x1 = 639;
    if (y1 > 479)
        y1 = 479;
    if (x2 > 639)
        x2 = 639;
    if (y2 > 479)
        y2 = 479;
    if (x1 < 0)
        x1 = 0;
    if (y1 < 0)
        y1 = 0;
    if (x2 < 0)
        x2 = 0;
    if (y2 < 0)
        y2 = 0;
    if (x1 > x2)
        SWAP(x1, x2);
    if (y1 > y2)
        SWAP(y1, y2);
    // left edge
    col = x1;
    for (row = y1; row <= y2; row++) {
        // 640x480
        // pixel_ptr = (char *)vga_pixel_ptr + (row<<10) + col ;
        // set pixel color
        //*(char *)pixel_ptr = pixel_color;
        VGA_PIXEL(col, row, pixel_color);
    }

    // right edge
    col = x2;
    for (row = y1; row <= y2; row++) {
        // 640x480
        // pixel_ptr = (char *)vga_pixel_ptr + (row<<10) + col ;
        // set pixel color
        //*(char *)pixel_ptr = pixel_color;
        VGA_PIXEL(col, row, pixel_color);
    }

    // top edge
    row = y1;
    for (col = x1; col <= x2; ++col) {
        // 640x480
        // pixel_ptr = (char *)vga_pixel_ptr + (row<<10) + col ;
        // set pixel color
        //*(char *)pixel_ptr = pixel_color;
        VGA_PIXEL(col, row, pixel_color);
    }

    // bottom edge
    row = y2;
    for (col = x1; col <= x2; ++col) {
        // 640x480
        // pixel_ptr = (char *)vga_pixel_ptr + (row<<10) + col ;
        // set pixel color
        //*(char *)pixel_ptr = pixel_color;
        VGA_PIXEL(col, row, pixel_color);
    }
}

/****************************************************************************************
* Draw a horixontal line on the VGA monitor
****************************************************************************************/
#define SWAP(X, Y) \
    do {           \
        int temp = X; \
        X = Y;        \
        Y = temp;     \
    } while (0)

void VGA_Hline(int x1, int y1, int x2, short pixel_color) {
    char *pixel_ptr;
    int row, col;

    /* check and fix box coordinates to be valid */
    if (x1 > 639)
        x1 = 639;
    if (y1 > 479)
        y1 = 479;
    if (x2 > 639)
        x2 = 639;
    if (x1 < 0)
        x1 = 0;
    if (y1 < 0)
        y1 = 0;
    if (x2 < 0)
        x2 = 0;
    if (x1 > x2)
        SWAP(x1, x2);
    // line
    row = y1;
    for (col = x1; col <= x2; ++col) {
        // 640x480
        // pixel_ptr = (char *)vga_pixel_ptr + (row<<10) + col ;
        // set pixel color
        //*(char *)pixel_ptr = pixel_color;
        VGA_PIXEL(col, row, pixel_color);
    }
}

/****************************************************************************************
* Draw a vertical line on the VGA monitor
****************************************************************************************/
#define SWAP(X, Y) \
    do {           \
        int temp = X; \
        X = Y;        \
        Y = temp;     \
    } while (0)

void VGA_Vline(int x1, int y1, int y2, short pixel_color) {
    char *pixel_ptr;
    int row, col;

    /* check and fix box coordinates to be valid */
    if (x1 > 639)
        x1 = 639;
    if (y1 > 479)
        y1 = 479;
    if (y2 > 479)
        y2 = 479;
    if (x1 < 0)
        x1 = 0;
    if (y1 < 0)
        y1 = 0;
    if (y2 < 0)
        y2 = 0;
    if (y1 > y2)
        SWAP(y1, y2);
    // line
    col = x1;
    for (row = y1; row <= y2; row++) {
        // 640x480
        // pixel_ptr = (char *)vga_pixel_ptr + (row<<10) + col ;
        // set pixel color
        //*(char *)pixel_ptr = pixel_color;
        VGA_PIXEL(col, row, pixel_color);
    }
}

/****************************************************************************************
* Draw a filled circle on the VGA monitor
****************************************************************************************/

void VGA_disc(int x, int y, int r, short pixel_color) {
    char *pixel_ptr;
    int row, col, rsqr, xc, yc;

    rsqr = r * r;

    for (yc = -r; yc <= r; yc++)
        for (xc = -r; xc <= r; xc++) {
            col = xc;
            row = yc;
            // add the r to make the edge smoother
            if (col * col + row * row <= rsqr + r) {
                col += x; // add the center point
                row += y; // add the center point
                // check for valid 640x480
                if (col > 639)
                    col = 639;
                if (row > 479)
                    row = 479;
                if (col < 0)
                    col = 0;
                if (row < 0)
                    row = 0;
                // pixel_ptr = (char *)vga_pixel_ptr + (row<<10) + col ;
                // set pixel color
                //*(char *)pixel_ptr = pixel_color;
                VGA_PIXEL(col, row, pixel_color);
            }
        }
}

/****************************************************************************************
* Draw a circle on the VGA monitor
****************************************************************************************/

void VGA_circle(int x, int y, int r, int pixel_color) {
    char *pixel_ptr;
    int row, col, rsqr, xc, yc;
    int col1, row1;
    rsqr = r * r;

    for (yc = -r; yc <= r; yc++) {
        // row = yc;
        col1 = (int)sqrt((float)(rsqr + r - yc * yc));
        // right edge
        col = col1 + x; // add the center point
        row = yc + y;   // add the center point
        // check for valid 640x480
        if (col > 639)
            col = 639;
        if (row > 479)
            row = 479;
        if (col < 0)
            col = 0;
        if (row < 0)
            row = 0;
        // pixel_ptr = (char *)vga_pixel_ptr + (row<<10) + col ;
        // set pixel color
        //*(char *)pixel_ptr = pixel_color;
        VGA_PIXEL(col, row, pixel_color);
        // left edge
        col = -col1 + x; // add the center point
        // check for valid 640x480
        if (col > 639)
            col = 639;
        if (row > 479)
            row = 479;
        if (col < 0)
            col = 0;
        if (row < 0)
            row = 0;
        // pixel_ptr = (char *)vga_pixel_ptr + (row<<10) + col ;
        // set pixel color
        //*(char *)pixel_ptr = pixel_color;
        VGA_PIXEL(col, row, pixel_color);
    }

    for (xc = -r; xc <= r; xc++) {
        // row = yc;
        row1 = (int)sqrt((float)(rsqr + r - xc * xc));
        // right edge
        col = xc + x; // add the center point
        row = row1 + y; // add the center point
        // check for valid 640x480
        if (col > 639)
            col = 639;
        if (row > 479)
            row = 479;
        if (col < 0)
            col = 0;
        if (row < 0)
            row = 0;
        // pixel_ptr = (char *)vga_pixel_ptr + (row<<10) + col ;
        // set pixel color
        //*(char *)pixel_ptr = pixel_color;
        VGA_PIXEL(col, row, pixel_color);
        // left edge
        row = -row1 + y; // add the center point
        // check for valid 640x480
        if (col > 639)
            col = 639;
        if (row > 479)
            row = 479;
        if (col < 0)
            col = 0;
        if (row < 0)
            row = 0;
        // pixel_ptr = (char *)vga_pixel_ptr + (row<<10) + col ;
        // set pixel color
        //*(char *)pixel_ptr = pixel_color;
        VGA_PIXEL(col, row, pixel_color);
    }
}

// =============================================
// === Draw a line
// =============================================
// plot a line
// at x1,y1 to x2,y2 with color
// Code is from David Rodgers,
//"Procedural Elements of Computer Graphics",1985
void VGA_line(int x1, int y1, int x2, int y2, short c) {
    int e;
    signed int dx, dy, j, temp;
    signed int s1, s2, xchange;
    signed int x, y;
    char *pixel_ptr;

    /* check and fix line coordinates to be valid */
    if (x1 > 639)
        x1 = 639;
    if (y1 > 479)
        y1 = 479;
    if (x2 > 639)
        x2 = 639;
    if (y2 > 479)
        y2 = 479;
    if (x1 < 0)
        x1 = 0;
    if (y1 < 0)
        y1 = 0;
    if (x2 < 0)
        x2 = 0;
    if (y2 < 0)
        y2 = 0;

    x = x1;
    y = y1;

    // take absolute value
    if (x2 < x1) {
        dx = x1 - x2;
        s1 = -1;
    } else if (x2 == x1) {
        dx = 0;
        s1 = 0;
    } else {
        dx = x2 - x1;
        s1 = 1;
    }

    if (y2 < y1) {
        dy = y1 - y2;
        s2 = -1;
    } else if (y2 == y1) {
        dy = 0;
        s2 = 0;
    } else {
        dy = y2 - y1;
        s2 = 1;
    }

    xchange = 0;

    if (dy > dx) {
        temp = dx;
        dx = dy;
        dy = temp;
        xchange = 1;
    }

    e = ((int)dy << 1) - dx;

    for (j = 0; j <= dx; j++) {
        // video_pt(x,y,c); //640x480
        // pixel_ptr = (char *)vga_pixel_ptr + (y<<10)+ x;
        // set pixel color
        //*(char *)pixel_ptr = c;
        VGA_PIXEL(x, y, c);

        if (e >= 0) {
            if (xchange == 1)
                x = x + s1;
            else
                y = y + s2;
            e = e - ((int)dx << 1);
        }

        if (xchange == 1)
            y = y + s2;
        else
            x = x + s1;

        e = e + ((int)dy << 1);
    }
}
