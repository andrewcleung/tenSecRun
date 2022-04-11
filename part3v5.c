
	/* This files provides address values that exist in the system */

#define SDRAM_BASE            0xC0000000
#define FPGA_ONCHIP_BASE      0xC8000000
#define FPGA_CHAR_BASE        0xC9000000

/* Cyclone V FPGA devices */
#define LEDR_BASE             0xFF200000
#define HEX3_HEX0_BASE        0xFF200020
#define HEX5_HEX4_BASE        0xFF200030
#define SW_BASE               0xFF200040
#define KEY_BASE              0xFF200050
#define TIMER_BASE            0xFF202000
#define PIXEL_BUF_CTRL_BASE   0xFF203020
#define CHAR_BUF_CTRL_BASE    0xFF203030

/* VGA colors */
#define WHITE 0xFFFF
#define YELLOW 0xFFE0
#define RED 0xF800
#define GREEN 0x07E0
#define BLUE 0x001F
#define CYAN 0x07FF
#define MAGENTA 0xF81F
#define GREY 0xC618
#define PINK 0xFC18
#define ORANGE 0xFC00

#define ABS(x) (((x) > 0) ? (x) : -(x))

/* Screen size. */
#define RESOLUTION_X 320
#define RESOLUTION_Y 240

/* Constants for animation */
#define BOX_LEN 2
#define NUM_BOXES 8

#define FALSE 0
#define TRUE 1
#define CLEAR 0
#define DRAW 1

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>

short int colors[] = {
	WHITE,
	YELLOW,
	RED,
	GREEN,
	BLUE,
	CYAN,
	MAGENTA,
	GREY,
	PINK,
	ORANGE,
};

struct box_d {
	//flag
	bool draw;
	//location
	int x;
	int y;
	int prevx;
	int prevy;
	int xdir;
	int ydir;
	short int color;
};

void plot_pixel(int x, int y, short int line_color);
void draw_line(int x0, int y0, int x1, int y1, short int line_color);
void clear_screen();
void swap(int *x, int *y);
void wait_for_vsync();
void draw_box(int x, int y, short int box_color);
void clear_draw(struct box_d *box);
void draw(struct box_d* box);


// Begin part3.c code for Lab 7

//number of box
#define NUMBOX 10
	
volatile int pixel_buffer_start; // global variable

int main(void)
{
    volatile int * pixel_ctrl_ptr = (int *)0xFF203020;
    // declare other variables(not shown)
	struct box_d box[NUMBOX];
    // initialize location and direction of rectangles(not shown)
	int i;
	for (i =0; i<NUMBOX ; i++) {
		box[i].draw = true;
		box[i].x = rand()%319;
		box[i].y = rand()%239;
		box[i].prevx = box[i].x;
		box[i].prevy = box[i].y;
		box[i].xdir = rand()%2 * 2 - 1;
		box[i].ydir = rand()%2 * 2 - 1;
		box[i].color = colors[(int)rand()%10];
	}
    /* set front pixel buffer to start of FPGA On-chip memory */
    *(pixel_ctrl_ptr + 1) = 0xC8000000; // first store the address in the 
                                        // back buffer
    /* now, swap the front/back buffers, to set the front buffer location */
    wait_for_vsync();
    /* initialize a pointer to the pixel buffer, used by drawing functions */
    pixel_buffer_start = *pixel_ctrl_ptr;
    clear_screen(); // pixel_buffer_start points to the pixel buffer
    /* set back pixel buffer to start of SDRAM memory */
    *(pixel_ctrl_ptr + 1) = 0xC0000000;
    pixel_buffer_start = *(pixel_ctrl_ptr + 1); // we draw on the back buffer
    clear_screen(); // pixel_buffer_start points to the pixel buffer

    while (1)
    {
        /* Erase any boxes and lines that were drawn in the last iteration */

		//clear_draw(box);
		clear_screen();
		int i;
        // code for updating the locations of boxes (not shown)
		for (i=0; i<NUMBOX; i++) {
			// Checks the direction
			if (box[i].x + box[i].xdir > 319 || box[i].x + box[i].xdir < 1)
				box[i].xdir = -box[i].xdir;
			if (box[i].y + box[i].ydir > 239 || box[i].y + box[i].ydir < 1)
				box[i].ydir = -box[i].ydir;
			
			box[i].prevx = box[i].x;
			box[i].prevy = box[i].y;
			
			box[i].x += box[i].xdir;
			box[i].y += box[i].ydir;
		}		
		// code for drawing the boxes and lines (not shown)
		draw(box);
		
        wait_for_vsync(); // swap front and back buffers on VGA vertical sync
        pixel_buffer_start = *(pixel_ctrl_ptr + 1); // new back buffer
    }
}

// code for subroutines (not shown)
void plot_pixel(int x, int y, short int line_color)
{
    *(short int *)(pixel_buffer_start + (y << 10) + (x << 1)) = line_color;
}

void draw_line(int x0, int y0, int x1, int y1, short int line_color) {
	bool is_steep = abs(y1-y0) > abs(x1-x0);
	if (is_steep) {
		swap(&x0, &y0);
		swap(&x1, &y1);
	}
	if (x0 > x1) {
		swap(&x0, &x1);
		swap(&y0, &y1);
	}
	int deltax = x1 - x0;
	int deltay = abs (y1-y0);
	int error = -(deltax/2);
	int y = y0;
	int y_step = y0 < y1 ? 1 : -1;
	
	int x;
	for (x = x0; x <= x1; x++) {
		if (is_steep) {
			plot_pixel(y, x, line_color);
		}
		else {
			plot_pixel(x, y, line_color);
		}
		error = error + deltay;
		if (error > 0) {
			y = y + y_step;
			error = error-deltax;
		}
	}
	return;
}

void swap(int *x, int *y) {
	int temp = *x;
	*x = *y;
	*y = temp;
	return;
}

void clear_screen() {
	int x;
	int y;
	for (x = 0; x < 320; x++) {
		for (y = 0; y < 240; y++) {
			*(short int *)(pixel_buffer_start + (y << 10) + (x << 1)) = 0x0;
		}
	}
}


void wait_for_vsync() {
	volatile int *pixel_ctrl_ptr = (int*) PIXEL_BUF_CTRL_BASE;
	register int status;
	
	*pixel_ctrl_ptr = 1;
	
	status = *(pixel_ctrl_ptr +3);
	while ((status & 0x01) !=0) {
		status = *(pixel_ctrl_ptr + 3);
	}
	return;
}

void draw_box(int x, int y, short int box_color) {
	volatile int *pixel_ctrl_ptr = (int*) PIXEL_BUF_CTRL_BASE;
	*(short int *)(pixel_buffer_start + (y << 10) + (x << 1)) = box_color;
	*(short int *)(pixel_buffer_start + (y+1 << 10) + (x << 1)) = box_color;
	*(short int *)(pixel_buffer_start + (y << 10) + (x+1 << 1)) = box_color;
	*(short int *)(pixel_buffer_start + (y+1 << 10) + (x+1 << 1)) = box_color;
}
		 
void clear_draw(struct box_d* box){
	
	int i;
	for (i = 0; i < NUMBOX; i++) {
		if (i == NUMBOX - 1) {
			draw_box(box[i].prevx, box[i].prevy, 0x0);
			draw_line(box[i].prevx, box[i].prevy, box[0].prevx, box[0].prevy, 0x0);
		}
		else {
			draw_box(box[i].prevx, box[i].prevy, 0x0);
			draw_line(box[i].prevx, box[i].prevy, box[i+1].prevx, box[i+1].prevy, 0x0);
		}
	}
}
void draw (struct box_d* box) {
	int i;
	for (i = 0; i < NUMBOX; i++) {
		if (i == NUMBOX - 1) {
			draw_box(box[i].x, box[i].y, box[i].color);
			draw_line(box[i].x, box[i].y, box[0].x, box[0].y, box[i].color);
		}
		else {
			draw_box(box[i].x, box[i].y, box[i].color);
			draw_line(box[i].x, box[i].y, box[i+1].x, box[i+1].y, box[i].color);
		}
	}
	return;
}