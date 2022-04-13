#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
/* This files provides address values that exist in the system */

#define BOARD "DE1-SoC"

/* Memory */
#define DDR_BASE 0x00000000
#define DDR_END 0x3FFFFFFF
#define A9_ONCHIP_BASE 0xFFFF0000
#define A9_ONCHIP_END 0xFFFFFFFF
#define SDRAM_BASE 0xC0000000
#define SDRAM_END 0xC3FFFFFF
#define FPGA_ONCHIP_BASE 0xC8000000
#define FPGA_ONCHIP_END 0xC803FFFF
#define FPGA_CHAR_BASE 0xC9000000
#define FPGA_CHAR_END 0xC9001FFF

/* Cyclone V FPGA devices */
#define LEDR_BASE 0xFF200000
#define HEX3_HEX0_BASE 0xFF200020
#define HEX5_HEX4_BASE 0xFF200030
#define SW_BASE 0xFF200040
#define KEY_BASE 0xFF200050
#define JP1_BASE 0xFF200060
#define JP2_BASE 0xFF200070
#define PS2_BASE 0xFF200100
#define PS2_DUAL_BASE 0xFF200108
#define JTAG_UART_BASE 0xFF201000
#define JTAG_UART_2_BASE 0xFF201008
#define IrDA_BASE 0xFF201020
#define TIMER_BASE 0xFF202000
#define AV_CONFIG_BASE 0xFF203000
#define PIXEL_BUF_CTRL_BASE 0xFF203020
#define CHAR_BUF_CTRL_BASE 0xFF203030
#define AUDIO_BASE 0xFF203040
#define VIDEO_IN_BASE 0xFF203060
#define ADC_BASE 0xFF204000

/* Cyclone V HPS devices */
#define HPS_GPIO1_BASE 0xFF709000
#define HPS_TIMER0_BASE 0xFFC08000
#define HPS_TIMER1_BASE 0xFFC09000
#define HPS_TIMER2_BASE 0xFFD00000
#define HPS_TIMER3_BASE 0xFFD01000
#define FPGA_BRIDGE 0xFFD0501C

/* ARM A9 MPCORE devices */
#define PERIPH_BASE 0xFFFEC000       // base address of peripheral devices
#define MPCORE_PRIV_TIMER 0xFFFEC600 // PERIPH_BASE + 0x0600

/* Interrupt controller (GIC) CPU interface(s) */
#define MPCORE_GIC_CPUIF 0xFFFEC100 // PERIPH_BASE + 0x100
#define ICCICR 0x00                 // offset to CPU interface control reg
#define ICCPMR 0x04                 // offset to interrupt priority mask reg
#define ICCIAR 0x0C                 // offset to interrupt acknowledge reg
#define ICCEOIR 0x10                // offset to end of interrupt reg
/* Interrupt controller (GIC) distributor interface(s) */
#define MPCORE_GIC_DIST 0xFFFED000 // PERIPH_BASE + 0x1000
#define ICDDCR 0x00                // offset to distributor control reg
#define ICDISER 0x100              // offset to interrupt set-enable regs
#define ICDICER 0x180              // offset to interrupt clear-enable regs
#define ICDIPTR 0x800              // offset to interrupt processor targets regs
#define ICDICFR 0xC00              // offset to interrupt configuration regs

/* Operating modes */
#define OM_SVC 0b10011;
#define OM_UNDEFINED 0b10011;
#define OM_USER 0b10000;
#define OM_ABORT 0b10111;
#define OM_IRQ 0b10010;
#define OM_FIQ 0b10001;

/* VGA colors */
#define BLACK 0x0
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

/* Game colour */
#define COLOR_PLATFORM 0xC618
#define COLOR_PLATFORM_BORDER 0x0
#define COLOR_START 0xF800
#define COLOR_END 0x001F
#define COLOR_PLAYER 0xF800

/* Animation definitions */
#define ABS(x) (((x) > 0) ? (x) : -(x))

/* Screen size. */
#define RESOLUTION_X 320
#define RESOLUTION_Y 240

/* Object block size */
/* DO NOT CHANGE THIS*/
#define BOX_LEN 8

#define BLOCK_RESOLUTION_X 40
#define BLOCK_RESOLUTION_Y 30

/* Player attributes */
#define PLAYER_SPEED_NORMAL 5

/* level attirbutes */
#define NUMBER_OF_LEVELS 2

/* Player size */
#define NUM_JOINTS 11
#define TEMP 100
#define HEAD_RADIUS 6
#define TORSO_LENGTH 8
#define TORSO_LENGTH_30 2
#define TORSO_LENGTH_60 6
#define JOINT_LENGTH_30 2
#define JOINT_LENGTH_45 4
#define JOINT_LENGTH_60 5

/**************************** Data structures ************************************/
/* On screen objects */
enum GameObject
{
    GAMEOBJ_EMPTY = 0, // default value of the enum is empty
    GAMEOBJ_PLATFORM_BLOCK,
    GAMEOBJ_FIREBALL,
    GAMEOBJ_SPIKE,
    GAMEOBJ_START,
    GAMEOBJ_END,
    GAMEOBJ_THANOS
};

enum PlayerState
{
    PLAYERSTATE_STILL = 0,
    PLAYERSTATE_LEFT,
    PLAYERSTATE_RIGHT
};

/* Use for declaring a position of an object, where x<320 and y<240 */
typedef struct position
{
    int x;
    int y;
} Position;

/* Use for declaring a relative position of an object, where x<40 and y<30 */
typedef struct position_boxlen
{
    int x;
    int y;
} Position_boxlen;

/* Information of the current player */
typedef struct player
{
    Position pos;
    enum PlayerState state;
    int horizontal_speed;
    int vertical_speed;
    bool jump;
} Player;

typedef struct levels
{
    int levelNumber;
    Position_boxlen start;
    Position_boxlen end;
    enum GameObject levelObjects[BLOCK_RESOLUTION_X][BLOCK_RESOLUTION_Y];
} Levels;

enum GameCurrentProgress
{
    GAMEPROG_BEFORE = 0,
    GAMEPROG_START,
    GAMEPROG_WIN,
    GAMEPROG_LOSE
};

/* State of the game */
typedef struct gamestate
{
    enum GameCurrentProgress progress;
    int level;
    Position_boxlen start;
    Position_boxlen end;
    Player myPlayer;
    /* Onscreen objects */
    enum GameObject currentObjects[BLOCK_RESOLUTION_X][BLOCK_RESOLUTION_Y];
} GameState;

/* Functions declarations */
/**************************** Interrupt controls ***********************************/
void disable_A9_interrupts(void);
void set_A9_IRQ_stack(void);
void config_GIC(void);
void config_KEYs(void);              // configure the KEYs
void config_mpcore_priv_timer(void); // configure A9 private timer
void enable_A9_interrupts(void);
void config_interrupt(int, int);
// Helper function to setup all devices
void setup_interrupts(void);

/* Interrupt routines */
void pushbutton_ISR(void);
void mpcore_priv_timer_ISR(void);

/**************************** Timer routines ***********************************/
void start_mpcore_priv_timer(void);
void stop_mpcore_priv_timer(void);

/**************************** VGA setup routine ***********************************/
void setupVGA(void);   // Setup the front and back buffer of the VGA display
void wait_for_vsync(); // Swap between front and back buffer
void clear_screen();

/**************************** Level setup routine ***********************************/
void setupLevels();
void setupLevels_lv1();
void updateLevel(Levels level);

/**************************** Draw routine ************************************/
/* Game objects */
void plot_pixel(int x, int y, short int line_color); // plot individual pixels on the VGA coordinate
void drawCurrentObjects();                           // draw the current objects on the screen
void drawPlatformBlock(int baseX, int baseY);        // draw the platform block
void drawStart(int baseX, int baseY);                // draw the starting block
void drawEnd(int baseX, int baseY);                  // draw the ending block
void drawEmpty(int baseX, int baseY);                // draw the empty space

/* Testing objects */
void drawTestBox(int vgaX, int vgaY);

/* Player */
void drawPlayerResting();
void drawPlayerRunningRight();
void drawPlayer(int x_box[NUM_JOINTS], int y_box[NUM_JOINTS]);

/* Shapes */
void drawLine(int x0, int y0, int x1, int y1, short int line_color);
void drawCircle(int x_center, int y_centerc, int r, short int line_color);
void drawBox(int vgaX, int vgaY, int boxLen, short int boxColor);
void drawBoxWithBorder(int vgaX, int vgaY, int boxLen, int borderWid, short int boxColor, short int borderColor);

/**************************** Animation routine ************************************/
void refreshAnimation();

/* Player phyisics */
void updatePlayerSpeed(); // updates the player's horizontal and veritcal speed

/**************************** Game handler ************************************/
void resetGame();
void updateCurrentGame();

/**************************** Input handler ************************************/
void ps2KeyboardInputHandler(char byte1, char byte2, char byte3);

/**************************** Helper Functions ************************************/
void swap(int *A, int *B);

/**************************** Global variables ************************************/
Levels gameLevels[NUMBER_OF_LEVELS];
GameState myGame;

/* VGA buffer */
volatile int pixel_buffer_start;

/* ===========!!!!WARNING!!!================= */
/* ===========!!!!DO NOT MODIFY BELOW!!!================= */

/* setup the KEY interrupts in the FPGA */
void config_KEYs()
{
    volatile int *KEY_ptr = (int *)0xFF200050; // KEY base address
    *(KEY_ptr + 2) = 0xF;                      // enable interrupts for all four KEYs
}

/* Setup the A9 private timer, you have to manually start the timer using start_mpcore_priv_timer */
void config_mpcore_priv_timer(void)
{
    volatile int *MPcore_private_timer_ptr = (int *)MPCORE_PRIV_TIMER;
    int counter = 1562500;                   // timeout = 1/(200 MHz) x 200x10^6 = 1 sec
    *(MPcore_private_timer_ptr) = counter;   // write to timer load register
    *(MPcore_private_timer_ptr + 2) = 0b110; // interrupt = 1 (enable interrupt),
    // mode = 1 (auto), enable = 0 (start timer later)
}

// Define the IRQ exception handler
void __attribute__((interrupt)) __cs3_isr_irq(void)
{
    // Read the ICCIAR from the CPU Interface in the GIC
    int interrupt_ID = *((int *)0xFFFEC10C);
    if (interrupt_ID == 73) // check if interrupt is from the KEYs
        pushbutton_ISR();
    else if (interrupt_ID == 29)
        mpcore_priv_timer_ISR();
    else
        while (1)
            ; // if unexpected, then stay here
    // Write to the End of Interrupt Register (ICCEOIR)
    *((int *)0xFFFEC110) = interrupt_ID;
}
// Define the remaining exception handlers
void __attribute__((interrupt)) __cs3_reset(void)
{
    while (1)
        ;
}
void __attribute__((interrupt)) __cs3_isr_undef(void)
{
    while (1)
        ;
}
void __attribute__((interrupt)) __cs3_isr_swi(void)
{
    while (1)
        ;
}
void __attribute__((interrupt)) __cs3_isr_pabort(void)
{
    while (1)
        ;
}
void __attribute__((interrupt)) __cs3_isr_dabort(void)
{
    while (1)
        ;
}
void __attribute__((interrupt)) __cs3_isr_fiq(void)
{
    while (1)
        ;
}

/*
 * Turn on interrupt in the ARM processor
 */
void enable_A9_interrupts(void)
{
    int status = 0b01010011;
    asm("msr cpsr, %[ps]"
        :
        : [ps] "r"(status));
}

/*
 * Turn off interrupt in the ARM processor
 */
void disable_A9_interrupts(void)
{
    int status = 0b11010011;
    asm("msr cpsr, %[ps]"
        :
        : [ps] "r"(status));
}

/*
 * Configure the Generic Interrupt Controller (GIC)
 */
void config_GIC(void)
{
    config_interrupt(73, 1); // configure the FPGA KEYs interrupt (73)
    config_interrupt(29, 1); // configure the A9 private timer (29)
    // Set Interrupt Priority Mask Register (ICCPMR). Enable all priorities
    *((int *)0xFFFEC104) = 0xFFFF;
    // Set the enable in the CPU Interface Control Register (ICCICR)
    *((int *)0xFFFEC100) = 1;
    // Set the enable in the Distributor Control Register (ICDDCR)
    *((int *)0xFFFED000) = 1;
}
/*
 * Configure registers in the GIC for an individual Interrupt ID. We
 * configure only the Interrupt Set Enable Registers (ICDISERn) and
 * Interrupt Processor Target Registers (ICDIPTRn). The default (reset)
 * values are used for other registers in the GIC
 */
void config_interrupt(int N, int CPU_target)
{
    int reg_offset, index, value, address;
    /* Configure the Interrupt Set-Enable Registers (ICDISERn).
     * reg_offset = (integer_div(N / 32) * 4; value = 1 << (N mod 32) */
    reg_offset = (N >> 3) & 0xFFFFFFFC;
    index = N & 0x1F;
    value = 0x1 << index;
    address = 0xFFFED100 + reg_offset;
    /* Using the address and value, set the appropriate bit */
    *(int *)address |= value;
    /* Configure the Interrupt Processor Targets Register (ICDIPTRn)
     * reg_offset = integer_div(N / 4) * 4; index = N mod 4 */
    reg_offset = (N & 0xFFFFFFFC);
    index = N & 0x3;
    address = 0xFFFED800 + reg_offset + index;
    /* Using the address and value, write to (only) the appropriate byte */
    *(char *)address = (char)CPU_target;
}

/*
 * Initialize the banked stack pointer register for IRQ mode
 */
void set_A9_IRQ_stack(void)
{
    int stack, mode;
    stack = 0xFFFFFFFF - 7; // top of A9 onchip memory, aligned to 8 bytes
    /* change processor to IRQ mode with interrupts disabled */
    mode = 0b11010010;
    asm("msr cpsr, %[ps]"
        :
        : [ps] "r"(mode));
    /* set banked stack pointer */
    asm("mov sp, %[ps]"
        :
        : [ps] "r"(stack));
    /* go back to SVC mode before executing subroutine return! */
    mode = 0b11010011;
    asm("msr cpsr, %[ps]"
        :
        : [ps] "r"(mode));
}
/* ===========!!!!WARNING!!!================= */
/* ===========!!!!DO NOT MODIFY ABOVE!!!================= */

/********************************************************************
 * Pushbutton - Interrupt Service Routine
 *
 * This routine checks which KEY has been pressed. It writes to HEX0
 *******************************************************************/
void pushbutton_ISR(void)
{
    /* KEY base address */
    volatile int *KEY_ptr = (int *)KEY_BASE;
    /* HEX display base address */
    volatile int *HEX3_HEX0_ptr = (int *)HEX3_HEX0_BASE;
    int press, HEX_bits;
    press = *(KEY_ptr + 3); // read the pushbutton interrupt register
    *(KEY_ptr + 3) = press; // Clear the interrupt
    if (press & 0x1)        // KEY0
        HEX_bits = 0b00111111;
    else if (press & 0x2) // KEY1
        HEX_bits = 0b00000110;
    else if (press & 0x4) // KEY2
        HEX_bits = 0b01011011;
    else // press & 0x8, which is KEY3
        HEX_bits = 0b01001111;
    *HEX3_HEX0_ptr = HEX_bits;
    return;
}

/********************************************************************
 * mpcore_priv_timer_ISR - Interrupt Service Routine
 *
 * This routine is specifically use for drawing the frames in the game
 *******************************************************************/
void mpcore_priv_timer_ISR(void)
{
    volatile int *MPcore_private_timer_ptr = (int *)MPCORE_PRIV_TIMER;

    // draw routine
    updatePlayerSpeed();
    refreshAnimation();
    myGame.myPlayer.pos.x += myGame.myPlayer.horizontal_speed;

    *(MPcore_private_timer_ptr + 3) = 1; // reset timer flag bit
}

// Helper function to setup all devices
void setup_interrupts(void)
{
    /* Interrupt setup routine */
    disable_A9_interrupts(); // disable interrupts in the A9 processor
    // Stack and GIC configurations
    set_A9_IRQ_stack(); // initialize the stack pointer for IRQ mode
    config_GIC();       // configure the general interrupt controller

    // Configure the devices
    config_KEYs(); // configure KEYs to generate interrupts
    config_mpcore_priv_timer();
    enable_A9_interrupts(); // enable interrupts in the A9 processor
}

/********************************************************************
 * start_mpcore_priv_timer
 *
 * starts the A9 private timer
 *******************************************************************/
void start_mpcore_priv_timer(void)
{
    volatile int *MPcore_private_timer_ptr = (int *)MPCORE_PRIV_TIMER;
    *(MPcore_private_timer_ptr + 2) = 0b111;
}

/********************************************************************
 * stop_mpcore_priv_timer
 *
 * stops the A9 private timer
 *******************************************************************/
void stop_mpcore_priv_timer(void)
{
    volatile int *MPcore_private_timer_ptr = (int *)MPCORE_PRIV_TIMER;
    *(MPcore_private_timer_ptr + 2) = 0b110;
}

/********************************************************************
 * setupVGA
 *
 * The function setups the front and back buffer of the VGA display
 *******************************************************************/
void setupVGA(void)
{
    volatile int *pixel_ctrl_ptr = (int *)0xFF203020;
    /* set front pixel buffer to start of FPGA On-chip memory */
    *(pixel_ctrl_ptr + 1) = 0xC8000000; // first store the address in the
                                        // back buffer
    /* now, swap the front/back buffers, to set the front buffer location */
    wait_for_vsync();
    /* initialize a pointer to the pixel buffer, used by drawing functions */
    pixel_buffer_start = *pixel_ctrl_ptr; // pixel_buffer_start points to the pixel buffer

    /* set back pixel buffer to start of SDRAM memory */
    *(pixel_ctrl_ptr + 1) = 0xC0000000;         // we draw on the back buffer
    pixel_buffer_start = *(pixel_ctrl_ptr + 1); // pixel_buffer_start points to the pixel buffer
}

/********************************************************************
 * wait_for_vsync
 *
 * The function swaps the front and back buffer
 *******************************************************************/
void wait_for_vsync()
{
    volatile int *pixel_ctrl_ptr = (int *)PIXEL_BUF_CTRL_BASE;
    register int status;

    *pixel_ctrl_ptr = 1;

    status = *(pixel_ctrl_ptr + 3);
    while ((status & 0x01) != 0)
    {
        status = *(pixel_ctrl_ptr + 3);
    }
    return;
}

/********************************************************************
 * clear_screen
 *
 * Clear the whole screen to white
 *******************************************************************/
void clear_screen()
{
    int x;
    int y;
    for (x = 0; x < RESOLUTION_X; x++)
    {
        for (y = 0; y < RESOLUTION_Y; y++)
        {
            plot_pixel(x, y, WHITE);
        }
    }
    refreshAnimation();
}

/********************************************************************
 * setupLevels
 *
 * Routine to setup all the leves
 *******************************************************************/
void setupLevels()
{
    setupLevels_lv1();
}

/********************************************************************
 * setupLevels_lv1
 *
 * setup level 1 object
 *******************************************************************/
void setupLevels_lv1()
{
    // update the level number
    gameLevels[0].levelNumber = 1;

    // draw the platform for level 1
    for (int x = 0; x < BLOCK_RESOLUTION_X; x++)
    {
        gameLevels[0].levelObjects[x][20] = GAMEOBJ_PLATFORM_BLOCK;
    }

    // create gaps
    for (int x = 20; x < 23; x++)
    {
        gameLevels[0].levelObjects[x][20] = GAMEOBJ_EMPTY;
    }

    // starting position
    gameLevels[0].levelObjects[2][20] = GAMEOBJ_START;
    gameLevels[0].start.x = 2;
    gameLevels[0].start.y = 20;

    // ending position
    gameLevels[0].levelObjects[38][20] = GAMEOBJ_END;
    gameLevels[0].end.x = 38;
    gameLevels[0].end.y = 20;
}

/********************************************************************
 * updateLevel(int level)
 *
 * update the current level
 *******************************************************************/
void updateLevel(Levels level)
{
    // updating the current object array
    for (int x = 0; x < BLOCK_RESOLUTION_X; x++)
    {
        for (int y = 0; y < BLOCK_RESOLUTION_Y; y++)
        {
            myGame.currentObjects[x][y] = level.levelObjects[x][y];
        }
    }
    myGame.start = level.start;
    myGame.end = level.end;
    myGame.level = level.levelNumber;
    // Update my player
    myGame.myPlayer.pos.x = myGame.start.x * BOX_LEN;                          // scale to VGA position
    myGame.myPlayer.pos.y = (myGame.start.y - 1 /* player height*/) * BOX_LEN; // scale to VGA position
}

/********************************************************************
 * plot_pixels
 *
 * draw the pixel on the screen
 *******************************************************************/
void plot_pixel(int x, int y, short int line_color)
{
    *(short int *)(pixel_buffer_start + (y << 10) + (x << 1)) = line_color;
}

/********************************************************************
 * drawCurrentObjects
 *
 * draw current objects on the screen
 *******************************************************************/
void drawCurrentObjects()
{
    for (int x = 0; x < BLOCK_RESOLUTION_X; x++)
    {
        for (int y = 0; y < BLOCK_RESOLUTION_Y; y++)
        {
            switch (myGame.currentObjects[x][y])
            {
            case GAMEOBJ_EMPTY:
                drawEmpty(x, y);
                break;
            case GAMEOBJ_PLATFORM_BLOCK:
                drawPlatformBlock(x, y);
                break;
            case GAMEOBJ_FIREBALL:
                break;
            case GAMEOBJ_SPIKE:
                break;
            case GAMEOBJ_START:
                drawStart(x, y);
                break;
            case GAMEOBJ_END:
                drawEnd(x, y);
                break;
            case GAMEOBJ_THANOS:
                break;
            default:
                drawEmpty(x, y);
                break;
            }
        }
    }
}

/********************************************************************
 * drawPlatformBlock
 *
 * draw all the platform blocks
 *******************************************************************/
void drawPlatformBlock(int baseX, int baseY)
{
    drawBoxWithBorder(baseX*BOX_LEN, baseY*BOX_LEN, BOX_LEN, 1, COLOR_PLATFORM, COLOR_PLATFORM_BORDER);
}

/********************************************************************
 * drawStart
 *
 * draw the starting block
 *******************************************************************/
void drawStart(int baseX, int baseY)
{
    drawBoxWithBorder(baseX*BOX_LEN, baseY*BOX_LEN, BOX_LEN, 1, COLOR_START, COLOR_PLATFORM_BORDER);
}

/********************************************************************
 * drawEnd
 *
 * draw the ending block
 *******************************************************************/
void drawEnd(int baseX, int baseY)
{
    drawBoxWithBorder(baseX*BOX_LEN, baseY*BOX_LEN, BOX_LEN, 1, COLOR_END, COLOR_PLATFORM_BORDER);
}

/********************************************************************
 * drawEmpty
 *
 * draw all the platform blocks
 *******************************************************************/
void drawEmpty(int baseX, int baseY)
{
    for (int x = 0; x < BOX_LEN; x++)
    {
        for (int y = 0; y < BOX_LEN; y++)
        {
            plot_pixel(baseX * BOX_LEN + x, baseY * BOX_LEN + y, WHITE);
        }
    }
}

void drawPlayerResting()
{
    int x_box[NUM_JOINTS];
    int y_box[NUM_JOINTS];
    /*draw human, from 100 x 100 */
    // Head
    x_box[0] = TEMP;
    y_box[0] = TEMP;

    // Neck
    x_box[1] = x_box[0];
    y_box[1] = y_box[0] + HEAD_RADIUS;
    // Left arm
    x_box[2] = x_box[0] - JOINT_LENGTH_45;
    y_box[2] = y_box[1] + JOINT_LENGTH_45;
    // Left hand
    x_box[3] = x_box[2] - JOINT_LENGTH_30;
    y_box[3] = y_box[2] + JOINT_LENGTH_60;
    // Right arm
    x_box[4] = x_box[0] + JOINT_LENGTH_30;
    y_box[4] = y_box[1] + JOINT_LENGTH_60;
    // Right hand
    x_box[5] = x_box[4] + JOINT_LENGTH_45;
    y_box[5] = y_box[4] + JOINT_LENGTH_45;
    // Balls
    x_box[6] = x_box[1];
    y_box[6] = y_box[1] + TORSO_LENGTH;
    // left leg
    x_box[7] = x_box[6] - JOINT_LENGTH_30;
    y_box[7] = y_box[6] + JOINT_LENGTH_60;
    // left foot
    x_box[8] = x_box[7] - JOINT_LENGTH_45;
    y_box[8] = y_box[7] + JOINT_LENGTH_45;
    // right leg
    x_box[9] = x_box[6] + JOINT_LENGTH_45;
    y_box[9] = y_box[6] + JOINT_LENGTH_45;
    // right foot
    x_box[10] = x_box[9] + JOINT_LENGTH_30;
    y_box[10] = y_box[9] + JOINT_LENGTH_60;

    drawPlayer(x_box, y_box);
}

void drawPlayerRunningRight()
{
    int x_box[NUM_JOINTS];
    int y_box[NUM_JOINTS];
    /*draw human, from 100 x 100 */
    // Head
    x_box[0] = TEMP + 32;
    y_box[0] = TEMP;

    // Neck
    x_box[1] = x_box[0] - 2;
    y_box[1] = y_box[0] + HEAD_RADIUS;
    // Left arm
    x_box[2] = x_box[0] - JOINT_LENGTH_60;
    y_box[2] = y_box[1] - JOINT_LENGTH_30;
    // Left hand
    x_box[3] = x_box[2] - JOINT_LENGTH_45;
    y_box[3] = y_box[2] + JOINT_LENGTH_45;
    // Right arm
    x_box[4] = x_box[0] + JOINT_LENGTH_45;
    y_box[4] = y_box[1] + JOINT_LENGTH_45;
    // Right hand
    x_box[5] = x_box[4] + JOINT_LENGTH_60;
    y_box[5] = y_box[4] - JOINT_LENGTH_30;
    // Balls
    x_box[6] = x_box[1] - TORSO_LENGTH_30;
    y_box[6] = y_box[1] + TORSO_LENGTH_60;
    // left leg
    x_box[7] = x_box[6] - JOINT_LENGTH_45;
    y_box[7] = y_box[6] + JOINT_LENGTH_45;
    // left foot
    x_box[8] = x_box[7] - JOINT_LENGTH_45;
    y_box[8] = y_box[7] + JOINT_LENGTH_45;
    // right leg
    x_box[9] = x_box[6] + JOINT_LENGTH_45;
    y_box[9] = y_box[6] + JOINT_LENGTH_45;
    // right foot
    x_box[10] = x_box[9] - JOINT_LENGTH_45;
    y_box[10] = y_box[9] + JOINT_LENGTH_45;

    drawPlayer(x_box, y_box);
}

/*
 * draw Player
 */
void drawPlayer(int x_box[NUM_JOINTS], int y_box[NUM_JOINTS])
{
    // draw head
    drawCircle(x_box[0], x_box[0], HEAD_RADIUS, COLOR_PLAYER);
    // draw neck to left arm
    drawLine(x_box[1], y_box[1], x_box[2], y_box[2],
             COLOR_PLAYER);
    // draw left arm to left hand
    drawLine(x_box[2], y_box[2], x_box[3], y_box[3],
             COLOR_PLAYER);
    // draw neck to right arm
    drawLine(x_box[1], y_box[1], x_box[4], y_box[4],
             COLOR_PLAYER);
    // draw right arm to right hand
    drawLine(x_box[4], y_box[4], x_box[5], y_box[5],
             COLOR_PLAYER);
    // draw neck to balls
    drawLine(x_box[1], y_box[1], x_box[6], y_box[6],
             COLOR_PLAYER);
    // draw balls to left leg
    drawLine(x_box[6], y_box[6], x_box[7], y_box[7],
             COLOR_PLAYER);
    // draw left leg to left foot
    drawLine(x_box[7], y_box[7], x_box[8], y_box[8],
             COLOR_PLAYER);
    // draw balls to right leg
    drawLine(x_box[6], y_box[6], x_box[9], y_box[9],
             COLOR_PLAYER);
    // draw right leg to right foot
    drawLine(x_box[9], y_box[9], x_box[10], y_box[10],
             COLOR_PLAYER);
}

/********************************************************************
 * drawTestBox(int vgaX, int vgaY)
 *
 * the function draws a testing object  in the VGA coordinates
 *******************************************************************/
void drawTestBox(int vgaX, int vgaY)
{
    drawBox(vgaX, vgaY, BOX_LEN, GREEN);
}

/********************************************************************
 * void refreshAnimation();
 *
 * the function refreshes all the drawing using the timer
 *******************************************************************/
void refreshAnimation()
{
    volatile int *pixel_ctrl_ptr = (int *)0xFF203020;
    pixel_buffer_start = *(pixel_ctrl_ptr + 1); // new back buffer
    drawCurrentObjects();
    drawTestBox(myGame.myPlayer.pos.x, myGame.myPlayer.pos.y);
    wait_for_vsync();
}

/********************************************************************
 * void updatePlayerSpeed();
 *
 * updates the player's horizontal and veritcal speed
 *******************************************************************/
void updatePlayerSpeed()
{
    // horizontal
    int baseSpeed = PLAYER_SPEED_NORMAL;
    switch (myGame.myPlayer.state)
    {
    case PLAYERSTATE_LEFT:
        myGame.myPlayer.horizontal_speed = -baseSpeed;
        break;
    case PLAYERSTATE_RIGHT:
        myGame.myPlayer.horizontal_speed = baseSpeed;
        break;
    case PLAYERSTATE_STILL:
        myGame.myPlayer.horizontal_speed = 0;
        break;
    default:
        myGame.myPlayer.horizontal_speed = 0;
    }
}

/********************************************************************
 * void resetGame();
 *
 * reset the game to its initial state
 *******************************************************************/
void resetGame()
{
    return;
}

/********************************************************************
 * void updateCurrentGame();
 *
 * update the current player and game state based on the flags
 *******************************************************************/
void updateCurrentGame()
{
    return;
}

/********************************************************************
 * void ps2KeyboardInputHandler(char byte1, char byte2, char byte3);
 *
 * the function handles different input by the keyboard
 *******************************************************************/
void ps2KeyboardInputHandler(char byte1, char byte2, char byte3)
{
    if (byte3 == 0xF0)
        return;
    // player movements
    if (byte2 == 0xF0)
    {
        myGame.myPlayer.state = PLAYERSTATE_STILL;
        return;
    }
    // move to the left
    if (byte3 == 0x1C)
    {
        myGame.myPlayer.state = PLAYERSTATE_LEFT;
        return;
    }
    if (byte3 == 0x23)
    {
        myGame.myPlayer.state = PLAYERSTATE_RIGHT;
        return;
    }
}

/********************************************************************
 * drawLine(int x0, int y0, int x1, int y1, short int line_color)
 *
 * the function draws a line in the VGA coordinates
 *******************************************************************/
void drawLine(int x0, int y0, int x1, int y1, short int line_color)
{
    bool is_steep = ABS(y1 - y0) > ABS(x1 - x0);
    if (is_steep)
    {
        swap(&x0, &y0);
        swap(&x1, &y1);
    }
    if (x0 > x1)
    {
        swap(&x0, &x1);
        swap(&y0, &y1);
    }
    int deltax = x1 - x0;
    int deltay = ABS(y1 - y0);
    int error = -(deltax / 2);
    int y = y0;
    int y_step = 0;
    if (y0 < y1)
        y_step = 1;
    else
        y_step = -1;

    for (int x = x0; x < x1; ++x)
    {
        if (is_steep)
            plot_pixel(y, x, line_color);
        else
            plot_pixel(x, y, line_color);

        error = error + deltay;

        if (error > 0)
        {
            y = y + y_step;
            error = error - deltax;
        }
    }
}

/********************************************************************
 * drawCircle(int x_center, int y_center, int r)
 *
 * Draw a circle on the VGA coordinate
 *******************************************************************/
void drawCircle(int x_center, int y_center, int r, short int line_color)
{
    int x = 0, y = r;
    int d = 3 - 2 * r;
    while (y >= x)
    {
        plot_pixel(x_center + x, y_center + y, line_color);
        plot_pixel(x_center - x, y_center + y, line_color);
        plot_pixel(x_center + x, y_center - y, line_color);
        plot_pixel(x_center - x, y_center - y, line_color);
        plot_pixel(x_center + y, y_center + x, line_color);
        plot_pixel(x_center - y, y_center + x, line_color);
        plot_pixel(x_center + y, y_center - x, line_color);
        plot_pixel(x_center - y, y_center - x, line_color);
        x++;
        if (d > 0)
        {
            y--;
            d = 10 + 4 * (x - y) + d;
        }
        else
            d = 6 + d + 4 * x;
    }
}

/********************************************************************
 * void drawBox(int vgaX, int vgaY, int boxLen, short int boxColor);
 *
 * Draw a box on the VGA coordinate
 *******************************************************************/
void drawBox(int vgaX, int vgaY, int boxLen, short int boxColor)
{
    for (int x = 0; x < boxLen; x++)
    {
        for (int y = 0; y < boxLen; y++)
        {
            plot_pixel(vgaX + x, vgaY + y, boxColor);
        }
    }
}

/********************************************************************
 * void drawBoxWithBorder(int vgaX, int vgaY, int boxLen,int borderWid, short int boxColor, short int borderColor);
 *
 * Draw a box with borderon the VGA coordinate
 *******************************************************************/
void drawBoxWithBorder(int vgaX, int vgaY, int boxLen, int borderWid, short int boxColor, short int borderColor)
{
    for (int x = 0; x < boxLen; x++)
    {
        for (int y = 0; y < boxLen; y++)
        {
            if (x < borderWid || y < borderWid || x >= boxLen - borderWid || y >= boxLen - borderWid)
                plot_pixel(vgaX + x, vgaY + y, borderColor);
            else
                plot_pixel(vgaX + x, vgaY + y, boxColor);
        }
    }
}
    /********************************************************************
     * swap(int *A, int *B)
     *
     * the function swaps the value of A and B
     * Note A and B are pointers
     *******************************************************************/
    void swap(int *A, int *B)
    {
        int temp = *A;
        *A = *B;
        *B = temp;
    }

    int main(void)
    {
        // Setup all the levels before the game
        setupLevels();
        updateLevel(gameLevels[0]);

        // Interrupt setup routine
        setup_interrupts();

        // VGA setup routine
        setupVGA();
        clear_screen();

        start_mpcore_priv_timer();

        /* Declare volatile pointers to I/O registers (volatile means that IO load
          and store instructions will be used to access these pointer locations,
          instead of regular memory loads and stores) */
        volatile int *PS2_ptr = (int *)PS2_BASE;
        int PS2_data, RVALID;
        char byte1 = 0, byte2 = 0, byte3 = 0;
        // PS/2 mouse needs to be reset (must be already plugged in)
        *(PS2_ptr) = 0xFF; // reset

        while (1)
        {
            PS2_data = *(PS2_ptr);      // read the Data register in the PS/2 port
            RVALID = PS2_data & 0x8000; // extract the RVALID field
            if (RVALID)
            {
                /* shift the next data byte into the display */
                byte1 = byte2;
                byte2 = byte3;
                byte3 = PS2_data & 0xFF;
                ps2KeyboardInputHandler(byte1, byte2, byte3);
            }
        }
    }