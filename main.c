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
#define BOX_LEN 10

#define BLOCK_RESOLUTION_X 32
#define BLOCK_RESOLUTION_Y 24

/* Physics */
#define PLAYER_SPEED_NORMAL 8
#define GRAVITY -10
#define PLAYER_INITIAL_UP_VELOCITY 40

/* level attirbutes */
#define NUMBER_OF_LEVELS 3

/* Player size in base coordinate */
#define PLAYER_HEIGHT_BASE 2
#define PLAYER_WIDTH_BASE 1
#define PLAYER_HEIGHT 20
#define PLAYER_WIDTH 10

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
    bool airborne;
} Player;

typedef struct levels
{
    int levelNumber;
    Position_boxlen start;
    Position_boxlen end;
    enum GameObject levelObjects[BLOCK_RESOLUTION_X][BLOCK_RESOLUTION_Y];
} Levels;

typedef struct timer
{
    int frame;
    int countDown;
} Timer;

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
void setupLevels_lv2();
void setupLevels_lv3();
void updateLevel(Levels level);

/**************************** Draw routine ************************************/
/* Game objects */
void plot_pixel(int x, int y, short int line_color); // plot individual pixels on the VGA coordinate
void drawCurrentObjects();                           // draw the current objects on the screen
void drawPlatformBlock(int baseX, int baseY);        // draw the platform block
void drawStart(int baseX, int baseY);                // draw the starting block
void drawEnd(int baseX, int baseY);                  // draw the ending block
void drawEmpty(int baseX, int baseY);                // draw the empty space

/* Timer */
void drawTimer(int count);

/* Title screen */
void drawBigTitle();

/* Winning screen */
void drawWinningScreen();

/* Game losing screen */
void drawLosingScreen();

/* Testing objects */
void drawTestBox(int vgaX, int vgaY);

/* Player */
void drawPlayerResting(int vgaX, int vgaY);
void drawPlayerRunningRight(int vgaX, int vgaY);
void drawPlayerRunningLeft(int vgaX, int vgaY);
void drawPlayerJumping(int vgaX, int vgaY);
void drawPlayer(int vgaX, int vgaY); // determine the animation of the player

/* Obstacles */
void drawFireball(int baseX, int baseY);
void drawSpike(int baseX, int baseY);

/* Letters */
void drawA(int vgaX, int vgaY, short int textColour);
void drawB(int vgaX, int vgaY, short int textColour);
void drawC(int vgaX, int vgaY, short int textColour);
void drawD(int vgaX, int vgaY, short int textColour);
void drawE(int vgaX, int vgaY, short int textColour);
void drawF(int vgaX, int vgaY, short int textColour);
void drawG(int vgaX, int vgaY, short int textColour);
void drawH(int vgaX, int vgaY, short int textColour);
void drawI(int vgaX, int vgaY, short int textColour);
void drawJ(int vgaX, int vgaY, short int textColour);
void drawK(int vgaX, int vgaY, short int textColour);
void drawL(int vgaX, int vgaY, short int textColour);
void drawM(int vgaX, int vgaY, short int textColour);
void drawN(int vgaX, int vgaY, short int textColour);
void drawO(int vgaX, int vgaY, short int textColour);
void drawP(int vgaX, int vgaY, short int textColour);
void drawQ(int vgaX, int vgaY, short int textColour);
void drawR(int vgaX, int vgaY, short int textColour);
void drawS(int vgaX, int vgaY, short int textColour);
void drawT(int vgaX, int vgaY, short int textColour);
void drawU(int vgaX, int vgaY, short int textColour);
void drawV(int vgaX, int vgaY, short int textColour);
void drawW(int vgaX, int vgaY, short int textColour);
void drawX(int vgaX, int vgaY, short int textColour);
void drawY(int vgaX, int vgaY, short int textColour);
void drawZ(int vgaX, int vgaY, short int textColour);

/* Numbers */
void draw0(int vgaX, int vgaY, short int textColour);
void draw1(int vgaX, int vgaY, short int textColour);
void draw2(int vgaX, int vgaY, short int textColour);
void draw3(int vgaX, int vgaY, short int textColour);
void draw4(int vgaX, int vgaY, short int textColour);
void draw5(int vgaX, int vgaY, short int textColour);
void draw6(int vgaX, int vgaY, short int textColour);
void draw7(int vgaX, int vgaY, short int textColour);
void draw8(int vgaX, int vgaY, short int textColour);
void draw9(int vgaX, int vgaY, short int textColour);

/* Shapes */
void drawLine(int x0, int y0, int x1, int y1, short int line_color);
void drawCircle(int x_center, int y_centerc, int r, short int line_color);
void drawBox(int vgaX, int vgaY, int boxLen, short int boxColor);
void drawBoxWithBorder(int vgaX, int vgaY, int boxLen, int borderWid, short int boxColor, short int borderColor);

/**************************** Animation routine ************************************/
void refreshAnimation();

/* Player phyisics */
void updatePlayerStatus(); // updates the player's horizontal and veritcal speed, his current location
void hitboxCheck();        // check if the player is in contact with any fireballs or spikes

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
Timer myTimer;

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
    int counter = 6250000 * 2;                   // timeout = 1/(200 MHz) x 200x10^6 = 1 sec
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
    __asm("msr cpsr, %[ps]"
        :
        : [ps] "r"(status));
}

/*
 * Turn off interrupt in the ARM processor
 */
void disable_A9_interrupts(void)
{
    int status = 0b11010011;
    __asm("msr cpsr, %[ps]"
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
    __asm("msr cpsr, %[ps]"
        :
        : [ps] "r"(mode));
    /* set banked stack pointer */
    __asm("mov sp, %[ps]"
        :
        : [ps] "r"(stack));
    /* go back to SVC mode before executing subroutine return! */
    mode = 0b11010011;
    __asm("msr cpsr, %[ps]"
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
    /* Switch address */
    volatile int *SW_ptr = (int *)SW_BASE;
    /* HEX display base address */
    volatile int *HEX3_HEX0_ptr = (int *)HEX3_HEX0_BASE;
    int press, HEX_bits;
    press = *(KEY_ptr + 3); // read the pushbutton interrupt register
    *(KEY_ptr + 3) = press; // Clear the interrupt
    if (press & 0x1)        // KEY0
    {
        HEX_bits = 0b00111111;
        clear_screen();
        if ((*SW_ptr) < 3 && (*SW_ptr) >= 0)
            updateLevel(gameLevels[*SW_ptr]);
        else
            updateLevel(gameLevels[2]);
        myGame.progress = GAMEPROG_START;
    }
    else if (press & 0x2) // KEY1
        HEX_bits = 0b00000110;
    else if (press & 0x4) // KEY2
        HEX_bits = 0b01011011;
    else // press & 0x8, which is KEY3
    {
        resetGame();
        HEX_bits = 0b01001111;
    }
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

    // frame counter
    if (myGame.progress == GAMEPROG_START)
    {
        if (myTimer.countDown == 0)
        {
            myGame.progress = GAMEPROG_LOSE;
        }
        else if (myTimer.frame == 32 / 2)
        {
            myTimer.countDown--;
            myTimer.frame = 0;
        }
        else
        {
            myTimer.frame++;
        }
    }
    // draw routine
    updatePlayerStatus();
    hitboxCheck();
    refreshAnimation();

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
    volatile int *pixel_ctrl_ptr = (int *)PIXEL_BUF_CTRL_BASE;
    pixel_buffer_start = *(pixel_ctrl_ptr + 1); // new back buffer
    for (x = 0; x < RESOLUTION_X; x++)
    {
        for (y = 0; y < RESOLUTION_Y; y++)
        {
            plot_pixel(x, y, WHITE);
        }
    }
    wait_for_vsync();
    pixel_buffer_start = *(pixel_ctrl_ptr + 1); // new back buffer

    for (x = 0; x < RESOLUTION_X; x++)
    {
        for (y = 0; y < RESOLUTION_Y; y++)
        {
            plot_pixel(x, y, WHITE);
        }
    }
    wait_for_vsync();
    pixel_buffer_start = *(pixel_ctrl_ptr + 1); // new back buffer
}

/********************************************************************
 * setupLevels
 *
 * Routine to setup all the leves
 *******************************************************************/
void setupLevels()
{
    setupLevels_lv1();
    setupLevels_lv2();
    setupLevels_lv3();
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

    gameLevels[0].levelObjects[10][19] = GAMEOBJ_PLATFORM_BLOCK;

    gameLevels[0].levelObjects[10][10] = GAMEOBJ_PLATFORM_BLOCK;
    gameLevels[0].levelObjects[9][10] = GAMEOBJ_PLATFORM_BLOCK;
    gameLevels[0].levelObjects[8][10] = GAMEOBJ_PLATFORM_BLOCK;
    gameLevels[0].levelObjects[11][6] = GAMEOBJ_PLATFORM_BLOCK;

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
    gameLevels[0].levelObjects[30][20] = GAMEOBJ_END;
    gameLevels[0].end.x = 30;
    gameLevels[0].end.y = 20;
}

/********************************************************************
 * setupLevels_lv2
 *
 * setup level 2 object
 *******************************************************************/
void setupLevels_lv2()
{
    // update the level number
    gameLevels[1].levelNumber = 2;

    // draw platforms
    for (int x = 0; x <= 30; x++)
    {
        gameLevels[1].levelObjects[x][0] = GAMEOBJ_PLATFORM_BLOCK;
    }
    for (int x = 6; x <= 27; x++)
    {
        gameLevels[1].levelObjects[x][5] = GAMEOBJ_PLATFORM_BLOCK;
    }
    for (int x = 1; x <= 17; x++)
    {
        gameLevels[1].levelObjects[x][10] = GAMEOBJ_PLATFORM_BLOCK;
    }
    for (int x = 21; x <= 27; x++)
    {
        gameLevels[1].levelObjects[x][14] = GAMEOBJ_PLATFORM_BLOCK;
    }
    for (int x = 14; x <= 18; x++)
    {
        gameLevels[1].levelObjects[x][18] = GAMEOBJ_PLATFORM_BLOCK;
    }
    for (int x = 0; x <= 6; x++)
    {
        gameLevels[1].levelObjects[x][19] = GAMEOBJ_PLATFORM_BLOCK;
    }
    for (int x = 7; x <= 12; x++)
    {
        gameLevels[1].levelObjects[x][22] = GAMEOBJ_PLATFORM_BLOCK;
    }
    for (int x = 19; x <= 31; x++)
    {
        gameLevels[1].levelObjects[x][22] = GAMEOBJ_PLATFORM_BLOCK;
    }

    // draw fireballs
    for (int x = 4; x <= 5; x++)
    {
        gameLevels[1].levelObjects[x][5] = GAMEOBJ_FIREBALL;
    }
    for (int y = 5; y <= 13; y++)
    {
        gameLevels[1].levelObjects[28][y] = GAMEOBJ_FIREBALL;
    }
    for (int x = 0; x <= 9; x++)
    {
        gameLevels[1].levelObjects[x][15] = GAMEOBJ_FIREBALL;
    }
    for (int y = 15; y <= 18; y++)
    {
        gameLevels[1].levelObjects[10][y] = GAMEOBJ_FIREBALL;
        gameLevels[1].levelObjects[25][y] = GAMEOBJ_FIREBALL;
    }
    for (int y = 0; y <= 22; y++)
    {
        gameLevels[1].levelObjects[31][y] = GAMEOBJ_FIREBALL;
    }

    // draw spikes
    for (int x = 19; x <= 25; x++)
    {
        gameLevels[1].levelObjects[x][21] = GAMEOBJ_SPIKE;
    }
    gameLevels[1].levelObjects[30][21] = GAMEOBJ_SPIKE;
    for (int x = 0; x <= 31; x++)
    {
        gameLevels[1].levelObjects[x][23] = GAMEOBJ_SPIKE;
    }

    // starting position
    gameLevels[1].levelObjects[2][19] = GAMEOBJ_START;
    gameLevels[1].start.x = 2;
    gameLevels[1].start.y = 19;

    // ending position
    gameLevels[1].levelObjects[28][22] = GAMEOBJ_END;
    gameLevels[1].end.x = 28;
    gameLevels[1].end.y = 22;
}

/********************************************************************
 * setupLevels_lv3
 *
 * setup level 3 object
 *******************************************************************/
void setupLevels_lv3()
{
    // update the level number
    gameLevels[2].levelNumber = 3;

    // draw platforms
    for (int x = 0; x <= 30; x++)
    {
        gameLevels[1].levelObjects[x][0] = GAMEOBJ_PLATFORM_BLOCK;
    }
    for (int x = 6; x <= 27; x++)
    {
        gameLevels[2].levelObjects[x][5] = GAMEOBJ_PLATFORM_BLOCK;
    }
    for (int x = 1; x <= 17; x++)
    {
        gameLevels[2].levelObjects[x][10] = GAMEOBJ_PLATFORM_BLOCK;
    }
    for (int x = 21; x <= 27; x++)
    {
        gameLevels[2].levelObjects[x][14] = GAMEOBJ_PLATFORM_BLOCK;
    }
    for (int x = 14; x <= 18; x++)
    {
        gameLevels[2].levelObjects[x][18] = GAMEOBJ_PLATFORM_BLOCK;
    }
    for (int x = 0; x <= 6; x++)
    {
        gameLevels[2].levelObjects[x][19] = GAMEOBJ_PLATFORM_BLOCK;
    }
    for (int x = 7; x <= 12; x++)
    {
        gameLevels[2].levelObjects[x][22] = GAMEOBJ_PLATFORM_BLOCK;
    }
    for (int x = 19; x <= 31; x++)
    {
        gameLevels[2].levelObjects[x][22] = GAMEOBJ_PLATFORM_BLOCK;
    }

    // draw fireballs
    for (int x = 4; x <= 5; x++)
    {
        gameLevels[2].levelObjects[x][5] = GAMEOBJ_FIREBALL;
    }
    for (int y = 5; y <= 13; y++)
    {
        gameLevels[2].levelObjects[28][y] = GAMEOBJ_FIREBALL;
    }
    for (int x = 0; x <= 9; x++)
    {
        gameLevels[2].levelObjects[x][15] = GAMEOBJ_FIREBALL;
    }
    for (int y = 15; y <= 18; y++)
    {
        gameLevels[2].levelObjects[10][y] = GAMEOBJ_FIREBALL;
        gameLevels[2].levelObjects[25][y] = GAMEOBJ_FIREBALL;
    }
    for (int y = 0; y <= 22; y++)
    {
        gameLevels[2].levelObjects[31][y] = GAMEOBJ_FIREBALL;
    }
    for (int x = 0; x <= 7; x++)
    {
        gameLevels[2].levelObjects[x][0] = GAMEOBJ_FIREBALL;
    }
    for (int y = 11; y <= 14; y++)
    {
        gameLevels[2].levelObjects[17][y] = GAMEOBJ_FIREBALL;
    }
    for (int y = 16; y <= 21; y++)
    {
        gameLevels[2].levelObjects[13][y] = GAMEOBJ_FIREBALL;
        gameLevels[2].levelObjects[30][y] = GAMEOBJ_FIREBALL;
    }

    // draw spikes
    for (int x = 19; x <= 25; x++)
    {
        gameLevels[2].levelObjects[x][21] = GAMEOBJ_SPIKE;
    }
    for (int x = 0; x <= 31; x++)
    {
        gameLevels[2].levelObjects[x][23] = GAMEOBJ_SPIKE;
    }
    for (int x = 9; x <= 11; x++)
    {
        gameLevels[2].levelObjects[x][4] = GAMEOBJ_SPIKE;
    }
    for (int x = 17; x <= 19; x++)
    {
        gameLevels[2].levelObjects[x][4] = GAMEOBJ_SPIKE;
    }
    for (int x = 25; x <= 26; x++)
    {
        gameLevels[2].levelObjects[x][4] = GAMEOBJ_SPIKE;
    }
    gameLevels[2].levelObjects[1][9] = GAMEOBJ_SPIKE;
    for (int x = 15; x <= 17; x++)
    {
        gameLevels[2].levelObjects[x][9] = GAMEOBJ_SPIKE;
    }
    gameLevels[2].levelObjects[7][21] = GAMEOBJ_SPIKE;

    // starting position
    gameLevels[2].levelObjects[2][19] = GAMEOBJ_START;
    gameLevels[2].start.x = 2;
    gameLevels[2].start.y = 19;

    // ending position
    gameLevels[2].levelObjects[28][22] = GAMEOBJ_END;
    gameLevels[2].end.x = 28;
    gameLevels[2].end.y = 22;
}

/********************************************************************
 * updateLevel(int level)
 *
 * update the current level
 *******************************************************************/
void updateLevel(Levels level)
{
    // clear currnet level objects
    // updating the current object array
    for (int x = 0; x < BLOCK_RESOLUTION_X; x++)
    {
        for (int y = 0; y < BLOCK_RESOLUTION_Y; y++)
        {
            myGame.currentObjects[x][y] = GAMEOBJ_EMPTY;
        }
    }

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
    myGame.myPlayer.pos.x = myGame.start.x * BOX_LEN;                                           // scale to VGA position
    myGame.myPlayer.pos.y = (myGame.start.y - PLAYER_HEIGHT_BASE /* player height*/) * BOX_LEN; // scale to VGA position
    // reset player flags
    myGame.progress = GAMEPROG_BEFORE;
    myGame.myPlayer.state = PLAYERSTATE_STILL;
    myGame.myPlayer.jump = false;
    myGame.myPlayer.airborne = false;

    // Reset speeds
    myGame.myPlayer.horizontal_speed = 0;
    myGame.myPlayer.vertical_speed = 0;

    // reset timer
    myTimer.frame = 0;
    myTimer.countDown = 10;
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
                // ignored using baseX and baseY as function parameter, use vga instead
                drawFireball(x * BOX_LEN, y * BOX_LEN);
                break;
            case GAMEOBJ_SPIKE:
                // ignored using baseX and baseY as function parameter, use vga instead
                drawSpike(x * BOX_LEN, y * BOX_LEN);
                break;
            case GAMEOBJ_START:
                drawStart(x, y);
                break;
            case GAMEOBJ_END:
                drawEnd(x, y);
                break;
            default:
                drawEmpty(x, y);
                break;
            }
        }
    }
    // Drawing the timer
    drawTimer(myTimer.countDown);
}

/********************************************************************
 * drawPlatformBlock
 *
 * draw all the platform blocks
 *******************************************************************/
void drawPlatformBlock(int baseX, int baseY)
{
    drawBoxWithBorder(baseX * BOX_LEN, baseY * BOX_LEN, BOX_LEN, 1, COLOR_PLATFORM, COLOR_PLATFORM_BORDER);
}

/********************************************************************
 * drawStart
 *
 * draw the starting block
 *******************************************************************/
void drawStart(int baseX, int baseY)
{
    drawBoxWithBorder(baseX * BOX_LEN, baseY * BOX_LEN, BOX_LEN, 1, COLOR_START, COLOR_PLATFORM_BORDER);
}

/********************************************************************
 * drawEnd
 *
 * draw the ending block
 *******************************************************************/
void drawEnd(int baseX, int baseY)
{
    drawBoxWithBorder(baseX * BOX_LEN, baseY * BOX_LEN, BOX_LEN, 1, COLOR_END, COLOR_PLATFORM_BORDER);
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

/********************************************************************
 * drawBigTitle
 *
 * draw the big title on the screen
 * Characters has a height of 15 blocks, starting from 7
 *******************************************************************/
void drawBigTitle()
{

    // number 1
    for (int baseY = 7; baseY < 22; baseY++)
        drawBox(5 * BOX_LEN, baseY * BOX_LEN, BOX_LEN, BLACK);
    // number 0
    for (int baseX = 8; baseX < 12; baseX++)
    {
        for (int baseY = 7; baseY < 22; baseY++)
        {
            if (baseX == 8 || baseY == 7 || baseX == 11 || baseY == 21)
                drawBox(baseX * BOX_LEN, baseY * BOX_LEN, BOX_LEN, BLACK);
        }
    }
    // R
    for (int baseY = 14; baseY < 22; baseY++)
        drawBox(13 * BOX_LEN, baseY * BOX_LEN, BOX_LEN, MAGENTA);
    for (int baseX = 13; baseX < 17; baseX++)
        drawBox(baseX * BOX_LEN, 14 * BOX_LEN, BOX_LEN, MAGENTA);
    drawBox(16 * BOX_LEN, 15 * BOX_LEN, BOX_LEN, MAGENTA);
    drawBox(16 * BOX_LEN, 16 * BOX_LEN, BOX_LEN, MAGENTA);

    // U
    for (int baseY = 14; baseY < 22; baseY++)
        drawBox(18 * BOX_LEN, baseY * BOX_LEN, BOX_LEN, MAGENTA);
    for (int baseY = 14; baseY < 22; baseY++)
        drawBox(21 * BOX_LEN, baseY * BOX_LEN, BOX_LEN, MAGENTA);
    for (int baseX = 19; baseX < 22; baseX++)
        drawBox(baseX * BOX_LEN, 21 * BOX_LEN, BOX_LEN, MAGENTA);

    // N
    for (int baseY = 14; baseY < 22; baseY++)
    {
        drawBox(23 * BOX_LEN, baseY * BOX_LEN, BOX_LEN, MAGENTA);
        drawBox(27 * BOX_LEN, baseY * BOX_LEN, BOX_LEN, MAGENTA);
    }
    drawBox(24 * BOX_LEN, 15 * BOX_LEN, BOX_LEN, MAGENTA);
    drawBox(24 * BOX_LEN, 16 * BOX_LEN, BOX_LEN, MAGENTA);
    drawBox(25 * BOX_LEN, 17 * BOX_LEN, BOX_LEN, MAGENTA);
    drawBox(25 * BOX_LEN, 18 * BOX_LEN, BOX_LEN, MAGENTA);
    drawBox(26 * BOX_LEN, 19 * BOX_LEN, BOX_LEN, MAGENTA);
    drawBox(26 * BOX_LEN, 20 * BOX_LEN, BOX_LEN, MAGENTA);
}

/********************************************************************
 * drawWinningScreen()
 *
 * draw the win screen when player wins
 *******************************************************************/
void drawWinningScreen()
{
    // Y
    for (int baseY = 5; baseY <= 9; baseY++)
        drawBox(8 * BOX_LEN, baseY * BOX_LEN, BOX_LEN, BLACK);
    drawBox(6 * BOX_LEN, 3 * BOX_LEN, BOX_LEN, BLACK);
    drawBox(10 * BOX_LEN, 3 * BOX_LEN, BOX_LEN, BLACK);
    drawBox(7 * BOX_LEN, 4 * BOX_LEN, BOX_LEN, BLACK);
    drawBox(9 * BOX_LEN, 4 * BOX_LEN, BOX_LEN, BLACK);

    // O
    for (int baseX = 13; baseX <= 15; baseX++)
    {
        drawBox(baseX * BOX_LEN, 3 * BOX_LEN, BOX_LEN, BLACK);
        drawBox(baseX * BOX_LEN, 9 * BOX_LEN, BOX_LEN, BLACK);
    }
    for (int baseY = 4; baseY <= 8; baseY++)
    {
        drawBox(12 * BOX_LEN, baseY * BOX_LEN, BOX_LEN, BLACK);
        drawBox(16 * BOX_LEN, baseY * BOX_LEN, BOX_LEN, BLACK);
    }

    // U
    for (int baseY = 3; baseY <= 8; baseY++)
    {
        drawBox(18 * BOX_LEN, baseY * BOX_LEN, BOX_LEN, BLACK);
        drawBox(22 * BOX_LEN, baseY * BOX_LEN, BOX_LEN, BLACK);
    }
    for (int baseX = 19; baseX <= 21; baseX++)
    {
        drawBox(baseX * BOX_LEN, 9 * BOX_LEN, BOX_LEN, BLACK);
    }

    // W
    for (int baseY = 12; baseY <= 17; baseY++)
    {
        drawBox(6 * BOX_LEN, baseY * BOX_LEN, BOX_LEN, BLACK);
        drawBox(10 * BOX_LEN, baseY * BOX_LEN, BOX_LEN, BLACK);
    }
    for (int baseY = 14; baseY <= 17; baseY++)
    {
        drawBox(8 * BOX_LEN, baseY * BOX_LEN, BOX_LEN, BLACK);
    }
    drawBox(7 * BOX_LEN, 18 * BOX_LEN, BOX_LEN, BLACK);
    drawBox(9 * BOX_LEN, 18 * BOX_LEN, BOX_LEN, BLACK);

    // I
    for (int baseX = 12; baseX <= 16; baseX++)
    {
        drawBox(baseX * BOX_LEN, 12 * BOX_LEN, BOX_LEN, BLACK);
        drawBox(baseX * BOX_LEN, 18 * BOX_LEN, BOX_LEN, BLACK);
    }
    for (int baseY = 13; baseY <= 17; baseY++)
    {
        drawBox(14 * BOX_LEN, baseY * BOX_LEN, BOX_LEN, BLACK);
    }

    // N
    for (int baseY = 12; baseY <= 18; baseY++)
    {
        drawBox(18 * BOX_LEN, baseY * BOX_LEN, BOX_LEN, BLACK);
        drawBox(22 * BOX_LEN, baseY * BOX_LEN, BOX_LEN, BLACK);
    }
    drawBox(19 * BOX_LEN, 13 * BOX_LEN, BOX_LEN, BLACK);
    drawBox(19 * BOX_LEN, 14 * BOX_LEN, BOX_LEN, BLACK);
    drawBox(20 * BOX_LEN, 14 * BOX_LEN, BOX_LEN, BLACK);
    drawBox(20 * BOX_LEN, 15 * BOX_LEN, BOX_LEN, BLACK);
    drawBox(20 * BOX_LEN, 16 * BOX_LEN, BOX_LEN, BLACK);
    drawBox(21 * BOX_LEN, 16 * BOX_LEN, BOX_LEN, BLACK);
    drawBox(21 * BOX_LEN, 17 * BOX_LEN, BOX_LEN, BLACK);
}

/********************************************************************
 * drawGameOver()
 *
 * draw the win screen when player loses
 *******************************************************************/
void drawLosingScreen()
{
    // Y
    for (int baseY = 5; baseY <= 9; baseY++)
        drawBox(8 * BOX_LEN, baseY * BOX_LEN, BOX_LEN, BLACK);
    drawBox(6 * BOX_LEN, 3 * BOX_LEN, BOX_LEN, BLACK);
    drawBox(10 * BOX_LEN, 3 * BOX_LEN, BOX_LEN, BLACK);
    drawBox(7 * BOX_LEN, 4 * BOX_LEN, BOX_LEN, BLACK);
    drawBox(9 * BOX_LEN, 4 * BOX_LEN, BOX_LEN, BLACK);

    // O
    for (int baseX = 13; baseX <= 15; baseX++)
    {
        drawBox(baseX * BOX_LEN, 3 * BOX_LEN, BOX_LEN, BLACK);
        drawBox(baseX * BOX_LEN, 9 * BOX_LEN, BOX_LEN, BLACK);
    }
    for (int baseY = 4; baseY <= 8; baseY++)
    {
        drawBox(12 * BOX_LEN, baseY * BOX_LEN, BOX_LEN, BLACK);
        drawBox(16 * BOX_LEN, baseY * BOX_LEN, BOX_LEN, BLACK);
    }

    // U
    for (int baseY = 3; baseY <= 8; baseY++)
    {
        drawBox(18 * BOX_LEN, baseY * BOX_LEN, BOX_LEN, BLACK);
        drawBox(22 * BOX_LEN, baseY * BOX_LEN, BOX_LEN, BLACK);
    }
    for (int baseX = 19; baseX <= 21; baseX++)
    {
        drawBox(baseX * BOX_LEN, 9 * BOX_LEN, BOX_LEN, BLACK);
    }

    // L
    for (int baseY = 12; baseY <= 18; baseY++)
    {
        drawBox(4 * BOX_LEN, baseY * BOX_LEN, BOX_LEN, BLACK);
    }
    for (int baseX = 4; baseX <= 8; baseX++)
    {
        drawBox(baseX * BOX_LEN, 18 * BOX_LEN, BOX_LEN, BLACK);
    }

    // O
    for (int baseX = 11; baseX <= 13; baseX++)
    {
        drawBox(baseX * BOX_LEN, 12 * BOX_LEN, BOX_LEN, BLACK);
        drawBox(baseX * BOX_LEN, 18 * BOX_LEN, BOX_LEN, BLACK);
    }
    for (int baseY = 13; baseY <= 17; baseY++)
    {
        drawBox(10 * BOX_LEN, baseY * BOX_LEN, BOX_LEN, BLACK);
        drawBox(14 * BOX_LEN, baseY * BOX_LEN, BOX_LEN, BLACK);
    }

    // S
    for (int baseX = 17; baseX <= 19; baseX++)
    {
        drawBox(baseX * BOX_LEN, 12 * BOX_LEN, BOX_LEN, BLACK);
        drawBox(baseX * BOX_LEN, 15 * BOX_LEN, BOX_LEN, BLACK);
        drawBox(baseX * BOX_LEN, 18 * BOX_LEN, BOX_LEN, BLACK);
    }
    drawBox(16 * BOX_LEN, 13 * BOX_LEN, BOX_LEN, BLACK);
    drawBox(20 * BOX_LEN, 13 * BOX_LEN, BOX_LEN, BLACK);
    drawBox(16 * BOX_LEN, 14 * BOX_LEN, BOX_LEN, BLACK);
    drawBox(20 * BOX_LEN, 16 * BOX_LEN, BOX_LEN, BLACK);
    drawBox(20 * BOX_LEN, 17 * BOX_LEN, BOX_LEN, BLACK);
    drawBox(16 * BOX_LEN, 17 * BOX_LEN, BOX_LEN, BLACK);

    // E
    for (int baseX = 22; baseX <= 26; baseX++)
    {
        drawBox(baseX * BOX_LEN, 12 * BOX_LEN, BOX_LEN, BLACK);
        drawBox(baseX * BOX_LEN, 18 * BOX_LEN, BOX_LEN, BLACK);
        if (baseX != 26)
            drawBox(baseX * BOX_LEN, 15 * BOX_LEN, BOX_LEN, BLACK);
    }
    for (int baseY = 12; baseY <= 18; baseY++)
    {
        drawBox(22 * BOX_LEN, baseY * BOX_LEN, BOX_LEN, BLACK);
    }
}

/*******************************************************************
 * drawPlayerResting(int vgaX, int vgaY)
 *
 * draws player in its resting state
 *******************************************************************/
void drawPlayerResting(int vgaX, int vgaY)
{
    // Draw head
    for (int i = 0; i < 4; i++)
    {
        plot_pixel(vgaX + 4 + i, vgaY, COLOR_PLAYER);
        plot_pixel(vgaX + 4 + i, vgaY + 7, COLOR_PLAYER);
    }
    for (int i = 0; i < 6; i++)
    {
        plot_pixel(vgaX + 3 + i, vgaY + 1, COLOR_PLAYER);
        plot_pixel(vgaX + 3 + i, vgaY + 6, COLOR_PLAYER);
    }
    for (int j = 0; j < 4; j++)
    {
        for (int i = 0; i < 8; i++)
        {
            plot_pixel(vgaX + 2 + i, vgaY + 2 + j, COLOR_PLAYER);
        }
    }

    // Draw body and arms
    plot_pixel(vgaX + 3, vgaY + 8, COLOR_PLAYER);
    plot_pixel(vgaX + 4, vgaY + 8, COLOR_PLAYER);

    plot_pixel(vgaX + 2, vgaY + 9, COLOR_PLAYER);
    plot_pixel(vgaX + 4, vgaY + 9, COLOR_PLAYER);
    plot_pixel(vgaX + 5, vgaY + 9, COLOR_PLAYER);

    for (int i = 0; i < 2; i++)
    {
        plot_pixel(vgaX + 1, vgaY + 10 + i, COLOR_PLAYER);
        plot_pixel(vgaX + 4, vgaY + 10 + i, COLOR_PLAYER);
        plot_pixel(vgaX + 5, vgaY + 10 + i, COLOR_PLAYER);
    }

    plot_pixel(vgaX + 1, vgaY + 12, COLOR_PLAYER);
    plot_pixel(vgaX + 4, vgaY + 12, COLOR_PLAYER);
    plot_pixel(vgaX + 6, vgaY + 12, COLOR_PLAYER);

    // Draw legs
    plot_pixel(vgaX + 2, vgaY + 13, COLOR_PLAYER);
    plot_pixel(vgaX + 4, vgaY + 13, COLOR_PLAYER);

    plot_pixel(vgaX + 4, vgaY + 14, COLOR_PLAYER);
    plot_pixel(vgaX + 5, vgaY + 14, COLOR_PLAYER);

    plot_pixel(vgaX + 4, vgaY + 15, COLOR_PLAYER);
    plot_pixel(vgaX + 6, vgaY + 15, COLOR_PLAYER);

    for (int i = 0; i < 4; i++)
    {
        plot_pixel(vgaX + 3 - i, vgaY + 16 + i, COLOR_PLAYER);
        plot_pixel(vgaX + 7, vgaY + 16 + i, COLOR_PLAYER);
    }
}

/********************************************************************
 * drawPlayerRunningRight(int vgaX, int vgaY)
 *
 * draws player in its running state in the right direction
 *******************************************************************/
void drawPlayerRunningRight(int vgaX, int vgaY)
{
    // Draw head
    for (int i = 0; i < 4; i++)
    {
        plot_pixel(vgaX + 4 + i, vgaY, COLOR_PLAYER);
        plot_pixel(vgaX + 4 + i, vgaY + 7, COLOR_PLAYER);
    }
    for (int i = 0; i < 6; i++)
    {
        plot_pixel(vgaX + 3 + i, vgaY + 1, COLOR_PLAYER);
        plot_pixel(vgaX + 3 + i, vgaY + 6, COLOR_PLAYER);
    }
    for (int j = 0; j < 4; j++)
    {
        for (int i = 0; i < 8; i++)
        {
            plot_pixel(vgaX + 2 + i, vgaY + 2 + j, COLOR_PLAYER);
        }
    }

    // Draw body and arms
    for (int i = 0; i < 4; i++)
    {
        plot_pixel(vgaX + 1 + i, vgaY + 8, COLOR_PLAYER);
    }

    for (int j = 0; j < 3; j++)
    {
        plot_pixel(vgaX, vgaY + 9 + j, COLOR_PLAYER);
        plot_pixel(vgaX + 4, vgaY + 9 + j, COLOR_PLAYER);
    }

    plot_pixel(vgaX + 5, vgaY + 10, COLOR_PLAYER);

    plot_pixel(vgaX + 3, vgaY + 11, COLOR_PLAYER);
    plot_pixel(vgaX + 4, vgaY + 11, COLOR_PLAYER);
    plot_pixel(vgaX + 6, vgaY + 11, COLOR_PLAYER);
    plot_pixel(vgaX + 7, vgaY + 11, COLOR_PLAYER);
    plot_pixel(vgaX + 8, vgaY + 11, COLOR_PLAYER);

    // Draw legs
    plot_pixel(vgaX + 3, vgaY + 12, COLOR_PLAYER);

    plot_pixel(vgaX + 2, vgaY + 13, COLOR_PLAYER);
    plot_pixel(vgaX + 3, vgaY + 13, COLOR_PLAYER);

    plot_pixel(vgaX + 2, vgaY + 14, COLOR_PLAYER);
    plot_pixel(vgaX + 4, vgaY + 14, COLOR_PLAYER);
    plot_pixel(vgaX + 5, vgaY + 14, COLOR_PLAYER);

    plot_pixel(vgaX + 1, vgaY + 15, COLOR_PLAYER);
    plot_pixel(vgaX + 2, vgaY + 15, COLOR_PLAYER);
    plot_pixel(vgaX + 5, vgaY + 15, COLOR_PLAYER);

    plot_pixel(vgaX + 1, vgaY + 16, COLOR_PLAYER);
    plot_pixel(vgaX + 4, vgaY + 16, COLOR_PLAYER);
    plot_pixel(vgaX + 5, vgaY + 16, COLOR_PLAYER);

    plot_pixel(vgaX + 0, vgaY + 17, COLOR_PLAYER);
    plot_pixel(vgaX + 1, vgaY + 17, COLOR_PLAYER);
    plot_pixel(vgaX + 4, vgaY + 17, COLOR_PLAYER);

    plot_pixel(vgaX + 0, vgaY + 18, COLOR_PLAYER);
    plot_pixel(vgaX + 4, vgaY + 18, COLOR_PLAYER);

    plot_pixel(vgaX + 0, vgaY + 19, COLOR_PLAYER);
}

/********************************************************************
 * drawPlayerRunningLeft(int vgaX, int vgaY)
 *
 * draws player in its running state in the left direction
 *******************************************************************/
void drawPlayerRunningLeft(int vgaX, int vgaY)
{
    // Draw head
    for (int i = 0; i < 4; i++)
    {
        plot_pixel(vgaX + 2 + i, vgaY, COLOR_PLAYER);
        plot_pixel(vgaX + 2 + i, vgaY + 7, COLOR_PLAYER);
    }
    for (int i = 0; i < 6; i++)
    {
        plot_pixel(vgaX + 1 + i, vgaY + 1, COLOR_PLAYER);
        plot_pixel(vgaX + 1 + i, vgaY + 6, COLOR_PLAYER);
    }
    for (int j = 0; j < 4; j++)
    {
        for (int i = 0; i < 8; i++)
        {
            plot_pixel(vgaX + i, vgaY + 2 + j, COLOR_PLAYER);
        }
    }

    // Draw body and arms
    for (int i = 0; i < 4; i++)
    {
        plot_pixel(vgaX + 5 + i, vgaY + 8, COLOR_PLAYER);
    }

    for (int j = 0; j < 3; j++)
    {
        plot_pixel(vgaX + 9, vgaY + 9 + j, COLOR_PLAYER);
        plot_pixel(vgaX + 5, vgaY + 9 + j, COLOR_PLAYER);
    }

    plot_pixel(vgaX + 4, vgaY + 10, COLOR_PLAYER);

    plot_pixel(vgaX + 1, vgaY + 11, COLOR_PLAYER);
    plot_pixel(vgaX + 2, vgaY + 11, COLOR_PLAYER);
    plot_pixel(vgaX + 3, vgaY + 11, COLOR_PLAYER);
    plot_pixel(vgaX + 5, vgaY + 11, COLOR_PLAYER);
    plot_pixel(vgaX + 6, vgaY + 11, COLOR_PLAYER);

    // Draw legs
    plot_pixel(vgaX + 6, vgaY + 12, COLOR_PLAYER);

    plot_pixel(vgaX + 6, vgaY + 13, COLOR_PLAYER);
    plot_pixel(vgaX + 7, vgaY + 13, COLOR_PLAYER);

    plot_pixel(vgaX + 4, vgaY + 14, COLOR_PLAYER);
    plot_pixel(vgaX + 5, vgaY + 14, COLOR_PLAYER);
    plot_pixel(vgaX + 7, vgaY + 14, COLOR_PLAYER);

    plot_pixel(vgaX + 4, vgaY + 15, COLOR_PLAYER);
    plot_pixel(vgaX + 7, vgaY + 15, COLOR_PLAYER);
    plot_pixel(vgaX + 8, vgaY + 15, COLOR_PLAYER);

    plot_pixel(vgaX + 4, vgaY + 16, COLOR_PLAYER);
    plot_pixel(vgaX + 5, vgaY + 16, COLOR_PLAYER);
    plot_pixel(vgaX + 8, vgaY + 16, COLOR_PLAYER);

    plot_pixel(vgaX + 5, vgaY + 17, COLOR_PLAYER);
    plot_pixel(vgaX + 8, vgaY + 17, COLOR_PLAYER);
    plot_pixel(vgaX + 9, vgaY + 17, COLOR_PLAYER);

    plot_pixel(vgaX + 5, vgaY + 18, COLOR_PLAYER);
    plot_pixel(vgaX + 9, vgaY + 18, COLOR_PLAYER);

    plot_pixel(vgaX + 9, vgaY + 19, COLOR_PLAYER);
}

/********************************************************************
 * drawPlayerJumping(int vgaX, int vgaY)
 *
 * draws player in its jumping state
 *******************************************************************/
void drawPlayerJumping(int vgaX, int vgaY)
{
    // Draw head
    for (int i = 0; i < 4; i++)
    {
        plot_pixel(vgaX + 3 + i, vgaY, COLOR_PLAYER);
        plot_pixel(vgaX + 3 + i, vgaY + 7, COLOR_PLAYER);
    }
    for (int i = 0; i < 6; i++)
    {
        plot_pixel(vgaX + 2 + i, vgaY + 1, COLOR_PLAYER);
        plot_pixel(vgaX + 2 + i, vgaY + 6, COLOR_PLAYER);
    }
    for (int j = 0; j < 4; j++)
    {
        for (int i = 0; i < 8; i++)
        {
            plot_pixel(vgaX + 1 + i, vgaY + 2 + j, COLOR_PLAYER);
        }
    }

    // Draw body and arms
    plot_pixel(vgaX + 4, vgaY + 8, COLOR_PLAYER);

    for (int i = 0; i < 8; i++)
    {
        plot_pixel(vgaX + 1 + i, vgaY + 9, COLOR_PLAYER);
    }

    for (int j = 0; j < 4; j++)
    {
        plot_pixel(vgaX + 4, vgaY + 10 + j, COLOR_PLAYER);
    }

    plot_pixel(vgaX, vgaY + 10, COLOR_PLAYER);
    plot_pixel(vgaX + 9, vgaY + 10, COLOR_PLAYER);

    plot_pixel(vgaX + 1, vgaY + 11, COLOR_PLAYER);
    plot_pixel(vgaX + 8, vgaY + 11, COLOR_PLAYER);

    // Draw legs
    plot_pixel(vgaX + 2, vgaY + 14, COLOR_PLAYER);
    plot_pixel(vgaX + 3, vgaY + 14, COLOR_PLAYER);
    plot_pixel(vgaX + 5, vgaY + 14, COLOR_PLAYER);
    plot_pixel(vgaX + 6, vgaY + 14, COLOR_PLAYER);

    for (int j = 0; j < 2; j++)
    {
        plot_pixel(vgaX + 1, vgaY + 15 + j, COLOR_PLAYER);
        plot_pixel(vgaX + 7, vgaY + 15 + j, COLOR_PLAYER);
    }

    for (int j = 0; j < 2; j++)
    {
        plot_pixel(vgaX + 2, vgaY + 17 + j, COLOR_PLAYER);
        plot_pixel(vgaX + 6, vgaY + 17 + j, COLOR_PLAYER);
    }
}

/********************************************************************
 * void drawPlayer(int vgaX, int vgaY)
 *
 * determine the animation of the player
 *******************************************************************/
void drawPlayer(int vgaX, int vgaY)
{
      if (myGame.myPlayer.airborne == true)
    {
        drawPlayerJumping(vgaX, vgaY);
        return;
    }
    switch (myGame.myPlayer.state)
    {
    case PLAYERSTATE_LEFT:
        drawPlayerRunningLeft(vgaX, vgaY);
        return;
    case PLAYERSTATE_RIGHT:
        drawPlayerRunningRight(vgaX, vgaY);
        return;
    case PLAYERSTATE_STILL:
        drawPlayerResting(vgaX, vgaY);
        return;
    default:
        drawPlayerResting(vgaX, vgaY);
        return;
    }
}
/********************************************************************
 * drawFireball(int baseX, int baseY)
 *
 * draws fireball
 *******************************************************************/
void drawFireball(int baseX, int baseY)
{
    plot_pixel(baseX, baseY, RED);
    plot_pixel(baseX + 3, baseY, RED);
    plot_pixel(baseX + 7, baseY, RED);

    plot_pixel(baseX, baseY + 1, RED);
    plot_pixel(baseX + 3, baseY + 1, RED);
    plot_pixel(baseX + 5, baseY + 1, RED);
    plot_pixel(baseX + 7, baseY + 1, RED);

    plot_pixel(baseX + 2, baseY + 2, RED);
    plot_pixel(baseX + 6, baseY + 2, RED);
    plot_pixel(baseX + 9, baseY + 2, RED);

    plot_pixel(baseX + 1, baseY + 3, RED);
    plot_pixel(baseX + 3, baseY + 3, RED);
    plot_pixel(baseX + 5, baseY + 3, RED);
    plot_pixel(baseX + 6, baseY + 3, RED);
    plot_pixel(baseX + 9, baseY + 3, RED);

    plot_pixel(baseX + 1, baseY + 4, RED);
    plot_pixel(baseX + 2, baseY + 4, RED);
    plot_pixel(baseX + 4, baseY + 4, RED);
    plot_pixel(baseX + 5, baseY + 4, RED);
    plot_pixel(baseX + 6, baseY + 4, ORANGE);
    plot_pixel(baseX + 7, baseY + 4, RED);
    plot_pixel(baseX + 8, baseY + 4, RED);
    plot_pixel(baseX + 9, baseY + 4, RED);

    plot_pixel(baseX, baseY + 5, RED);
    plot_pixel(baseX + 1, baseY + 5, RED);
    plot_pixel(baseX + 2, baseY + 5, RED);
    plot_pixel(baseX + 3, baseY + 5, RED);
    plot_pixel(baseX + 4, baseY + 5, ORANGE);
    plot_pixel(baseX + 5, baseY + 5, ORANGE);
    plot_pixel(baseX + 6, baseY + 5, ORANGE);
    plot_pixel(baseX + 7, baseY + 5, ORANGE);
    plot_pixel(baseX + 8, baseY + 5, RED);
    plot_pixel(baseX + 9, baseY + 5, RED);

    plot_pixel(baseX, baseY + 6, RED);
    plot_pixel(baseX + 1, baseY + 6, RED);
    plot_pixel(baseX + 2, baseY + 6, ORANGE);
    plot_pixel(baseX + 3, baseY + 6, ORANGE);
    plot_pixel(baseX + 4, baseY + 6, YELLOW);
    plot_pixel(baseX + 5, baseY + 6, YELLOW);
    plot_pixel(baseX + 6, baseY + 6, YELLOW);
    plot_pixel(baseX + 7, baseY + 6, ORANGE);
    plot_pixel(baseX + 8, baseY + 6, RED);

    plot_pixel(baseX, baseY + 7, RED);
    plot_pixel(baseX + 1, baseY + 7, RED);
    plot_pixel(baseX + 2, baseY + 7, ORANGE);
    plot_pixel(baseX + 3, baseY + 7, YELLOW);
    plot_pixel(baseX + 4, baseY + 7, YELLOW);
    plot_pixel(baseX + 5, baseY + 7, WHITE);
    plot_pixel(baseX + 6, baseY + 7, YELLOW);
    plot_pixel(baseX + 7, baseY + 7, ORANGE);
    plot_pixel(baseX + 8, baseY + 7, RED);

    plot_pixel(baseX + 1, baseY + 8, RED);
    plot_pixel(baseX + 2, baseY + 8, RED);
    plot_pixel(baseX + 3, baseY + 8, YELLOW);
    plot_pixel(baseX + 4, baseY + 8, YELLOW);
    plot_pixel(baseX + 5, baseY + 8, WHITE);
    plot_pixel(baseX + 6, baseY + 8, WHITE);
    plot_pixel(baseX + 7, baseY + 8, RED);

    plot_pixel(baseX + 3, baseY + 9, RED);
    plot_pixel(baseX + 6, baseY + 9, RED);
}

/********************************************************************
 * drawSpike(int baseX, int baseY)
 *
 * draws spike object
 *******************************************************************/
void drawSpike(int baseX, int baseY)
{
    for (int i = 0; i <= 9; i++)
    {
        plot_pixel(baseX + 4, baseY + i, BLACK);
        plot_pixel(baseX + 5, baseY + i, BLACK);
    }
    for (int i = 2; i <= 9; i++)
    {
        plot_pixel(baseX + 3, baseY + i, BLACK);
        plot_pixel(baseX + 6, baseY + i, BLACK);
    }
    for (int i = 4; i <= 9; i++)
    {
        plot_pixel(baseX + 2, baseY + i, BLACK);
        plot_pixel(baseX + 7, baseY + i, BLACK);
    }
    for (int i = 6; i <= 9; i++)
    {
        plot_pixel(baseX + 1, baseY + i, BLACK);
        plot_pixel(baseX + 8, baseY + i, BLACK);
    }
    for (int i = 8; i <= 9; i++)
    {
        plot_pixel(baseX, baseY + i, BLACK);
        plot_pixel(baseX + 9, baseY + i, BLACK);
    }
}

/********************************************************************
 * void drawTimer(int count)
 *
 * the function draws the timer at the corner of the display
 *******************************************************************/
void drawTimer(int count)
{
    Position firstNumberPos;
    Position secondNumberPos;
    firstNumberPos.x = 0;
    firstNumberPos.y = 0;
    secondNumberPos.x = 10;
    secondNumberPos.y = 0;

    switch (count)
    {
    case 0:
        draw0(firstNumberPos.x, firstNumberPos.y, MAGENTA);
        break;
    case 1:
        draw1(firstNumberPos.x, firstNumberPos.y, MAGENTA);
        break;
    case 2:
        draw2(firstNumberPos.x, firstNumberPos.y, MAGENTA);
        break;
    case 3:
        draw3(firstNumberPos.x, firstNumberPos.y, MAGENTA);
        break;
    case 4:
        draw4(firstNumberPos.x, firstNumberPos.y, MAGENTA);
        break;
    case 5:
        draw5(firstNumberPos.x, firstNumberPos.y, MAGENTA);
        break;
    case 6:
        draw6(firstNumberPos.x, firstNumberPos.y, MAGENTA);
        break;
    case 7:
        draw7(firstNumberPos.x, firstNumberPos.y, MAGENTA);
        break;
    case 8:
        draw8(firstNumberPos.x, firstNumberPos.y, MAGENTA);
        break;
    case 9:
        draw9(firstNumberPos.x, firstNumberPos.y, MAGENTA);
        break;
    case 10:
        draw1(firstNumberPos.x, firstNumberPos.y, MAGENTA);
        draw0(secondNumberPos.x, secondNumberPos.y, MAGENTA);
        break;
    default:
        draw0(firstNumberPos.x, firstNumberPos.y, MAGENTA);
    }
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
    switch (myGame.progress)
    {
    case GAMEPROG_BEFORE:
        drawBigTitle();
        break;
    case GAMEPROG_START:
        drawCurrentObjects();
        drawPlayer(myGame.myPlayer.pos.x, myGame.myPlayer.pos.y);
        break;
    case GAMEPROG_LOSE:
        drawCurrentObjects();
        drawPlayer(myGame.myPlayer.pos.x, myGame.myPlayer.pos.y);
        drawLosingScreen();
        break;
    case GAMEPROG_WIN:
        drawCurrentObjects();
        drawPlayer(myGame.myPlayer.pos.x, myGame.myPlayer.pos.y);
        drawWinningScreen();
        break;
    default:
        drawBigTitle();
    }
    wait_for_vsync();
    pixel_buffer_start = *(pixel_ctrl_ptr + 1); // new back buffer
}

/********************************************************************
 * void updatePlayerStatus();
 *
 * Checks if the player have touch any object
 * updates the player's horizontal and veritcal speed
 *******************************************************************/
void updatePlayerStatus()
{
    // Only update when the game started
    if (myGame.progress != GAMEPROG_START)
        return;
    // Horizontal bound checkings
    int baseSpeed = PLAYER_SPEED_NORMAL;

    // Define more easy to read variable
    const int playerVgaPosX = myGame.myPlayer.pos.x;
    const int playerVgaPosXRight = myGame.myPlayer.pos.x + PLAYER_WIDTH - 1;
    const int playerVgaPosY = myGame.myPlayer.pos.y;
    const int playerVgaPosYBottom = myGame.myPlayer.pos.y + PLAYER_HEIGHT - 1;

    switch (myGame.myPlayer.state)
    {
    case PLAYERSTATE_LEFT:
        // Screen resolution bound checking
        if (playerVgaPosX < (0 + baseSpeed))
        {
            myGame.myPlayer.horizontal_speed = 0;
            break;
        }
        // Platform bound checking
        for (int yPos = myGame.myPlayer.pos.y; yPos < myGame.myPlayer.pos.y + PLAYER_HEIGHT; yPos++)
        {
            if (myGame.currentObjects[((int)(playerVgaPosX - baseSpeed) / BOX_LEN)][yPos / BOX_LEN] == GAMEOBJ_PLATFORM_BLOCK ||
                myGame.currentObjects[((int)(playerVgaPosX - baseSpeed) / BOX_LEN)][yPos / BOX_LEN] == GAMEOBJ_START ||
                myGame.currentObjects[((int)(playerVgaPosX - baseSpeed) / BOX_LEN)][yPos / BOX_LEN] == GAMEOBJ_END)
            {
                myGame.myPlayer.horizontal_speed = 0;
                goto updatePlayerStatusHorizontalReturn;
            }
        }
        myGame.myPlayer.horizontal_speed = -baseSpeed;
        break;

    case PLAYERSTATE_RIGHT:
        // Screen resolution bound checking
        if (playerVgaPosXRight >= (RESOLUTION_X - baseSpeed))
        {
            myGame.myPlayer.horizontal_speed = 0;
            break;
        }
        // Platform bound checking
        for (int yPos = myGame.myPlayer.pos.y; yPos < myGame.myPlayer.pos.y + PLAYER_HEIGHT; yPos++)
        {
            if (myGame.currentObjects[((int)(playerVgaPosXRight + baseSpeed) / BOX_LEN)][yPos / BOX_LEN] == GAMEOBJ_PLATFORM_BLOCK ||
                myGame.currentObjects[((int)(playerVgaPosXRight + baseSpeed) / BOX_LEN)][yPos / BOX_LEN] == GAMEOBJ_START ||
                myGame.currentObjects[((int)(playerVgaPosXRight + baseSpeed) / BOX_LEN)][yPos / BOX_LEN] == GAMEOBJ_END)
            {
                myGame.myPlayer.horizontal_speed = 0;
                goto updatePlayerStatusHorizontalReturn;
            }
        }
        myGame.myPlayer.horizontal_speed = baseSpeed;
        break;
    case PLAYERSTATE_STILL:
        myGame.myPlayer.horizontal_speed = 0;
        break;
    default:
        myGame.myPlayer.horizontal_speed = 0;
    }

// Update the player's horizontal position
updatePlayerStatusHorizontalReturn:
    myGame.myPlayer.pos.x += myGame.myPlayer.horizontal_speed;

    if (myGame.myPlayer.airborne)
    {
        myGame.myPlayer.vertical_speed += GRAVITY;
    }
    // JumpMechanics
    if (myGame.myPlayer.jump)
    {
        myGame.myPlayer.jump = false;
        myGame.myPlayer.airborne = true;
        myGame.myPlayer.vertical_speed = PLAYER_INITIAL_UP_VELOCITY;
    }
    // bound checking
    // landing check
    for (int xPos = playerVgaPosX; xPos <= playerVgaPosXRight; xPos++)
    {
        // empty ground check
        if (myGame.currentObjects[((int)(xPos) / BOX_LEN)][(playerVgaPosYBottom+1) / BOX_LEN] == GAMEOBJ_EMPTY)
        {
            myGame.myPlayer.airborne = true;
        }
        // roof check
        if (myGame.myPlayer.vertical_speed > 0)
        {
            for (int yPos = playerVgaPosY; yPos >= playerVgaPosY - myGame.myPlayer.vertical_speed; yPos--)
            {
                // check of upper block first
                // roof check
                if (myGame.currentObjects[((int)(xPos) / BOX_LEN)][yPos / BOX_LEN] == GAMEOBJ_PLATFORM_BLOCK ||
                    myGame.currentObjects[((int)(xPos) / BOX_LEN)][yPos / BOX_LEN] == GAMEOBJ_START ||
                    myGame.currentObjects[((int)(xPos) / BOX_LEN)][yPos / BOX_LEN] == GAMEOBJ_END)
                {
                    myGame.myPlayer.vertical_speed = 0;
                    myGame.myPlayer.pos.y = yPos + 1; // update the y position
                    goto updatePlayerStatusVerticalReturn;
                }
                // spike checking
                if (myGame.currentObjects[((int)(xPos) / BOX_LEN)][yPos / BOX_LEN] == GAMEOBJ_FIREBALL ||
                    myGame.currentObjects[((int)(xPos) / BOX_LEN)][yPos / BOX_LEN] == GAMEOBJ_SPIKE)
                {
                    myGame.myPlayer.vertical_speed = 0;
                    myGame.myPlayer.pos.y = yPos; // update the y position, the player walked into the spike
                    goto updatePlayerStatusVerticalReturn;
                }
            }
            for (int yPos = playerVgaPosY; yPos >= playerVgaPosY - myGame.myPlayer.vertical_speed; yPos--)
            {
                if (yPos < 0)
                {
                    myGame.myPlayer.vertical_speed = 0;
                    myGame.myPlayer.pos.y = 0; // update the y posotion
                    goto updatePlayerStatusVerticalReturn;
                }
            }
        }
        if (myGame.myPlayer.vertical_speed < 0)
        {
            for (int yPos = playerVgaPosY; yPos <= playerVgaPosY - myGame.myPlayer.vertical_speed; yPos++)
            {
                // ground check
                if (myGame.currentObjects[((int)(xPos) / BOX_LEN)][(yPos + PLAYER_HEIGHT - 1) / BOX_LEN] == GAMEOBJ_PLATFORM_BLOCK ||
                    myGame.currentObjects[((int)(xPos) / BOX_LEN)][(yPos + PLAYER_HEIGHT - 1) / BOX_LEN] == GAMEOBJ_START ||
                    myGame.currentObjects[((int)(xPos) / BOX_LEN)][(yPos + PLAYER_HEIGHT - 1) / BOX_LEN] == GAMEOBJ_END)
                {
                    myGame.myPlayer.vertical_speed = 0;
                    myGame.myPlayer.airborne = false;
                    myGame.myPlayer.pos.y = yPos - 1; // update the y position
                    goto updatePlayerStatusVerticalReturn;
                }
                if (myGame.currentObjects[((int)(xPos) / BOX_LEN)][(yPos + PLAYER_HEIGHT - 1) / BOX_LEN] == GAMEOBJ_FIREBALL ||
                    myGame.currentObjects[((int)(xPos) / BOX_LEN)][(yPos + PLAYER_HEIGHT - 1) / BOX_LEN] == GAMEOBJ_SPIKE)
                {
                    myGame.myPlayer.vertical_speed = 0;
                    myGame.myPlayer.airborne = false;
                    myGame.myPlayer.pos.y = yPos; // update the y position
                    goto updatePlayerStatusVerticalReturn;
                }
            }
            for (int yPos = playerVgaPosY; yPos <= playerVgaPosY - myGame.myPlayer.vertical_speed; yPos++)
            {
                if (yPos + PLAYER_HEIGHT - 1 >= RESOLUTION_Y)
                {
                    myGame.myPlayer.vertical_speed = 0;
                    myGame.myPlayer.pos.y = RESOLUTION_Y - 1 - PLAYER_HEIGHT;
                    myGame.progress = GAMEPROG_LOSE;
                    goto updatePlayerStatusVerticalReturn;
                }
            }
        }
    }
updatePlayerStatusVerticalReturn:
    myGame.myPlayer.pos.y -= myGame.myPlayer.vertical_speed;
}

/********************************************************************
 * void hitboxCheck();
 *
 * check if the player has hit anything
 *******************************************************************/
void hitboxCheck()
{
    // Define more easy to read variable
    const int playerVgaPosX = myGame.myPlayer.pos.x;
    const int playerVgaPosXRight = myGame.myPlayer.pos.x + PLAYER_WIDTH - 1;

    // Fireball and spike bound checking
    for (int yPos = myGame.myPlayer.pos.y; yPos < myGame.myPlayer.pos.y + PLAYER_HEIGHT; yPos++)
    {
        if (myGame.currentObjects[((int)(playerVgaPosX) / BOX_LEN)][yPos / BOX_LEN] == GAMEOBJ_FIREBALL ||
            myGame.currentObjects[((int)(playerVgaPosX) / BOX_LEN)][yPos / BOX_LEN] == GAMEOBJ_SPIKE)
        {
            myGame.progress = GAMEPROG_LOSE;
            return;
            ;
        }
    }
    // Fireball and spike bound checking
    for (int yPos = myGame.myPlayer.pos.y; yPos < myGame.myPlayer.pos.y + PLAYER_HEIGHT; yPos++)
    {
        if (myGame.currentObjects[((int)(playerVgaPosXRight) / BOX_LEN)][yPos / BOX_LEN] == GAMEOBJ_FIREBALL ||
            myGame.currentObjects[((int)(playerVgaPosXRight) / BOX_LEN)][yPos / BOX_LEN] == GAMEOBJ_SPIKE)
        {
            myGame.progress = GAMEPROG_LOSE;
            return;
        }
    }

    // Winning check
    if (myGame.currentObjects[((playerVgaPosXRight + playerVgaPosX) / (2 * BOX_LEN))][(myGame.myPlayer.pos.y + PLAYER_HEIGHT) / BOX_LEN] == GAMEOBJ_END)
        myGame.progress = GAMEPROG_WIN;
}

/********************************************************************
 * void resetGame();
 *
 * reset the game to its initial state
 *******************************************************************/
void resetGame()
{
    stop_mpcore_priv_timer();

    // clear both VGA buffers
    clear_screen();
    updateLevel(gameLevels[0]);
    updatePlayerStatus();

    start_mpcore_priv_timer();

    return;
}

/********************************************************************
 * void updateCurrentGame();
 *
 * update the current player and game state based on the flags
 * Reserve for player phyiscs
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
    // start the game when any key is being pressed
    if (byte3 == 0xF0)
        return;
    // player movements
    // Jump mechanics
    if (byte3 == 0x1D && myGame.myPlayer.airborne == false)
        myGame.myPlayer.jump = true;

    if (byte2 == 0xF0 && byte3 != 0x1D) // make sure the the button released is not the jump button
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

/********************************************************************
 * draw[letter](int vgaX, int vgaY, short int textColour)
 *
 * the following functions draws out their respective letters
 *******************************************************************/
void drawA(int vgaX, int vgaY, short int textColour)
{
    for (int i = 1; i <= 3; i++)
    {
        plot_pixel(vgaX + i, vgaY, textColour);
        plot_pixel(vgaX + i, vgaY + 5, textColour);
    }

    for (int j = 1; j <= 9; j++)
    {
        plot_pixel(vgaX, vgaY + j, textColour);
        plot_pixel(vgaX + 4, vgaY + j, textColour);
    }
}

void drawB(int vgaX, int vgaY, short int textColour)
{
    for (int i = 0; i <= 3; i++)
    {
        plot_pixel(vgaX + i, vgaY, textColour);
        plot_pixel(vgaX + i, vgaY + 4, textColour);
        plot_pixel(vgaX + i, vgaY + 9, textColour);
    }

    for (int j = 1; j <= 8; j++)
    {
        if (j != 4)
        {
            plot_pixel(vgaX, vgaY + j, textColour);
            plot_pixel(vgaX + 4, vgaY + j, textColour);
        }
    }
}

void drawC(int vgaX, int vgaY, short int textColour)
{
    for (int i = 0; i <= 4; i++)
    {
        plot_pixel(vgaX + i, vgaY, textColour);
        plot_pixel(vgaX + i, vgaY + 9, textColour);
    }
    for (int j = 0; j <= 9; j++)
    {
        plot_pixel(vgaX, vgaY + j, textColour);
    }
}

void drawD(int vgaX, int vgaY, short int textColour)
{
    for (int i = 0; i <= 3; i++)
    {
        plot_pixel(vgaX + i, vgaY, textColour);
        plot_pixel(vgaX + i, vgaY + 9, textColour);
    }

    for (int j = 1; j <= 8; j++)
    {
        plot_pixel(vgaX, vgaY + j, textColour);
        plot_pixel(vgaX + 4, vgaY + j, textColour);
    }
}

void drawE(int vgaX, int vgaY, short int textColour)
{
    for (int i = 0; i <= 4; i++)
    {
        plot_pixel(vgaX + i, vgaY, textColour);
        plot_pixel(vgaX + i, vgaY + 9, textColour);
    }
    for (int i = 0; i <= 3; i++)
    {
        plot_pixel(vgaX + i, vgaY + 4, textColour);
    }
    for (int j = 0; j <= 9; j++)
    {
        plot_pixel(vgaX, vgaY + j, textColour);
    }
}

void drawF(int vgaX, int vgaY, short int textColour)
{
    for (int i = 0; i <= 4; i++)
    {
        plot_pixel(vgaX + i, vgaY, textColour);
    }
    for (int i = 0; i <= 3; i++)
    {
        plot_pixel(vgaX + i, vgaY + 4, textColour);
    }
    for (int j = 0; j <= 9; j++)
    {
        plot_pixel(vgaX, vgaY + j, textColour);
    }
}

void drawG(int vgaX, int vgaY, short int textColour)
{
    for (int i = 1; i <= 3; i++)
    {
        plot_pixel(vgaX + i, vgaY, textColour);
        plot_pixel(vgaX + i, vgaY + 9, textColour);
    }
    for (int j = 1; j <= 8; j++)
    {
        plot_pixel(vgaX, vgaY + j, textColour);
        if (j != 3 && j != 4)
        {
            plot_pixel(vgaX + 4, vgaY + j, textColour);
        }
    }
    for (int i = 2; i <= 4; i++)
    {
        plot_pixel(vgaX + i, vgaY + 5, textColour);
    }
}

void drawH(int vgaX, int vgaY, short int textColour)
{
    for (int j = 0; j <= 9; j++)
    {
        plot_pixel(vgaX, vgaY + j, textColour);
        plot_pixel(vgaX + 4, vgaY + j, textColour);
    }
    for (int i = 1; i <= 3; i++)
    {
        plot_pixel(vgaX + i, vgaY + 4, textColour);
    }
}

void drawI(int vgaX, int vgaY, short int textColour)
{
    for (int i = 0; i <= 4; i++)
    {
        plot_pixel(vgaX + i, vgaY, textColour);
        plot_pixel(vgaX + i, vgaY + 9, textColour);
    }
    for (int j = 0; j <= 9; j++)
    {
        plot_pixel(vgaX + 2, vgaY + j, textColour);
    }
}

void drawJ(int vgaX, int vgaY, short int textColour)
{
    for (int i = 0; i <= 4; i++)
    {
        plot_pixel(vgaX + i, vgaY, textColour);
    }
    for (int j = 0; j <= 8; j++)
    {
        plot_pixel(vgaX + 3, vgaY + j, textColour);
    }
    plot_pixel(vgaX, vgaY + 7, textColour);
    plot_pixel(vgaX, vgaY + 8, textColour);
    plot_pixel(vgaX + 1, vgaY + 9, textColour);
    plot_pixel(vgaX + 2, vgaY + 9, textColour);
}

void drawK(int vgaX, int vgaY, short int textColour)
{
    for (int j = 0; j <= 9; j++)
    {
        plot_pixel(vgaX, vgaY + j, textColour);
    }
    for (int i = 1; i <= 4; i++)
    {
        plot_pixel(vgaX + i, vgaY + 4 - i, textColour);
        plot_pixel(vgaX + i, vgaY + 5 + i, textColour);
    }
    plot_pixel(vgaX + 4, vgaY, textColour);
    plot_pixel(vgaX + 4, vgaY + 9, textColour);
}

void drawL(int vgaX, int vgaY, short int textColour)
{
    for (int j = 0; j <= 9; j++)
    {
        plot_pixel(vgaX, vgaY + j, textColour);
    }
    for (int i = 1; i <= 4; i++)
    {
        plot_pixel(vgaX + i, vgaY + 9, textColour);
    }
}

void drawM(int vgaX, int vgaY, short int textColour)
{
    for (int j = 0; j <= 9; j++)
    {
        plot_pixel(vgaX, vgaY + j, textColour);
        plot_pixel(vgaX + 4, vgaY + j, textColour);
    }
    for (int j = 1; j <= 2; j++)
    {
        plot_pixel(vgaX + 1, vgaY + j, textColour);
        plot_pixel(vgaX + 2, vgaY + 2 + j, textColour);
        plot_pixel(vgaX + 3, vgaY + j, textColour);
    }
}

void drawN(int vgaX, int vgaY, short int textColour)
{
    for (int j = 0; j <= 9; j++)
    {
        plot_pixel(vgaX, vgaY + j, textColour);
        plot_pixel(vgaX + 4, vgaY + j, textColour);
    }
    for (int j = 2; j <= 3; j++)
    {
        plot_pixel(vgaX + 1, vgaY + j, textColour);
        plot_pixel(vgaX + 2, vgaY + 2 + j, textColour);
        plot_pixel(vgaX + 3, vgaY + 4 + j, textColour);
    }
}

void drawO(int vgaX, int vgaY, short int textColour)
{
    for (int j = 1; j <= 8; j++)
    {
        plot_pixel(vgaX, vgaY + j, textColour);
        plot_pixel(vgaX + 4, vgaY + j, textColour);
    }
    for (int i = 1; i <= 3; i++)
    {
        plot_pixel(vgaX + i, vgaY, textColour);
        plot_pixel(vgaX + i, vgaY + 9, textColour);
    }
}

void drawP(int vgaX, int vgaY, short int textColour)
{
    for (int j = 0; j <= 9; j++)
    {
        plot_pixel(vgaX, vgaY + j, textColour);
    }
    for (int i = 1; i <= 3; i++)
    {
        plot_pixel(vgaX + i, vgaY, textColour);
        plot_pixel(vgaX + i, vgaY + 4, textColour);
    }
    for (int j = 1; j <= 3; j++)
    {
        plot_pixel(vgaX + 4, vgaY + j, textColour);
    }
}

void drawQ(int vgaX, int vgaY, short int textColour)
{
    for (int j = 1; j <= 8; j++)
    {
        plot_pixel(vgaX, vgaY + j, textColour);
        if (j != 8)
        {
            plot_pixel(vgaX + 4, vgaY + j, textColour);
        }
    }
    for (int i = 1; i <= 3; i++)
    {
        plot_pixel(vgaX + i, vgaY, textColour);
        if (i != 3)
        {
            plot_pixel(vgaX + i, vgaY + 9, textColour);
        }
    }
    plot_pixel(vgaX + 2, vgaY + 6, textColour);
    plot_pixel(vgaX + 2, vgaY + 7, textColour);
    plot_pixel(vgaX + 3, vgaY + 8, textColour);
    plot_pixel(vgaX + 4, vgaY + 9, textColour);
}

void drawR(int vgaX, int vgaY, short int textColour)
{
    for (int j = 0; j <= 9; j++)
    {
        plot_pixel(vgaX, vgaY + j, textColour);
    }
    for (int i = 1; i <= 3; i++)
    {
        plot_pixel(vgaX + i, vgaY, textColour);
        plot_pixel(vgaX + i, vgaY + 5, textColour);
        plot_pixel(vgaX + i, vgaY + 5 + i, textColour);
    }
    plot_pixel(vgaX + 4, vgaY + 9, textColour);
    for (int j = 1; j <= 4; j++)
    {
        plot_pixel(vgaX + 4, vgaY + j, textColour);
    }
}

void drawS(int vgaX, int vgaY, short int textColour)
{
    for (int i = 1; i <= 3; i++)
    {
        plot_pixel(vgaX + i, vgaY, textColour);
        plot_pixel(vgaX + i, vgaY + 4, textColour);
        plot_pixel(vgaX + i, vgaY + 9, textColour);
    }
    for (int j = 1; j <= 3; j++)
    {
        plot_pixel(vgaX, vgaY + j, textColour);
    }
    for (int j = 5; j <= 8; j++)
    {
        plot_pixel(vgaX + 4, vgaY + j, textColour);
    }
    plot_pixel(vgaX + 4, vgaY + 1, textColour);
    plot_pixel(vgaX, vgaY + 8, textColour);
}

void drawT(int vgaX, int vgaY, short int textColour)
{
    for (int i = 0; i <= 4; i++)
    {
        plot_pixel(vgaX + i, vgaY, textColour);
    }
    for (int j = 0; j <= 9; j++)
    {
        plot_pixel(vgaX + 2, vgaY + j, textColour);
    }
}

void drawU(int vgaX, int vgaY, short int textColour)
{
    for (int j = 0; j <= 8; j++)
    {
        plot_pixel(vgaX, vgaY + j, textColour);
        plot_pixel(vgaX + 4, vgaY + j, textColour);
    }
    for (int i = 1; i <= 4; i++)
    {
        plot_pixel(vgaX + i, vgaY + 9, textColour);
    }
}

void drawV(int vgaX, int vgaY, short int textColour)
{
    for (int j = 0; j <= 4; j++)
    {
        plot_pixel(vgaX, vgaY + j, textColour);
        plot_pixel(vgaX + 4, vgaY + j, textColour);
    }
    for (int j = 5; j <= 8; j++)
    {
        plot_pixel(vgaX + 1, vgaY + j, textColour);
        plot_pixel(vgaX + 3, vgaY + j, textColour);
    }
    plot_pixel(vgaX + 2, vgaY + 9, textColour);
}

void drawW(int vgaX, int vgaY, short int textColour)
{
    for (int j = 0; j <= 6; j++)
    {
        plot_pixel(vgaX, vgaY + j, textColour);
        plot_pixel(vgaX + 4, vgaY + j, textColour);
    }
    for (int j = 3; j <= 6; j++)
    {
        plot_pixel(vgaX + 2, vgaY + j, textColour);
    }
    for (int j = 7; j <= 9; j++)
    {
        plot_pixel(vgaX + 1, vgaY + j, textColour);
        plot_pixel(vgaX + 3, vgaY + j, textColour);
    }
}

void drawX(int vgaX, int vgaY, short int textColour)
{
    for (int j = 0; j <= 1; j++)
    {
        plot_pixel(vgaX, vgaY + j, textColour);
        plot_pixel(vgaX + 4, vgaY + j, textColour);
    }
    for (int j = 2; j <= 3; j++)
    {
        plot_pixel(vgaX + 1, vgaY + j, textColour);
        plot_pixel(vgaX + 3, vgaY + j, textColour);
    }
    for (int j = 4; j <= 5; j++)
    {
        plot_pixel(vgaX + 2, vgaY + j, textColour);
    }
    for (int j = 6; j <= 7; j++)
    {
        plot_pixel(vgaX + 1, vgaY + j, textColour);
        plot_pixel(vgaX + 3, vgaY + j, textColour);
    }
    for (int j = 8; j <= 9; j++)
    {
        plot_pixel(vgaX, vgaY + j, textColour);
        plot_pixel(vgaX + 4, vgaY + j, textColour);
    }
}

void drawY(int vgaX, int vgaY, short int textColour)
{
    for (int j = 0; j <= 1; j++)
    {
        plot_pixel(vgaX, vgaY + j, textColour);
        plot_pixel(vgaX + 4, vgaY + j, textColour);
    }
    for (int j = 2; j <= 3; j++)
    {
        plot_pixel(vgaX + 1, vgaY + j, textColour);
        plot_pixel(vgaX + 3, vgaY + j, textColour);
    }
    for (int j = 4; j <= 9; j++)
    {
        plot_pixel(vgaX + 2, vgaY + j, textColour);
    }
}

void drawZ(int vgaX, int vgaY, short int textColour)
{
    for (int i = 0; i <= 4; i++)
    {
        plot_pixel(vgaX + i, vgaY, textColour);
        plot_pixel(vgaX + i, vgaY + 9, textColour);
    }
    plot_pixel(vgaX + 4, vgaY + 1, textColour);
    plot_pixel(vgaX + 3, vgaY + 2, textColour);
    plot_pixel(vgaX + 3, vgaY + 3, textColour);
    plot_pixel(vgaX, vgaY + 4, textColour);
    plot_pixel(vgaX + 1, vgaY + 4, textColour);
    plot_pixel(vgaX + 2, vgaY + 4, textColour);
    plot_pixel(vgaX + 2, vgaY + 5, textColour);
    plot_pixel(vgaX + 3, vgaY + 5, textColour);
    plot_pixel(vgaX + 4, vgaY + 5, textColour);
    plot_pixel(vgaX + 1, vgaY + 6, textColour);
    plot_pixel(vgaX + 1, vgaY + 7, textColour);
    plot_pixel(vgaX, vgaY + 8, textColour);
}

/********************************************************************
 * draw[Number](int vgaX, int vgaY, short int textColour)
 *
 * the following functions draws out their respective numbers
 *******************************************************************/
void draw0(int vgaX, int vgaY, short int textColour)
{
    for (int j = 1; j <= 8; j++)
    {
        plot_pixel(vgaX, vgaY + j, textColour);
        plot_pixel(vgaX + 4, vgaY + j, textColour);
    }
    for (int i = 1; i <= 3; i++)
    {
        plot_pixel(vgaX + i, vgaY, textColour);
        plot_pixel(vgaX + i, vgaY + 9, textColour);
    }
}

void draw1(int vgaX, int vgaY, short int textColour)
{
    for (int j = 0; j <= 9; j++)
    {
        plot_pixel(vgaX + 4, vgaY + j, textColour);
    }
}

void draw2(int vgaX, int vgaY, short int textColour)
{
    for (int i = 0; i <= 4; i++)
    {
        plot_pixel(vgaX + i, vgaY + 9, textColour);
    }
    for (int i = 1; i <= 3; i++)
    {
        plot_pixel(vgaX + i, vgaY, textColour);
    }
    for (int j = 1; j <= 4; j++)
    {
        plot_pixel(vgaX + 4, vgaY + j, textColour);
    }
    plot_pixel(vgaX, vgaY + 1, textColour);
    plot_pixel(vgaX, vgaY + 2, textColour);
    for (int i = 0; i <= 3; i++)
    {
        plot_pixel(vgaX + i, vgaY + 8 - i, textColour);
    }
}

void draw3(int vgaX, int vgaY, short int textColour)
{
    for (int j = 1; j <= 8; j++)
    {
        if (j != 4)
        {
            plot_pixel(vgaX + 4, vgaY + j, textColour);
        }
    }
    for (int i = 1; i <= 3; i++)
    {
        plot_pixel(vgaX + i, vgaY, textColour);
        if (i != 1)
        {
            plot_pixel(vgaX + i, vgaY + 4, textColour);
        }
        plot_pixel(vgaX + i, vgaY + 9, textColour);
    }
    plot_pixel(vgaX, vgaY + 1, textColour);
    plot_pixel(vgaX, vgaY + 8, textColour);
}

void draw4(int vgaX, int vgaY, short int textColour)
{
    for (int j = 0; j <= 9; j++)
    {
        plot_pixel(vgaX + 4, vgaY + j, textColour);
    }
    for (int j = 0; j <= 4; j++)
    {
        plot_pixel(vgaX, vgaY + j, textColour);
    }
    for (int i = 1; i <= 4; i++)
    {
        plot_pixel(vgaX + i, vgaY + 5, textColour);
    }
}

void draw5(int vgaX, int vgaY, short int textColour)
{
    for (int i = 1; i <= 3; i++)
    {
        plot_pixel(vgaX + i, vgaY + 4, textColour);
        plot_pixel(vgaX + i, vgaY + 9, textColour);
    }
    for (int i = 0; i <= 4; i++)
    {
        plot_pixel(vgaX + i, vgaY, textColour);
    }
    for (int j = 1; j <= 4; j++)
    {
        plot_pixel(vgaX, vgaY + j, textColour);
    }
    for (int j = 5; j <= 8; j++)
    {
        plot_pixel(vgaX + 4, vgaY + j, textColour);
    }
    plot_pixel(vgaX, vgaY + 8, textColour);
}

void draw6(int vgaX, int vgaY, short int textColour)
{
    for (int j = 1; j <= 8; j++)
    {
        plot_pixel(vgaX, vgaY + j, textColour);
    }
    for (int i = 1; i <= 3; i++)
    {
        plot_pixel(vgaX + i, vgaY, textColour);
        plot_pixel(vgaX + i, vgaY + 4, textColour);
        plot_pixel(vgaX + i, vgaY + 9, textColour);
    }
    for (int j = 5; j <= 8; j++)
    {
        plot_pixel(vgaX + 4, vgaY + j, textColour);
    }
    plot_pixel(vgaX + 4, vgaY + 1, textColour);
}

void draw7(int vgaX, int vgaY, short int textColour)
{
    for (int i = 0; i <= 4; i++)
    {
        plot_pixel(vgaX + i, vgaY, textColour);
    }
    plot_pixel(vgaX + 4, vgaY + 1, textColour);
    plot_pixel(vgaX + 3, vgaY + 2, textColour);
    plot_pixel(vgaX + 3, vgaY + 3, textColour);
    plot_pixel(vgaX + 2, vgaY + 4, textColour);
    plot_pixel(vgaX + 2, vgaY + 5, textColour);
    plot_pixel(vgaX + 1, vgaY + 6, textColour);
    plot_pixel(vgaX + 1, vgaY + 7, textColour);
    plot_pixel(vgaX, vgaY + 8, textColour);
    plot_pixel(vgaX, vgaY + 9, textColour);
}

void draw8(int vgaX, int vgaY, short int textColour)
{
    for (int j = 1; j <= 8; j++)
    {
        if (j != 4)
        {
            plot_pixel(vgaX, vgaY + j, textColour);
            plot_pixel(vgaX + 4, vgaY + j, textColour);
        }
    }
    for (int i = 1; i <= 3; i++)
    {
        plot_pixel(vgaX + i, vgaY, textColour);
        plot_pixel(vgaX + i, vgaY + 4, textColour);
        plot_pixel(vgaX + i, vgaY + 9, textColour);
    }
}

void draw9(int vgaX, int vgaY, short int textColour)
{
    for (int j = 1; j <= 8; j++)
    {
        plot_pixel(vgaX + 4, vgaY + j, textColour);
    }
    for (int i = 1; i <= 3; i++)
    {
        plot_pixel(vgaX + i, vgaY, textColour);
        plot_pixel(vgaX + i, vgaY + 4, textColour);
        plot_pixel(vgaX + i, vgaY + 9, textColour);
    }
    for (int j = 1; j <= 3; j++)
    {
        plot_pixel(vgaX, vgaY + j, textColour);
    }
    plot_pixel(vgaX, vgaY + 8, textColour);
}

int main(void)
{
    // Setup all the levels before the game
    setupLevels();
    clear_screen();
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