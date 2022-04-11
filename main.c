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

/* Functions declarations */
/* interrupt controls */
void disable_A9_interrupts(void);
void set_A9_IRQ_stack(void);
void config_GIC(void);
void config_KEYs(void);
void enable_A9_interrupts(void);
void config_interrupt(int, int);

/* Interrupt routines */
void pushbutton_ISR(void);

/* VGA setup routine */
void setupVGA(void);   // Setup the front and back buffer of the VGA display
void wait_for_vsync(); // Swap between front and back buffer
void clear_screen();

/* Data structures */
/* On screen objects */
enum GameObject
{
    EMPTY = 0, // default value of the enum is empty
    PLAYER,
    PLATFORM_BLOCK,
    FIREBALL,
    SPIKE,
    START,
    END,
    THANOS
};

enum PlayerState
{
    STILL = 0,
    LEFT,
    RIGHT
};

typedef struct position
{
    int x;
    int y;
} Position;

typedef struct player
{
    Position pos;
    enum PlayerState state;
    bool jump;
} Player;

/* Global variables */
/* Onscreen objects */
enum GameObject currentObjects[BLOCK_RESOLUTION_X][BLOCK_RESOLUTION_Y];
Player myPlayer;

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

// Define the IRQ exception handler
void __attribute__((interrupt)) __cs3_isr_irq(void)
{
    // Read the ICCIAR from the CPU Interface in the GIC
    int interrupt_ID = *((int *)0xFFFEC10C);
    if (interrupt_ID == 73) // check if interrupt is from the KEYs
        pushbutton_ISR();
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
    pixel_buffer_start = *pixel_ctrl_ptr;
    clear_screen(); // pixel_buffer_start points to the pixel buffer
    /* set back pixel buffer to start of SDRAM memory */
    *(pixel_ctrl_ptr + 1) = 0xC0000000;
    pixel_buffer_start = *(pixel_ctrl_ptr + 1); // we draw on the back buffer
    clear_screen();                             // pixel_buffer_start points to the pixel buffer
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
 * wait_for_vsync
 *
 * The function swaps the front and back buffer
 *******************************************************************/
void clear_screen()
{
    int x;
    int y;
    for (x = 0; x < RESOLUTION_X; x++)
    {
        for (y = 0; y < RESOLUTION_Y; y++)
        {
            *(short int *)(pixel_buffer_start + (y << 10) + (x << 1)) = WHITE;
        }
    }
}

int main(void)
{
    /* Interrupt setup routine */
    disable_A9_interrupts(); // disable interrupts in the A9 processor
    set_A9_IRQ_stack();      // initialize the stack pointer for IRQ mode
    config_GIC();            // configure the general interrupt controller
    config_KEYs();           // configure KEYs to generate interrupts
    enable_A9_interrupts();  // enable interrupts in the A9 processor

    /* VGA setup routine */
    setupVGA();

    while (1) // wait for an interrupt
        ;
}