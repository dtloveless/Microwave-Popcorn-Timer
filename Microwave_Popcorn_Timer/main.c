#include <stdio.h>
#include "msp.h"

#define   LED1 BIT0 // P1.0 LED1 for blink indicator
#define   LED2 BIT0 // P2.0 LED2 red segment for pop indicator
#define   BUZZ BIT7 // P1.7 Buzzer output
#define MIC_RE BIT0 // P5.0 digital microphone rising-edge input
#define MIC_FE BIT1 // P5.1 digital microphone falling-edge input
#define    POT BIT0 // P4.0 analog potentiometer input
#define     CE BIT0 // P6.0 chip enable (CS)
#define  RESET BIT6 // P6.6 reset
#define     DC BIT7 // P6.7 data/control
#define TIMER_PERIOD 3277 // 1638/32768Hz * 2 = 0.1s
#define WIDTH 84    // Display width
#define HEIGHT 48   // Display height

/* Function Prototypes */
void ports_init(void);
void timer_init(void);
void ADC_init(void);
int sample_ADC(void);
void GLCD_command_write(unsigned char);
void GLCD_data_write(unsigned char);
void GLCD_init(void);
void GLCD_clear(void);
void GLCD_setCursor(unsigned char, unsigned char);
void GLCD_putchar(int);
void SPI_init(void);
void SPI_write(unsigned char);
void print_float(float);
void display_time(void);
void print_message(void);
void trigger_alarm(void);
void TA0_N_IRQHandler(void);
void PORT1_IRQHandler(void);

/* Global Variables */
float cutoff;     // Seconds between pops
float time = 0.0; // Start timer at 0.0s
int pop = 0;      // Pop condition
int end = 0;      // End condition

// Character encoding for GLCD display
const int font_table[][6] = {
    {0x3e, 0x51, 0x49, 0x45, 0x3e, 0x00}, // 0 //  0
    {0x00, 0x42, 0x7f, 0x40, 0x00, 0x00}, // 1 //  1
    {0x42, 0x61, 0x51, 0x49, 0x46, 0x00}, // 2 //  2
    {0x21, 0x41, 0x45, 0x4b, 0x31, 0x00}, // 3 //  3
    {0x18, 0x14, 0x12, 0x7f, 0x10, 0x00}, // 4 //  4
    {0x27, 0x45, 0x45, 0x45, 0x39, 0x00}, // 5 //  5
    {0x3c, 0x4a, 0x49, 0x49, 0x30, 0x00}, // 6 //  6
    {0x01, 0x71, 0x09, 0x05, 0x03, 0x00}, // 7 //  7
    {0x36, 0x49, 0x49, 0x49, 0x36, 0x00}, // 8 //  8
    {0x0e, 0x49, 0x49, 0x29, 0x1e, 0x00}, // 9 //  9
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, //   // 10
    {0x7e, 0x11, 0x11, 0x11, 0x7e, 0x00}, // A // 11
    {0x7f, 0x49, 0x49, 0x49, 0x36, 0x00}, // B // 12
    {0x3e, 0x41, 0x41, 0x41, 0x22, 0x00}, // C // 13
    {0x7f, 0x41, 0x41, 0x41, 0x3e, 0x00}, // D // 14
    {0x7f, 0x49, 0x49, 0x49, 0x41, 0x00}, // E // 15
    {0x7f, 0x09, 0x09, 0x09, 0x01, 0x00}, // F // 16
    {0x3e, 0x41, 0x49, 0x49, 0x7a, 0x00}, // G // 17
    {0x7f, 0x08, 0x08, 0x08, 0x7f, 0x00}, // H // 18
    {0x41, 0x41, 0x7f, 0x41, 0x41, 0x00}, // I // 19
    {0x20, 0x40, 0x40, 0x40, 0x3f, 0x00}, // J // 20
    {0x7f, 0x08, 0x14, 0x22, 0x41, 0x00}, // K // 21
    {0x7f, 0x40, 0x40, 0x40, 0x40, 0x00}, // L // 22
    {0x7f, 0x02, 0x0c, 0x02, 0x7f, 0x00}, // M // 23
    {0x7f, 0x04, 0x08, 0x10, 0x7f, 0x00}, // N // 24
    {0x3e, 0x41, 0x41, 0x41, 0x3e, 0x00}, // O // 25
    {0x7f, 0x09, 0x09, 0x09, 0x06, 0x00}, // P // 26
    {0x3e, 0x41, 0x51, 0x61, 0x7e, 0x00}, // Q // 27
    {0x7f, 0x09, 0x19, 0x29, 0x46, 0x00}, // R // 28
    {0x26, 0x49, 0x49, 0x49, 0x32, 0x00}, // S // 29
    {0x01, 0x01, 0x7f, 0x01, 0x01, 0x00}, // T // 30
    {0x3f, 0x40, 0x40, 0x40, 0x3f, 0x00}, // U // 31
    {0x1f, 0x20, 0x40, 0x20, 0x1f, 0x00}, // V // 32
    {0x3f, 0x40, 0x38, 0x40, 0x3f, 0x00}, // W // 33
    {0x63, 0x14, 0x08, 0x14, 0x63, 0x00}, // X // 34
    {0x03, 0x04, 0x78, 0x04, 0x03, 0x00}, // Y // 35
    {0x61, 0x51, 0x49, 0x45, 0x43, 0x00}, // Z // 36
    {0x00, 0x00, 0x5f, 0x00, 0x00, 0x00}, // ! // 37
    {0x20, 0x10, 0x08, 0x04, 0x02, 0x00}, // / // 38
    {0x00, 0x60, 0x60, 0x00, 0x00, 0x00}, // . // 39
    {0x48, 0x54, 0x54, 0x54, 0x20, 0x00}, // s // 40
    {0x14, 0x08, 0x3e, 0x08, 0x14, 0x00}, // * // 41
};

/**
 * main.c
 */
void main(void)
{
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer

    ports_init();   // Configure Ports
    timer_init();   // Configure Timer_A0
    ADC_init();     // Configure ADC14
    GLCD_init();    // Configure GLCD

    while(1);       // Main loop
}

/*
 * Displays time since last pop vs cutoff interval
 */
void display_time(void){
    GLCD_clear();
    print_float(time);
    GLCD_putchar(38);
    print_float(cutoff);
    GLCD_putchar(40);
    if(pop){
        GLCD_putchar(10);
        GLCD_putchar(41);
        pop = 0;
    }
}

/*
 * Prints a floating point number with 2 digits
 * (including one decimal point)
 */
void print_float(float num){
    char num_str[3];
    sprintf(num_str, "%.1f", num);

    int i;
    for(i = 0; i < 3; i++){
        switch(num_str[i]){
            case '0':
                GLCD_putchar(0);
                break;
            case '1':
                GLCD_putchar(1);
                break;
            case '2':
                GLCD_putchar(2);
                break;
            case '3':
                GLCD_putchar(3);
                break;
            case '4':
                GLCD_putchar(4);
                break;
            case '5':
                GLCD_putchar(5);
                break;
            case '6':
                GLCD_putchar(6);
                break;
            case '7':
                GLCD_putchar(7);
                break;
            case '8':
                GLCD_putchar(8);
                break;
            case '9':
                GLCD_putchar(9);
                break;
            case '.':
                GLCD_putchar(39);
                break;
        }
    }
}

/*
 * Flash lights and sound buzzer 4 times when popcorn is done
 */
void trigger_alarm(void){
    GLCD_clear();

    int i;
    for(i = 0; i < 4; i++){
        P1->OUT |= BUZZ;        // Activate buzzer
        print_message();        // Display "POPCORN READY!"
        __delay_cycles(300000); // Wait
        P1->OUT &= ~BUZZ;       // Buzzer off
        GLCD_clear();           // Clear message
        __delay_cycles(300000); // Wait
    }
    print_message();            // Lingering message
}

/*
 * Display "POPCORN READY!"
 */
void print_message(void){
    GLCD_clear();
    GLCD_putchar(26); // P
    GLCD_putchar(25); // O
    GLCD_putchar(26); // P
    GLCD_putchar(13); // C
    GLCD_putchar(25); // O
    GLCD_putchar(28); // R
    GLCD_putchar(24); // N
    GLCD_putchar(10); //
    GLCD_putchar(28); // R
    GLCD_putchar(15); // E
    GLCD_putchar(11); // A
    GLCD_putchar(14); // D
    GLCD_putchar(35); // Y
    GLCD_putchar(37); // !
}

/*
 * Handle microphone interrupts
 */
void PORT5_IRQHandler(void){
    if(P5->IFG & MIC_RE){
        P5->IFG &= ~MIC_RE; // Clear interrupt flag
        P2->OUT |= LED2;    // Turn on LED2
        pop = 1;            // Register a 'pop'
    }
    if(P5->IFG & MIC_FE){
        P5->IFG &= ~MIC_FE; // Clear interrupt flag

        // Debounce signal
        int i;
        for(i=0;i<600;i++);

        time = 0;           // Reset time when a pop is registered
        P2->OUT &= ~LED2;   // Turn off LED2
    }
}

/*
 * Handle timer interrupts
 */
void TA0_N_IRQHandler(void){
    TIMER_A0->CTL &= ~TIMER_A_CTL_IFG; // Clear interrupt flag
    if(!end){
        P1->OUT ^= LED1;               // Toggle blinking LED (P1.0)

        cutoff = sample_ADC() / 819.4; // (0, 4096) -> (0, 5)
        time += 0.2;                   // Increment time
        display_time();                // Update display

        if(time >= cutoff){            // Alarm condition
            time = 0;                  // Reset time to prevent double activation
            end = 1;                   // Meet end condition
            trigger_alarm();           // Sound alarm
        }
    }
}

/*
 * Displays a character
 */
void GLCD_putchar(int c){
    int i;
    for(i = 0; i < 6; i++)
        GLCD_data_write(font_table[c][i]);
}

/*
 * Clears the GLCD by writing zeros to the entire screen
 */
void GLCD_clear(void){
    int i;
    for (i = 0; i < (WIDTH * HEIGHT / 8); i++){
        GLCD_data_write(0x00);
    }
    GLCD_setCursor(0,0);
}

/*
 * Write to GLCD controller data register
 */
void GLCD_data_write(unsigned char data){
    P6->OUT |= DC;      // select data register
    SPI_write(data);    // send data via SPI
}

/*
 * Moves the cursor
 */
void GLCD_setCursor(unsigned char x, unsigned char y){
    GLCD_command_write(0x80 | x);   // set column
    GLCD_command_write(0x40 | y);   // set bank row (8 rows per bank)
}

/*
 * Send the initialization commands to PCD8544 GLCD controller
 */
void GLCD_init(void){
    SPI_init();
    // Hardware reset of GLCD controller
    P6->OUT |= RESET;   // deassert Reset PIN

    // Send command words to the display
    GLCD_command_write(0x21);   // extended command mode
    GLCD_command_write(0x98);   // set LCD Vop
    GLCD_command_write(0x04);   // set temp coefficient
    GLCD_command_write(0x17);   // set bias mode
    GLCD_command_write(0x20);   // leave extended mode
    GLCD_command_write(0x0C);   // set display to normal operating mode
}

/*
 * Write to GLCD controller command register
 */
void GLCD_command_write(unsigned char data){
    P6->OUT &= ~DC;     // select command register
    SPI_write(data);    // send data via SPI
}

/*
 * Sends data to the SPI transmit buffer
 */
void SPI_write(unsigned char data){
    P6->OUT &= ~CE;         // assert /CE
    EUSCI_B0->TXBUF = data; // write data
    while (EUSCI_B0->STATW & BIT0); // wait for transmission to be done
    P6->OUT |= CE;          // deassert /CE
}

/*
 * Configure the SPI on UCB0
 */
void SPI_init(void){
    // Configure eUSCI_B0
    EUSCI_B0->CTLW0 = 0x0001;     // Put UCB0 in reset mode
    EUSCI_B0->CTLW0 = 0x69C1;     // Determine protocol parameters (PH=0, PL=1, MSB first, Master, SPI, SMCLK)
    EUSCI_B0->BRW = 3;            // 3 MHz / 3 = 1 MHz
    EUSCI_B0->CTLW0 &= ~0x0001;   // enable UCB0 after configuration

    P1->SEL0 |= BIT5 | BIT6;      // P1.5 and P1.6 for UCB0
    P1->SEL1 &= ~(BIT5 | BIT6);   // P1.5 and P1.6 for UCB0

    P6->DIR |= (CE | RESET | DC); // P6.0, P6.6, P6.7 are output pins
    P6->OUT |= CE;                // CE idle high
    P6->OUT &= ~RESET;            // assert Reset PIN
}

/*
 * Sample ADC14 and perform Analog-to-Digital Conversion (ADC)
 */
int sample_ADC(void){
    int result;

    ADC14->CTL0 |= 1;       // Start a conversion
    while (!ADC14->IFGR0);  // Wait until conversion is complete
    result = ADC14->MEM[0]; // Read conversion result

    return result;
}

/*
 * Configure ADC14
 */
void ADC_init(void){
    // -> Configure P4.7 for A6 (analog input channel 6)
    P6->SEL0 |= POT;
    P6->SEL1 |= POT;
    // -> Configure CTL0
    ADC14->CTL0  = 0x00000010;  // Power on ADC and disable during config
    ADC14->CTL0 |= 0x04080310;  // Configure CTL0 according to instructions
    // -> Configure CTL1
    ADC14->CTL1  = 0x00000020;  // 12-bit resolution (see instructions)
    ADC14->CTL1 |= 0x00000000;  // Configure for memory register 0
    // -> Configure MCTL[0] (memory register 0) to receive input channel 6
    ADC14->MCTL[0] = 0x06;      // A6 input (P4.7), single ended, Vref = AVCC
    // Enable ADC14
    ADC14->CTL0 |= 0x02;        // Enable ADC14 after configuration
}

/*
 * Configure Timer_A0 in up/down mode with predefined period
 * using ACLK (supplied by 32768 Hz LFTX)
 */
void timer_init(void){
    PJ->SEL0 |= BIT0 | BIT1; // Needed for clock/timer

    // Configure clock system
    CS->KEY = CS_KEY_VAL;
    CS->CTL2 |= CS_CTL2_LFXT_EN;
    while(CS->IFG & CS_IFG_LFXTIFG)
        CS->CLRIFG |= CS_CLRIFG_CLR_LFXTIFG;
    CS->CTL1 |= CS_CTL1_SELA_0;
    CS->CLKEN |= CS_CLKEN_REFOFSEL;
    CS->CTL1 &= ~(CS_CTL1_SELS_MASK | CS_CTL1_DIVS_MASK);
    CS ->CTL1 |= CS_CTL1_SELS_2;
    CS->KEY = 0;

    // Configure Timer_A0
    TIMER_A0->CCR[0] = TIMER_PERIOD;
    TIMER_A0->CTL = TIMER_A_CTL_TASSEL_1 | TIMER_A_CTL_MC_3 | TIMER_A_CTL_CLR | TIMER_A_CTL_IE;

    NVIC->ISER[0] = 1 << ((TA0_N_IRQn) & 31);
    __enable_irq();
}

/*
 * Configure Ports
 */
void ports_init(void){
    // Configure P1
    P1->DIR |=   LED1 | BUZZ;       // Set LED1 and buzzer as outputs
    P1->OUT &= ~(LED1 | BUZZ);      // Initialize LED1 and buzzer to off

    // Configure P2
    P2->DIR |=  LED2;               // Set LED2 as output
    P2->OUT &= ~LED2;               // Initialize to off

    // Configure P5
    P5->DIR &= ~(MIC_RE | MIC_FE);  // Set MIC as input
    P5->IE  |=  MIC_RE | MIC_FE;    // Enable interrupts for MIC
    P5->IES &= ~MIC_RE;             // Rising edge interrupt
    P5->IES |=  MIC_FE;             // Falling-edge interrupt

    // Port-level Interrupt Enable for Port 5
    NVIC->ISER[1] |= 0x00000080;
    __enable_interrupts();
}
