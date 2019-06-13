// DEVCFG2
#pragma config FPLLIDIV = DIV_2 // PLL Input Divider (1x Divider)
#pragma config FPLLMUL = MUL_20 // PLL Multiplier (24x Multiplier)
#pragma config UPLLIDIV = DIV_2 // USB PLL Input Divider (12x Divider)
#pragma config UPLLEN = OFF // USB PLL Enable (Disabled and Bypassed)
#pragma config FPLLODIV = DIV_1 // System PLL Output Clock Divider (PLL Divide by 256)
// DEVCFG1
#pragma config FNOSC = PRIPLL // Oscillator Selection Bits (Primary Osc w/PLL (XT+,HS+,EC+PLL))
#pragma config FSOSCEN = ON // Secondary Oscillator Enable (Enabled)
#pragma config IESO = ON // Internal/External Switch Over (Enabled)
#pragma config POSCMOD = HS // Primary Oscillator Configuration (HS osc mode)
#pragma config OSCIOFNC = ON // CLKO Output Signal Active on the OSCO Pin (Enabled)
#pragma config FPBDIV = DIV_8 // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/8)
#pragma config FCKSM = CSDCMD // Clock Switching and Monitor Selection (Clock Switch Disable, FSCM Disabled)
#pragma config WDTPS = PS1048576 // Watchdog Timer Postscaler (1:1048576)
#pragma config FWDTEN = OFF // Watchdog Timer Enable (WDT Disabled (SWDTEN Bit Controls))
// DEVCFG0
#pragma config DEBUG = OFF // Background Debugger Enable (Debugger is disabled)
#pragma config ICESEL = ICS_PGx2 // ICE/ICD Comm Channel Select (ICE EMUC2/EMUD2 pins shared with PGC2/PGD2)
#pragma config PWP = OFF // Program Flash Write Protect (Disable)
#pragma config BWP = OFF // Boot Flash Write Protect bit (Protection Disabled)
#pragma config CP = OFF // Code Protect (Protection Disabled)

#include <p32xxxx.h>
#include <plib.h> // Include the PIC32 Peripheral Library.

#define StatePOR 0x00
#define StateIDLE 0x10
#define StateSEND 0x20
#define StateRECEIVE 0x30
#define SYSCLK 80000000L
#define DESIRED_BAUDRATE 9600
#define RE  LATDbits.LATD8
#define DE  LATDbits.LATD11

/*
void delay(int t) { // 1 ms di delay
    int n = t * 1900; //1900 è un numero ricavato sperimentalmente
    while (n > 0) {
        n--;
    }
}
 */

void initializeUART();
void initializePortsIO();
char CheckButton(unsigned port);

int oldButtonState;
int newButtonState;
char state = StatePOR;
char sendRdy = 1;
char receiveRdy = 0;
char data = 0;

int main(void) {

    initializePortsIO();
    initializeUART();
    // Must enable glocal interrupts - in this case, we are using multi-vector mode
    INTEnableSystemMultiVectoredInt();

    int d13Pressed;
    LATDbits.LATD7 = 0;

    while (1) {

        d13Pressed = CheckButton(PORTGbits.RG6);

        switch (state) {
            case StatePOR:
                state = StateIDLE;
                break;
            case StateIDLE:
                //rimango in ascolto sul canale
                RE = 0;
                DE = 0;

                if (d13Pressed && sendRdy) {
                    state = StateSEND;
                }

                if (receiveRdy) {
                    state = StateRECEIVE;
                }

                break;
            case StateSEND:
                //TODO: capire come fare il send e come/quando settare i flag
                RE = 0;
                DE = 1;
                sendRdy = 0;
                putcUART1('A'); // Transmit 'A' through UART
                break;
            case StateRECEIVE:
                receiveRdy = 0;
                state = StateIDLE; // torno in IDLE solo quando finisco di leggere i dati
                if (data == 'B') {
                    LATDbits.LATD7 = LATDbits.LATD7 ^ 1;
                }
                break;
            default:
                break;
        }
    }
    return 1;
}

void __ISR(_UART1_VECTOR, ipl2) IntUart1Handler(void) {
    // Is this an RX interrupt?
    if (mU1RXGetIntFlag()) {
        // Clear the RX interrupt Flag
        mU1RXClearIntFlag();
        data = (char) ReadUART1(); // Read data from Rx
        receiveRdy = 1;
    }

    //chiama l'interrupt quando ha finito di trasmettere
    if (mU1TXGetIntFlag()) {
        mU1TXClearIntFlag();
        state = StateIDLE;
        sendRdy = 1;
    }
}

void initializeUART() {
    // Optimize PIC32 performance and return peripheral bus frequency
    unsigned int pbClk = SYSTEMConfig(SYSCLK, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);
    // Abilita UART1 and set baud rate to DESIRED_BAUDRATE=9600
    OpenUART1(UART_EN, UART_RX_ENABLE | UART_TX_ENABLE, pbClk / 16 / DESIRED_BAUDRATE - 1);
    // Configure UART1 RX Interrupt
    ConfigIntUART1(UART_INT_PR2 | UART_RX_INT_EN | UART_TX_INT_EN);
    while (BusyUART1());
}

void initializePortsIO() {
    //R0 -- Rx D0 -> settato da config di UART
    //DI -- Tx D1 -> settato da config di UART
    TRISGbits.TRISG6 = 1; //D13 bottone
    TRISDbits.TRISD7 = 0; //D5 LED
    TRISDbits.TRISD8 = 0; //D6 RE  Read Enable
    TRISDbits.TRISD11 = 0; //D7 DE  Send Enable
    /*
     RE: attivazione uscita del ricevitore (attivazione a livello basso)
     DE: attivazione uscita Driver (attivazione a livello alto)
     */
}

//controllo se un bottone è stato premuto

char CheckButton(unsigned port) {
    int temp = 0;
    newButtonState = !port;
    if (oldButtonState > newButtonState) {
        temp = 1;
    }
    oldButtonState = !port;
    return temp;
}