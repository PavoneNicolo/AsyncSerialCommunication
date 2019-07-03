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
#define StatePAIRING 0x05
#define StateIDLE 0x10
#define StateSEND 0x20
#define StateRECEIVE 0x30
#define StateFETCH 0x035
#define StateIGNORE 0x038
#define StateEMPTYTX 0x40
#define SYSCLK 80000000L
#define DESIRED_BAUDRATE 9600
#define DE  LATDbits.LATD11
#define PAIRLED LATDbits.LATD1
#define PAIRBTN PORTDbits.RD0

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
char CheckButton(unsigned port, int oldBtnIndex);

int oldButtonStates [2] = {1, 1};
char state = StatePOR;
char sendRdy = 1;
char receiveRdy = 0;
char emptyTx = 0;
char data;
char skipData = 0;
char str[5] = {0xFF, 0x02, 0x02, 0x01, 0x00};

char address = 0x00;

int main(void) {
    /* per usarlo bisogna settare la heap memory nelle proprietà del progetto alla sezione linker -> xc23
    int *buffer;
    buffer = (int *) malloc(10 * sizeof (int));
    */
    initializePortsIO();
    initializeUART();
    // Must enable glocal interrupts - in this case, we are using multi-vector mode
    INTEnableSystemMultiVectoredInt();

    char d4Pressed;
    char d2Pressed;
    LATDbits.LATD7 = 0;
    PAIRLED = 0;
    DE = 0;

    while (1) {

        d4Pressed = CheckButton(PORTDbits.RD6, 0);
        d2Pressed = CheckButton(PORTDbits.RD4, 1);

        switch (state) {
            case StatePOR:
                if (d2Pressed) {
                    // mettere un timeout per farlo uscire dal pairing
                    receiveRdy = 0;
                    state = StatePAIRING;
                    address = 0xFF;
                    PAIRLED = 1;
                }
                break;
            case StatePAIRING:
                if (receiveRdy) {
                    if (data == address) {
                        DE = 1;
                        putcUART1(0xFF);
                        while (BusyUART1());
                        DE = 0;
                    } else {
                        address = data;
                        state = StateIDLE;
                        DE = 1;
                        putsUART1(str);
                        while (BusyUART1());
                        DE = 0;
                    }
                }
                receiveRdy = 0;
                break;
            case StateIDLE:
                PAIRLED = 0;
                // azzerare skipData se arriva 255/0XFF
                if (emptyTx) {
                    //sendRdy = 0;
                    state = StateEMPTYTX;
                }

                if (d4Pressed && sendRdy) {
                    DE = 1;
                    state = StateSEND;
                }

                if (receiveRdy) {
                    state = StateRECEIVE;
                }

                break;
            case StateSEND:
                state = StateIDLE;
                sendRdy = 0;
                putcUART1('B'); // Transmit 'A' through UART
                //putsUART1(str);
                break;
            case StateRECEIVE:
                if (data == address) {
                    state = StateFETCH; //leggo i prossimi 2 byte
                    receiveRdy = 0;
                }

                receiveRdy = 0;
                state = StateIDLE; // torno in IDLE solo quando finisco di leggere i dati
                if (data == 'A') {
                    LATDbits.LATD7 = LATDbits.LATD7 ^ 1;
                }
                break;
                //case StateFETCH:
                //skipData = lunghezza duration messaggio
                //se devo scartare vado in StateIGNORE altrimenti vado in StateREAD
                //break;
            case StateIGNORE:
                if (skipData == 0) {
                    state = StateIDLE;
                }
                break;
                //verrà sostituito dalla funzione che aspetterà che si svuoti il Tx
            case StateEMPTYTX:
                while (BusyUART1());
                DE = 0;
                emptyTx = 0;
                sendRdy = 1;
                state = StateIDLE;
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
        // Read data from Rx
        data = (char) ReadUART1();
        receiveRdy = 1;
        skipData--;
    }

    //chiama l'interrupt quando ha finito di trasmettere
    if (mU1TXGetIntFlag()) {
        mU1TXClearIntFlag();
        //sendRdy = 1;
        emptyTx = 1;
    }
}

void initializeUART() {
    // Optimize PIC32 performance and return peripheral bus frequency
    unsigned int pbClk = SYSTEMConfig(SYSCLK, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);
    // Abilita UART1 and set baud rate to DESIRED_BAUDRATE=9600
    OpenUART1(UART_EN | UART_ODD_PAR_8BIT, UART_RX_ENABLE | UART_TX_ENABLE, pbClk / 16 / DESIRED_BAUDRATE - 1);
    // Configure UART1 RX Interrupt
    ConfigIntUART1(UART_INT_PR2 | UART_RX_INT_EN | UART_TX_INT_EN);
    while (BusyUART1());
}

void initializePortsIO() {
    //R0 -- Rx D0 -> settato da config di UART
    //DI -- Tx D1 -> settato da config di UART
    TRISDbits.TRISD1 = 0; // LED2 PAIRLED
    TRISDbits.TRISD0 = 1; // D2 BTN
    TRISDbits.TRISD4 = 1; // D2 BTN
    TRISGbits.TRISG6 = 1; //D13 bottone
    TRISDbits.TRISD6 = 1; //D4 bottone
    TRISDbits.TRISD7 = 0; //D5 LED
    TRISDbits.TRISD11 = 0; //D7 DE  Send/Receive Enable
}


//controllo se un bottone è stato premuto

char CheckButton(unsigned port, int oldBtnIndex) {
    char temp = 0;
    if (oldButtonStates[oldBtnIndex] & !port) {
        temp = 1;
    }
    oldButtonStates[oldBtnIndex] = port;
    return temp;
}