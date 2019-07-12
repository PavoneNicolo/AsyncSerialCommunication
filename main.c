// DEVCFG2
#pragma config FPLLIDIV = DIV_2 // PLL Input Divider (1x Divider)
#pragma config FPLLMUL = MUL_20 // PLL Multiplier (24x Multiplier)
#pragma config UPLLIDIV = DIV_2 // USB PLL Input Divider (12x Divider)
#pragma config UPLLEN = OFF     // USB PLL Enable (Disabled and Bypassed)
#pragma config FPLLODIV = DIV_1 // System PLL Output Clock Divider (PLL Divide by 256)
// DEVCFG1
#pragma config FNOSC = PRIPLL    // Oscillator Selection Bits (Primary Osc w/PLL (XT+,HS+,EC+PLL))
#pragma config FSOSCEN = ON      // Secondary Oscillator Enable (Enabled)
#pragma config IESO = ON         // Internal/External Switch Over (Enabled)
#pragma config POSCMOD = HS      // Primary Oscillator Configuration (HS osc mode)
#pragma config OSCIOFNC = ON     // CLKO Output Signal Active on the OSCO Pin (Enabled)
#pragma config FPBDIV = DIV_8    // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/8)
#pragma config FCKSM = CSDCMD    // Clock Switching and Monitor Selection (Clock Switch Disable, FSCM Disabled)
#pragma config WDTPS = PS1048576 // Watchdog Timer Postscaler (1:1048576)
#pragma config FWDTEN = OFF      // Watchdog Timer Enable (WDT Disabled (SWDTEN Bit Controls))
// DEVCFG0
#pragma config DEBUG = OFF       // Background Debugger Enable (Debugger is disabled)
#pragma config ICESEL = ICS_PGx2 // ICE/ICD Comm Channel Select (ICE EMUC2/EMUD2 pins shared with PGC2/PGD2)
#pragma config PWP = OFF         // Program Flash Write Protect (Disable)
#pragma config BWP = OFF         // Boot Flash Write Protect bit (Protection Disabled)
#pragma config CP = OFF          // Code Protect (Protection Disabled)


#include <p32xxxx.h>
#include <plib.h> // Include the PIC32 Peripheral Library.
#include "DHT22.h"

//STATES
#define StatePOR 0x00
#define StatePAIRING 0x05
#define StatePAIRDISCARD 0x06
#define StateIDLE 0x10
#define StateRECEIVE 0x20
#define StateREAD 0x21
#define StateDISCARD 0x22
#define StatePROCESS 0x25
#define StateSEND 0x30
#define StateEMPTYTX 0x35

#define PAIRValue 0xFF

#define CmdCOLLECT 0x01
#define CmdCOMMAND 0x02

#define DvcTEMP1_ID 0x01
#define DvcTEMP1_PIN PORTBbits.RB1
#define DvcLED1_ID 0x02
#define DvcLED1_PIN LATDbits.LATD7
#define DvcHUM1_ID 0x03
#define DvcHUM1_PIN PORTBbits.RB1
#define DvcLED2_ID 0x04
#define DvcLED2_PIN LATBbits.LATB14


#define SYSCLK 80000000L
#define DESIRED_BAUDRATE 9600
#define DE LATDbits.LATD11
#define PAIRBTN PORTDbits.RD0
#define PAIRLED LATDbits.LATD1

#define bitN(arg, n) (((arg) >> (n)) & 1)

//FUNCTIONS
void StartTimeout();
void initializeUART();
void initializePortsIO();
char CheckButton(unsigned port, int oldBtnIndex);
void SerialSend(char body[], int length);
int getDeviceData(char deviceID);

int oldButtonStates[2] = {1, 1};
char state = StatePOR;
char sendRdy = 1;
char receiveRdy = 0;
char emptyTx = 0;
char data = 0x00;
char address = 0x00;
char matchedAddress = 0;
int dataLengthCounter = 0;
char i = 0;
char action;
char timeoutCount = 0;
char timeoutFlag = 0;

void delay(int t) {
    int n = t * 1900;
    while (n > 0) {
        n--;
    }
}

int main(void) {
    SYSTEMConfigPerformance(SYSCLK);
    initializePortsIO();
    initializeUART();
    initDHT22();
    // Must enable glocal interrupts - in this case, we are using multi-vector mode
    INTEnableSystemMultiVectoredInt();
    OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_256, 0xf423); // Timeout 20s (con count a 100)

    int temp;
    char *msgBody;
    char discardPair[2] = {0x00, 0x00};
    char collectResponse[3] = {0x01, 0x01, 0x01};
    char deviceList[4] = {DvcTEMP1_ID, 0x02, DvcLED1_ID, 0x01};
    int d2Pressed;
    unsigned char cmdID = 0;
    unsigned char cmdData = 0;
    LATDbits.LATD7 = 0;
    DE = 0;
    PAIRLED = 0;
    int dataLength = 0;
    while (1) {

        //        d4Pressed = CheckButton(PORTDbits.RD6, 0);
        d2Pressed = CheckButton(PAIRBTN,oldButtonStates[0]);

        switch (state) {
            case StatePOR:
                //stato di primo boot :(address = 0;)
                //non ? nel canale degli orfani, aspetto il BUT2 per mettermi in quel canale
                PAIRLED = 0;
                if (d2Pressed) {
                    PAIRLED = 1;
                    address = 0xFF;
                    state = StatePAIRING;
                    receiveRdy = 0;
                    StartTimeout();
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
                        PAIRLED = 0;
                        int length = sizeof (deviceList) / sizeof (char);
                        SerialSend(deviceList, length);
                    }
                    receiveRdy = 0;
                }
                if (timeoutFlag) {
                    state = StatePOR;
                }
                break;
            case StateIDLE:
                //                if (emptyTx) {
                //                    //sendRdy = 0;
                //                    state = StateEMPTYTX;
                //                }
                //                if (d4Pressed && sendRdy) {
                //                    DE = 1;
                //                    state = StateSEND;
                //                } 
                if (receiveRdy) {
                    state = StateRECEIVE;
                    if (dataLengthCounter > 0) {
                        if (matchedAddress) {
                            state = StateREAD;
                        } else {
                            state = StateDISCARD;
                        }
                    }
                    receiveRdy = 0;
                }
                break;
            case StateREAD:
                //malloc richiede memoria nell'heap, assegnata nelle propriet? del progetto
                msgBody = (char *) malloc(dataLengthCounter * sizeof (char));
                while (dataLengthCounter != 0) {
                    if (receiveRdy) {
                        //praticamente dato che passa prima nell'interrupt msgDuration è stato già decrementato e quindi si sfancula
                        msgBody[dataLength - dataLengthCounter] = data;
                        receiveRdy = 0;
                        dataLengthCounter--;
                    }
                }
                state = StatePROCESS;
                break;
            case StateDISCARD:
                while (dataLengthCounter != 0) {
                    if (receiveRdy) {
                        //praticamente dato che passa prima nell'interrupt msgDuration è stato già decrementato e quindi si sfancula
                        //                        msgBody[size - msgDuration] = data;
                        receiveRdy = 0;
                        dataLengthCounter--;
                    }
                }
                state = StateIDLE;
                //                if (msgDuration == 0) {
                //                    state = StateIDLE;
                //                    matchedAddress = 0;
                //                }
                break;
            case StatePROCESS:
                action = msgBody[0];
                switch (action) {
                    case CmdCOLLECT:
                        //ciclo molteplici device che sono richiesti nel body
                        //                        for (i = 0; i< sizeof (msgBody) - 1; i++) {
                        //                            //TODO
                        ////                            getDeviceData(msgBody[i]);
                        //                            // usare questi DeviceID per fare cose
                        //                        }
                        //ciclo un device singolo
                        temp = getDeviceData(msgBody[1]);
                        collectResponse[0] = 0x01;
                        collectResponse[1] = (char) (temp >> 8);
                        collectResponse[2] = (char) (temp & 0xFF);
                        break;
                    case CmdCOMMAND:
                        for (i = 1; i < sizeof (msgBody) - 1; i++) {
                            if (i == 1) { // è un device ID
                                cmdID = msgBody[i];
                            } else { // è data
                                cmdData = msgBody[i];
                            }
                        }
                        switch (cmdID) {
                            case DvcLED1_ID:
                                if (cmdData == 0x00) {
                                    DvcLED1_PIN = 0;
                                } else {
                                    DvcLED1_PIN = 1;
                                }
                                break;
                            case DvcLED2_ID:
                                if (cmdData == 0x00) {
                                    DvcLED2_PIN = 0;
                                } else {
                                    DvcLED2_PIN = 1;
                                }
                                break;
                        }
                        break;
                    default:
                        //TODO come ultima cosa magari: Controllo errori
                        break;
                }
                state = StateSEND;
                break;
            case StateSEND:
                state = StateIDLE;
                sendRdy = 0;
                int length = sizeof (collectResponse) / sizeof (char);
                SerialSend(collectResponse, length);
                break;
            case StateRECEIVE:
                receiveRdy = 0;
                matchedAddress = 0;
                if (data == 0xff) {
                    state = StatePAIRDISCARD;
                } else {
                    if (data == address) {
                        matchedAddress = 1;
                    }
                    state = StateIDLE;
                    while (!receiveRdy);
                    dataLengthCounter = data;
                    dataLength = dataLengthCounter;
                }
                break;
            case StatePAIRDISCARD:
                StartTimeout();
                if (timeoutFlag) {
                    state = StateIDLE;
                }
                if (receiveRdy) {
                    receiveRdy = 0;
                    if (data != 0xFF) {
                        state = StateIDLE;
                    }
                }
                break;
                //            case StateEMPTYTX: //TODO fulminare questo state in quanto sar? incluso in un metodo di send futuro
                //                while (BusyUART1())
                //                    ;
                //                DE = 0;
                //                emptyTx = 0;
                //                sendRdy = 1;
                //                state = StateIDLE;
                //                break;
            default:
                break;
        }
    }
    return 1;
}

void StartTimeout() {
    timeoutFlag = 0;
    timeoutCount = 0;
    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);
}

void __ISR(_UART1_VECTOR, ipl2) IntUart1Handler(void) {
    // Is this an RX interrupt?
    if (mU1RXGetIntFlag()) {
        // Clear the RX interrupt Flag
        mU1RXClearIntFlag();
        // Read data from Rx
        data = 0;
        data = (char) ReadUART1();
        receiveRdy = 1;
    }
    //chiama l'interrupt quando ha finito di trasmettere
    if (mU1TXGetIntFlag()) {
        mU1TXClearIntFlag();
        //sendRdy = 1;
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
    TRISDbits.TRISD6 = 0; //D4 tempsens
    TRISDbits.TRISD1 = 0; //LED2
    TRISDbits.TRISD0 = 1; //D2 BUT del pinguino
    TRISDbits.TRISD4 = 1; //D2 BUT del pinguino
    TRISDbits.TRISD7 = 0; //D5 DevLED1
    TRISBbits.TRISB14 = 0; //D9 DevLED2
    TRISDbits.TRISD11 = 0; //D7 DE  Send/Receive Enable
}

//controllo se un bottone ? stato premuto

char CheckButton(unsigned port, int oldBtnIndex) {
    char temp = 0;
    if (oldButtonStates[oldBtnIndex] & !port) {
        temp = 1;
    }
    oldButtonStates[oldBtnIndex] = port;
    return temp;
}

void SerialSend(char body[], int length) { //TODO aggiungere checksum??
    char message[length + 3];
    int i = 0;
    message[0] = address;
    message[1] = length;
    for (i = 0; i < length; i++) {
        message[i + 2] = body[i];
    }
    message[length + 2] = 0x00;
    DE = 1;
    putsUART1(message);
    while (BusyUART1());
    DE = 0;
}

int getDeviceData(char deviceID) {
    int result;
    switch (deviceID) {
        case DvcLED1_ID:
            //TODO rimpiazzare con adc di un motore o cose simili
            result = 0x1111;
            break;
        case DvcTEMP1_ID:
            //da mandare i 2 byte della temp
            result = readTemperature();
            break;
        case DvcHUM1_ID:
            //da mandare i 2 byte dell'hum
            result = readHumidity();
            break;
        default:
            result = 0xFFFF;
            break;
    }
    return result;
}

void __ISR(_TIMER_2_VECTOR, ipl2) handlesTimer2Ints(void) {
    // **make sure iplx matches the timer?s interrupt priority level
    timeoutCount++;
    if (timeoutCount == 100) {
        timeoutFlag = 1;
        ConfigIntTimer2(T2_INT_OFF | T2_INT_PRIOR_2);
    }
    mT2ClearIntFlag();
    // Clears the interrupt flag so that the program returns to the main loop
} // END Timer2 ISR
