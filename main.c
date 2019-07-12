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

// LIBRARIES
#include <p32xxxx.h>
#include <plib.h> // Include the PIC32 Peripheral Library.
#include "DHT22.h"

//STATES
#define StatePOR 0x00
#define StatePAIRING 0x05
#define StateIDLE 0x10
#define StateRECEIVE 0x20
#define StateREAD 0x21
#define StateDISCARD 0x22
#define StatePROCESS 0x25
#define StateSEND 0x30
#define StateEMPTYTX 0x35

#define CmdCOLLECT 0x01
#define CmdCOMMAND 0x02

#define DvcTEMP1_ID 0x01
#define DvcTEMP1_PIN PORTBbits.RB1
#define DvcLED1_ID 0x02
#define DvcLED1_PIN LATDbits.LATD7

#define SYSCLK 80000000L
#define DESIRED_BAUDRATE 9600
#define DE LATDbits.LATD11
#define PAIRBTN PORTDbits.RD0
#define PAIRLED LATDbits.LATD1

#define bitN(arg, n) (((arg) >> (n)) & 1)

//FUNCTIONS
void initializeUART();
void initializePortsIO();
char CheckButton(unsigned port, int oldBtnIndex);
void SerialSend(char body[], int length);
int getDeviceData(char deviceID);

//typedef struct {
//    char deviceID;
//    char deviceType;
//    int PortAddress;
//} Device;

int oldButtonStates[2] = {1, 1};
char state = StatePOR;
char sendRdy = 1;
char receiveRdy = 0;
char emptyTx = 0;
char result = 0x00;
char address = 0x00;
char matchedAddress = 0;
int msgDuration = 0;
char i = 0;
char action;

//const Device Led1 = {.deviceID = 0x01, .deviceType = 0x01, .PortAddress = 7 , .myPort = &PORTD};
//const Device Temp1 = {.deviceID = 0x02, .deviceType = 0x02, .PortAddress = 9 , .myPort = &LATD};

//    //TODO da fare struct per device *(Pavo ? un pavone e non vuole le struct)
//    Device deviceList[2] = {Led1, Temp1}; // 0x01 = deviceID , 0x14 = deviceType(LED)

int main(void) {
    short temperature = 0;
    unsigned int humidity = 0;
    initializePortsIO();
    initializeUART();
    // Must enable glocal interrupts - in this case, we are using multi-vector mode
    INTEnableSystemMultiVectoredInt();
    int temp;
    char *msgBody;
    char collectResponse[3] = {0x00, 0x00, 0x00};
    char deviceList[4] = {DvcTEMP1_ID, 0x02, DvcLED1_ID, 0x01};
    int d4Pressed;
    int d2Pressed;
    LATDbits.LATD7 = 0;
    DE = 0;
    PAIRLED = 0;
    int currentMsgDuration = 0;
    int size = 0;
    initDHT22();
    
    while (1) {

        d4Pressed = CheckButton(PORTDbits.RD6, 0);
        d2Pressed = CheckButton(PAIRBTN, 1);
        
        //temperature = readTemperature();
        humidity = readHumidity();

        printf("ciao");
        
        switch (state) {
            case StatePOR:
                //stato di primo boot :(address = 0;)
                //non ? nel canale degli orfani, aspetto il BUT2 per mettermi in quel canale
                if (d2Pressed) {
                    PAIRLED = 1;
                    address = 0xFF;
                    state = StatePAIRING;
                    receiveRdy = 0;
                }
                break;
            case StatePAIRING:
                if (receiveRdy) {
                    if (result == address) {
                        DE = 1;
                        putcUART1(0xFF);
                        while (BusyUART1())
                            ;
                        DE = 0;
                    } else {
                        address = result;
                        state = StateIDLE;
                        PAIRLED = 0;
                        int length = sizeof (deviceList) / sizeof (char);
                        SerialSend(deviceList, length);
                    }
                    receiveRdy = 0;
                }
                break;
            case StateIDLE:
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
                    if (msgDuration > 0) {
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
                msgBody = (char *) malloc(msgDuration * sizeof (char));
                size = sizeof (msgBody) / sizeof (char);
                //L'errore è qua
                while (msgDuration != 0) {
                    if (receiveRdy) {
                        //praticamente dato che passa prima nell'interrupt msgDuration è stato già decrementato e quindi si sfancula
                        msgBody[currentMsgDuration - msgDuration - 1] = result;
                        receiveRdy = 0;
                    }
                }
                state = StatePROCESS;
                break;
            case StateDISCARD:
                if (msgDuration == 0) {
                    state = StateIDLE;
                    matchedAddress = 0;
                }
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
                        collectResponse[1] = (char) (temp & 0xFF);
                        collectResponse[2] = (char) (temp >> 8);

                        break;
                    case CmdCOMMAND:
                        for (i = 0; i < sizeof (msgBody) - 1; i++) {
                            //TODO
                            // usare questi DeviceID per fare cose
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
                state = StateIDLE; // torno in IDLE solo quando finisco di leggere i dati
                if (result == address) {
                    matchedAddress = 1;
                } else {
                    matchedAddress = 0;
                }
                while (!receiveRdy)
                    ;
                msgDuration = result;
                currentMsgDuration = msgDuration;
                break;
            case StateEMPTYTX: //TODO fulminare questo state in quanto sar? incluso in un metodo di send futuro
                while (BusyUART1())
                    ;
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
        result = (char) ReadUART1();
        receiveRdy = 1;
        msgDuration--;
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
    TRISDbits.TRISD6 = 0; //D4 sens. temp
    TRISDbits.TRISD1 = 0; //LED2
    TRISDbits.TRISD0 = 1; //D2 BUT del pinguino
    TRISDbits.TRISD4 = 1; //D2 BUT del pinguino
    TRISDbits.TRISD7 = 0; //D5 LED
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
            result = 0x2222;
            break;
    }
    return result;
}
