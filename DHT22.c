#include "DHT22.h"

// DHT22 sensor connection (here sensor data pin is connected to pin RB0)
#define DHT22_PIN_OUTPUT LATDbits.LATD6
#define DHT22_PIN_INPUT PORTDbits.RD6
#define DHT22_PIN_STATE TRISDbits.TRISD6

#define SYSCLK 80000000L

int count = 0;
int sigLength = 0;
int iterator = 0;
char retry = 0;
char data[5];
int countBit = 0;

void startSignal();
void readBit(int index);
void getSigLength();
void delay_us(long us);

void initDHT22() {
    DHT22_PIN_STATE = 0;
    DHT22_PIN_OUTPUT = 1;
    SYSTEMConfigPerformance(SYSCLK);
    INTEnableSystemMultiVectoredInt();
    OpenTimer3(T3_ON | T3_SOURCE_INT | T3_PS_1_16, 0x0004);
}

short readTemperature() {
    short temperature = 0;

    data[0] = data[1] = data[2] = data[3] = data[4] = 0;

    startSignal();

    getSigLength();
    while (DHT22_PIN_INPUT == 1); // aspetto che il DHT risponda allo start signal
    ConfigIntTimer3(T3_INT_OFF | T3_INT_PRIOR_3);
    if (sigLength < 100) { // DHT22 porta da alto a basso per 80us

        getSigLength();
        while (DHT22_PIN_INPUT == 0);
        ConfigIntTimer3(T3_INT_OFF | T3_INT_PRIOR_3);

        if (sigLength < 100) { // DHT22 porta da basso ad alto per 80us
            //Start procedure andata a buon fine
            for (iterator = 0; iterator < 40; iterator++) { // il DHT22 manda 40 bit di dati + checksum
                readBit(iterator);
            }
        }
    }

    // DA CONTROLLARE IL CHECKSUM SE E' ESATTO
    //temperature = ((((data[2] & 0x7F)) << 8) | data[3])*0.1;
    //humidity = ((data[0] << 8) | data[1])*0.1;

    // controllo il segno della temperatura (1 negativo; 0 positivo)
    /*if (data[2] & 0x80) {
        temperature *= -1;
    }*/

    // controllo checksum
    /*
    if ((data[0] + data[1] + data[2] + data[3]) == data[4]) {
        temperature = (data[2] << 8) | data[3];
        retry = 0;
        return temperature;
    }*/
    
    temperature = (data[2] << 8) | data[3];

    return temperature;
    
    
    
    
    //se il checksum non corrisponde riprovo per almeno 10 tentativi
    if (retry > 10) {
        return 0;
    }

    retry++;
    return readTemperature();
}

short readHumidity() {
    short hum = 0;

    data[0] = data[1] = data[2] = data[3] = data[4] = 0;

    startSignal();

    getSigLength();
    while (DHT22_PIN_INPUT == 1); // aspetto che il DHT risponda allo start signal
    ConfigIntTimer3(T3_INT_OFF | T3_INT_PRIOR_3);
    if (sigLength < 100) { // DHT22 porta da alto a basso per 80us

        getSigLength();
        while (DHT22_PIN_INPUT == 0);
        ConfigIntTimer3(T3_INT_OFF | T3_INT_PRIOR_3);

        if (sigLength < 100) { // DHT22 porta da basso ad alto per 80us
            //Start procedure andata a buon fine
            for (iterator = 0; iterator < 40; iterator++) { // il DHT22 manda 40 bit di dati + checksum
                readBit(iterator);
            }
        }
    }

    // DA CONTROLLARE IL CHECKSUM SE E' ESATTO
    //temperature = ((((data[2] & 0x7F)) << 8) | data[3])*0.1;
    //humidity = ((data[0] << 8) | data[1])*0.1;

    // controllo il segno della temperatura (1 negativo; 0 positivo)
    /*if (data[2] & 0x80) {
        temperature *= -1;
    }*/

    // controllo checksum
    /*
    if ((data[0] + data[1] + data[2] + data[3]) == data[4]) {
        humidity = (data[0] << 8) | data[1];
        retry = 0;
        return humidity;
    }*/
    
    hum = ((data[0] << 8) | (data[1] & 0xFF)) & 0xFFFF;
    
    return hum;
    
    
    
    
    //se il checksum non corrisponde riprovo per almeno 10 tentativi
    if (retry > 10) {
        return 0;
    }

    retry++;
    return readHumidity();
}

void startSignal() {
    DHT22_PIN_STATE = 0;
    DHT22_PIN_OUTPUT = 0;
    delay_us(1000);
    DHT22_PIN_OUTPUT = 1;
    delay_us(40);
    DHT22_PIN_STATE = 1;
}

char j = 7; // PROVVISORIA -> DA ELIMINARE

void readBit(int index) {

    while (DHT22_PIN_INPUT == 1); // aspetto che il DHT22 porti da alto a basso

    getSigLength();
    while (DHT22_PIN_INPUT == 0);
    ConfigIntTimer3(T3_INT_OFF | T3_INT_PRIOR_3);

    if (sigLength < 100) { // 50us basso (prima di ogni bit)
        getSigLength();
        while (DHT22_PIN_INPUT == 1); // riporta a 1 per mandare il bit
        ConfigIntTimer3(T3_INT_OFF | T3_INT_PRIOR_3);
        if (sigLength > 50) { // 70us = bit a 1; 26-28us = bit a 0; parte dal pi� significativo
            data[index / 8] = data[index / 8] | 1 << j;
        }

        if (j == 0) {
            j = 8;
        }
        j--;
    }
    countBit++;
}

void getSigLength() {
    sigLength = 0;
    ConfigIntTimer3(T3_INT_ON | T3_INT_PRIOR_3);
}

void delay_us(long us) {
    count = 0;
    ConfigIntTimer3(T3_INT_ON | T3_INT_PRIOR_3);
    while (count < us);
    ConfigIntTimer3(T3_INT_OFF | T3_INT_PRIOR_3);
}

void __ISR(_TIMER_3_VECTOR, ipl3) handlesTimer3Ints(void) {
    count++;
    sigLength++;
    mT3ClearIntFlag();
}