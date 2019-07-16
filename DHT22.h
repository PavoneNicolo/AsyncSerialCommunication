#include <p32xxxx.h>
#include <plib.h> // Include the PIC32 Peripheral Library.
//void initDHT22();
typedef struct DHT22{
    IoPortId portId;
    unsigned int pin;
}DHT22;

short readTemperature(DHT22 DHT);
short readHumidity(DHT22 DHT);


