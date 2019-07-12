#include <p32xxxx.h>
#include <plib.h> // Include the PIC32 Peripheral Library.
#include "SG90.h"

int count = 0;
int pwm = 0;
int angle = 0;
int targetAngle = 0;
int increment = 0;

void initServo(){
    OpenTimer4(T4_ON | T4_SOURCE_INT | T4_PS_1_16, 0x0018); // 5 micro //  con 5 micro riesco a dividere 180 gradi in 200 step
    OpenTimer3(T3_ON | T3_SOURCE_INT | T3_PS_1_32, 0xC34F); // 20 milli
    INTEnableSystemMultiVectoredInt();
    ConfigIntTimer3(T3_INT_ON | T3_INT_PRIOR_3); //timer3 sempre acceso 
    TRISDbits.TRISD8 = 0;
}
void goTo(int target){
    targetAngle=target;
}
long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
void __ISR(_TIMER_4_VECTOR, ipl4) handlesTimer4Ints(void) {
    // **make sure iplx matches the timer?s interrupt priority level
    //tengo alto motore per 1000 micro (200 count) , 
    // per i prossimi 200 si deve fare un operazione
    count++;
    if (count <= (110 + pwm)) {
        SERVO_PIN = 1;
    } else {
        SERVO_PIN = 0;
        count = 0;
        ConfigIntTimer4(T4_INT_OFF | T2_INT_PRIOR_4);
    }    
    mT4ClearIntFlag();
    // Clears the interrupt flag so that the program returns to the main loop
} // END Timer2 ISR

void __ISR(_TIMER_3_VECTOR, ipl3) handlesTimer3Ints(void) {
    // **make sure iplx matches the timer?s interrupt priority level
    if(angle == targetAngle){
        increment = 0;
    }else{
        if(angle>targetAngle){
            increment = -1;
        }else{
            increment = 1;
        }
    }
    angle+=increment;
    pwm = map(angle, 0, 180, 0, 340);
    ConfigIntTimer4(T4_INT_ON | T4_INT_PRIOR_4);
    mT3ClearIntFlag();
    // Clears the interrupt flag so that the program returns to the main loop
}

