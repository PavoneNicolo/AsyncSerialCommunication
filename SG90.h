#define SYSCLK 80000000L

#define SERVO_PIN LATDbits.LATD8

void initServo();
void goTo(int target);
