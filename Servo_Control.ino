//i2c Servo-LED driver board pins
#define Throttle      0
#define FrontSteering 1
#define Red_LED       2
#define Green_LED     3
#define Blue_LED      4
//////////////////////////////Timing////
unsigned long interval = 300;
unsigned long previousMillis = 0;

int FrontCenter = 1474;  // Using writeMicroseconds()
int SoftRight =  1224;    //center - 250
int SoftLeft =   1712;    //center + 238
int HardRight = 974;
int HardLeft =  1950;
int ThrottleCenter = 1446;
int ThrottleSpeed = 1750;
int SlowForward = 1600;
int FastForward = 1966;
int SlowReverse = 1300;
int FastReverse = 956;
//////////////////////I2C LED CONTROL/////////////////
int Red_State = 4096;
int Red_State2 = 0;
int Green_State = 4096;
int Green_State2 = 0;
int Blue_State = 4096;
int Blue_State = 0;

//////////////////////////function to send I2C data/////////////
void SendData(byte instruction)                 
{
  if (currentMillis - previousMillis > interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    // if the LED is off turn it on and vice-versa:
    if (Red_State == 4096) //LOW
      Red_State = 0;  //HIGH
    Red_State2 = 4096;
    else
      Red_State = 4096; //LOW
    Red_State2 = 0;

    if (Green_State == 4096) //LOW
      Green_State = 0;  //HIGH
    Green_State2 = 4096;
    else
      Green_State = 4096;  //LOW
    Green_State2 = 0;

    if (Blue_State == 4096)  //LOW
      Blue_State = 0; //HIGH
    Blue_State2 = 4096;
    else
      Blue_State = 4096;  //LOW
    Blue_State2 = 0;
  }
  // Serial.println(instruction);
  switch (instruction) {
    case 0:
      pwm.setPWM(Throttle, 0, ThrottleCenter);             //    pwm.setPWM(servonum, 0, pulselen);
      pwm.setPWM(FrontSteering, 0, FrontCenter);
      pwm.setPWM(Red_LED, Red_State, Red_State2);     //setPWM(channel, on time, off time) 0 to 4096
      pwm.setPWM(Green_LED, 4096, 0);               //LEDs totally off use setPWM(pin, 4096, 0);
      pwm.setPWM(Blue_LED, 4096, 0);                //LEDs totally off use setPWM(pin, 4096, 0);
      break;
    case 11:   //hard left
      pwm.setPWM(Throttle, 0, SlowForward);
      pwm.setPWM(FrontSteering, 0, HardLeft);
      pwm.setPWM(Red_LED, 4096, 0);               //LEDs totally off use setPWM(pin, 4096, 0);
      pwm.setPWM(Green_LED, 4096, 0);               //LEDs totally off use setPWM(pin, 4096, 0);
      pwm.setPWM(Blue_LED, Blue_State, Blue_State2);
      break;
    case 12:   //hard right
      pwm.setPWM(Throttle, 0, SlowForward);
      pwm.setPWM(FrontSteering, 0, HardRight);
      pwm.setPWM(Red_LED, 4096, 0);               //LEDs totally off use setPWM(pin, 4096, 0);
      pwm.setPWM(Green_LED, 4096, 0);               //LEDs totally off use setPWM(pin, 4096, 0);
      pwm.setPWM(Blue_LED, Blue_State, Blue_State2);
      break;
    case 20:   //fast straight
      pwm.setPWM(Throttle, 0, ThrottleSpeed);
      pwm.setPWM(FrontSteering, 0, FrontCenter);
      pwm.setPWM(Red_LED, Red_State);
      pwm.setPWM(Green_LED, 4096, 0);               //LEDs totally off use setPWM(pin, 4096, 0);
      pwm.setPWM(Blue_LED, Red_State, Red_State2);
      break;
    case 21:   //fast left
      pwm.setPWM(Throttle, 0, ThrottleSpeed);
      pwm.setPWM(FrontSteering, 0, SoftLeft);
      pwm.setPWM(Red_LED, 4096, 0);               //LEDs totally off use setPWM(pin, 4096, 0);
      pwm.setPWM(Green_LED, Green_State, Green_State2);
      pwm.setPWM(Blue_LED, 4096, 0);                //LEDs totally off use setPWM(pin, 4096, 0);
      break;
    case 22:   //fast right
      pwm.setPWM(Throttle, 0, ThrottleSpeed);
      pwm.setPWM(FrontSteering, 0, SoftRight);
      pwm.setPWM(Red_LED, 4096, 0);               //LEDs totally off use setPWM(pin, 4096, 0);
      pwm.setPWM(Green_LED, Green_State, Green_State2);
      pwm.setPWM(Blue_LED, 4096, 0);                //LEDs totally off use setPWM(pin, 4096, 0);
      break;

    case 30:    // your hand is close to the sensor
      // Serial.println("Reverse");
      pwm.setPWM(Throttle, 0, SlowReverse);
      pwm.setPWM(FrontSteering, 0, SoftRight);
      break;
   }
  )

