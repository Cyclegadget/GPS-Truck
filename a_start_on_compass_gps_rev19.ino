/*
Rev 7 I changed to the AdaFruit IMU with tilt compensation
 Magnetic declination: -2° 0' WEST
 Declination is NEGATIVE
 GPS wires... green = gnd, red = +5, blue = TX, white = RX
 Compass wires... orange = gnd, White = +5, red to A5, Brown to A4
 GPS on 0, 1  RX1, TX1
 Compass I2C A4,A5 SDA, SDL
 NOKIA LCD PINS 5,6,7,8,9
 Master I2C A4,A5 SDA, SDL
 BUTTONs ARE ON PIN RESET,2,3,4  Brown,Orange,White, Yel
 LEDs are on pins
 potpin = A0 white
 Servos are on I2C board 0,1 and LEDs on 2,3,4
 */

#include <TinyGPS++.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x43);
#include <SPI.h>                //SOFTWARE SPI
#include <Adafruit_GFX.h>       //NOKIA LIBRARY
#include <Adafruit_PCD8544.h>   //NOKIA LIBRARY

#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_9DOF.h>
#include <Adafruit_L3GD20_U.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>

/* Assign a unique ID to the sensors */
Adafruit_9DOF                 dof   = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);

///////////////LCD Settings and Pins/////////////
// Software SPI (slower updates, more flexible pin options):
// pin 9 - Serial clock out (SCLK)
// pin 8 - Serial data out (DIN)
// pin 7 - Data/Command select (D/C)
// pin 6 - LCD chip select (CS)            LCD IS 6*14
// pin 5 - LCD reset (RST)
Adafruit_PCD8544 display = Adafruit_PCD8544(9, 8, 7, 6, 5);

// Waypoints
const double First_LAT =     38.896713;
const double First_LNG =    -89.438370;   // one
const double Second_LAT =     38.897108;
const double Second_LNG =   -89.438749;   //two
const double Third_LAT =     38.897097;
const double Third_LNG =   -89.439131;   //three
const double Fourth_LAT =    38.896650;
const double Fourth_LNG =   -89.438798;   //four
const double Fifth_LAT =    38.896713;
const double Fifth_LNG =    -89.438370;   //five
const double Sixth_LAT =     0.00;
const double Sixth_LNG =     0.00;   //six
const double Seventh_LAT =   38.638595;
const double Seventh_LNG =  -90.271740;    //Seven
const double Eighth_LAT =    38.638892; /////////////
const double Eighth_LNG =   -90.271611;   //Eight
const double Nineth_LAT =    38.638595;
const double Nineth_LNG =   -90.271740;   //ninth
const double Tenth_LAT =     38.638477;
const double Tenth_LNG =    -90.271741;   // Tenth
const double Eleventh_LAT =  38.638233;
const double Eleventh_LNG = -90.271758;   //Eleventh
const double Twelveth_LAT =  38.637919;
const double Twelveth_LNG = -90.272353;   //tweleth
const double Last_LAT    =     0.0;
const double Last_LNG    =     0.0;       // Last Stop point

double WAYPOINT_DIST_TOLERANE = 1;   // tolerance in meters to waypoint; once within this tolerance, will advance to the next waypoint
#define NUMBER_WAYPOINTS 26          // enter the numebr of way points here (will run from 0 to (n-1)) put actual amount of waypoints long and lat
int waypointNumber = 0;            // current waypoint number; will run from 0 to (NUMBER_WAYPOINTS -1); start at -1 and gets initialized during setup()
const double WaypointList[NUMBER_WAYPOINTS] = {
  First_LAT, First_LNG, Second_LAT, Second_LNG, Third_LAT, Third_LNG, Fourth_LAT, Fourth_LNG, Fifth_LAT, Fifth_LNG, Sixth_LAT, Sixth_LNG, Seventh_LAT, Seventh_LNG, Eighth_LAT, Eighth_LNG, Nineth_LAT,
  Nineth_LNG, Tenth_LAT, Tenth_LNG, Eleventh_LAT, Eleventh_LNG, Twelveth_LAT, Twelveth_LNG, Last_LAT, Last_LNG
};
/////////////////////////////////////////////////////temp variables for waypoints

double CURRENT_DESTINATION_LAT = 0.000000;      // where our next LAT target location will be loaded
double CURRENT_DESTINATION_LNG = 0.000000;      // where our next LNG target location will be loaded
double MyLatAverage = 0.000000;   //current GPS Position LAT
double MyLngAverage = 0.000000;   //current GPS Position LNG

static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps; // The TinyGPS++ object
byte SpeedArray[11] = {
  0xA0, 0xA1, 0x00, 0x04, 0x05, 0x00, 0x05, 0x01, 0x01, 0x0D, 0x0A
};//115200
byte UpdateArray[10] = {
  0xA0, 0xA1, 0x00, 0x03, 0x0E, 0x0A, 0x01, 0x05, 0x0D, 0x0A
}; // update rate 10x

int potpin = A0;  // analog pin used to connect the potentiometer
int val;    // variable to read the value from the analog pin
float DEC_ANGLE = -1.8;         //0.069 North direction adjustment
// Compass navigation
double courseToWayPoint;        //GPS course suggestion
int GPSheading = 0;             //Heading determined by GPS
double distanceKm = 5;
double distanceMeters = 1000;        // distance place holder set to 100 to get started
int targetHeading;              // where we want to go to reach current waypoint
int currentHeading = 0;             // where we are actually facing now
int headingError;               // signed (+/-) difference between targetHeading and currentHeading
int HEADING_TOLERANCE = 8;     // tolerance +/- (in degrees) within which we don't attempt to turn to intercept targetHeadi

//0 = Stop, 1 = Forward, 2 = Right Turn, 3 = Left Turn, 4 Hard Right, 5 = Hard Left, 6 = Reverse
byte NewDirection = 0;   // 0 = Straight, 1 = left, 2 = right
byte NewSpeed = 0;       // 0 = stop, 10 = slow, 20 = fast
bool UpdateIsCurrent = 0;     // this will keep track if our update is current and if the truck is in a known position
bool WeAreGO = 0;            // this will allow the truck to travel via Push Button and Coordinates to go to
byte PrintTimeCounter = 0;   //counter for timing LCD printing
byte LcdPrintRotate = 1;  //Rotate between screen displays

int32_t lastMicros;
int32_t lastMillis;
int32_t lastPosUpdate;
int32_t lastBlink;
////////////////////////////Debounced buttons////////////////////////
int ButtonsArray[] = {2, 3, 4}; //Buttons on the Teensy 3.1
const int TotalButtons = (sizeof(ButtonsArray) / 2);
boolean ButtonStates[TotalButtons];
boolean LastButtonStates[TotalButtons];    //LOW
boolean ButtonIsPressed[TotalButtons];
unsigned long LastDebounceTime[TotalButtons];
unsigned long DebounceTime = 50;
/////////////////////////////////////////////////SERVO INSTRUCTIONS/////
//byte instruction = 0;     //the switch case control
byte ServoData = 0;      //variable to be used with SendData()
int32_t lastCommandUpdate;                // the last time a command was received
int32_t LastPrintTime;                // the last time a command was received
bool newUpdate;    //we have a new update
//////////////////////////////////////////////LED PINS AND VARIABLES
#define RED_LEFT    2   // LIST OF LED PINS
#define GREEN_LEFT  3
#define BLUE_LEFT   4
#define RED_RIGHT   5
#define GREEN_RIGHT 6
#define BLUE_RIGHT  7
bool RL_State = false;   // LIST OF LED BLINK STATES
bool GL_State = false;
bool BL_State = false;
bool RR_State = false;
bool GR_State = false;
bool BR_State = false;
bool LEDStateArray[6] = {
  RL_State, GL_State, BL_State, RR_State, false, false
}; // led state array still in progress... perhaps say at the end of function led 1 = LedState 1 etc. led 6 = LedState 6

void setup()
{
  for (int i = 0; i < TotalButtons; i++) {
    pinMode(ButtonsArray[i], INPUT_PULLUP);        // set pin to input for button
  }
  LastButtonStates[0] = HIGH;
  Serial.begin(115200);
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  display.begin();  // init done
  display.setContrast(90);  // you can change the contrast around to adapt the display
  display.clearDisplay();   // clears the screen and buffer
  // text display tests
  display.setTextSize(1);
  display.setTextColor(BLACK);
  display.setCursor(0, 0);
  display.print("Initializing I2C devices...");

  /* Initialise the AdaFruit sensors */
  initSensors();
  // save I2C bitrate
  TWBR = 12; // upgrade to 400KHz!
  delay(1000);

  Serial1.begin(GPSBaud);
  SendData(0);   //STOP

  for (int i = 2; i < 8; i++) {
    pinMode(i, OUTPUT);  // configure LED for output ON PINS 2 TO 7
  }

  ///////////////////////////////////////////GPS SPEED UP TO 10HZ 115,200 BAUD /////////////
  for (int i = 0; i < 10; i++) {                                                          ///
    Serial1.write(UpdateArray[i]);                                                        ///
    delay(10);    //was 5                                                                 ///
  }                                                                                       ///
  delay(500);                                                                             ///
                                                                                          ///
  for (int i = 0; i < 11; i++) {                                                          ///
    Serial1.write(SpeedArray[i]);                                                         ///
    delay(10);    //was 5                                                                 ///
  }                                                                                       ///
                                                                                          ///
  Serial1.begin(115200);                                                                  ///
  ///////////////////////////////////////////////////////////////////////////////////////////

  CURRENT_DESTINATION_LAT = WaypointList[waypointNumber], 10;     // where our next LAT target location will be loaded
  CURRENT_DESTINATION_LNG = WaypointList[waypointNumber + 1], 10;     // where our next LNG target location will be loaded
  waypointNumber++; //update the waypoint counter
  display.clearDisplay();   // clears the screen and buffer
  display.setCursor(0, 0);
  display.print("LAT ");
  display.println(CURRENT_DESTINATION_LAT, 6);
  display.print("LNG ");
  display.println(CURRENT_DESTINATION_LNG, 6);
  display.print("waiting on gps");
  for (int i = 0; i < 1;) {
    while (Serial1.available() > 0)gps.encode(Serial1.read());

    if (gps.location.isUpdated()) {
      i = 1;
    }
    // blink LEDS to indicate activity
    if ((lastMillis + 600) < millis()) {
      lastMillis = millis();
      RL_State  = !RL_State;
      RR_State  = !RR_State;
      digitalWrite(RED_LEFT, RL_State);
      digitalWrite(RED_RIGHT, RR_State);
    }
  }
  digitalWrite(RED_LEFT, LOW);
  digitalWrite(RED_RIGHT, LOW);
}

void loop()
{
  if (WeAreGO == 0) {
    ButtonChecker();  //check for button Press
    if (ButtonStates[0] == LOW) {
      WeAreGO = 1;  //we have a fix we are ready to go..
    }
  }
  // AdaFruit sensor events
  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_vec_t   orientation;

  /* Read the accelerometer and magnetometer */
  accel.getEvent(&accel_event);
  mag.getEvent(&mag_event);

  while (Serial1.available() > 0) gps.encode(Serial1.read());  // read GPS as data arrives

  if (gps.location.isUpdated())
  {
    lastPosUpdate = millis();      // mark when this update took place
    MyLatAverage = gps.location.lat(), 6;
    MyLngAverage =  gps.location.lng(), 6;
    courseToWayPoint = TinyGPSPlus::courseTo(MyLatAverage, MyLngAverage, CURRENT_DESTINATION_LAT, CURRENT_DESTINATION_LNG);
    distanceKm = gps.distanceBetween(MyLatAverage, MyLngAverage, CURRENT_DESTINATION_LAT, CURRENT_DESTINATION_LNG) / 1000;
    // Serial.println(courseToWayPoint);
    if (distanceKm < 10) {
      distanceMeters = (distanceKm * 1000);
    }
    else
    {
      distanceMeters = 9999;
    }
  }

  ////////////////////////blink the LED for Direction indication/////////////
  if (millis() > lastBlink + 500)
  {
    lastBlink = millis();
    LedBlinker(); //blink the leds to indicate travel status
    if (PrintTimeCounter == 2) { // count 5 times equals 2.5 seconds
      if (WeAreGO == 0) {
        val = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023)
        val = map(val, 0, 1023, -60, 60);     // scale it to use it with dec value
        DEC_ANGLE = val;
      }

      LCDPrinter();
      PrintTimeCounter = 0;  //reset counter
    }
    if (LcdPrintRotate == 10) {
      LcdPrintRotate = 0;
    }
    PrintTimeCounter++;
    LcdPrintRotate++;
  }

  while (Serial1.available() > 0) gps.encode(Serial1.read());  // read GPS as data arrives
  ///////////////////////check to make sure there is a fresh update
  if (millis() > (lastPosUpdate + 1800))
  {
    UpdateIsCurrent = 0;  // Our last update is too old
    //Serial.println("oops update is too old");
  }
  else
  {
    UpdateIsCurrent = 1;  // Our last update is current
    //Serial.println("good update time");
  }
  ////////////////here is where we work with the distance // check to see if we have reached the current waypoint************************************************

  if (distanceMeters <= WAYPOINT_DIST_TOLERANE) { // || Serial.available() > 0) {

    distanceMeters = 9999; // reset the waypoint distance to allow time for recaluation //  nextWaypoint();
    waypointNumber++;  //  nextWaypoint
    CURRENT_DESTINATION_LAT = WaypointList[waypointNumber];      // where our next LAT target location will be loaded
    waypointNumber++;
    CURRENT_DESTINATION_LNG = WaypointList[waypointNumber];      // where our next LNG target location will be loaded
    if (Serial.available() > 0) {
      Serial.read();
    }
    SendData(0);   //STOP the truck to show we have arrived at a waypoint
    //read the GPS while we pause
    for (int i = 0; i <= 2000; i++) {
      while (Serial1.available() > 0) gps.encode(Serial1.read());  // read GPS as data arrives
      if (gps.location.isUpdated())
      {
        MyLatAverage = gps.location.lat(), 6;
        MyLngAverage =  gps.location.lng(), 6;
        delay(1);
      }
    }

    Serial.println(waypointNumber);
    lcd.clear();
    lcd.print("LAT ");
    lcd.print(CURRENT_DESTINATION_LAT, 6);
    lcd.setCursor ( 0, 1 );        // go to the next line
    lcd.print("LNG ");
    lcd.print(CURRENT_DESTINATION_LNG, 6);


    if (CURRENT_DESTINATION_LAT == 0.00)
    {
      while (1)
      {
        Serial.print("Stop");           //STOP RIGHT HERE FOR NOW PLAN ON SENDING A STOP SERVO
        WeAreGO = 0;   // stop the truck we have lost permission, we are out of waypoints
        SendData(0);   //STOP
        delay(300);
      }
    }
  }
  while (Serial1.available() > 0) gps.encode(Serial1.read());  // read GPS as data arrives
  //Mag//////////////////////////////////////////////////
  /* Use the new fusionGetOrientation function to merge accel/mag data */
  if (dof.fusionGetOrientation(&accel_event, &mag_event, &orientation))
  {
    currentHeading = orientation.heading;
    currentHeading += DEC_ANGLE;
    if (currentHeading > 0) {
      currentHeading = 360 - abs(currentHeading);             // Check for wrap due to addition of declination.
    }
    if (currentHeading < 0) {
      currentHeading = abs(currentHeading);
    }
    // Serial.println(currentHeading);
    calcDesiredTurn();
    if (gps.course.isUpdated())
    {
      GPSheading = gps.course.deg();    // heading determined by the GPS
    }
    while (Serial1.available() > 0) gps.encode(Serial1.read());  // read GPS as data arrives
    /////////////////////////<<<<<SERVO DATA SEND>>>>>>>>>>>>>>/////////////////

    if (WeAreGO == 1)  //((UpdateIsCurrent != 0) &&
    { // Our last update is current, we have another waypoint and have not been stopped
      ServoData = (NewDirection + NewSpeed);
      SendData(ServoData);  // sends one byte 0 = straight, 1 = left, 2 = right, 3 = straight + Speed  0 = stop, 10 = slow, 20 = fast
    }
    else
    {
      ServoData = 0;
      SendData(ServoData);  // sends one byte    //Throttle Center ||  88
    }
  }
  while (Serial1.available() > 0) gps.encode(Serial1.read());  // read GPS as data arrives
}




































