
void ButtonChecker() // function to check buttons and save the states
{
  for (int i = 0; i <= (TotalButtons); i++){

    boolean reading = digitalRead(ButtonsArray[i]);
    if (reading != LastButtonStates[i]) {
      LastDebounceTime[i] = millis();
      // Serial.println(i);
    }
    if ((millis() - LastDebounceTime[i]) > DebounceTime) {

      //////////////////////////
      if (reading != ButtonStates[i]) {
        ButtonStates[i] = reading;
      }
    }
  }
}

void initSensors()  // Initialises the AdaFruit 9DOF sensors used for heading
{
  if(!accel.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    lcd.print("Ooops, no accel");
    while(1);
  }
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    lcd.print("Ooops, no mag");
    while(1);
  }
}

//////////////////////////function to send I2C data/////////////

void LCDPrinter(){
  if(WeAreGO == 0){
    lcd.setCursor ( 0, 0 );
    lcd.print("                    ");
    lcd.setCursor ( 0, 0 );
    lcd.print(currentHeading);
    lcd.setCursor ( 0, 1 );
    lcd.print("                    ");
    lcd.setCursor ( 0, 1 );
    lcd.print(DEC_ANGLE);
    lcd.setCursor ( 0, 2 );
    lcd.print("Press button #2 ");
    lcd.setCursor ( 0, 3 );
    lcd.print("To Start Truck");
  }
  if(WeAreGO == 1 && LcdPrintRotate < 5){
    lcd.setCursor ( 0, 0 );        // go to the first line
    lcd.print("                    ");
    lcd.setCursor ( 0, 0 );        // go to the first line
    lcd.print("LAT");
    lcd.print(CURRENT_DESTINATION_LAT,6);
    lcd.setCursor ( 15, 0 );
    lcd.print("TAR");
    lcd.setCursor ( 0, 1 );        // go to the second line
    lcd.print("                    ");
    lcd.setCursor ( 0, 1 );        // go to the second line
    lcd.print("LNG");
    lcd.print(CURRENT_DESTINATION_LNG,6);
    lcd.setCursor ( 14, 1 );
    lcd.print(targetHeading);
    lcd.setCursor ( 0, 2 );        // go to the next line
    lcd.print("                    ");
    lcd.setCursor ( 0, 2 );
    lcd.print("Lat");
    lcd.print(MyLatAverage,6);
    lcd.setCursor ( 14, 2 );
    lcd.print(currentHeading);
    lcd.setCursor ( 0, 3 );        // go to the next line
    lcd.print("                    ");
    lcd.setCursor ( 0, 3 );
    lcd.print("Lng");
    lcd.print(MyLngAverage,6);
    lcd.setCursor ( 14, 3 );
    lcd.print(headingError);
  }
    if(WeAreGO == 1 && LcdPrintRotate > 5){
    lcd.clear();
    lcd.setCursor ( 0, 0 );        // go to the first line
    lcd.print("KMs ");
    lcd.print(distanceKm);
    lcd.setCursor ( 0, 1 );        // go to the second line
    lcd.print("Mters ");
    lcd.print(distanceMeters);
    lcd.setCursor ( 0, 2 );
    lcd.print("GPSheading ");
    lcd.print(GPSheading);
    lcd.setCursor ( 0, 3 );
    lcd.print("ServoData ");
    lcd.print(ServoData);
   }
}

void LedBlinker() {
  //one byte 0 = straight, 1 = left, 2 = right, 3 = straight2 (+ Speed  0 = stop, 10 = slow, 20 = fast)
  //possible combinations  0, 11, 12, 20, 21, 22
  //we have a fix but the command = stop  WeAreGO = 1
  if(WeAreGO == 0)
  {
    NewDirection = 0;
    NewSpeed = 0;
  }
  switch (NewDirection + NewSpeed) {

  case 0:    //stop with no fix
    if(WeAreGO == 0){
      RL_State = !RR_State;   // LIST OF LED BLINK STATES
      GL_State = false;
      BL_State = false;
      RR_State = !RR_State;
      GR_State = false;
      BR_State = false;
    }
    else     // we have a go but no next waypoint
    {
      RL_State = false;   // LIST OF LED BLINK STATES
      GL_State = !GR_State;
      BL_State = false;
      RR_State = false;
      GR_State = !GR_State;
      BR_State = false;
    }
    break;
  case 11:   //slow left
    RL_State = false;   // LIST OF LED BLINK STATES
    GL_State = false;
    BL_State = !BL_State;
    RR_State = false;
    GR_State = !GR_State;
    BR_State = false;
    break;
  case 12:   //slow right
    RL_State = false;   // LIST OF LED BLINK STATES
    GL_State = !GL_State;
    BL_State = false;
    RR_State = false;
    GR_State = false;
    BR_State = !BR_State;
    break;
  case 20:   //fast straight
    RL_State = !BR_State;   // LIST OF LED BLINK STATES
    GL_State = false;
    BL_State = !BR_State;
    RR_State = !BR_State;
    GR_State = false;
    BR_State = !BR_State;
    break;
  case 21:   //fast left
    RL_State = !BL_State;   // LIST OF LED BLINK STATES
    GL_State = false;
    BL_State = !BL_State;
    RR_State = false;
    GR_State = false;
    BR_State = false;
    break;
  case 22:   //fast right
    RL_State = false;   // LIST OF LED BLINK STATES
    GL_State = false;
    BL_State = false;
    RR_State = !BR_State;
    GR_State = false;
    BR_State = !BR_State;
    break;
  }
  digitalWrite(RED_LEFT,RL_State);
  digitalWrite(GREEN_LEFT,GL_State);
  digitalWrite(BLUE_LEFT,BL_State);
  digitalWrite(RED_RIGHT,RR_State);
  digitalWrite(GREEN_RIGHT,GR_State);
  digitalWrite(BLUE_RIGHT,BR_State);
}









