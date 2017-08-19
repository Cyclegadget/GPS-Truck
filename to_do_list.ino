 /*
 sketch was based on A_gps_guided_car
 Receiver Read Sketch could be handy
 
 ****wanted items****
 Different speeds depending on distance to waypoint or next waypoint is harder
 Start button to allow truck to begin
 
 Stop if gps fix is lost
 Leds right, left, fix, forward, stop
 
 leds change color at waypoint change
 
 I2C display showing current position, next waypoint, distance to next way, ready/mode
 start button, stop button
 I2C output to slave
 Slow the steering down maybe on slave
 
 work on different outputs right left forward stop
 Make LED arrays to control led states LIKE SET ALL TO FALSE BUT ONE OR TWO LEDS*****
 
 IF LAST WAYPOINT AND DISTANCE IS CLOSE ENOUGH STOP****make last waypoint "0.00" and stop on 0 check
 
 make serial monitor printing slower
 I need more 5 volt sources
 Start button ... waiting on a fix
 
 Show current waypoint going to 
 print distance to 
 
 
 Change direction results from 1,2,99 to something readable
 add left and right LEDs
 add message at end of setup that says we are waiting on a fix
 load actual corrdinates
 #define FULL_SPEED 70
 
 Decimal Degrees = Degrees + minutes/60 + seconds/3600
 *Lat first*
START      38.63151388888889    START POSITION
           90.27190833333333
           
           38.63115833333333///
           90.27217222222222//
           
           38.631391666666666//
           90.27288055555556//
           
           38.63209166666667//
           90.27270555555556//
           
           38.632580555555556//
           90.27276666666667//
           
           38.63251388888889//
           90.27228611111111//
           
           38.632019444444445
           90.27203055555556
           
FINISH     38.63151388888889
           90.27190833333333          
 
 
 DONE
 
 Average gps readings?
 add gps speed up...... a gps checker2
 I2C is max speed 
 
 TWBR   prescaler   Frequency
 
  12       1       400   kHz  (the maximum supported frequency)
  32       1       200   kHz
  72       1       100   kHz  (the default)
 152       1        50   kHz
  78       4        25   kHz
 158       4        12.5 kHz
 
    double mylat = gps.location.lat();
    Serial.print(mylat,6);
    Serial.print(F(" Long="));
    double mylng = gps.location.lng();
    Serial.print(mylng,6);
 
 */
