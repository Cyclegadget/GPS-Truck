void calcDesiredTurn()
{
  // calculate where we need to turn to head to destination

    targetHeading = courseToWayPoint;  // load the best degree to go to ??????????????????????????????????
    headingError = targetHeading - currentHeading;
  //headingError = targetHeading - GPSheading;
  // adjust for compass wrap
  if (headingError < -180)
    headingError += 360;
  if (headingError > 180)
    headingError -= 360;

  if (distanceMeters > 10)    //decide how fast we can travel between points
  {
    NewSpeed = 20;       // 0 = stop, 10 = slow, 20 = fast
  }
  else
  {
    NewSpeed = 10;       // 0 = stop, 10 = slow, 20 = fast
    //NewSpeed = 20;
  }
  //Serial.println(headingError);
  // calculate which way to turn to intercept the targetHeading

  if(distanceMeters > 20)
  {
    HEADING_TOLERANCE = 20;
  }
  else
  {
    HEADING_TOLERANCE = 8;
  }

  if (abs(headingError) <= HEADING_TOLERANCE)      // if within tolerance, don't turn
  {
    NewDirection = 0;           // go Straight      // 0 = Straight, 1 = left, 2 = right
  }
  else if (headingError < 0)     // if headingError is positive steer left
  {
    NewDirection = 1;  //steer left
  }
  else if (headingError > 0)     // if headingError is negative steer right
  {
    NewDirection = 2;       //steer right
  }
}



