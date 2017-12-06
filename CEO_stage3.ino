/*
 * Demo line-following code for the Pololu Zumo Robot
 *
 * This code will follow a black line on a white background, using a
 * PID-based algorithm.  It works decently on courses with smooth, 6"
 * radius curves and has been tested with Zumos using 30:1 HP and
 * 75:1 HP motors.  Modifications might be required for it to work
 * well on different courses or with different motors.
 *
 * http://www.pololu.com/catalog/product/2506
 * http://www.pololu.com
 * http://forum.pololu.com
 */

#include <QTRSensors.h>
#include <ZumoReflectanceSensorArray.h>
#include <ZumoMotors.h>
#include <ZumoBuzzer.h>
#include <Pushbutton.h>
#include <SPI.h>  
#include <Pixy.h>

Pixy pixy;
ZumoBuzzer buzzer;
ZumoReflectanceSensorArray reflectanceSensors;
ZumoMotors motors;
Pushbutton button(ZUMO_BUTTON);
int lastError = 0;
int m1Speed = 0;
int m2Speed = 0;

// This is the maximum speed the motors will be allowed to turn.
// (400 lets the motors go at top speed; decrease to impose a speed limit)
const int MAX_SPEED = 400;
int stage = 1;
const int objectWidth = 110;

void setup()
{
  motors.setSpeeds(0,0);
  // Play a little welcome song
  buzzer.play(">g32>>c32");

  // Initialize the reflectance sensors module
  reflectanceSensors.init();
  
  // Wait for the user button to be pressed and released
//  THIS LINE WAITFORBUTTON IS COMMENTED OUT BECAUSE AFTER INTEGRATE WITH PIXY IT CANNOT CALL THIS FUNCTION
//  button.waitForButton();

  // Wait 1 second and then begin automatic sensor calibration
  // by rotating in place to sweep the sensors over the line
  delay(1000);
  int i;
  for(i = 0; i < 80; i++)
  {
    if ((i > 10 && i <= 30) || (i > 50 && i <= 70))
      motors.setSpeeds(-200, 200);
    else
      motors.setSpeeds(200, -200);
    reflectanceSensors.calibrate();

    // Since our counter runs to 80, the total delay will be
    // 80*20 = 1600 ms.
    delay(20);
  }
  motors.setSpeeds(0,0);

  // Turn off LED to indicate we are through with calibration
  digitalWrite(13, LOW);
  buzzer.play(">g32>>c32");

  // Wait for the user button to be pressed and released
//  button.waitForButton();

  // Play music and wait for it to finish before we start driving.
  pixy.init();
//  while(buzzer.isPlaying());
  delay(3000);
  buzzer.play("L16 cdegreg4");
}

void loop()
{
  unsigned int sensors[6];

  // Get the position of the line.  Note that we *must* provide the "sensors"
  // argument to readLine() here, even though we are not interested in the
  // individual sensor readings
  int position = reflectanceSensors.readLine(sensors);

  // Our "error" is how far we are away from the center of the line, which
  // corresponds to position 2500.
  int error = position - 2500;
  
  // Get motor speed difference using proportional and derivative PID terms
  // (the integral term is generally not very useful for line following).
  // Here we are using a proportional constant of 1/4 and a derivative
  // constant of 6, which should work decently for many Zumo motor choices.
  // You probably want to use trial and error to tune these constants for
  // your particular Zumo and line course.
  int speedDifference = 0.15 * error + 40 * (error - lastError);
  
  lastError = error;
  m1Speed = MAX_SPEED + speedDifference;
  m2Speed = MAX_SPEED - speedDifference;
  
  // Here we constrain our motor speeds to be between 0 and MAX_SPEED.
  // Generally speaking, one motor will always be turning at MAX_SPEED
  // and the other will be at MAX_SPEED-|speedDifference| if that is positive,
  // else it will be stationary.  For some applications, you might want to
  // allow the motor speed to go negative so that it can spin in reverse.
  
  static int i = 0;
  int j;
  uint16_t blocks;
  char buf[32]; 
  
  // grab blocks!
  blocks = pixy.getBlocks();
  
  // If there are detect blocks, print them!
  if (blocks)
  {
    i++;
    
    // do this (print) every 50 frames because printing every
    // frame would bog down the Arduino
    if (i%1==0)
    {
      sprintf(buf, "Detected %d:\n", blocks);
      Serial.print(buf);
      int maxWidthDetected = 0;
      bool detected = false;
      int posX = 0;
      for (j=0; j<blocks; j++)
      {
        sprintf(buf, "  block %d: ", j);
//        Serial.print();
        // > 50 then detect zone B
        // for bonus detect width>250
        if((pixy.blocks[j].signature == 1) && (stage == 1)){
//          if(maxWidthDetected > objectWidth){
//            stage = 2;
//          }
          detected = true;
          if(pixy.blocks[j].width > maxWidthDetected)
            maxWidthDetected = pixy.blocks[j].width;
          
        }
      }

      if(stage == 1){
        if(detected && maxWidthDetected > objectWidth){
          stage = 2;
        }
      }
    }
  }
  if(stage == 2){
    m1Speed = 0;
    m2Speed = 0;
  }
  
  if (m1Speed < 0)
    m1Speed = 0;
  if (m2Speed < 0)
    m2Speed = 0;
  if (m1Speed > MAX_SPEED)
    m1Speed = MAX_SPEED;
  if (m2Speed > MAX_SPEED)
    m2Speed = MAX_SPEED;

  motors.setSpeeds(m1Speed, m2Speed);
}
