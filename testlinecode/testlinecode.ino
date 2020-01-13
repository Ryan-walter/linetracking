#include <Pixy2.h>
#include <PIDLoop.h>
#include <ZumoMotors.h>
#include <ZumoBuzzer.h>

// Zumo speeds, maximum allowed is 400
#define ZUMO_FAST        200
#define ZUMO_SLOW        150
#define X_CENTER         (pixy.frameWidth/2)
int intrsectcounter = 0;
int instruct = 1;

Pixy2 pixy;
ZumoMotors motors;
ZumoBuzzer buzzer;

PIDLoop headingLoop(5000, 0, 0, false);

void setup()
{
  Serial.begin(115200);
  
  Serial.print("Starting...\n");
  motors.setLeftSpeed(0);
  motors.setRightSpeed(0);

  pixy.init();
  // Turn on both lamps, upper and lower for maximum exposure
  pixy.setLamp(1, 1);
  // change to the line_tracking program.  Note, changeProg can use partial strings, so for example,
  // you can change to the line_tracking program by calling changeProg("line") instead of the whole
  // string changeProg("line_tracking")
  pixy.changeProg("line");

  // look straight and down
  pixy.setServos(500, 850);
  pixy.line.setDefaultTurn(-90);
}


void loop() {
  int8_t res;
  int32_t error;
  int left, right;
  char buf[96];


  // Get latest data from Pixy, including main vector, new intersections and new barcodes.
  res = pixy.line.getMainFeatures();

  if (res <= 0)
  {
    motors.setLeftSpeed(0);
    motors.setRightSpeed(0);
    Serial.print("I see nothing. ");
    Serial.println(res);
    return;
  }

  if (res & LINE_VECTOR) // Follows a line and corrects itself as neccessary.
  {
    error = (int32_t)pixy.line.vectors->m_x1 - (int32_t)X_CENTER;
    pixy.line.vectors->print();
    headingLoop.update(error);
    left = headingLoop.m_command;
    right = -headingLoop.m_command;

    Serial.print("Following a line. ");
    Serial.println(res);

    if (pixy.line.vectors->m_y0 > pixy.line.vectors->m_y1) // Cehcks if y-coord of head is less than tail, which means vector is pointing forward.
    {
      if (pixy.line.vectors->m_flags & LINE_FLAG_INTERSECTION_PRESENT) // Slows down when intersection present.
      {
        left += ZUMO_SLOW;
        right += ZUMO_SLOW;
        Serial.println("Slowing down for intersection.");
      }
      else // Else continues forward.
      {
        left += ZUMO_FAST;
        right += ZUMO_FAST;
      }
    }
    else  // If the vector is pointing down, or down-ish, we need to go backwards to follow.
    {
      left -= ZUMO_SLOW;
      right -= ZUMO_SLOW;
      Serial.println("Going backwards.");
    }
    motors.setLeftSpeed(left);
    motors.setRightSpeed(right);
  }

  if (res & LINE_BARCODE) // Checks for any barcodes.
  {
    if (pixy.line.barcodes->m_code == 5) // Checks if detected barcode is barcode 5.
    {
      Serial.println("I see a barcode.");
      delay(600);
      stopMoving();
      delay(500);
      if (instruct == 0) // If instruct variable is 0, turns left.
      {
        Serial.println("Turning left");
        motors.setLeftSpeed(-250);
        motors.setRightSpeed(250);
        delay(550);
        keepTurning(instruct);
      }
      if (instruct == 1) // If instruct variable is 1, turns right.
      {
        Serial.println("Turning right.");
        motors.setLeftSpeed(250);
        motors.setRightSpeed(-250);
        delay(500);
        keepTurning(instruct);
      }
      if (instruct == 2) // If instruct variable is 2, continues forward.
      {
        Serial.println("Continuing straight.");
        return;
      }
    }
  }
}

void stopMoving() // Function created to stop motors
{
  motors.setLeftSpeed(0);
  motors.setRightSpeed(0);
  Serial.println("Stopped motors.");
}

void keepTurning(int x) // Function meant to override parts of code so robot behaves differently when turning.
{
  int8_t res;
  int direc = x;
  res = pixy.line.getMainFeatures();
  
  while (res <= 0) // While turning, if the Pixy camera sees nothing, robot will continue turning until it sees something (a line).
  {
    res = pixy.line.getMainFeatures();
    Serial.println(res);
    if(direc == 0) // Checks if previously turning left.
    {
      Serial.println("Continuing left turn.");
      motors.setLeftSpeed(-250);
      motors.setRightSpeed(250);
    }
    if(direc == 1) // Checks if previously turning right.
    {
      Serial.println("Continuing right turn.");
      motors.setLeftSpeed(250);
      motors.setRightSpeed(-250);
    }
  }
  
  if (res & LINE_VECTOR) // Robot will turn 90 degrees before it stops.
  {
    if (direc == 0) // Checks if previously turning left.
    {
      while (pixy.line.vectors->m_x0 > pixy.line.vectors->m_x1) // Continues to turn until tail cordinate of vector is greater than the coordinate for the head.
      {                                                         // Signifies that robot fully turned 90 degrees.
        Serial.println("Finishing left turn.");
        motors.setLeftSpeed(-250);
        motors.setRightSpeed(250);
      }
    }
    if (direc == 1) // Checks if previously turning left.
    {
      while (pixy.line.vectors->m_x0 > pixy.line.vectors->m_x1) // Continues to turn until tail cordinate of vector is less than the coordinate for the head.
      {                                                         // Signifies that robot fully turned 90 degrees.
        Serial.println("Finishing right turn.");
        motors.setLeftSpeed(250);
        motors.setRightSpeed(-250);
        //Serial.println(pixy.line.vectors->m_x0);
        //Serial.println(pixy.line.vectors->m_x1);
      }
    }
    stopMoving();
    delay(250);
    Serial.println("Finished.");
    return;
  }
}
