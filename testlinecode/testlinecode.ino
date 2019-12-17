#include <Pixy2.h>
#include <PIDLoop.h>
#include <ZumoMotors.h>
#include <ZumoBuzzer.h>

// Zumo speeds, maximum allowed is 400
#define ZUMO_FAST        100
#define ZUMO_SLOW        75
#define X_CENTER         (pixy.frameWidth/2)
#define res              (pixy.line.getMainFeatures())
#define leftMotor        100
#define rightMotor       75
String direc = "None";
int counter = 0;
bool wait;

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
  pixy.setServos(550, 800);

  pixy.line.setMode(LINE_MODE_TURN_DELAYED);
}

void loop()
{
  int8_t res;
  int32_t error;
  int left, right;
  char buf[96];
  bool x;

  if (pixy.line.vectors->m_flags&LINE_FLAG_INTERSECTION_PRESENT)
  {
    Serial.println("At intersection.    Direction: " + direc);
    delay(150);
    motors.setLeftSpeed(0);
    motors.setRightSpeed(0);
  }

  if (res & LINE_BARCODE) // If Pixy sees a barcode, it will start to turn left or right
  {
    Serial.println("I see a barcode:");
    if (pixy.line.barcodes->m_code == 5) // Checks to see if its the barcode associated with a right turn
    {
      direc = "right";
      Serial.println(" 5.   Turning " + direc);
      motors.setLeftSpeed(leftMotor);
      motors.setRightSpeed(-rightMotor);
      delay(50);
    }
    if (pixy.line.barcodes->m_code == 0) // Checks to see if its the barcode associated with a left turn
    {
      direc = "left";
      Serial.println(" 5.   Turning " + direc);
      motors.setLeftSpeed(-leftMotor);
      motors.setRightSpeed(rightMotor);
      delay(50);
    }
  }

  if (res <= 0) // If Pixy sees nothing, it will stop. If it was in the process of turning it will continue turning in that direction
  {
    Serial.print("I see nothing.");
    if (direc == "right") // Checks string to see if it previously was in the process of turning right
    {
      Serial.println("    Direction: " + direc + "    Continuing turn");
      motors.setLeftSpeed(leftMotor);
      motors.setRightSpeed(-rightMotor);
    }
    if (direc == "left") // Checks string to see if it was previously in the process of turning left
    {
      Serial.println("    Direction: " + direc + "    Continuing turn");
      motors.setLeftSpeed(leftMotor);
      motors.setRightSpeed(-rightMotor);
    }
    if (direc == "None") // Checks if string is empty, which means that Pixy was not previously turning
    {
      motors.setLeftSpeed(0);
      motors.setRightSpeed(0);
      Serial.println("    Direction: " + direc + "    Stopping");
    }
  }

  if (res & LINE_VECTOR) // If Pixy sees a line, it will correct itself, track, and follow that line. If it was previously turning, it will stop at the line and continue foward.
  {
    error = (int32_t)pixy.line.vectors->m_x1 - (int32_t)X_CENTER; // Corrects itself
    pixy.line.vectors->print();
    headingLoop.update(error);
    left = headingLoop.m_command;
    right = -headingLoop.m_command;
    motors.setLeftSpeed(right);
    motors.setRightSpeed(left); 
    
    Serial.print("I see a line.");

    if (direc == "right") //&& pixy.line.vectors->m_x0 > pixy.line.vectors->m_x1); // Checks if it was previously turning right
    {
      motors.setLeftSpeed(0);
      motors.setRightSpeed(0);
      direc = "None";
      Serial.println("    Direction: " + direc + "    Finishing turn") ;
      counter == 0;
      delay(500);
    }

    if (direc == "left") //&& pixy.line.vectors->m_x0 < pixy.line.vectors->m_x1); // Checks if it was previously turning left
    {
      motors.setLeftSpeed(0);
      motors.setRightSpeed(0);
      direc = "None";
      counter == 0;
      Serial.println("    Direction: " + direc + "    Finishing turn") ;
      delay(500);
    }

    if (direc == "None") //&& pixy.line.vectors->m_y0 > pixy.line.vectors->m_y1) // Checks if it was previously going foward
    {
      Serial.println("    Direction: " + direc + "    Continuing Forward");
      motors.setLeftSpeed(ZUMO_FAST);
      motors.setRightSpeed(ZUMO_FAST);
    }
  }
}


/*void waitForCommand()
{
  while (wait)
  {
    Serial.println("Waiting");
    if (res & LINE_BARCODE)
    {
      wait = false;
      return;
    }
  }
}*/
