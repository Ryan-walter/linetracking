#include <Pixy2.h>
#include <PIDLoop.h>
#include <ZumoMotors.h>
#include <ZumoBuzzer.h>

// Zumo speeds, maximum allowed is 400
#define ZUMO_FAST        100
#define ZUMO_SLOW        50
#define X_CENTER         (pixy.frameWidth/2)
#define res              (pixy.line.getMainFeatures())
#define leftMotor        200
#define rightMotor       200
String direc;
int counter = 0;

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

  if (res & LINE_BARCODE) // If Pixy sees a barcode, it will start to turn left or right
  {
    Serial.println("I see a barcode");
    if (pixy.line.barcodes->m_code == 5) // Checks to see if its the barcode associated with a right turn
    {
      direc = "right";
      Serial.println("I see barcode 5");
      motors.setLeftSpeed(leftMotor);
      motors.setRightSpeed(-rightMotor);
      Serial.println("Turning " + direc);
      delay(50);
    }
    if (pixy.line.barcodes->m_code == 0) // Checks to see if its the barcode associated with a left turn
    {
      direc = "left";
      Serial.println("I see barcode 0");
      motors.setLeftSpeed(-leftMotor);
      motors.setRightSpeed(rightMotor);
      Serial.println("Turning " + direc);
      delay(50);
    }
  }

  if (res <= 0) // If Pixy sees nothing, it will stop. If it was in the process of turning it will continue turning in that direction
  {
    if (direc == "right") // Checks string to see if it previously was in the process of turning right
    {
      Serial.println("Still turning right");
      motors.setLeftSpeed(leftMotor);
      motors.setRightSpeed(-rightMotor);
    }
    if (direc == "left") // Checks string to see if it was previously in the process of turning left
    {
      Serial.println("Still turning left");
      motors.setLeftSpeed(leftMotor);
      motors.setRightSpeed(-rightMotor);
    }
    if (direc == "") // Checks if string is empty, which means that Pixy was not previously turning
    {
      motors.setLeftSpeed(0);
      motors.setRightSpeed(0);
      Serial.print("stop ");
      Serial.println(res);
    }
  }

  if (res & LINE_VECTOR) // If Pixy sees a line, it will correct itself, track, and follow that line. If it was previously turning, it will stop at the line and continue foward.
  {
    error = (int32_t)pixy.line.vectors->m_x1 - (int32_t)X_CENTER; // Corrects itself
    pixy.line.vectors->print();
    headingLoop.update(error);
    left = headingLoop.m_command;
    right = -headingLoop.m_command;

    Serial.println("I see a vector.");

    if (direc == "left" || "right"); // Checks if it was previously turning
    {
      if (pixy.line.vectors->m_x0 > pixy.line.vectors->m_x1) // If it was turning, it stops the motors since Pixy detected a new line and resets string variable
      {
        motors.setLeftSpeed(0);
        motors.setRightSpeed(0);
        direc = "";
        counter == 0;
        Serial.println("DONE");
      }
    }
//problem here, maybe check direction to be empty so it doesn't get confused 
    if (pixy.line.vectors->m_y0 > pixy.line.vectors->m_y1) // Checks if vector is pointing upwards
    {
      if (res & LINE_INTERSECTION) // Checks for and counts intersections
      {
        counter++;
        Serial.println(counter);
        if (counter == 3) // Stops only on the third intersection
        {
          delay(250);
          motors.setLeftSpeed(0);
          motors.setRightSpeed(0);
          counter == 0;
          Serial.println(counter);
          return;
        }
      }
      else // Continue forward
      {
        left += ZUMO_FAST;
        right += ZUMO_FAST;
      }
    }
    else  // If the vector is pointing down, Pixy goes backwards
    {
      left -= ZUMO_SLOW;
      right -= ZUMO_SLOW;
    }
    motors.setLeftSpeed(left);
    motors.setRightSpeed(right);
  }




}
