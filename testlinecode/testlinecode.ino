#include <Pixy2.h>
#include <PIDLoop.h>
#include <ZumoMotors.h>
#include <ZumoBuzzer.h>

#define ZUMO_FAST        200
#define ZUMO_SLOW        150
#define X_CENTER         (pixy.frameWidth/2)
//#define res              (pixy.line.getMainFeatures())
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
  pixy.line.getMainFeatures();

  if (res&LINE_VECTOR) // If Pixy sees a line, it will correct itself, track, and follow that line. If it was previously turning, it will stop at the line and continue foward.
  {
    error = (int32_t)pixy.line.vectors->m_x1 - (int32_t)X_CENTER;
    pixy.line.vectors->print();
    headingLoop.update(error);
    left = headingLoop.m_command;
    right = -headingLoop.m_command;
    motors.setLeftSpeed(right);
    motors.setRightSpeed(left);

    Serial.print("I see a line.");
    if (pixy.line.vectors->m_y0 > pixy.line.vectors->m_y1)
    {
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
        left += ZUMO_FAST;
        right += ZUMO_FAST;
        motors.setLeftSpeed(left);
        motors.setRightSpeed(right);
      }
    }
  }
}
