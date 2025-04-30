#include "Aria/Aria.h"

class Joystick {
public:
  Joystick() {
    joy_handler.init();
    joy_handler.setSpeeds(50, 700);
  }

  bool joy_connected() const { return joy_handler.haveJoystick(); }

  ArActionDesired *fire(ArActionDesired current_desired) {
    printf("x:%6.1f, y:%6.1f, tth:%6.1f, vel:%7.1f mpacs: %3d\n",
           myRobot->getX(), myRobot->getY(), myRobot->getTh(),
           myRobot->getVel(), myRobot->getMotorPacCount());
    if (joy_connected() * *joy_handler.getButton(1) ||
        joy_handler.getButton(2)) {
      int rot, trans;

      joy_handler.getAdjusted(&rot, &trans);

      desired.setVel(trans);
      desired.setDeltaHeading(-rot);

      return &desired;
    } else {
      desired.setVel(0);
      desired.setDeltaHeading(0);
      return &desired;
    }
  }

private:
  ArJoyHandler joy_handler;
  ArActionDesired desired;
};
