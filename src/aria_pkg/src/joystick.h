#include "Aria/Aria.h"

class Joystick {
public:
    Joystick(double dead_zone = 0.1) 
        : dead_zone_(dead_zone), 
          connected_(false),
          myJoyHandler()
    {
        if(myJoyHandler.init()) {
            connected_ = true;
            myJoyHandler.setSpeeds(100, 100); // Default axis scaling
            ArLog::log(ArLog::Normal, "Joystick initialized successfully");
        } else {
            ArLog::log(ArLog::Terse, "Joystick initialization failed");
        }
    }

    // Get normalized axes (-1.0 to 1.0) with dead zone applied
    void getAxes(double& x, double& y) {
        if(!connected_) {
            x = y = 0.0;
            return;
        }

        int rawX, rawY;
        myJoyHandler.getAdjusted(&rawX, &rawY);
        
        // Convert to normalized values
        x = clamp((rawX / 100.0), -1.0, 1.0);
        y = clamp((rawY / 100.0), -1.0, 1.0);

        // Apply dead zone
        applyDeadZone(x);
        applyDeadZone(y);
    }

    // Get button states (ARIA supports up to 16 buttons)
    bool getButton(int button_num) {
        return connected_ ? myJoyHandler.getButton(button_num) : false;
<<<<<<< HEAD
<<<<<<< HEAD
    }

    // Get all button states as bitmask
    unsigned int getButtons() {
        return connected_ ? myJoyHandler.getButton() : 0;
    }
=======
    } 
>>>>>>> 85dbbc9 (joystick: fix get button to use correct function)
=======
    } 
>>>>>>> 85dbbc9 (joystick: fix get button to use correct function)

    bool hasJoystick() const { return connected_; }

private:
    ArJoyHandler myJoyHandler;
    bool connected_;
    double dead_zone_;

    void applyDeadZone(double& value) {
        if(fabs(value) < dead_zone_) {
            value = 0.0;
        } else {
            // Rescale remaining values to 0-1 range
            value = copysign((fabs(value) - dead_zone_) / (1.0 - dead_zone_), value);
        }
    }

    double clamp(double value, double min, double max) {
        return (value < min) ? min : (value > max) ? max : value;
    }
<<<<<<< HEAD
<<<<<<< HEAD
};
=======
};
>>>>>>> 85dbbc9 (joystick: fix get button to use correct function)
=======
};
>>>>>>> 85dbbc9 (joystick: fix get button to use correct function)
