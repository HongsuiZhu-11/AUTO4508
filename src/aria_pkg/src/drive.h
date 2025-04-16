#include "Aria/Aria.h"

class Drive {
public:
    Drive(ArRobot* robot, double linear_scale = 500.0, double angular_scale = 50.0)
        : myRobot(robot), 
          linear_scale_(linear_scale), 
          angular_scale_(angular_scale),
          desired_linear_(0.0),
          desired_angular_(0.0) 
    {
        // Validate robot pointer
        if (!robot) {
            ArLog::log(ArLog::Terse, "Drive: Invalid robot pointer!");
            Aria::exit(1);
        }
    }

    // Set both linear and angular velocities (normalized values)
    void drive(double linear, double angular) {
        desired_linear_ = linear;
        desired_angular_ = angular;
        applySpeeds();
    }

    // Drive straight with specified linear velocity (angular = 0)
    void driveForward(double speed) {
        drive(speed, 0.0);
    }

    // Set angular velocity while maintaining current linear speed
    void driveAtAngle(double angular) {
        desired_angular_ = angular;
        applySpeeds();
    }

    // Stop all motion
    void stop() {
        drive(0.0, 0.0);
    }

    // Individual setters
    void setLinear(double speed) {
        desired_linear_ = speed;
        applySpeeds();
    }

    void setAngular(double angular) {
        desired_angular_ = angular;
        applySpeeds();
    }

private:
    void applySpeeds() {
        myRobot->lock();
        myRobot->setVel(desired_linear_ * linear_scale_);
        myRobot->setRotVel(desired_angular_ * angular_scale_);
        myRobot->unlock();
    }

    ArRobot* myRobot;
    double linear_scale_;
    double angular_scale_;
    double desired_linear_;
    double desired_angular_;
};