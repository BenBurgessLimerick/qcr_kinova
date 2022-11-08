#pragma once

class KinovaJoint {
  public:  
    KinovaJoint() : position(0), velocity(0), effort(0), velocity_command(0) {}
    double position;
    double velocity;
    double effort;
    double position_command;
    double velocity_command;
    double effort_command;
};