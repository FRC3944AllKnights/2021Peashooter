#ifndef ELEVATOR_H
#define ELEVATOR_H
#include <iostream>
#include <string>
#include "rev/CANSparkMax.h"
#include "ctre/Phoenix.h"


class Elevator{
    public:
    Elevator();
    void init();
    void Elevate(bool Elevate, bool Down);

    private:
    rev::CANSparkMax ElevatorMotor{5, rev::CANSparkMax::MotorType::kBrushless};

};
#endif