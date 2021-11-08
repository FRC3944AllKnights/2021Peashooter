#include "Elevator.h"

Elevator::Elevator(){
}

void Elevator::init(){

}

void Elevator::Elevate(bool elevate, bool down){
    if(elevate) {
        ElevatorMotor.Set(-0.6);

    }
    else if(down) {
        ElevatorMotor.Set(0.6);
    }
    else {
        ElevatorMotor.Set(0);   
    }
}