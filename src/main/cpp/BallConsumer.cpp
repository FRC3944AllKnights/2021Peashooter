#include "BallConsumer.h"

BallConsumer::BallConsumer(){
}

void BallConsumer::init(){

}

void BallConsumer::Retractor(double RetractUp, double RetractDown){
    if(RetractUp) {
        if(RetractorSwitchUp.Get()){
            RetractorMotor.Set(ControlMode::PercentOutput, 0);
        }
        else {
            RetractorMotor.Set(ControlMode::PercentOutput, -0.4);
        }
    }

    else if(true) {
        if(RetractorSwitchDown.Get()){
            RetractorMotor.Set(ControlMode::PercentOutput, 0);
        }
        else {
            RetractorMotor.Set(ControlMode::PercentOutput, 0.4);
        }
    }

    else{
        RetractorMotor.Set(ControlMode::PercentOutput, 0);
    }

    if (++_loops >= 20) {
		_loops = 0;
        _sb.append("\tUp Switch");
	    _sb.append(std::to_string(RetractorSwitchUp.Get()));
        _sb.append("\tDown Switch");
	    _sb.append(std::to_string(RetractorSwitchDown.Get()));

        

		//printf("%s\n",_sb.c_str());
        _sb.clear();
	}
}

void BallConsumer::Intake(bool suck, bool expel){
    if(suck){
         IntakeMotor.Set(-0.8);

    }
    else if(expel){
        IntakeMotor.Set(0.4);
    }
    else{
        IntakeMotor.Set(0.0);
    }
}