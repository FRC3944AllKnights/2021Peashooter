#include "WheelSpinnies.h"

WheelSpinnies::WheelSpinnies(){
}

void WheelSpinnies::init(){
 spinnymotorLeft.RestoreFactoryDefaults();
    
    // set PID coefficients
    spinnypidleft.SetP(kP);
    spinnypidleft.SetI(kI);
    spinnypidleft.SetD(kD);
    spinnypidleft.SetIZone(kIz);
    spinnypidleft.SetFF(kFF);
    spinnypidleft.SetOutputRange(kMinOutput, kMaxOutput);

 spinnymotorRight.RestoreFactoryDefaults();
    
    // set PID coefficients
    spinnypidright.SetP(kP);
    spinnypidright.SetI(kI);
    spinnypidright.SetD(kD);
    spinnypidright.SetIZone(kIz);
    spinnypidright.SetFF(kFF);
    spinnypidright.SetOutputRange(kMinOutput, kMaxOutput);
}

void WheelSpinnies::spinrev(bool revUp, bool holdBall){ //bool up1, bool down1, bool up2, bool down2, bool revUp){

/*
    if (up1 && up1pressed == false){
        up1pressed = true;
        velocity1 += 50;
    }
    else if (up1 == false && up1pressed){
        up1pressed = false;
    }

    if (down1 && down1pressed == false){
        down1pressed = true;
        velocity1 -= 50;
    }
    else if (down1 == false && down1pressed){
        down1pressed = false;
    }

    if (up2 && up2pressed == false){
        up2pressed = true;
        velocity2 += 50;
    }
    else if (up2 == false && up2pressed){
        up2pressed = false;
    }

    if (down2 && down2pressed == false){
        down2pressed = true;
        velocity2 -= 50;
    }
    else if (down2 == false && down2pressed){
        down2pressed = false;
    }
*/
    if (revUp){
        spinnypidright.SetReference(velocity1, rev::ControlType::kVelocity);
        spinnypidleft.SetReference(-velocity2, rev::ControlType::kVelocity);
        if (bottomMotor >= (velocity1 - 200) && bottomMotor <= (velocity1 + 200)){
            feeder.Set(ControlMode::PercentOutput, -1);
        }
        else{
            feeder.Set(ControlMode::PercentOutput, 0);
        }
    }
    //else if(holdBall){
    //    feeder.Set(ControlMode::PercentOutput, -0.3);
    //}
    else{
        spinnymotorRight.Set(0);
        spinnymotorLeft.Set(0);
        feeder.Set(ControlMode::PercentOutput, 0);
    }

    if (++_loops >= 20) {
        bottomMotor = spinnyencoderright.GetVelocity();
		_loops = 0;
        _sb.append("\tV1:");
	    _sb.append(std::to_string(velocity1));
	    _sb.append("\tspd1:");
	    _sb.append(std::to_string(spinnyencoderright.GetVelocity()));

        _sb.append("\tV2:");
	    _sb.append(std::to_string(velocity2));
	    _sb.append("\tspd2:");
	    _sb.append(std::to_string(spinnyencoderleft.GetVelocity()));

		//printf("%s\n",_sb.c_str());
        _sb.clear();
	}
}

void WheelSpinnies::MoveServo(bool green, bool yellow, bool blue, bool red){
    //max 0.8 min 0.2
    if(green){
        ShooterServoRight.Set(0.2);
        ShooterServoLeft.Set(0.2);
        velocity1 = 4000;
        velocity2 = 600;
    }
    
    else if(yellow){
        ShooterServoRight.Set(0.52);
        ShooterServoLeft.Set(0.52);
        velocity1 = 4000;
        velocity2 = 1000;
    }

    else if(blue){
        ShooterServoRight.Set(0.69);
        ShooterServoLeft.Set(0.69);
        velocity1 = 4000;
        velocity2 = 1000;
    }

    else if(red){
        ShooterServoRight.Set(0.71);
        ShooterServoLeft.Set(0.71);
        velocity1 = 4000;
        velocity2 = 1000;
    }

    
}
    