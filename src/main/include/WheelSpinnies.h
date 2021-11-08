#ifndef WHEELSPINNIES_H
#define WHEELSPINNIES_H
#include <iostream>
#include <string>

#include "rev/CANSparkMax.h"
#include "ctre/Phoenix.h"
#include "frc/Servo.h"

class WheelSpinnies{
    public:
        WheelSpinnies();
        void init();
        void MoveServo(bool green, bool yellow, bool blue, bool red);
        void spinrev(bool revUp, bool holdBall); // bool up1, bool down1, bool up2, bool down2, bool revUp);
        std::string _sb;
	    int _loops = 0;
        int velocity1 = 4000;
        int velocity2 = 1000;
        double bottomMotor = 0;
        double topMotor = 0;
        bool up1pressed = false;
        bool up2pressed = false;
        bool down1pressed = false;
        bool down2pressed = false;
        double kP = 5e-4, kI = 5e-7, kD = 1e-8, kIz = 0, kFF = 0, kMaxOutput = 1.0, kMinOutput = -1.0;
        const double MaxRPM = 5700;


    
    private:
        WPI_TalonSRX feeder{14};

        rev::CANSparkMax spinnymotorLeft{7, rev::CANSparkMax::MotorType::kBrushless};
        rev::CANPIDController spinnypidleft = spinnymotorLeft.GetPIDController();
        rev::CANEncoder spinnyencoderleft = spinnymotorLeft.GetEncoder();

        rev::CANSparkMax spinnymotorRight{8, rev::CANSparkMax::MotorType::kBrushless};
        rev::CANPIDController spinnypidright = spinnymotorRight.GetPIDController();
        rev::CANEncoder spinnyencoderright = spinnymotorRight.GetEncoder();

        frc::Servo ShooterServoLeft {1};
        frc::Servo ShooterServoRight {2};
};
#endif