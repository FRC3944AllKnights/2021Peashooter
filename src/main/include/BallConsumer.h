#ifndef BALLCONSUMER_H
#define BALLCONSUMER_H
#include <iostream>
#include <string>
#include <frc/DigitalInput.h>
#include "rev/CANSparkMax.h"

#include "ctre/Phoenix.h"

class BallConsumer{
    public:
        BallConsumer();
        bool IntakeRetractorUp;
        bool IntakeRetractorDown;
        bool SpoolUp;
        bool SpoolDown;
        void Retractor(double RetractUp, double RetractDown);
        void init();
        void Intake(bool suck, bool expel);

    private: 
        std::string _sb;
        int _loops = 0;

        WPI_TalonSRX RetractorMotor{19};
        rev::CANSparkMax IntakeMotor{10, rev::CANSparkMax::MotorType::kBrushless};
        frc::DigitalInput RetractorSwitchUp{2};
        frc::DigitalInput RetractorSwitchDown{1};
};
#endif