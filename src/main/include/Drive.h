#ifndef DRIVE_H
#define DRIVE_H
#include <frc/drive/DifferentialDrive.h>
#include <frc/SpeedControllerGroup.h>
#include <ctre/Phoenix.h>
#include "rev/CANSparkMax.h"

class Drive{
    public:
      Drive();
      void init();
      void Arcade(double x, double y);
      rev::CANSparkMax frontLeft{4, rev::CANSparkMax::MotorType::kBrushless};
      rev::CANSparkMax rearLeft{2, rev::CANSparkMax::MotorType::kBrushless};
      rev::CANSparkMax frontRight{3, rev::CANSparkMax::MotorType::kBrushless};
      rev::CANSparkMax rearRight{1, rev::CANSparkMax::MotorType::kBrushless};

    private:
    frc::SpeedControllerGroup Left{frontLeft, rearLeft};
    frc::SpeedControllerGroup Right{frontRight, rearRight};
    frc::DifferentialDrive m_robotDrive{Left, Right};
};

#endif