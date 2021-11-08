/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <thread> 

#include <frc/TimedRobot.h>
#include <frc/Joystick.h>
#include <frc/drive/DifferentialDrive.h>
#include "frc/smartdashboard/SmartDashboard.h"
#include "frc/I2C.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "AHRS.h"
#include <frc/SerialPort.h>


#include "Drive.h"
#include "WheelSpinnies.h"
#include "Turret.h"
//#include "colormatch.h"
#include "BallConsumer.h"
#include "Elevator.h"
#include "Encoder.h"
#include "JeVois.h"

#include <cstring>
#include <string>
#include <iostream>

/** This is a test
 * This is a demo program showing how to use Mecanum control with the
 * MecanumDrive class.
 */
class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override {
    // Invert the left side motors. You may need to change or remove this to
    // match your robot.
    LOCDrive.init();
    Shoot.init();
    //Sense.init();
    //Eat.init();
    BallTrack.init();
  }

  void AutonomousInit() override {
    isA = BallTrack.PathSelector();
    Encode.init(isA);
    //Encode.DriveAndTurn();
  }

  void AutonomousPeriodic() override {
    Encode.FollowTrajectory(isA);
    Eat.Intake(true, false);
  }

  void TeleopInit() override {
    //Encode.init(isA);
  }

  void TeleopPeriodic() override {
    /* Use the joystick X axis for lateral movement, Y axis for forward
     * movement, and Z axis for rotation.
     */
    LOCDrive.Arcade(-m_stick.GetRawAxis(1), m_stick.GetRawAxis(2));

    Eat.Retractor(m_stick.GetRawButton(12), m_stick.GetRawButton(11));
    Eat.Intake(m_stick.GetRawButton(7), m_stick.GetRawButton(8));
    Encode.getOdom();

    Elevate.Elevate(m_stick.GetRawButton(9), m_stick.GetRawButton(10));

    std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    double Xoffset = table->GetNumber("tx",0.0);
    double Yoffset = table->GetNumber("ty",0.0);
    double targetArea = table->GetNumber("ta",0.0);
    double targetSkew = table->GetNumber("ts",0.0);

    Susan.smartMan(m_stick.GetRawButton(12), m_stick.GetRawButton(11), m_stick.GetRawButton(2), Xoffset, Yoffset, targetSkew);
    Shoot.spinrev(m_stick.GetRawButton(1), m_stick.GetRawButton(9)); // m_stick.GetRawButton(8), m_stick.GetRawButton(7), m_stick.GetRawButton(10), m_stick.GetRawButton(9), m_stick.GetRawButton(1));
    Shoot.MoveServo(m_stick.GetRawButton(3), m_stick.GetRawButton(4), m_stick.GetRawButton(5), m_stick.GetRawButton(6));
  }

 private:
 //this is where to change variables
  static constexpr int kJoystickChannel = 0;

  Drive LOCDrive;
  Turret Susan;
  WheelSpinnies Shoot;
  //colormatch Sense;
  BallConsumer Eat;
  Elevator Elevate;
  Encoder Encode;
  jevois BallTrack;
  frc::Joystick m_stick{kJoystickChannel};
  bool isA;

};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif