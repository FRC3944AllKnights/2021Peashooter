#include "Encoder.h"

Encoder::Encoder(){

}

void Encoder::init(bool isRed){ 
    //init motors
    LOCDrive.frontLeft.RestoreFactoryDefaults();
    frontLeftPID.SetP(kP);
    frontLeftPID.SetI(kI);
    frontLeftPID.SetD(kD);
    frontLeftPID.SetIZone(kIz);
    frontLeftPID.SetFF(kFF);
    frontLeftPID.SetOutputRange(kMinOutput, kMaxOutput);

    LOCDrive.rearLeft.RestoreFactoryDefaults();
    rearLeftPID.SetP(kP);
    rearLeftPID.SetI(kI);
    rearLeftPID.SetD(kD);
    rearLeftPID.SetIZone(kIz);
    rearLeftPID.SetFF(kFF);
    rearLeftPID.SetOutputRange(kMinOutput, kMaxOutput);

    LOCDrive.frontRight.RestoreFactoryDefaults();
    frontRightPID.SetP(kP);
    frontRightPID.SetI(kI);
    frontRightPID.SetD(kD);
    frontRightPID.SetIZone(kIz);
    frontRightPID.SetFF(kFF);
    frontRightPID.SetOutputRange(kMinOutput, kMaxOutput);

    LOCDrive.rearRight.RestoreFactoryDefaults();
    rearRightPID.SetP(kP);
    rearRightPID.SetI(kI);
    rearRightPID.SetD(kD);
    rearRightPID.SetIZone(kIz);
    rearRightPID.SetFF(kFF);
    rearRightPID.SetOutputRange(kMinOutput, kMaxOutput);

    //init encoders
    encoder1.SetDistancePerPulse(ticks2meters);
    encoder1.SetReverseDirection(true);
    encoder2.SetDistancePerPulse(ticks2meters);

    //init trajectory
    GenerateTrajectory();

    //init odom
    encoder1.Reset();
    encoder2.Reset();
    gyroAngle = {units::degree_t(0)};
    pose = {units::meter_t(0), units::meter_t(0), gyroAngle};
    gyroAngle = {units::degree_t(-ahrs.GetAngle())};
    if (isRed){
      odom.ResetPosition(redTrajectory.InitialPose(), gyroAngle);
    }else{
        odom.ResetPosition(blueTrajectory.InitialPose(), gyroAngle);
    }
    m_timer.Start();

    bounce1go = false;
    bounce2go = true;
    bounce3go = false;
    bounce4go = false;
    m_timer.Reset();
}

void Encoder::getOdom(){
    /* These functions are compatible w/the WPI Gyro Class */
    frc::SmartDashboard::PutNumber(  "IMU_TotalYaw",         ahrs.GetAngle());
    frc::SmartDashboard::PutNumber(  "IMU_YawRateDPS",       ahrs.GetRate());
    gyroAngle = {units::degree_t(-ahrs.GetAngle())};
    pose = odom.Update(gyroAngle, units::meter_t(encoder1.GetDistance()), units::meter_t(encoder2.GetDistance()));

    if (++_loops >= 20) {
		_loops = 0;
        _sb.append("\tX: ");
	    _sb.append(std::to_string(pose.Translation().X().to<double>()));
        _sb.append(" Y: ");
        _sb.append(std::to_string(pose.Translation().Y().to<double>()));
        _sb.append(" Theta: ");
        _sb.append(std::to_string(pose.Rotation().Degrees().to<double>()));
        _sb.append(" vLeft ");
        _sb.append(std::to_string(vLeft));
        _sb.append(" vRight ");
        _sb.append(std::to_string(vRight));
		printf("%s\n",_sb.c_str());
        _sb.clear();
	}    
}

void Encoder::VelocityControl(units::meters_per_second_t x, units::radians_per_second_t theta){
    auto speeds = m_kinematics.ToWheelSpeeds({x, 0_mps, theta});
    vLeft = speeds.left.to<double>()/0.4775*gearRatio*60;
    vRight = -speeds.right.to<double>()/0.4775*gearRatio*60;

    if (vLeft == 0 && vRight == 0){
        LOCDrive.frontLeft.Set(0.0);
        LOCDrive.rearLeft.Set(0.0);
        LOCDrive.frontRight.Set(0.0);
        LOCDrive.rearRight.Set(0.0);
    } else{
        frontLeftPID.SetReference(vLeft, rev::ControlType::kVelocity);
        rearLeftPID.SetReference(vLeft, rev::ControlType::kVelocity);
        frontRightPID.SetReference(vRight, rev::ControlType::kVelocity);
        rearRightPID.SetReference(vRight, rev::ControlType::kVelocity);
    }
}

void Encoder::GenerateTrajectory(){
    bounceConfig.SetReversed(false);
    bounce1Trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
           frc::Pose2d(30_in, 90_in, 0_deg),
           {frc::Translation2d(70_in, 96_in)}, 
           frc::Pose2d(90_in, 150_in, 90_deg), 
           bounceConfig);

    bounceConfig.SetReversed(true);
    bounce2Trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
           frc::Pose2d(90_in, 150_in, 90_deg),
           {frc::Translation2d(90_in, 120_in),
           frc::Translation2d(100_in, 90_in),
           frc::Translation2d(120_in, 60_in),
           frc::Translation2d(120_in, 30_in),
           frc::Translation2d(130_in, 15_in),
           frc::Translation2d(150_in, 30_in),
           frc::Translation2d(170_in, 60_in), //this is where danny left us
           frc::Translation2d(180_in, 90_in),
           frc::Translation2d(180_in, 120_in)},
           frc::Pose2d(180_in, 150_in, 270_deg), 
           bounceConfig);

    bounceConfig.SetReversed(false);
    bounce3Trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
           frc::Pose2d(180_in, 150_in, 270_deg), 
           {frc::Translation2d(180_in, 60_in), 
           frc::Translation2d(195_in, 40_in),
           frc::Translation2d(230_in, 30_in),
           frc::Translation2d(250_in, 40_in)},
           frc::Pose2d(270_in, 150_in, 90_deg), 
           bounceConfig);

    bounceConfig.SetReversed(true);
    bounce4Trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
           frc::Pose2d(270_in, 150_in, 90_deg),
           {frc::Translation2d(280_in, 96_in)}, 
           frc::Pose2d(330_in, 90_in, 180_deg), 
           bounceConfig);
}

void Encoder::FollowBounceTrajectory(){
    getOdom();
    if (bounce1go){
        if (m_timer.Get() < bounce1Trajectory.TotalTime()) {
            // Get the desired pose from the trajectory.
            auto desiredPose = bounce1Trajectory.Sample(m_timer.Get());

            // Get the reference chassis speeds from the Ramsete Controller.
            auto refChassisSpeeds =
                italy.Calculate(pose, desiredPose);

            // Set the linear and angular speeds.
            ramseteOutputX = desiredPose.pose.Translation().X().to<double>();
            ramseteOutputTheta = refChassisSpeeds.omega.to<double>();
            VelocityControl(refChassisSpeeds.vx, refChassisSpeeds.omega);
        } else {
            bounce1go = false;
            bounce2go = false;
            bounce3go = false;
            bounce4go = false;
            m_timer.Reset();
            VelocityControl(0_mps, 0_rad_per_s);
        }
    }

    if (bounce2go){
        if (m_timer.Get() < bounce2Trajectory.TotalTime()) {
            // Get the desired pose from the trajectory.
            auto desiredPose = bounce2Trajectory.Sample(m_timer.Get());

            // Get the reference chassis speeds from the Ramsete Controller.
            auto refChassisSpeeds =
                italy.Calculate(pose, desiredPose);

            // Set the linear and angular speeds.
            ramseteOutputX = desiredPose.pose.Translation().X().to<double>();
            ramseteOutputTheta = refChassisSpeeds.omega.to<double>();
            VelocityControl(refChassisSpeeds.vx, refChassisSpeeds.omega);
        } else {
            bounce1go = false;
            bounce2go = false;
            bounce3go = false;//true;
            bounce4go = false;
            m_timer.Reset();
            VelocityControl(0_mps, 0_rad_per_s);
        }
    }

    if (bounce3go){
        if (m_timer.Get() < bounce3Trajectory.TotalTime()) {
            // Get the desired pose from the trajectory.
            auto desiredPose = bounce3Trajectory.Sample(m_timer.Get());

            // Get the reference chassis speeds from the Ramsete Controller.
            auto refChassisSpeeds =
                italy.Calculate(pose, desiredPose);

            // Set the linear and angular speeds.
            ramseteOutputX = desiredPose.pose.Translation().X().to<double>();
            ramseteOutputTheta = refChassisSpeeds.omega.to<double>();
            VelocityControl(refChassisSpeeds.vx, refChassisSpeeds.omega);
        } else {
            bounce1go = false;
            bounce2go = false;
            bounce3go = false;
            bounce4go = false;
            m_timer.Reset();
            VelocityControl(0_mps, 0_rad_per_s);
        }
    }

    if (bounce4go){
        if (m_timer.Get() < bounce4Trajectory.TotalTime()) {
            // Get the desired pose from the trajectory.
            auto desiredPose = bounce4Trajectory.Sample(m_timer.Get());

            // Get the reference chassis speeds from the Ramsete Controller.
            auto refChassisSpeeds =
                italy.Calculate(pose, desiredPose);

            // Set the linear and angular speeds.
            ramseteOutputX = desiredPose.pose.Translation().X().to<double>();
            ramseteOutputTheta = refChassisSpeeds.omega.to<double>();
            VelocityControl(refChassisSpeeds.vx, refChassisSpeeds.omega);
        } else {
            bounce1go = false;
            bounce2go = false;
            bounce3go = false;
            bounce4go = false;
            m_timer.Reset();
            VelocityControl(0_mps, 0_rad_per_s);
        }
    }
}

void Encoder::FollowTrajectory(bool isRed){
    getOdom();
    if (isRed){
       if (m_timer.Get() < redTrajectory.TotalTime()) {
      // Get the desired pose from the trajectory.
      auto desiredPose = redTrajectory.Sample(m_timer.Get());

      // Get the reference chassis speeds from the Ramsete Controller.
      auto refChassisSpeeds =
          italy.Calculate(pose, desiredPose);

      // Set the linear and angular speeds.
      ramseteOutputX = desiredPose.pose.Translation().X().to<double>();
      ramseteOutputTheta = refChassisSpeeds.omega.to<double>();
      VelocityControl(refChassisSpeeds.vx, refChassisSpeeds.omega);
      } else {
        VelocityControl(0_mps, 0_rad_per_s);
      }
   } else{
              if (m_timer.Get() < blueTrajectory.TotalTime()) {
      // Get the desired pose from the trajectory.
      auto desiredPose = blueTrajectory.Sample(m_timer.Get());

      // Get the reference chassis speeds from the Ramsete Controller.
      auto refChassisSpeeds =
          italy.Calculate(pose, desiredPose);

      // Set the linear and angular speeds.
      ramseteOutputX = desiredPose.pose.Translation().X().to<double>();
      ramseteOutputTheta = refChassisSpeeds.omega.to<double>();
      VelocityControl(refChassisSpeeds.vx, refChassisSpeeds.omega);
      } else {
        VelocityControl(0_mps, 0_rad_per_s);
      }
   }
}

void Encoder::DriveAndTurn(){
    while (pose.Translation().X().to<double>() < 2.438){
        frontLeftPID.SetReference(4500, rev::ControlType::kVelocity);
        rearLeftPID.SetReference(4500, rev::ControlType::kVelocity);
        frontRightPID.SetReference(-4500, rev::ControlType::kVelocity);
        rearRightPID.SetReference(-4500, rev::ControlType::kVelocity);
        getOdom();
    }
        frontLeftPID.SetReference(0, rev::ControlType::kVelocity);
        rearLeftPID.SetReference(0, rev::ControlType::kVelocity);
        frontRightPID.SetReference(0, rev::ControlType::kVelocity);
        rearRightPID.SetReference(0, rev::ControlType::kVelocity);
}