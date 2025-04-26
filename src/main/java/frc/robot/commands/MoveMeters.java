// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;

public class MoveMeters extends Command {
  /** Creates a new MoveMeters. */
  DrivetrainSubsystem m_drivetrain;

  double m_meters;
  double m_forwardSpeed;
  double m_rightSpeed;
  double m_angleSpeed;
  double startY;
  double startX;
  double whileX;
  double whileY;
  double distance;
  double distanceX;
  double distanceY;
  Pose2d pose;

  public MoveMeters(
      DrivetrainSubsystem drivetrain,
      double meters,
      double forwardSpeed,
      double rightSpeed,
      double angleSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    m_meters = Math.abs(meters);
    m_forwardSpeed = forwardSpeed;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pose = m_drivetrain.getPose();
    startX = pose.getX();
    startY = pose.getY();
    distance = 0;
    if(frc.robot.subsystems.RobotState.getInstance().L1Mode) {
      ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.11, 0, 0);
      switch(frc.robot.subsystems.RobotState.getInstance().L1selectedPosition) {
        case FarLeft:
          chassisSpeeds.vyMetersPerSecond = 0.4;
          m_meters = 0.19;//0.19;
          break;
        case MiddleLeft:
          chassisSpeeds.vyMetersPerSecond = 0.2;
          m_meters = 0.12;//0.1;
          break;
        case MiddleRight:
          chassisSpeeds.vyMetersPerSecond = -0.2;
          m_meters = 0.12;
          break;
        case FarRight:
          chassisSpeeds.vyMetersPerSecond = -0.4;
          m_meters = 0.19;
          break;
      }
      m_drivetrain.moveTowardsRotationTargetRobotRelative(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
      return;
    }
    m_drivetrain.drive(new ChassisSpeeds(m_forwardSpeed, m_rightSpeed, m_angleSpeed));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pose = m_drivetrain.getPose();
    whileX = pose.getX();
    whileY = pose.getY();
    distanceX = (startX - whileX);
    distanceY = (startY - whileY);
    distance = Math.sqrt(Math.pow(distanceX, 2) + Math.pow(distanceY, 2));

    SmartDashboard.putNumber("MOVE1METER/distance x", distanceX);
    SmartDashboard.putNumber("MOVE1METER/distance y", distanceY);
    SmartDashboard.putNumber("MOVE1METER/distance from start", distance);
    SmartDashboard.putNumber("MOVEMETERS/target distance", m_meters);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.pointWheelsInward();
    distance = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return distance >= m_meters;
    return distance >= m_meters;
  }
}
