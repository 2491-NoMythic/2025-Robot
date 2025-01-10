// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.RobotState;

public class LineUpLeft extends Command {
  /** Creates a new MoveMeters. */
  DrivetrainSubsystem drivetrain;
  double fSensorSpeed;
  double lSensorSpeed;
  double rotSensorSpeed;

  public LineUpLeft(
    DrivetrainSubsystem drivetrain, 
    double fSensorSpeed, 
    double lSensorSpeed,
    double rotSensorSpeed
  ) 
  {

    this.drivetrain = drivetrain;
    this.fSensorSpeed = fSensorSpeed;
    this.lSensorSpeed = lSensorSpeed;
    this.rotSensorSpeed = rotSensorSpeed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(
      !RobotState.getInstance().farLeftSensorTriggered 
      ){
        while(!RobotState.getInstance().farLeftSensorTriggered){
      drivetrain.drive(new ChassisSpeeds(fSensorSpeed, lSensorSpeed, rotSensorSpeed));
        }
    }
    else{
      while(RobotState.getInstance().farLeftSensorTriggered){
        drivetrain.drive(new ChassisSpeeds(fSensorSpeed, lSensorSpeed, rotSensorSpeed));
          }
    }
    
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return distance >= m_meters;
    return false;
  }
}
