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
import frc.robot.subsystems.RobotState.ReefOffset;

public class LineUp extends Command {
  /** Creates a new MoveMeters. */
  DrivetrainSubsystem drivetrain;
  double fSensorSpeed;
  double lSensorSpeed;
  double rotSensorSpeed;
  ReefOffset reefOffset;
  boolean sFLtrig;
  boolean sLtrig;
  boolean sRtrig;
  boolean sFRtrig;
  boolean movingLeft;
  boolean notSensed;


  public LineUp(
    DrivetrainSubsystem drivetrain,
    boolean movingLeft,
    boolean sFLtrig,
    boolean sLtrig,
    boolean sRtrig,
    boolean sFRtrig
  ) 
  {
    this.drivetrain = drivetrain;
    this.movingLeft = movingLeft;
     this.sFLtrig  = sFLtrig;
     this.sLtrig = sLtrig;
     this.sRtrig = sRtrig;
     this.sFRtrig = sFRtrig; 
    
      notSensed = false;
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
    reefOffset = RobotState.calcOffset(sFLtrig, sLtrig, sRtrig, sFRtrig);
    switch(reefOffset){
      case TOO_FAR_LEFT:
       if (movingLeft) {
        drivetrain.drive(new ChassisSpeeds(0, -0.2, 0));
      }
      else{
        drivetrain.drive(new ChassisSpeeds(0, -0.3, 0));
      }
      break;

      case ALIGNED_LEFT:
    if(movingLeft) {
      drivetrain.stop();
      drivetrain.pointWheelsInward();
    }
      else {
        drivetrain.drive(new ChassisSpeeds(0, -0.2, 0));
      }   
    break;

      case CENTERED:
        if(movingLeft) {
          drivetrain.drive(new ChassisSpeeds(0, 0.2, 0));
        }
        else {
          drivetrain.drive(new ChassisSpeeds(0, -0.2, 0));
        }   
      break;

      case ALIGNED_RIGHT:
      if(movingLeft) {
        drivetrain.drive(new ChassisSpeeds(0, 0.2, 0));
      }
      else {
        drivetrain.stop();
        drivetrain.pointWheelsInward();
      }   
    break;
      
    case TOO_FAR_RIGHT:
    if(movingLeft) {
      drivetrain.drive(new ChassisSpeeds(0, 0.3, 0));
    }
    else {
      drivetrain.drive(new ChassisSpeeds(0, 0.2, 0));
    }   
  break;

  case NOT_SENSED:
    notSensed = true;
  break;

  case UNKNOWN:
  drivetrain.stop();
  drivetrain.pointWheelsInward();   
break;



       
      


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
    return notSensed;
  }
}
