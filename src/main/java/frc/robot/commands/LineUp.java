// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.settings.ReefOffsetEnums;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.RobotState;

public class LineUp extends Command {
  /** Creates a new MoveMeters. */
  DrivetrainSubsystem drivetrain;
  double fSensorSpeed;
  double lSensorSpeed;
  double rotSensorSpeed;
  ReefOffsetEnums reefOffset;
  boolean sFLtrig;
  boolean sLtrig;
  boolean sRtrig;
  boolean sFRtrig;
  BooleanSupplier movingLeft;
  boolean notSensed;
  boolean finished;

/**
 * this command will align your robot side-to-side on one of the reef poles. 
 * @param drivetrain
 * @param movingLeft true if you are aligning on the left pole, false, if you are aligning on the right pole
 */
  public LineUp(DrivetrainSubsystem drivetrain, BooleanSupplier movingLeft) {
    this.drivetrain = drivetrain;
    this.movingLeft = movingLeft;
    addRequirements(drivetrain);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    notSensed = false;
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("LINEUP/notSensed", notSensed);
    SmartDashboard.putBoolean("LINEUP/movingLeft", movingLeft.getAsBoolean());
  //update sensor readings with Robot STate (which is updated periodically by DistanceSensors.java)
    sFLtrig = RobotState.getInstance().farLeftSensorTriggered;
    sLtrig = RobotState.getInstance().middleLeftSensorTriggered;
    sRtrig = RobotState.getInstance().middleRightSensorTriggered;
    sFRtrig = RobotState.getInstance().farRightSensorTriggered;
    
  //moves the robot based on what the Robot State declares is our location, relative to the reef lineup
    reefOffset = RobotState.getInstance().reefOffset;
    SmartDashboard.putString("sensing case", reefOffset.toString());
    switch(reefOffset){
      case TOO_FAR_LEFT:
        if (movingLeft.getAsBoolean()) {
          drivetrain.drive(new ChassisSpeeds(0, -0.2, 0));
        } else{
          drivetrain.drive(new ChassisSpeeds(0, -0.3, 0));
        }
        break;

      case ALIGNED_LEFT:
        if(movingLeft.getAsBoolean()) {
           drivetrain.stop();
          drivetrain.pointWheelsInward();
          finished = true;
        } else {
          drivetrain.drive(new ChassisSpeeds(0, 0.2, 0));
        }   
        break;

      case CENTERED:
        if(movingLeft.getAsBoolean()) {
          drivetrain.drive(new ChassisSpeeds(0, -0.2, 0));
        } else {
          drivetrain.drive(new ChassisSpeeds(0, 0.2, 0));
        }   
        break;

      case ALIGNED_RIGHT:
        if(movingLeft.getAsBoolean()) {
          drivetrain.drive(new ChassisSpeeds(0, -0.2, 0));
        } else {
          drivetrain.stop();
          drivetrain.pointWheelsInward();
          finished = true;
        }   
        break;
      
      case TOO_FAR_RIGHT:
        if(movingLeft.getAsBoolean()) {
          drivetrain.drive(new ChassisSpeeds(0, 0.3, 0));
        } else {
          drivetrain.drive(new ChassisSpeeds(0, 0.2, 0));
        }   
        break;

      case NOT_SENSED:
        notSensed = true;
        break;

      case UNKNOWN:
        drivetrain.stop();
        drivetrain.pointWheelsInward();
        finished = true;   
        break;  
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("command ended by condtions"+finished+notSensed);
    drivetrain.stop();
    finished = false;
    notSensed = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished||notSensed;
  }
}
