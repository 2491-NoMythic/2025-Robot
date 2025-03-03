// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.settings.PlacementLocations;
import frc.robot.subsystems.DrivetrainSubsystem;
import static frc.robot.settings.Constants.Field.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToPose extends Command {
  /** Creates a new DriveToPose. */
  Supplier<PlacementLocations> targetSpot;
  DrivetrainSubsystem drivetrain;
  Pose2d targetPose;
  public DriveToPose(Supplier<PlacementLocations> targetSpot, DrivetrainSubsystem drivetrain) {
    this.drivetrain = drivetrain;
    this.targetSpot = targetSpot;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch (targetSpot.get()) {
      case ReefA:
        if(DriverStation.getAlliance().get() == Alliance.Red) {
          targetPose = ReefARed;
        } else {
          targetPose = ReefABlue;
        }
          break;
      case ReefB:
        if(DriverStation.getAlliance().get() == Alliance.Red) {
          targetPose = ReefBRed;
        } else {
          targetPose = ReefBBlue;
        }
        break;
      case ReefC:
        if(DriverStation.getAlliance().get() == Alliance.Red) {
          targetPose = ReefCRed;
        } else {
          targetPose = ReefCBlue;
        } 
        break;
      case ReefD:
        if(DriverStation.getAlliance().get() == Alliance.Red) {
          targetPose = ReefDRed;
        } else {
          targetPose = ReefDBlue;
        }
        break;
      case ReefE:
        if(DriverStation.getAlliance().get() == Alliance.Red) {
          targetPose = ReefERed;
        } else {
          targetPose = ReefEBlue;
        }
        break;
      case ReefF:
        if(DriverStation.getAlliance().get() == Alliance.Red) {
          targetPose = ReefFRed;
        } else {
          targetPose = ReefFBlue;
        }
        break;
      case ReefG:
        if(DriverStation.getAlliance().get() == Alliance.Red) {
          targetPose = ReefGRed;
        } else {
          targetPose = ReefGBlue;
        }
        break;
      case ReefH:
        if(DriverStation.getAlliance().get() == Alliance.Red) {
          targetPose = ReefHRed;
        } else {
          targetPose = ReefHBlue;
        }
        break;
      case ReefI:
        if(DriverStation.getAlliance().get() == Alliance.Red) {
          targetPose = ReefIRed;
        } else {
          targetPose = ReefIBlue;
        }
        break;
      case ReefJ:
        if(DriverStation.getAlliance().get() == Alliance.Red) {
          targetPose = ReefJRed;
        } else {
          targetPose = ReefJBlue;
        }
        break;
      case ReefK:
        if(DriverStation.getAlliance().get() == Alliance.Red) {
          targetPose = ReefKRed;
        } else {
          targetPose = ReefKBlue;
        }
        break;
      case ReefL:
        if(DriverStation.getAlliance().get() == Alliance.Red) {
          targetPose = ReefLRed;
        } else {
          targetPose = ReefLBlue;
        }
        break;
      case Barge:
        if(DriverStation.getAlliance().get() == Alliance.Red) {
          targetPose = BargePoseRed;
        } else {
          targetPose = BargePoseBlue;
        }
      default:
        targetPose = BargePoseBlue;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.moveTowardsPose(targetPose);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drivetrain.getPositionTargetingError() < 0.01;
  }
}
