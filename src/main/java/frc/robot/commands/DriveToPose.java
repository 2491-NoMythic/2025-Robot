// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
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
  DoubleSupplier yMovementSupplierForBarge;
  public DriveToPose(Supplier<PlacementLocations> targetSpot, DrivetrainSubsystem drivetrain, DoubleSupplier yMovementSupplierBarge) {
    this.drivetrain = drivetrain;
    this.targetSpot = targetSpot;
    yMovementSupplierForBarge = yMovementSupplierBarge;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    boolean redAlliance = DriverStation.getAlliance().get() == Alliance.Red;
    switch (targetSpot.get()) {
      case ReefA:
        if(redAlliance) {
          targetPose = ReefARed;
        } else {
          targetPose = ReefABlue;
        }
          break;
      case ReefB:
        if(redAlliance) {
          targetPose = ReefBRed;
        } else {
          targetPose = ReefBBlue;
        }
        break;
      case ReefC:
        if(redAlliance) {
          targetPose = ReefCRed;
        } else {
          targetPose = ReefCBlue;
        } 
        break;
      case ReefD:
        if(redAlliance) {
          targetPose = ReefDRed;
        } else {
          targetPose = ReefDBlue;
        }
        break;
      case ReefE:
        if(redAlliance) {
          targetPose = ReefERed;
        } else {
          targetPose = ReefEBlue;
        }
        break;
      case ReefF:
        if(redAlliance) {
          targetPose = ReefFRed;
        } else {
          targetPose = ReefFBlue;
        }
        break;
      case ReefG:
        if(redAlliance) {
          targetPose = ReefGRed;
        } else {
          targetPose = ReefGBlue;
        }
        break;
      case ReefH:
        if(redAlliance) {
          targetPose = ReefHRed;
        } else {
          targetPose = ReefHBlue;
        }
        break;
      case ReefI:
        if(redAlliance) {
          targetPose = ReefIRed;
        } else {
          targetPose = ReefIBlue;
        }
        break;
      case ReefJ:
        if(redAlliance) {
          targetPose = ReefJRed;
        } else {
          targetPose = ReefJBlue;
        }
        break;
      case ReefK:
        if(redAlliance) {
          targetPose = ReefKRed;
        } else {
          targetPose = ReefKBlue;
        }
        break;
      case ReefL:
        if(redAlliance) {
          targetPose = ReefLRed;
        } else {
          targetPose = ReefLBlue;
        }
        break;
      case Barge:
        if(redAlliance) {
          targetPose = BargePoseRed;
        } else {
          targetPose = BargePoseBlue;
        }
      case Processor:
        if(redAlliance) {
          targetPose = ProcessorPoseRed;
        } else {
          targetPose = ProcessorPoseBlue;
        }
      default:
        targetPose = BargePoseBlue;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(targetPose == BargePoseBlue || targetPose == BargePoseRed) {
      drivetrain.moveTowardsBargePose(yMovementSupplierForBarge);
    } else {
      drivetrain.moveTowardsPose(targetPose);
    }
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
