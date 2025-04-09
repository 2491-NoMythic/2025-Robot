// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.settings.ElevatorEnums;
import frc.robot.settings.PlacementLocations;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.RobotState;

import static frc.robot.settings.Constants.Field.*;

public class DriveToPose extends Command {
  Supplier<PlacementLocations> targetSpot;
  DrivetrainSubsystem drivetrain;
  Pose2d targetPose;
  int cyclesGood;
  DoubleSupplier yMovementSupplierForBarge;
  /**
   * Drives the robot to a given position. 
   * @param targetSpot
   * @param drivetrain
   * @param yMovementSupplierBarge
   */
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
    cyclesGood = 0;
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
        break;
      case Processor:
        if(redAlliance) {
          targetPose = ProcessorPoseRed;
        } else {
          targetPose = ProcessorPoseBlue;
        }
        break;
      default:
        targetPose = BargePoseBlue;
    }
    SmartDashboard.putString("TESTINGPOSE/targeted pose", targetPose.toString());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(targetPose == BargePoseBlue || targetPose == BargePoseRed) {
      drivetrain.moveTowardsBargePose(yMovementSupplierForBarge);
    } else {
      drivetrain.moveTowardsPose(targetPose);
    }
    if(drivetrain.getPositionTargetingError() < 0.015) {
      cyclesGood++;
    } else {
      cyclesGood = 0;
    }
    SmartDashboard.putBoolean("TARGETINGPOSE/isAtRotationTArget", drivetrain.isAtRotationTarget());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(RobotState.getInstance().goForAlgae && !(RobotState.getInstance().deliveringCoralHeight == ElevatorEnums.Reef4 || RobotState.getInstance().deliveringCoralHeight == ElevatorEnums.Reef3)) {
      drivetrain.drive(new ChassisSpeeds(0.7, 0, 0));
    } else {
      drivetrain.pointWheelsInward();
    }
    cyclesGood = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if(DriverStation.isAutonomous()) {
    //   return false;
    // }
    return cyclesGood>3&&drivetrain.isAtRotationTarget();
  }
}
