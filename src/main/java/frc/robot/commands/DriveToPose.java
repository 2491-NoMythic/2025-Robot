// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
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
        targetPose = ReefA;
        break;
      case ReefB:
        targetPose = ReefB;
        break;
      case ReefC:
        targetPose = ReefC;
        break;
      case ReefD:
        targetPose = ReefD;
        break;
      case ReefE:
        targetPose = ReefE;
        break;
      case ReefF:
        targetPose = ReefF;
        break;
      case ReefG:
        targetPose = ReefG;
        break;
      case ReefH:
        targetPose = ReefH;
        break;
      case ReefI:
        targetPose = ReefI;
        break;
      case ReefJ:
        targetPose = ReefJ;
        break;
      case ReefK:
        targetPose = ReefK;
        break;
      case ReefL:
        targetPose = ReefL;
        break;
      default:
        targetPose = BargePose;
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
