// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.settings.Constants.DriveConstants;
import frc.robot.settings.Constants.FieldConstants;
import frc.robot.subsystems.RobotState;
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LineUpBarge extends Command {
  boolean isRed;
  DrivetrainSubsystem driveTrain;
  double distanceX;
  double desiredX;
  double speedX;
  double currentX;
  DoubleSupplier controllerSupplier;
  /** Creates a new LineUpBarge. */
  public LineUpBarge(DrivetrainSubsystem driveTrain, DoubleSupplier controllerSupplier) {
    this.driveTrain = driveTrain;
    this.controllerSupplier = controllerSupplier;
    addRequirements(driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotState.getInstance().bargeLineUp = true;
    Optional<Alliance> alliance = DriverStation.getAlliance();
    isRed = alliance.isPresent() && alliance.get() == Alliance.Red;
    if(isRed){
      driveTrain.setRotationTarget(180);
      desiredX = FieldConstants.RED_BARGE_SHOOT_X;
    }
    else{
      driveTrain.setRotationTarget(0);
      desiredX = FieldConstants.BLUE_BARGE_SHOOT_X;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentX = driveTrain.getPose().getX();
    SmartDashboard.putNumber("currentX", currentX);
    SmartDashboard.putNumber("desiredX", desiredX);
    if (isRed) {
      if (currentX < desiredX) {
        speedX = -1;
      } else {
        speedX = 1;
      }
      driveTrain.moveTowardsRotationTarget(speedX, controllerSupplier.getAsDouble() * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND * -1);
    } else {
      if (currentX < desiredX) {
        speedX = 1;
      } else {
        speedX = -1;
      }
      driveTrain.moveTowardsRotationTarget(speedX, controllerSupplier.getAsDouble() * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
    RobotState.getInstance().bargeLineUp = false;
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   return driveTrain.isAtRotationTarget()
    && (Math.abs(currentX - desiredX) < 0.2);
  }
}
