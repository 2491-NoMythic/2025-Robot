// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.settings.Constants.FieldConstants;
import frc.robot.subsystems.RobotState;
public class LineUpBarge extends Command {
  boolean isRed;
  DrivetrainSubsystem driveTrain;
  double distanceX;
  double desiredX;
  double speedX;
  double currentX;
  DoubleSupplier controllerYSupplier;
  /**
   * Points the robot at the barge and drives towards it.
   * Allows for control of side-to-side movement by the driver.
   * @param driveTrain
   * @param controllerYSupplier
   */
  public LineUpBarge(DrivetrainSubsystem driveTrain, DoubleSupplier controllerYSupplier) {
    this.driveTrain = driveTrain;
    this.controllerYSupplier = ()->0;
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
    SmartDashboard.putNumber("BARGELINUP/difference", Math.abs(currentX-desiredX));
    speedX = 4*(desiredX - currentX);
    SmartDashboard.putNumber("BARGELINUP/calculated speed", speedX);
    if(speedX>3) {
      speedX = 3;
    }
    if(speedX<-3) {
      speedX = -3;
    }
    if (isRed) {
      driveTrain.moveTowardsRotationTargetFieldRelative(speedX, 0);
    } else {
      driveTrain.moveTowardsRotationTargetFieldRelative(speedX, 0);
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
    && (Math.abs(currentX - desiredX) < 0.05);
  }
}
