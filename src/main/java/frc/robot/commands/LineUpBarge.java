// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.settings.Constants.FieldConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LineUpBarge extends Command {
  boolean isRed;
  DrivetrainSubsystem driveTrain;
  double startY;
  double startX;
  double distanceX;
  double desiredX;
  double distanceY;
  double desiredY;
  double speedX;
  double speedY;
  double currentX;
  double currentY;
  ChassisSpeeds chassisSpeeds;
  /** Creates a new LineUpBarge. */
  public LineUpBarge(DrivetrainSubsystem driveTrain) {
    this.driveTrain = driveTrain;
    addRequirements(driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    isRed = alliance.isPresent() && alliance.get() == Alliance.Red;
    if(isRed){
      driveTrain.setRotationTarget(180);
      desiredX = FieldConstants.RED_BARGE_SHOOT_X;
      desiredY = FieldConstants.RED_BARGE_SHOOT_Y;
    }
    else{
      driveTrain.setRotationTarget(0);
      desiredX = FieldConstants.BLUE_BARGE_SHOOT_X;
      desiredY = FieldConstants.BLUE_BARGE_SHOOT_Y;
    }
    startX = driveTrain.getPose().getX();
    startY = driveTrain.getPose().getY();
    distanceX = desiredX + startX;
    distanceY = desiredY + startY;
    
    if(distanceX < distanceY) {
      speedY = distanceX/distanceY;
      speedX = 1 - speedY;
    } else {
      speedX = distanceY/distanceX;
      speedY = 1 - speedX;
    }
    speedX *= 5;
    speedY *= 5;
    
    chassisSpeeds.vxMetersPerSecond = speedX;
    chassisSpeeds.vyMetersPerSecond = speedY;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.moveTowardsRotationTarget();
    driveTrain.drive(chassisSpeeds);
    currentX = driveTrain.getPose().getX();
    currentY = driveTrain.getPose().getY();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   return driveTrain.isAtRotationTarget()
    && (Math.abs(currentX - desiredX) < 0.2)
    && (Math.abs(currentY - desiredY) < 0.2);
  }
}
