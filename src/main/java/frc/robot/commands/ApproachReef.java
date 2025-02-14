// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.settings.Constants.DriveConstants;
import frc.robot.subsystems.DistanceSensors;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.RobotState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ApproachReef extends Command {
  private final double kP = 0.01;//0.0023;
  private final double kI = 0.001;
  private final double kD = 0.0;
  private final PIDController pidController = new PIDController(kP, kI, kD);

  DistanceSensors sensorSubsystem;
  DrivetrainSubsystem drivetrain;
  DoubleSupplier forwardSupplier;
  DoubleSupplier rightwardsSupplier;
  DoubleSupplier rotationSupplier;
  double invert;
  /** Creates a new ApproachReef. */
  public ApproachReef(
    DistanceSensors distanceSensorsSubssytem,
    DrivetrainSubsystem drivetrainSubsystem,
    DoubleSupplier translationXSupplier,
    DoubleSupplier translationYSupplier,
    DoubleSupplier rotationSupplier
  ) {
    drivetrain = drivetrainSubsystem;
    sensorSubsystem = distanceSensorsSubssytem;
    forwardSupplier = translationXSupplier;
    rightwardsSupplier = translationYSupplier;
    this.rotationSupplier = rotationSupplier;
    addRequirements(drivetrainSubsystem);
    pidController.setIZone(300);
    pidController.setTolerance(10);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      invert = -1;
    } else {
      invert = 1;
    }    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  //first, create chassisSpeeds object using controller inputs
    ChassisSpeeds speeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              forwardSupplier.getAsDouble()
                  * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND
                  * invert,
              rightwardsSupplier.getAsDouble()
                  * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND
                  * invert,
              rotationSupplier.getAsDouble()
                  * DriveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
              drivetrain.getPose().getRotation());
              
  //next, gather data from the sensor
    pidController.setSetpoint(DriveConstants.BUMPER_TO_SENSOR);
    double distance = sensorSubsystem.distanceOfFrontDistancer;
    SmartDashboard.putNumber("SLOWFRONT/sensorReading", distance);
  
  //if there is something within the sensors, range, keep the distance and use it to calculate our forward (actually backward) speed
    distance = (distance == 0) ? 3000 : distance;
    double calculatedSpeed = pidController.calculate(sensorSubsystem.distanceOfFrontDistancer);
    SmartDashboard.putNumber("SLOWFRONT/calculated speed", calculatedSpeed);
    SmartDashboard.putNumber("SLOWFRONT/controller input speed", speeds.vxMetersPerSecond);
  //dont use the calculated speed unless it is between -1 and 0 (we don't want to go too fast)
    if(distance<1300) {
      speeds.vxMetersPerSecond = Math.min(3, -calculatedSpeed);
    }
  //drive using controller inputs + calculated forward speed
    drivetrain.drive(speeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.pointWheelsInward();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }
}
