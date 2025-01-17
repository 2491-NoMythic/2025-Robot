package frc.robot.commands;

import edu.wpi.first.hal.simulation.DriverStationDataJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.settings.Constants.DriveConstants;
import frc.robot.subsystems.DistanceSensors;
import frc.robot.subsystems.DrivetrainSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import static frc.robot.settings.Constants.SensorConstants.*;

public class Drive extends Command {
  private final DistanceSensors distanceSensorsSubssytem;
  private final BooleanSupplier slowFront;

  private final DrivetrainSubsystem drivetrain;
  private final BooleanSupplier robotCentricMode;
  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;
  private final DoubleSupplier rotationSupplier;
  private int invert;

  private final double kP = 0.002;
  private final double kI = 0.0;
  private final double kD = 0.0;
  private final PIDController pidController = new PIDController(kP, kI, kD);

  /**
   * drives the robot at a specific forward velocity, sideways velocity, and rotational velocity.
   *
   * @param drivetrainSubsystem Swerve drive subsytem
   * @param robotCentricMode while this is pressed, the robot will drive in RobotCentric mode.
   *     Otherwise, it will default to field centric
   * @param translationXSupplier forward throttle (from -1 to 1). 1 will drive at full speed forward
   * @param translationYSupplier sideways throttle (from -1 to 1). 1 will drive at full speed to the
   *     right
   * @param rotationSupplier rotational throttle (from -1 to 1). 1 will drive at full speed
   *     clockwise
   */
  public Drive(
      DistanceSensors distanceSensorsSubssytem,
      BooleanSupplier slowFront,

      DrivetrainSubsystem drivetrainSubsystem,
      BooleanSupplier robotCentricMode,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      DoubleSupplier rotationSupplier) {

    this.distanceSensorsSubssytem = distanceSensorsSubssytem;
    this.slowFront = slowFront;


    this.drivetrain = drivetrainSubsystem;
    this.robotCentricMode = robotCentricMode;
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    this.rotationSupplier = rotationSupplier;
    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void execute() {
    // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of
    // field-oriented
    // movement
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      invert = -1;
    } else {
      invert = 1;
    }
    if (robotCentricMode.getAsBoolean()) {
      drivetrain.drive(
          new ChassisSpeeds(
              translationXSupplier.getAsDouble()
                  * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND
                  * invert,
              translationYSupplier.getAsDouble()
                  * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND
                  * invert,
              rotationSupplier.getAsDouble()
                  * DriveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));
    } else {
      ChassisSpeeds speeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              translationXSupplier.getAsDouble()
                  * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND
                  * invert,
              translationYSupplier.getAsDouble()
                  * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND
                  * invert,
              rotationSupplier.getAsDouble()
                  * DriveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
              drivetrain.getPose().getRotation());
      //may have to mess with what is "forward" (aka y or x, positive or negative)
      //this will SCALE bassed off DRIVER INPUT. Meaning the driver still has to be moving forward for it to move forward. This will need to be tuned to driver preference.
        // double setAxisValue =  (speeds.vxMetersPerSecond * distanceSensorsSubssytem.distanceOfFrontDistancer/SLOW_DOWN_RANGE);
        // SmartDashboard.putNumber("percentPower", distanceSensorsSubssytem.distanceOfFrontDistancer/SLOW_DOWN_RANGE);
        // SmartDashboard.putNumber("SLOWFRONT/axis value", setAxisValue);
        // SmartDashboard.putNumber("yspd", speeds.vxMetersPerSecond);
      //this is checking to ensure that it isnt going to A: SPEED UP the DT, and B: that the trigger is pressed
      pidController.setSetpoint(DriveConstants.BUMPER_TO_SENSOR);
      double distance = distanceSensorsSubssytem.distanceOfFrontDistancer;
      SmartDashboard.putNumber("SLOWFRONT/sensorReading", distance);
      distance = (distance == 0) ? 3000 : distance;
      double calculatedSpeed = pidController.calculate(distanceSensorsSubssytem.distanceOfFrontDistancer);
      SmartDashboard.putNumber("SLOWFRONT/calculated speed", calculatedSpeed);
      SmartDashboard.putNumber("SLOWFRONT/controller input speed", speeds.vxMetersPerSecond);
      if(slowFront.getAsBoolean() && distance<1300) {
        //setting vy NOT from field orientation means that this is the ROBOTS y. 
        speeds.vxMetersPerSecond = Math.max(-1, calculatedSpeed);
      }
        
        drivetrain.drive(speeds);
    }
    }
            
    @Override
    public void end(boolean interrupted) {
      drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }
}
