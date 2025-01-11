package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.settings.Constants.AutoAlignToReefConstants;
import frc.robot.settings.Constants.DriveConstants;
import frc.robot.settings.Constants.FieldConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.lang.reflect.Field;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class AutoAlignToReef extends Command {
  DrivetrainSubsystem m_drivetrain;
  DoubleSupplier desiredRobotAngleSupplier;
  DoubleSupplier rotationSupplier;
  double desiredRobotAngle;
  double currentHeading;
  double differenceAngle;
  double turningSpeed;
  PIDController speedController;
  DoubleSupplier translationXSupplier;
  DoubleSupplier translationYSupplier;
  BooleanSupplier run;
  double rotationSpeed;
  double allianceOffset;
  BooleanSupplier AlignToReef;
  DoubleSupplier Red_Reef_Side_Angle0_Sup;
  DoubleSupplier Red_Reef_Side_Angle1_Sup;
  DoubleSupplier Red_Reef_Side_Angle2_Sup;
  DoubleSupplier Red_Reef_Side_Angle3_Sup;
  DoubleSupplier Red_Reef_Side_Angle4_Sup;
  DoubleSupplier Red_Reef_Side_Angle5_Sup;
  DoubleSupplier Blue_Reef_Side_Angle0_Sup;
  DoubleSupplier Blue_Reef_Side_Angle1_Sup;
  DoubleSupplier Blue_Reef_Side_Angle2_Sup;
  DoubleSupplier Blue_Reef_Side_Angle3_Sup;
  DoubleSupplier Blue_Reef_Side_Angle4_Sup;
  DoubleSupplier Blue_Reef_Side_Angle5_Sup;
  /**
   * a command to automatically aim the robot at the speaker if odometry is
   * correct. This command also controls aiming from setpoints incase the odometry
   * isn't working.
   * 
   * @param drivetrain           the swerve drive subsystem
   * @param rotationSupplier     rotational throttle, from -1 to 1
   * @param translationXSupplier the forward throttle, from -1 to 1
   * @param translationYSupplier the sideways (to the right) throttle, from -1 to
   *                             1
   * @param run                  the button used to trigger this command
   */
  public AutoAlignToReef(DrivetrainSubsystem drivetrain, DoubleSupplier rotationSupplier,
      DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier, BooleanSupplier run) {
    m_drivetrain = drivetrain;
    speedController = new PIDController(
        AutoAlignToReefConstants.AUTO_AIM_ROBOT_kP,
        AutoAlignToReefConstants.AUTO_AIM_ROBOT_kI,
        AutoAlignToReefConstants.AUTO_AIM_ROBOT_kD);
    speedController.setTolerance(AutoAlignToReefConstants.ROBOT_ANGLE_TOLERANCE);
    this.rotationSupplier = rotationSupplier;
    this.translationXSupplier = translationXSupplier;
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    this.run = run;
    addRequirements(drivetrain);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("isRotateRunning", true);

  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      Red_Reef_Side_Angle0_Sup = () -> FieldConstants.RED_REEF_ANGLE_0;
      Red_Reef_Side_Angle1_Sup = () -> FieldConstants.RED_REEF_ANGLE_1;
      Red_Reef_Side_Angle2_Sup = () -> FieldConstants.RED_REEF_ANGLE_2;
      Red_Reef_Side_Angle3_Sup = () -> FieldConstants.RED_REEF_ANGLE_3;
      Red_Reef_Side_Angle4_Sup = () -> FieldConstants.RED_REEF_ANGLE_4;
      Red_Reef_Side_Angle5_Sup = () -> FieldConstants.RED_REEF_ANGLE_5;
    } else {
      Blue_Reef_Side_Angle0_Sup = () -> FieldConstants.BLUE_REEF_ANGLE_0;
      Blue_Reef_Side_Angle1_Sup = () -> FieldConstants.BLUE_REEF_ANGLE_1;
      Blue_Reef_Side_Angle2_Sup = () -> FieldConstants.BLUE_REEF_ANGLE_2;
      Blue_Reef_Side_Angle3_Sup = () -> FieldConstants.BLUE_REEF_ANGLE_3;
      Blue_Reef_Side_Angle4_Sup = () -> FieldConstants.BLUE_REEF_ANGLE_4;
      Blue_Reef_Side_Angle5_Sup = () -> FieldConstants.BLUE_REEF_ANGLE_5;
    }

    this.currentHeading = m_drivetrain.getOdometryRotation().getDegrees();
    SmartDashboard.putNumber("AIMROBOT/input degrees", currentHeading);
    SmartDashboard.putNumber("AIMROBOT/setpoint", speedController.getSetpoint());
    if (Math.abs(rotationSupplier.getAsDouble()) > 0.3) {
      rotationSpeed = rotationSupplier.getAsDouble() * DriveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
    } else {
      rotationSpeed = speedController.calculate(currentHeading);
    }
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      allianceOffset = Math.PI;
    } else {
      allianceOffset = 0;
    }

    m_drivetrain.drive(new ChassisSpeeds(0, 0, speedController.calculate(currentHeading)));

      // m_drivetrain.drive(new ChassisSpeeds(
    // translationXSupplier.getAsDouble() *
    // DriveConstants.MAX_VELOCITY_METERS_PER_SECOND,
    // translationYSupplier.getAsDouble() *
    // DriveConstants.MAX_VELOCITY_METERS_PER_SECOND,
    // speedController.calculate(differenceAngle)));

    SmartDashboard.putNumber("current Heading", m_drivetrain.getPose().getRotation().getDegrees() % 360);
    SmartDashboard.putNumber("difference", differenceAngle);
    SmartDashboard.putNumber("desired angle", desiredRobotAngle);
    SmartDashboard.putNumber("PID calculated output", speedController.calculate(differenceAngle));

  }



  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stop();
    SmartDashboard.putBoolean("isRotateRunning", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (!run.getAsBoolean());
  }
}