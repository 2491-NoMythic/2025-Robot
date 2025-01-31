package frc.robot.commands;

import static frc.robot.settings.Constants.DriveConstants.AUTO_AIM_ROBOT_kD;
import static frc.robot.settings.Constants.DriveConstants.AUTO_AIM_ROBOT_kI;
import static frc.robot.settings.Constants.DriveConstants.AUTO_AIM_ROBOT_kP;
import static frc.robot.settings.Constants.DriveConstants.ROBOT_ANGLE_TOLERANCE;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import java.util.function.DoubleSupplier;

public class RotateRobot extends Command {
  DrivetrainSubsystem m_drivetrain;
  DoubleSupplier desiredRobotAngleSupplier;
  double desiredRobotAngle;
  double currentHeading;
  double differenceAngle;
  double turningSpeed;


  public RotateRobot(DrivetrainSubsystem drivetrain, DoubleSupplier desiredRobotAngle) {
    m_drivetrain = drivetrain;
    this.desiredRobotAngleSupplier = desiredRobotAngle;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    desiredRobotAngle = desiredRobotAngleSupplier.getAsDouble();
    m_drivetrain.setRotationTarget(desiredRobotAngle);
    SmartDashboard.putBoolean("isRotateRunning", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // move robot to desired angle
    this.currentHeading = m_drivetrain.getPose().getRotation().getDegrees();
    m_drivetrain.moveTowardsRotationTarget();
    SmartDashboard.putNumber(
        "current Heading", m_drivetrain.getPose().getRotation().getDegrees() % 360);
    SmartDashboard.putNumber("difference", differenceAngle);
    SmartDashboard.putNumber("desired angle", desiredRobotAngle);

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
    return Math.abs(differenceAngle - 360) < ROBOT_ANGLE_TOLERANCE;
  }
}
