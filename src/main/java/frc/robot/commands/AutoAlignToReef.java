package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.settings.Constants.AutoAlignToReefConstants;
import frc.robot.settings.Constants.DriveConstants;
import frc.robot.settings.Constants.FieldConstants;
import frc.robot.settings.Constants.Vision;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.LimelightHelpers;
import java.lang.reflect.Field;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import frc.robot.helpers.MythicalMath;

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

    this.currentHeading = m_drivetrain.getOdometryRotation().getDegrees();
    SmartDashboard.putNumber("AIMROBOT/input degrees", currentHeading);
    SmartDashboard.putNumber("AIMROBOT/setpoint", speedController.getSetpoint());
    if (Math.abs(rotationSupplier.getAsDouble()) > 0.3) {
      rotationSpeed = rotationSupplier.getAsDouble() * DriveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
    } else {
      rotationSpeed = speedController.calculate(currentHeading);
    }

    speedController.setSetpoint(desiredRobotAngle);
    m_drivetrain.drive(new ChassisSpeeds(0, 0, rotationSpeed));

    Pose3d poseA = LimelightHelpers.getTargetPose3d_RobotSpace(Vision.APRILTAG_LIMELIGHTA_NAME);
    Double distanceA = MythicalMath.DistanceFromOrigin3d(poseA.getX(), poseA.getY(), poseA.getZ());

    Pose3d poseB = LimelightHelpers.getTargetPose3d_RobotSpace(Vision.APRILTAG_LIMELIGHTB_NAME);
    Double distanceB = MythicalMath.DistanceFromOrigin3d(poseB.getX(), poseB.getY(), poseB.getZ());

    Pose3d poseC = LimelightHelpers.getTargetPose3d_RobotSpace(Vision.APRILTAG_LIMELIGHTC_NAME);
    Double distanceC = MythicalMath.DistanceFromOrigin3d(poseC.getX(), poseC.getY(), poseC.getZ());

    double tagA = LimelightHelpers.getFiducialID(Vision.APRILTAG_LIMELIGHTA_NAME);
    double tagB = LimelightHelpers.getFiducialID(Vision.APRILTAG_LIMELIGHTB_NAME);
    double tagC = LimelightHelpers.getFiducialID(Vision.APRILTAG_LIMELIGHTC_NAME);

    double tagClosest;
    if (distanceA == null) {
      distanceA = 100.0;
    }
    if (distanceB == null) {
      distanceB = 100.0;
    }
    if (distanceC == null) {
      distanceC = 100.0;
    }

    if (distanceA > distanceB && distanceA > distanceC) {
      tagClosest = tagA;
    } else if (distanceB > distanceA && distanceB > distanceC) {
      tagClosest = tagB;
    } else if (distanceC > distanceA && distanceC > distanceB) {
      tagClosest = tagC;
    }

    if (DriverStation.getAlliance().get() == Alliance.Red) {
      if (tagClosest == 0) {
        desiredRobotAngle = Red_Reef_Side_Angle0_Sup.getAsDouble();
      } else if (tagClosest == 1) {
        desiredRobotAngle = Red_Reef_Side_Angle1_Sup.getAsDouble();
      } else if (tagClosest == 2) {
        desiredRobotAngle = Red_Reef_Side_Angle2_Sup.getAsDouble();
      } else if (tagClosest == 3) {
        desiredRobotAngle = Red_Reef_Side_Angle3_Sup.getAsDouble();
      } else if (tagClosest == 4) {
        desiredRobotAngle = Red_Reef_Side_Angle4_Sup.getAsDouble();
      } else if (tagClosest == 5) {
        desiredRobotAngle = Red_Reef_Side_Angle5_Sup.getAsDouble();
      }
    } else {
      if (tagClosest == 0) {
        desiredRobotAngle = Blue_Reef_Side_Angle0_Sup.getAsDouble();
      } else if (tagClosest == 1) {
        desiredRobotAngle = Blue_Reef_Side_Angle1_Sup.getAsDouble();
      } else if (tagClosest == 2) {
        desiredRobotAngle = Blue_Reef_Side_Angle2_Sup.getAsDouble();
      } else if (tagClosest == 3) {
        desiredRobotAngle = Blue_Reef_Side_Angle3_Sup.getAsDouble();
      } else if (tagClosest == 4) {
        desiredRobotAngle = Blue_Reef_Side_Angle4_Sup.getAsDouble();
      } else if (tagClosest == 5) {
        desiredRobotAngle = Blue_Reef_Side_Angle5_Sup.getAsDouble();
      }
    }
    // targetpose robot space
    // g get fidutial id for ll closest to the orgin

    // how do we determine the desired robot angle anywhere on the field, to the
    // side of the reef?
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