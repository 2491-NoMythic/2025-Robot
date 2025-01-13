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
import static frc.robot.settings.Constants.Vision.*;
import frc.robot.settings.Constants.DriveConstants;
import frc.robot.settings.Constants.Vision;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.LimelightHelpers;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import frc.robot.helpers.MythicalMath;

public class AutoAngleAtReef extends Command {
  DrivetrainSubsystem m_drivetrain;
  DoubleSupplier rotationSupplier;
  double currentHeading;
  double differenceAngle;
  PIDController speedController;
  DoubleSupplier translationXSupplier;
  DoubleSupplier translationYSupplier;
  double rotationSpeed;
  BooleanSupplier AlignToReef;

  /**
   * a command to automatically aim the robot at the reef if a limelight can see
   * an april tag.
   * 
   * @param drivetrain           the swerve drive subsystem
   * @param rotationSupplier     rotational throttle, from -1 to 1
   * @param translationXSupplier the forward throttle, from -1 to 1
   * @param translationYSupplier the sideways (to the right) throttle, from -1 to 1
   */
  public AutoAngleAtReef(DrivetrainSubsystem drivetrain, DoubleSupplier rotationSupplier,
      DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier) {
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
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //sets our desired angle to our current angle so that if no april tag is seen our robot won't try to change it's rotation at all
    double desiredRobotAngle = m_drivetrain.getHeadingDegrees();
    //step 1: determine which tag ID is the closest tag that our robot sees
    //if a limelight is not connected, it's pose returned will be (0, 0, 0). Becuase of this we check for connection and set the distance to a very high number if there is no connection
    Pose3d poseA = LimelightHelpers.getTargetPose3d_RobotSpace(Vision.APRILTAG_LIMELIGHTA_NAME);
    Double distanceA = (Limelight.getInstance().isConnected(APRILTAG_LIMELIGHTA_NAME)) ? MythicalMath.DistanceFromOrigin3d(poseA.getX(), poseA.getY(), poseA.getZ()) : 100;

    Pose3d poseB = LimelightHelpers.getTargetPose3d_RobotSpace(Vision.APRILTAG_LIMELIGHTB_NAME);
    Double distanceB = (Limelight.getInstance().isConnected(APRILTAG_LIMELIGHTB_NAME)) ? MythicalMath.DistanceFromOrigin3d(poseB.getX(), poseB.getY(), poseB.getZ()) : 100;

    Pose3d poseC = LimelightHelpers.getTargetPose3d_RobotSpace(Vision.APRILTAG_LIMELIGHTC_NAME);
    Double distanceC = (Limelight.getInstance().isConnected(APRILTAG_LIMELIGHTC_NAME)) ? MythicalMath.DistanceFromOrigin3d(poseC.getX(), poseC.getY(), poseC.getZ()) : 100;

    double tagA = LimelightHelpers.getFiducialID(Vision.APRILTAG_LIMELIGHTA_NAME);
    double tagB = LimelightHelpers.getFiducialID(Vision.APRILTAG_LIMELIGHTB_NAME);
    double tagC = LimelightHelpers.getFiducialID(Vision.APRILTAG_LIMELIGHTC_NAME);

    double tagClosest; /*the tag ID of the closest april tag to our robot */
    if (distanceA > distanceB && distanceA > distanceC) {
      tagClosest = tagA;
    } else if (distanceB > distanceA && distanceB > distanceC) {
      tagClosest = tagB;
    } else if (distanceC > distanceA && distanceC > distanceB) {
      tagClosest = tagC;
    } else {
      tagClosest = 2491;
    }

  //based on the ID of the tag, determine which angle is likely to align us with the closest side of the reef
  //this will line us up with the blue alliance reef when we are near tags on the blue alliance side, and red alliance reef when we are near tags on the red alliance side
    int tagClosestInt = (int) tagClosest;
    switch (tagClosestInt) {
      case 1:
      case 6:
      case 15:
      case 16:
      case 22:
        desiredRobotAngle = 120;
        break;
      case 5:
      case 11:
      case 12:
      case 17:
        desiredRobotAngle = 60;
        break;
      case 10:
      case 18:
        desiredRobotAngle = 0;
        break;
      case 3:
      case 4:
      case 9:
      case 13:
      case 19:
        desiredRobotAngle = -60;
        break;
      case 2:
      case 8:
      case 14:
      case 20:
        desiredRobotAngle = -120;
        break;
      case 7:
      case 21:
        desiredRobotAngle = 180;
        break;
    }
    
    SmartDashboard.putNumber("AIMROBOT/closest tag", tagClosestInt);
    SmartDashboard.putNumber("AIMROBOT/current Heading", m_drivetrain.getHeadingDegrees() % 360);
    SmartDashboard.putNumber("AIMROBOT/desired angle", desiredRobotAngle);
    
  //finally, calculate our rotation speed based on our desired angle and our speedController, 
  //UNLESS the driver is pushing the rotation joystick more than a third of it's max tilt, then driver has control over rotation too:
    currentHeading = m_drivetrain.getHeadingDegrees();
    if (Math.abs(rotationSupplier.getAsDouble()) > 0.3) {
      rotationSpeed = rotationSupplier.getAsDouble() * DriveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
    } else {
      speedController.setSetpoint(desiredRobotAngle);
      rotationSpeed = speedController.calculate(currentHeading);
      SmartDashboard.putNumber("AIMROBOT/PID calculated output", speedController.calculate(desiredRobotAngle));
    }
  //drive the robot based on the calculated speed for ratotation
    double allianceOffset = (DriverStation.getAlliance().get() == Alliance.Red) ? Math.PI : 0; //this is needed for driving with field relative speeds
    m_drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
      translationXSupplier.getAsDouble() * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND,
      translationYSupplier.getAsDouble() * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND,
      rotationSpeed,
      new Rotation2d(m_drivetrain.getPose().getRotation().getRadians()+allianceOffset)));
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
    //ends command if none of the limelights see an april tag
    return (
      !Limelight.getInstance().isConnected(APRILTAG_LIMELIGHTA_NAME)
      && !Limelight.getInstance().isConnected(APRILTAG_LIMELIGHTB_NAME)
      && !Limelight.getInstance().isConnected(APRILTAG_LIMELIGHTC_NAME)
    );
  }
}