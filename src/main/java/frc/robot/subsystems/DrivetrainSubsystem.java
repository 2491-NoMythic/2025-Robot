// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.settings.Constants.DriveConstants.AUTO_AIM_ROBOT_kD;
import static frc.robot.settings.Constants.DriveConstants.AUTO_AIM_ROBOT_kI;
import static frc.robot.settings.Constants.DriveConstants.AUTO_AIM_ROBOT_kP;
import static frc.robot.settings.Constants.DriveConstants.BL_DRIVE_MOTOR_ID;
import static frc.robot.settings.Constants.DriveConstants.BL_STEER_ENCODER_ID;
import static frc.robot.settings.Constants.DriveConstants.BL_STEER_MOTOR_ID;
import static frc.robot.settings.Constants.DriveConstants.BR_DRIVE_MOTOR_ID;
import static frc.robot.settings.Constants.DriveConstants.BR_STEER_ENCODER_ID;
import static frc.robot.settings.Constants.DriveConstants.BR_STEER_MOTOR_ID;
import static frc.robot.settings.Constants.DriveConstants.CANIVORE_DRIVETRAIN;
import static frc.robot.settings.Constants.DriveConstants.DRIVETRAIN_PIGEON_ID;
import static frc.robot.settings.Constants.DriveConstants.DRIVE_ODOMETRY_ORIGIN;
import static frc.robot.settings.Constants.DriveConstants.FL_DRIVE_MOTOR_ID;
import static frc.robot.settings.Constants.DriveConstants.FL_STEER_ENCODER_ID;
import static frc.robot.settings.Constants.DriveConstants.FL_STEER_MOTOR_ID;
import static frc.robot.settings.Constants.DriveConstants.FR_DRIVE_MOTOR_ID;
import static frc.robot.settings.Constants.DriveConstants.FR_STEER_ENCODER_ID;
import static frc.robot.settings.Constants.DriveConstants.FR_STEER_MOTOR_ID;
import static frc.robot.settings.Constants.DriveConstants.ROBOT_ANGLE_TOLERANCE;
import static frc.robot.settings.Constants.Field.*;
import static frc.robot.settings.Constants.Vision.APRILTAG_LIMELIGHTA_NAME;
import static frc.robot.settings.Constants.Vision.APRILTAG_LIMELIGHTB_NAME;
import static frc.robot.settings.Constants.Vision.APRILTAG_LIMELIGHTC_NAME;
import static frc.robot.settings.Constants.Vision.FIELD_CORNER;
import static frc.robot.settings.Constants.Vision.FIELD_CORNER_FOR_INTAKE;

import java.util.Arrays;
import java.util.Collections;

// import java.util.logging.Logger;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.helpers.MotorLogger;
import frc.robot.helpers.MythicalMath;
import frc.robot.settings.Constants.DriveConstants;
import frc.robot.settings.Constants;
import frc.robot.settings.ReefSideEnum;

public class DrivetrainSubsystem extends SubsystemBase {
  //These are our swerve drive kinematics and Pigeon (gyroscope)
  public SwerveDriveKinematics kinematics = DriveConstants.kinematics;

  private final Pigeon2 pigeon = new Pigeon2(DRIVETRAIN_PIGEON_ID, CANIVORE_DRIVETRAIN);

  /**
   * These are our modules. We initialize them in the constructor. 0 = Front Left 1 = Front Right 2 = Back Left 3 = Back Right
   */
  private final SwerveModule[] modules;
  /**
	 * These are the angles the wheels are at. This is mostly used to stop the robot without changing the angles.
	 */
  private final Rotation2d[] lastAngles;
  /**
	 * This is a number that keeps track of how many times the steering motor has rotated.
	 */
  private int accumulativeLoops;
	/**
	 * This is the odometer.
	 */
  private final SwerveDrivePoseEstimator odometer;
  private final Field2d m_field = new Field2d();

  Limelight limelight;
  MotorLogger[] motorLoggers;
  PIDController speedController;
  PIDController rotationSpeedController;

  public DrivetrainSubsystem() {
    rotationSpeedController = new PIDController(AUTO_AIM_ROBOT_kP, AUTO_AIM_ROBOT_kI, AUTO_AIM_ROBOT_kD);
    rotationSpeedController.setTolerance(ROBOT_ANGLE_TOLERANCE);
    rotationSpeedController.enableContinuousInput(-180, 180);
    this.limelight = Limelight.getInstance();
    Preferences.initDouble("FL offset", 0);
    Preferences.initDouble("FR offset", 0);
    Preferences.initDouble("BL offset", 0);
    Preferences.initDouble("BR offset", 0);
    PathPlannerLogging.setLogActivePathCallback(
        (poses) -> m_field.getObject("path").setPoses(poses));
    SmartDashboard.putData("Field", m_field);
    SmartDashboard.putBoolean("Vision/force use limelight", false);

    // Creates and configures each of the four swerve modules used in the drivetrain, along with their motor loggers.
    modules = new SwerveModule[4];
    lastAngles =
        new Rotation2d[] {
          new Rotation2d(), new Rotation2d(), new Rotation2d(), new Rotation2d()
        }; // manually make empty angles to avoid null errors.

    modules[0] =
        new SwerveModule(
            "FL",
            FL_DRIVE_MOTOR_ID,
            FL_STEER_MOTOR_ID,
            FL_STEER_ENCODER_ID,
            Rotation2d.fromRotations(Preferences.getDouble("FL offset", 0)),
            CANIVORE_DRIVETRAIN);
    modules[1] =
        new SwerveModule(
            "FR",
            FR_DRIVE_MOTOR_ID,
            FR_STEER_MOTOR_ID,
            FR_STEER_ENCODER_ID,
            Rotation2d.fromRotations(Preferences.getDouble("FR offset", 0)),
            CANIVORE_DRIVETRAIN);
    modules[2] =
        new SwerveModule(
            "BL",
            BL_DRIVE_MOTOR_ID,
            BL_STEER_MOTOR_ID,
            BL_STEER_ENCODER_ID,
            Rotation2d.fromRotations(Preferences.getDouble("BL offset", 0)),
            CANIVORE_DRIVETRAIN);
    modules[3] =
        new SwerveModule(
            "BR",
            BR_DRIVE_MOTOR_ID,
            BR_STEER_MOTOR_ID,
            BR_STEER_ENCODER_ID,
            Rotation2d.fromRotations(Preferences.getDouble("BR offset", 0)),
            CANIVORE_DRIVETRAIN);

    DataLog log = DataLogManager.getLog();
    motorLoggers =
        new MotorLogger[] {
          new MotorLogger("/drivetrain/motorFL"),
          new MotorLogger("/drivetrain/motorFR"),
          new MotorLogger( "/drivetrain/motorBL"),
          new MotorLogger("/drivetrain/motorBR"),
        };
    // configures the odometer
    odometer =
        new SwerveDrivePoseEstimator(
            kinematics, getGyroscopeRotation(), getModulePositions(), DRIVE_ODOMETRY_ORIGIN);
    odometer.setVisionMeasurementStdDevs(VecBuilder.fill(0.5, 0.5, 99999999));
  }
  // This is the main 'get' section
	/**
	 * Gets the robot pose.
	 * @return
	 */
  public Pose2d getPose() {
    return odometer.getEstimatedPosition();
  }
  /**
	 * Returns the gyroscope rotation.
	 * @return
	 */
	public Rotation2d getGyroscopeRotation() {
		return pigeon.getRotation2d();
	}

  public double getPigeonPitch(){
    double pitch = pigeon.getPitch().getValueAsDouble();
    return pitch;
  }

  public double getPigeonRoll(){
    double roll = pigeon.getRoll().getValueAsDouble();
    return roll;
  }
  
  /**
   * @return a rotation2D of the angle according to the odometer
   */
  public Rotation2d getOdometryRotation() {
    return odometer.getEstimatedPosition().getRotation();
  }
  /**
	 * Returns the angle as degrees instead of rotations
	 * @return the angle in degreeds instead of rotations
	 */
	public double headingAsDegrees(){
		return getOdometryRotation().getDegrees();
	}
  /** 
   * Returns the heading of the robot, but only out of 360, not accumulative 
   * */
  public double getHeadingLooped() {
    accumulativeLoops =
        (int)
            (headingAsDegrees()
                / 180); // finding the amount of times that 360 goes into the heading, as an int
    return headingAsDegrees() - 180 * (accumulativeLoops);
  }
  	/**
	 * Returns what directions the swerve modules are pointed in
	 * @return the positions of the swerve modules
	 */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) positions[i] = modules[i].getPosition();
    return positions;
  }
	/**
	 * Returns the swerve module states (a mxiture of speed and angle)
	 * @return the speeds and angles of the swerve modules
	 */
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) states[i] = modules[i].getState();
    return states;
  }
  /**
   * Gets the speed of the robot
   * @return the module states in terms of speed
   */
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }
  //This is the odometry section. It has odometry-related functions.
  /**
	 * Resets the odometry of the robot.
	 * @param pose
	 */
  public void resetOdometry(Pose2d pose) {
    odometer.resetPosition(getGyroscopeRotation(), getModulePositions(), pose);
  } 
  /**
   * Sets the gyro to the specified position.
   */
  public void setGyroscope(double angleDeg) {
    resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(angleDeg)));
  }
  /**
   * Sets the gyroscope angle to zero.  
   */
  public void zeroGyroscope() {
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red) {
      setGyroscope(180);
    } else {
      setGyroscope(0);
    }
  }
  //This is the set section that sends commands to the modules
  /**
   * Calls the findOffset fucntion for each module.
   */
  public void setEncoderOffsets() {
    Preferences.setDouble("FL offset", modules[0].findOffset());
    Preferences.setDouble("FR offset", modules[1].findOffset());
    Preferences.setDouble("BL offset", modules[2].findOffset());
    Preferences.setDouble("BR offset", modules[3].findOffset());
  }
  /**
   * Sets a given module to a given module state. 
   * @param i the ID of the module
   * @param desiredState the speed and angle as a SwerveModuleState
   */
  private void setModule(int i, SwerveModuleState desiredState) {
    modules[i].setDesiredState(desiredState);
    lastAngles[i] = desiredState.angle;
  }
  
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.MAX_VELOCITY_METERS_PER_SECOND);
    for (int i = 0; i < 4; i++) {
      setModule(i, desiredStates[i]);
    }
  }
  /** Sets the modules speed and rotation to zero. */
  //TODO: Make a version that works with States. 
  public void pointWheelsForward() {
    for (int i = 0; i < 4; i++) {
      setModule(i, new SwerveModuleState(0, new Rotation2d()));
    }
  }
  /**
   * Points all of the wheels towards the center of the robot, making it harder to push. 
   */
  public void pointWheelsInward() {
    setModule(0, new SwerveModuleState(0, Rotation2d.fromDegrees(-135)));
    setModule(1, new SwerveModuleState(0, Rotation2d.fromDegrees(135)));
    setModule(2, new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    setModule(3, new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }
  /**
   * The function that actually lets us drive the robot.
   * @param chassisSpeeds the desired speed and direction
   */
  public void drive(ChassisSpeeds chassisSpeeds) {
    if (Preferences.getBoolean("AntiTipActive", false)) {
      if (pigeon.getRoll().getValueAsDouble() > 3) {
        chassisSpeeds.vxMetersPerSecond = chassisSpeeds.vxMetersPerSecond
            + (pigeon.getRoll().getValueAsDouble()/10);
      } else if (pigeon.getRoll().getValueAsDouble() < -3) {
        chassisSpeeds.vxMetersPerSecond = chassisSpeeds.vxMetersPerSecond
            + (-pigeon.getRoll().getValueAsDouble()/10);
      }
      if (pigeon.getPitch().getValueAsDouble() > 3) {
        chassisSpeeds.vyMetersPerSecond = chassisSpeeds.vyMetersPerSecond
            + (pigeon.getPitch().getValueAsDouble()/10);
      } else if (pigeon.getPitch().getValueAsDouble() < -3) {
        chassisSpeeds.vyMetersPerSecond = chassisSpeeds.vyMetersPerSecond
            + (-pigeon.getPitch().getValueAsDouble()/10);
      }
    }
    
    SwerveModuleState[] desiredStates =
        kinematics.toSwerveModuleStates(ChassisSpeeds.discretize(chassisSpeeds, 0.02));
    double maxSpeed = Collections.max(Arrays.asList(desiredStates)).speedMetersPerSecond;
    if (maxSpeed <= DriveConstants.DRIVE_DEADBAND_MPS) {
      for (int i = 0; i < 4; i++) {
        stop();
      }
    } else {
      setModuleStates(desiredStates);
    }
  }
  /**
   * Stops the robot. 
   */
  public void stop() {
    for (int i = 0; i < 4; i++) {
      modules[i].setDesiredState(new SwerveModuleState(0, lastAngles[i]));
    }
  }

  //This is the odometry section
  /**
   * Updates the odometry
   */
  public void updateOdometry() {
    odometer.updateWithTime(Timer.getFPGATimestamp(), getGyroscopeRotation(), getModulePositions());
  }

  /**
   * Provide the odometry a vision pose estimate, only if there is a trustworthy pose available.
   *
   * <p>Each time a vision pose is supplied, the odometry pose estimation will change a little,
   * larger pose shifts will take multiple calls to complete.
   */
  public void updateOdometryWithVision() {
    LimelightHelpers.SetRobotOrientation(
        APRILTAG_LIMELIGHTA_NAME,
        odometer.getEstimatedPosition().getRotation().getDegrees(),
        0,
        0,
        0,
        0,
        0);
    LimelightHelpers.SetRobotOrientation(
        APRILTAG_LIMELIGHTB_NAME,
        odometer.getEstimatedPosition().getRotation().getDegrees(),
        0,
        0,
        0,
        0,
        0);
    LimelightHelpers.SetRobotOrientation(
        APRILTAG_LIMELIGHTC_NAME,
        odometer.getEstimatedPosition().getRotation().getDegrees(),
        0,
        0,
        0,
        0,
        0);

    PoseEstimate estimate = limelight.getTrustedPose();
    if (estimate != null) {
      boolean doRejectUpdate = false;
      if (Math.abs(pigeon.getAngularVelocityZWorld().getValueAsDouble()) > 720) {
        doRejectUpdate = true;
      }
      if (estimate.tagCount == 0) {
        doRejectUpdate = true;
      }
      if (!doRejectUpdate) {
        odometer.addVisionMeasurement(estimate.pose, estimate.timestampSeconds);
      }
      RobotState.getInstance().LimelightsUpdated = true;
    } else {
      RobotState.getInstance().LimelightsUpdated = false;
    }
  }
  /**
   * Set the odometry using the current apriltag estimate, disregarding the pose trustworthyness.
   *
   * <p>You only need to run this once for it to take effect.
   */
  public void forceUpdateOdometryWithVision() {
    PoseEstimate estimate = limelight.getTrustedPose();
    if (estimate != null) {
      resetOdometry(estimate.pose);
    } else {
      System.err.println(
          "No valid limelight estimate to reset from. (Drivetrain.forceUpdateOdometryWithVision)");
    }
  }
  /** Prepares to rotate the robot to a specific angle
   * @param desiredAngle the angle to rotate the robot to (in degrees relative to the field)
   */
  public void setRotationTarget(double desiredAngle) {
    rotationSpeedController.setSetpoint(desiredAngle);
  }
  /** Applies power to the motors to rotate the robot to the angle set by 
   * {@link #setRotationTarget(double) setRotationTarget}
   */
  public void moveTowardsRotationTarget(double vx, double vy) {
    drive(new ChassisSpeeds(vx, vy, rotationSpeedController.calculate(getPose().getRotation().getDegrees()
    )));
  }
  public boolean isAtRotationTarget() {
    return rotationSpeedController.atSetpoint();
  }
  public boolean drivetrainInIntakeZones() {
    boolean inIntakeZone = false;
    double intakeZoneLength = 3;//lenth along the y axis of the "intake zone" (which we make up)
    double intakeZoneWidth = 1.7; //length along the x axis of the "intake zone" (which we make up)
    if(
      (getPose().getY()>FIELD_CORNER_FOR_INTAKE.getY()-intakeZoneLength || getPose().getY()<intakeZoneLength) //robot is in the Y ranges for the intake zone
      && (getPose().getX()>FIELD_CORNER_FOR_INTAKE.getX()-intakeZoneWidth || getPose().getX()<intakeZoneWidth)) { //robot is in the X ranges for the intake zone
      inIntakeZone = true;
    }
    return inIntakeZone;
  }
  /*
   * Logs important data for the drivetrain
   */
  private void logDrivetrainData(){
    SmartDashboard.putNumber("DRIVETRAIN/Robot Angle", getOdometryRotation().getDegrees());
    SmartDashboard.putString("DRIVETRAIN/Robot Location", getPose().getTranslation().toString());
    SmartDashboard.putNumber("DRIVETRAIN/forward speed", getChassisSpeeds().vxMetersPerSecond);
    SmartDashboard.putNumber(
        "DRIVETRAIN/rotational speed", Math.toDegrees(getChassisSpeeds().omegaRadiansPerSecond));
    SmartDashboard.putNumber(
        "DRIVETRAIN/gyroscope rotation degrees", getPose().getRotation().getDegrees());
    SmartDashboard.putNumber(
        "DRIVETRAIN/degrees per second", Math.toDegrees(getChassisSpeeds().omegaRadiansPerSecond));

    Logger.recordOutput("MyStates", getModuleStates());
    Logger.recordOutput("Position", odometer.getEstimatedPosition());
    Logger.recordOutput("Gyro", getGyroscopeRotation());
  }
/**
 * this method updates the closest ReefSide value in robot state by finding the distance to each reefSide, then finding the smallest of those, then using that to update RobotState
 */
  private void updateClosestReefSide() {
    Pose2d robotPose = getPose();
    Pose2d frontCenterPose;
    Pose2d frontLeftPose;
    Pose2d frontRightPose;
    Pose2d backCenterPose;
    Pose2d backRightPose;
    Pose2d backLeftPose;
  //sets the positions of the reef sides based on our alliance
    if(DriverStation.getAlliance().get() == Alliance.Red) {
      frontCenterPose = RED_FRONT_CENTER_REEFSIDE_POSE;
      frontLeftPose = RED_FRONT_LEFT_REEFSIDE_POSE;
      frontRightPose = RED_FRONT_RIGHT_REEFSIDE_POSE;
      backCenterPose = RED_BACK_CENTER_REEFSIDE_POSE;
      backLeftPose = RED_BACK_LEFT_REEFSIDE_POSE;
      backRightPose = RED_BACK_RIGHT_REEFSIDE_POSE;
    } else {
      frontCenterPose = BLUE_FRONT_CENTER_REEFSIDE_POSE;
      frontLeftPose = BLUE_FRONT_LEFT_REEFSIDE_POSE;
      frontRightPose = BLUE_FRONT_RIGHT_REEFSIDE_POSE;
      backCenterPose = BLUE_BACK_CENTER_REEFSIDE_POSE;
      backLeftPose = BLUE_BACK_LEFT_REEFSIDE_POSE;
      backRightPose = BLUE_BACK_RIGHT_REEFSIDE_POSE;
    }
//finds the distnaces to each reef side
    double distanceToFrontCenter = MythicalMath.distanceBetweenTwoPoses(robotPose, frontCenterPose);
    double distanceToFrontRight = MythicalMath.distanceBetweenTwoPoses(robotPose, frontRightPose);
    double distanceToFrontLeft = MythicalMath.distanceBetweenTwoPoses(robotPose, frontLeftPose);
    double distanceToBackCenter = MythicalMath.distanceBetweenTwoPoses(robotPose, backCenterPose);
    double distanceToBackRight = MythicalMath.distanceBetweenTwoPoses(robotPose, backRightPose);
    double distanceToBackLeft = MythicalMath.distanceBetweenTwoPoses(robotPose, backLeftPose);
//finds the distance to a reef side that is the smallest, then sets robot state based on the distance that the minimum is equal to.
    double closestDistance = Math.min(distanceToFrontCenter, Math.min(distanceToFrontRight, Math.min(distanceToFrontLeft, Math.min(distanceToBackRight, Math.min(distanceToBackCenter, distanceToBackLeft)))));
    if(closestDistance == distanceToFrontCenter) {RobotState.getInstance().closestReefSide = ReefSideEnum.middleClose;}
    if(closestDistance == distanceToFrontRight) {RobotState.getInstance().closestReefSide = ReefSideEnum.processorClose;}
    if(closestDistance == distanceToFrontLeft) {RobotState.getInstance().closestReefSide = ReefSideEnum.bargeClose;}
    if(closestDistance == distanceToBackCenter) {RobotState.getInstance().closestReefSide = ReefSideEnum.middleFar;}
    if(closestDistance == distanceToBackRight) {RobotState.getInstance().closestReefSide = ReefSideEnum.processorFar;}
    if(closestDistance == distanceToBackLeft) {RobotState.getInstance().closestReefSide = ReefSideEnum.bargeFar;}
    SmartDashboard.putString("REEFLINEUP/closest side", RobotState.getInstance().closestReefSide.toString());
    SmartDashboard.putNumber("REEFLINEUP/closest side distance", closestDistance);
  }
  //This is the things the subsystem does periodically. 
  @Override
  public void periodic() {
    SmartDashboard.putNumber("pose2d X", getPose().getX());
    SmartDashboard.putNumber("pose2d Y", getPose().getY());
    updateOdometry();
    // sets the robot orientation for each of the limelights, which is required for the
    if (Preferences.getBoolean("Use Limelight", false)) {
      if (SmartDashboard.getBoolean("Vision/force use limelight", false)) {
        forceUpdateOdometryWithVision();
      } else {
        updateOdometryWithVision();
      }
    } else {
      RobotState.getInstance().LimelightsUpdated = false;
    }

    m_field.setRobotPose(odometer.getEstimatedPosition());
    RobotState.getInstance().odometerOrientation = getOdometryRotation().getDegrees();
    // updates logging for all drive motors on the swerve modules

    for (int i = 0; i < 4; i++) {
      if(Preferences.getBoolean("Motor Logging", false)){
      motorLoggers[i].log(modules[i].getDriveMotor());
      }
    }
    logDrivetrainData();
    updateClosestReefSide();
    SmartDashboard.putBoolean("DRIVETRAIN/inIntakeZone", drivetrainInIntakeZones());
  }

}
