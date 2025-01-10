// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.settings.Constants.CTREConfigs;
import frc.robot.settings.Constants.DriveConstants;

public class SwerveModule {
  // The mechanical bits
  private final TalonFX m_driveMotor;
  private final TalonFX m_steerMotor;
  private final CANcoder m_steerEncoder;
  private final Rotation2d m_steerEncoderOffset;
  private TalonFXConfiguration drive_Configuration;
  private TalonFXConfiguration steer_Configuration;
  private CANcoderConfiguration enc_Configuration;
  //Shuffleboard stuff
  ShuffleboardTab debugInfo;
  //Variables
  /** The target wheel angle in rotations. [-.5, .5] */
  private double m_desiredSteerAngle;
  /** The target wheel speed in rotations per second */
  private double m_desiredDriveSpeed;
  //Controls
  private VelocityVoltage m_driveControl = new VelocityVoltage(0);
  private PositionDutyCycle m_steerControl = new PositionDutyCycle(0);
  private NeutralOut m_neutralControl = new NeutralOut();

  /**
   * Constructs a SwerveModule with a drive motor, steering motor, and steering encoder.
   * Also takes a rotation 2d offset for directions and a canivore name. 
   *
   *
   * @param driveMotorChannel (drive motor id, integer)
   * @param steerMotorChannel (steering motor id, integer)
   * @param steerEncoderChannel (steering encoder id, integer)
   * @param steerEncoderOffset (how far the wheel is offset, rotation2d)
   * @param canivoreName (name of the canivore, string)
   */
  public SwerveModule(
      String moduleName,
      int driveMotorChannel,
      int steerMotorChannel,
      int steerEncoderChannel,
      Rotation2d steerEncoderOffset,
      String canivoreName) {
    m_driveMotor = new TalonFX(driveMotorChannel, canivoreName);
    m_steerMotor = new TalonFX(steerMotorChannel, canivoreName);
    m_steerEncoder = new CANcoder(steerEncoderChannel, canivoreName);
    m_steerEncoderOffset = steerEncoderOffset;
    drive_Configuration = CTREConfigs.driveMotorConfig;
    steer_Configuration = CTREConfigs.steerMotorConfig;
    enc_Configuration = CTREConfigs.steerEncoderConfig;

    enc_Configuration.MagnetSensor.MagnetOffset = -m_steerEncoderOffset.getRotations();
    steer_Configuration.Feedback.FeedbackRemoteSensorID = steerEncoderChannel;  
    // Apply the configurations.
    m_driveMotor.getConfigurator().apply(drive_Configuration);
    m_steerMotor.getConfigurator().apply(steer_Configuration);
    m_steerEncoder.getConfigurator().apply(enc_Configuration);
  }
  //This section is the 'direct get' section. Everything that gets something directly from a motor is in here.
  public TalonFX getDriveMotor() {
    return m_driveMotor;
  }

  public TalonFX getSteerMotor() {
    return m_steerMotor;
  }
  /**
   * Returns the current encoder distance of the drive motor.
   *
   * @return The current distance of the drive motor in meters.
   */
  public double getDriveDistanceMeters() {
    return (m_driveMotor.getPosition().getValueAsDouble()
        * DriveConstants.DRIVETRAIN_ROTATIONS_TO_METERS);
  }
  /**
   * Returns the current encoder angle of the steer motor.
   *
   * @return The current encoder angle of the steer motor.
   */
  public Rotation2d getRotation() {
    return Rotation2d.fromRotations(
        MathUtil.inputModulus(m_steerMotor.getPosition().getValueAsDouble(), -0.5, 0.5));
  }

  /**
   * Returns the current encoder velocity of the drive motor.
   *
   * @return The current velocity of the drive motor in meters/second.
   */
  public double getSpeedMetersPerSecond() {
    return (m_driveMotor.getVelocity().getValueAsDouble()
        * DriveConstants.DRIVETRAIN_ROTATIONS_TO_METERS);
  }
    /**
   * finds the current encoder position, it removes the current offset so we just get the raw
   * position
   *
   * @return
   */
  public double findOffset() {
    return MathUtil.inputModulus(
        (m_steerEncoder.getPosition().getValueAsDouble() + m_steerEncoderOffset.getRotations()),
        -0.5,
        0.5);
  }
  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getSpeedMetersPerSecond(), getRotation());
  }

  /**
   * Returns the current position of the module. Includes the modules rotation and the modules
   * distance driven.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDriveDistanceMeters(), getRotation());
  }
  //This is the direction section. It recives instructions and sets the desired state of the swerve module. 
  /**
   * Returns the target angle of the wheel.
   *
   * @return The target angle of the wheel in degrees.
   */
  public double getTargetAngle() {
    return m_desiredSteerAngle;
  }

  /**
   * Returns the target speed of the wheel.
   *
   * @return The target speed of the wheel in meters/second.
   */
  public double getTargetSpeedMetersPerSecond() {
    return m_desiredDriveSpeed;
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {

    if (desiredState.angle == null) {
      DriverStation.reportWarning("Cannot set module angle to null.", true);
    }

    // Optimize the reference state to avoid spinning further than 90 degrees.
    desiredState.optimize(getRotation());

    m_desiredSteerAngle = MathUtil.inputModulus(desiredState.angle.getRotations(), -0.5, 0.5);
    m_desiredDriveSpeed =
        desiredState.speedMetersPerSecond / DriveConstants.DRIVETRAIN_ROTATIONS_TO_METERS;

    if (Math.abs(m_desiredDriveSpeed) <= 0.001) {
      m_driveMotor.setControl(m_neutralControl);
    } else {
      m_driveMotor.setControl(
          m_driveControl
              .withVelocity(m_desiredDriveSpeed)
              .withFeedForward(
                  (m_desiredDriveSpeed / DriveConstants.MAX_VELOCITY_RPS_EMPIRICAL)
                      * 12)); // TODO verify that this feedforward is effective
    }
    m_steerMotor.setControl(m_steerControl.withPosition(m_desiredSteerAngle));

  }
}
