// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.helpers.MotorLogger;

import static frc.robot.settings.Constants.ClimberConstants.*;
import static frc.robot.settings.Constants.DriveConstants.CANIVORE_DRIVETRAIN;
import static frc.robot.settings.Constants.ElevatorConstants.ELEVATOR_MOTOR_2_ID;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class ClimberSubsystem extends SubsystemBase {
  TalonFX climberMotor1;
  MotorLogger pulleyMotorLogger;
  MotorLogger wheelMotorLogger;
  Servo climberServo;
  SparkMax climberWheels;
  SparkMaxConfig climberWheelsConfig;
  int loops = 0;
  /** Creates a new CimberSubsystem. */
  public ClimberSubsystem() {
    climberMotor1 = new TalonFX(CLIMBER_MOTOR_ID, CANIVORE_DRIVETRAIN);
    climberServo = new Servo(SERVO_CHANNEL);
    climberWheels = new SparkMax(CLIMBER_WHEELS_MOTOR_ID, MotorType.kBrushless);
    //TODO spend some time figuring out how to use the absolute encoder with the motor.

    climberWheelsConfig = new SparkMaxConfig();
    climberWheelsConfig.smartCurrentLimit(CLIMBER_WHEELS_CURRENT_LIMIT, CLIMBER_WHEELS_CURRENT_LIMIT, CLIMBER_WHEELS_RPM_LIMIT);
    climberWheels.configure(climberWheelsConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    //configures sensor and motor based on whether we are on the practice bot or competition bot
    if(Preferences.getBoolean("CompBot", true)) {
      climberMotor1.getConfigurator().apply(ClimberMotorConfigComp);
      encoderConfig.MagnetSensor.MagnetOffset = COMP_ENCODER_OFFSET;
    } else {
      climberMotor1.getConfigurator().apply(ClimberMotorConfigPrac);
      encoderConfig.MagnetSensor.MagnetOffset = PRAC_ENCODER_OFFSET;
    }
    encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    if(Preferences.getBoolean("Elevator", false)) {
      climberMotor1.getConfigurator().apply(new HardwareLimitSwitchConfigs()
        .withForwardLimitRemoteTalonFX(new TalonFX(ELEVATOR_MOTOR_2_ID, CANIVORE_DRIVETRAIN))
        .withForwardLimitEnable(true)
        .withForwardLimitAutosetPositionEnable(false)
        .withForwardLimitType(ForwardLimitTypeValue.NormallyClosed)
        .withReverseLimitEnable(true)
        .withReverseLimitAutosetPositionEnable(false)
        .withReverseLimitRemoteTalonFX(new TalonFX(ELEVATOR_MOTOR_2_ID, CANIVORE_DRIVETRAIN))
        .withReverseLimitType(ReverseLimitTypeValue.NormallyClosed));
    }

    pulleyMotorLogger = new MotorLogger("/climber/pulleymotor");
    wheelMotorLogger = new MotorLogger("/climber/wheelmotor");
  }
  /**
   * takes a desired angle and moves climberMotor1 to it
   * @param angle the desired angle
   */
  public void stopClimber(){
    climberMotor1.set(0);
  }
  public void setClimberPower(double power) {
    climberMotor1.set(power);
  }
  public void setClimberPower(DoubleSupplier power) {
    climberMotor1.set(power.getAsDouble());
  }
  private void logMotors(){
    wheelMotorLogger.log(climberMotor1);
    pulleyMotorLogger.log(climberWheels);
  }
  public void setMotorTorqueFOC(double current) {
    climberMotor1.setControl(new TorqueCurrentFOC(current));
  }
  /**
   * sets the climberServo to one of two states
   * @param state true is when the rachet is active, false is when it is not
   */
  public void setServo(BooleanSupplier rachet){
    if(rachet.getAsBoolean()){
      climberServo.setAngle(CLIMBER_RACHET_TRUE);
    }
    else {
      climberServo.setAngle(CLIMBER_RACHET_FALSE);
    }
  }

  public void runWheels(double speed){
    climberWheels.set(speed);
  }
  public void runWheels(DoubleSupplier speed){
    climberWheels.set(speed.getAsDouble());
  }

  public void stopWheels(){
    climberWheels.set(0);
  }

  public void powerCheck() {

    SmartDashboard.putNumber("AlgaeMotorCurrent", climberWheels.getOutputCurrent());
    // if we are at 80%+ percent of the current limit, assume it's becuse we have an
    // algae
    if (climberWheels.getOutputCurrent() > CLIMBER_WHEELS_CURRENT_LIMIT * 0.95) {
      loops++;
      if (loops > 10) {
        RobotState.getInstance().climberIn = true;
      }
    } else {
      RobotState.getInstance().climberIn = false;
      loops = 0;
    }
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(Preferences.getBoolean("Motor Logging", false)){
      logMotors();
    }
    powerCheck();
    SmartDashboard.putNumber("servo angle", climberServo.getAngle());
    if(!RobotState.getInstance().climberIn && RobotState.getInstance().funnelDown){
      runWheels(0.25);
    }
  }
}
