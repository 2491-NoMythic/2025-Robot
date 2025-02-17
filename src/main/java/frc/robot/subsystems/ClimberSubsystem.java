// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkAbsoluteEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.helpers.MotorLogger;

import static frc.robot.settings.Constants.ClimberConstants.*;

public class ClimberSubsystem extends SubsystemBase {
  TalonFX climberMotor1;
  MotorLogger motorLogger1;
  CANcoder climberAngleSensor;
  /** Creates a new CimberSubsystem. */
  public ClimberSubsystem() {
    climberMotor1 = new TalonFX(CLIMBER_MOTOR_ID);
    climberAngleSensor = new CANcoder(CLIMBER_CANCODER_ID);
    //TODO spend some time figuring out how to use the absolute encoder with the motor.
    
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
    climberAngleSensor.getConfigurator().apply(encoderConfig);

    motorLogger1 = new MotorLogger("/climber/motor1");
  }

  public double getClimberAngle() {
    return climberAngleSensor.getAbsolutePosition().getValueAsDouble();
  }
  public void setClimberAngle(double angle) {
    climberMotor1.setControl(new PositionVoltage(angle));
  }
  public void stopClimber(){
    climberMotor1.set(0);
  }
  private void logMotors(){
    motorLogger1.log(climberMotor1);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(Preferences.getBoolean("Motor Logging", false)){
      logMotors();
    }
    if(Math.abs(climberAngleSensor.getAbsolutePosition().getValueAsDouble() - CLIMBER_CLIMBED_ANGLE) < 2) {
      RobotState.getInstance().climbed = true;
    }
  }
}
