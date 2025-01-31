// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.revrobotics.spark.SparkAbsoluteEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.helpers.MotorLogger;

import static frc.robot.settings.Constants.ClimberConstants.*;

public class CimberSubsystem extends SubsystemBase {
  TalonFX climberMotor1;
  Servo climberServo;
  MotorLogger motorLogger1;
  /** Creates a new CimberSubsystem. */
  public CimberSubsystem() {
    climberMotor1 = new TalonFX(CLIMBER_MOTOR_1_ID);
    climberMotor1.getConfigurator().apply(ClimberMotorConfig);
    climberServo = new Servo(CLIMBER_SERVO_ID);
//TODO spend some time figuring out how to use the absolute encoder with the motor.
    FeedbackConfigs krakenSensorConfigs = new FeedbackConfigs()
      .withFeedbackRemoteSensorID(2491)
      .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
      .withFeedbackRotorOffset(2491);
    climberMotor1.getConfigurator().apply(krakenSensorConfigs);

    motorLogger1 = new MotorLogger("/climber/motor1");
  }
  public void setKrakenPose(double angle) {
    climberMotor1.setControl(new PositionVoltage(angle));
  }
  public void runClimber(double speed){
    climberMotor1.set(speed);
  }
  public void stopClimber(){
    climberMotor1.set(0);
  }
  /**
   * sets the servo to an angle between 0 and 180
   * If angle is lower than 0, the angle will be set to 0. If greater than 180, the angle will be set to 180
   * @param angle
   */
  public void setServo(double angle) {
    climberServo.setAngle(angle);
  }
  private void logMotors(){
    motorLogger1.log(climberMotor1);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    logMotors();
  }
}
