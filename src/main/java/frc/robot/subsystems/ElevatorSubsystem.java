// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.helpers.MotorLogger;
import frc.robot.settings.ElevatorEnums;
import frc.robot.subsystems.RobotState;

import static frc.robot.settings.Constants.ElevatorConstants.*;
import edu.wpi.first.wpilibj.Preferences;

public class ElevatorSubsystem extends SubsystemBase {
  private TalonFX elevatorMotor1;
  private TalonFX elevatorMotor2;
  private TalonFXConfiguration eleMotorConfig;
  private double zeroPoint;
  MotorLogger motorLogger1;
  MotorLogger motorLogger2;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    elevatorMotor1 = new TalonFX(ELEVATOR_MOTOR_1_ID);
    elevatorMotor2 = new TalonFX(ELEVATOR_MOTOR_2_ID);
    eleMotorConfig = new TalonFXConfiguration()
      .withCurrentLimits(new CurrentLimitsConfigs()
        .withSupplyCurrentLimit(30)
        .withSupplyCurrentLimitEnable(true))
      .withMotionMagic(new MotionMagicConfigs()
        .withMotionMagicAcceleration(MOTION_MAGIC_ELEVATOR_ACCLERATION)
        .withMotionMagicCruiseVelocity(MOTION_MAGIC_ELEVATOR_VELOCITY)
        .withMotionMagicJerk(MOTION_MAGIC_ELEVATOR_JERK));
    if (Preferences.getBoolean("CompBot", true)){  
      eleMotorConfig.Slot0 = new Slot0Configs()
        .withKP(0)
        .withKG(0)
        .withKA(0)
        .withKV(0);
    } else {
      eleMotorConfig = new TalonFXConfiguration()
      .withSlot0(new Slot0Configs()
        .withKP(0)
        .withKG(0)
        .withKA(0)
        .withKV(0));
    }
    elevatorMotor1.getConfigurator().apply(eleMotorConfig);
    elevatorMotor2.setControl(new Follower(ELEVATOR_MOTOR_1_ID, false));

    motorLogger1 = new MotorLogger("/elevator/motor1");
    motorLogger2 = new MotorLogger("/elevator/motor2");
  }
  private void logMotors(){
    motorLogger1.log(elevatorMotor1);
    motorLogger2.log(elevatorMotor2);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(Preferences.getBoolean("Motor Logging", false)){
    logMotors();
    }
    if(elevatorMotor1.getForwardLimit().getValueAsDouble() > 0.1){
      setZero(HEIGHT_AT_LIMIT_SWITCH);
      RobotState.getInstance().elevatorZeroSet = true;
    }
  }
  /**
   * tells the elevator motor what rotations it will have to reach for the elevator to be touching the ground (this will never happen, just theoritically) <p>
   * this is necessary so that the elevator has a reference point to calculate the position of any height off the ground. Run this before ever setting the elevator to a position
   * @param theDistance the distance that the elevator is from the ground, in millimeters
   */
  public void setZero(double theDistance){
    double rotationsFromGround = theDistance * ELEVATOR_MILLIMETERS_TO_ROTATIONS;
    zeroPoint = elevatorMotor1.getPosition().getValueAsDouble() - rotationsFromGround;   
    }
  /**
   * Makes the elevator move to a position relative to the ground. It does this by changing the setpoint for the motor's onboard PID controller
   * @param height the desired height, in millimeters off the ground
   */
  public void setElevatorPosition(double height){
    double targetRotations = calculateRotations(height);
    MotionMagicVoltage request = new MotionMagicVoltage(targetRotations);
    elevatorMotor1.setControl(request);
  }
  public void setElevatorPositionDynamicConfigs(double height, double acceleration, double velocity, double jerk) {
    double targetRotations = calculateRotations(height);
    DynamicMotionMagicVoltage request = new DynamicMotionMagicVoltage(targetRotations, velocity, acceleration, jerk);
    elevatorMotor1.setControl(request);
  }
  /**
   * sets the height of the elvator using constants associated with different values of ElevatorEnums
   * @param height
   */
  public void setElevatorPosition(ElevatorEnums height){
    switch(height){
      case Reef1:
        setElevatorPosition(REEF_LEVEL_1_MILLIMETERS);
        break;
      case Reef2:
        setElevatorPosition(REEF_LEVEL_2_MILLIMETERS);
        if(elevatorMotor1.getClosedLoopError().getValueAsDouble() < ELEVATOR_THRESHOLD){
          RobotState.getInstance().elevatorIsHigh = true;
        }
        break;
      case Reef3:
        setElevatorPosition(REEF_LEVEL_3_MILLIMETERS);
        if(elevatorMotor1.getClosedLoopError().getValueAsDouble() < ELEVATOR_THRESHOLD){
          RobotState.getInstance().elevatorIsHigh = true;
        }
        break;
      case Reef4:
        setElevatorPosition(REEF_LEVEL_4_MILLIMETERS);
        if(elevatorMotor1.getClosedLoopError().getValueAsDouble() < ELEVATOR_THRESHOLD){
          RobotState.getInstance().elevatorIsHigh = true;
        }
        break;
      case HumanPlayer:
        setElevatorPosition(HUMAN_PLAYER_STATION_MILLIMETERS);
        if(elevatorMotor1.getClosedLoopError().getValueAsDouble() < ELEVATOR_THRESHOLD){
          RobotState.getInstance().elevatorIsHigh = false;
        }
        break;
      case Bottom:
        setElevatorPosition(HEIGHT_AT_LIMIT_SWITCH);
        break;
      case AlgaeInProcessor:
        setElevatorPosition(PROCESSOR_HEIGHT_MILLIMETERS);
        break;
      case Barge:
        setElevatorPosition(BARGE_SHOOT_MILLIMETERS);
        break;
    }
  }
  /**
   * calculate the target rotations for the motor based on a desired height off the ground
   * @param desiredHeight height off the ground, in millimeters
   * @return the taret position for the motor, in rotations
   */
  private double calculateRotations(double desiredHeight) {
    return (desiredHeight*ELEVATOR_MILLIMETERS_TO_ROTATIONS) + zeroPoint;
  }
  /**
   * asks if the error on the closed loop is less than our ELEVATOR_THRESHOLD constant
   * @return true if closed loop error is less than our threshold, false otherwise
   */
  public boolean isElevatorAtPose() {
    return elevatorMotor1.getClosedLoopError().getValueAsDouble() < ELEVATOR_THRESHOLD;
  }
  /**
   * the current reference for the onboard ClosedLoopController
   * @return the reference, in rotations
   */
  public double getPIDTarget() {
    return elevatorMotor1.getClosedLoopReference().getValueAsDouble();
  }
  /**
   * stops the elevator by setting it's target to wherever it is right now
   */
  public void stopElevator(){
    elevatorMotor1.setControl(new PositionVoltage(elevatorMotor1.getPosition().getValueAsDouble()));
  }

}
