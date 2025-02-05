// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.helpers.MotorLogger;
import frc.robot.settings.ElevatorEnums;

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
    if (Preferences.getBoolean("CompBot", true)){  
      eleMotorConfig = new TalonFXConfiguration()
      .withSlot0(new Slot0Configs()
        .withKP(1)
        .withKS(0)
        .withKA(0)
        .withKV(0))
      .withCurrentLimits(new CurrentLimitsConfigs()
        .withSupplyCurrentLimit(100)
        .withSupplyCurrentLimitEnable(true));
      //We are not yet sure on whether or not we are using MotionMagic.
        //.withMotionMagic(new MotionMagicConfigs()
        //.withMotionMagicAcceleration(2491)
        //.withMotionMagicCruiseVelocity(2491)
        //.withMotionMagicJerk(2491));
    } else {
      eleMotorConfig = new TalonFXConfiguration()
      .withSlot0(new Slot0Configs()
        .withKP(1)
        .withKS(0)
        .withKA(0)
        .withKV(0))
      .withCurrentLimits(new CurrentLimitsConfigs()
        .withSupplyCurrentLimit(100)
        .withSupplyCurrentLimitEnable(true));
      //We are not yet sure on whether or not we are using MotionMagic.
        //.withMotionMagic(new MotionMagicConfigs()
        //.withMotionMagicAcceleration(2491)
        //.withMotionMagicCruiseVelocity(2491)
        //.withMotionMagicJerk(2491));
    }
    elevatorMotor1.getConfigurator().apply(eleMotorConfig);
    elevatorMotor2.getConfigurator().apply(eleMotorConfig);
    elevatorMotor2.setControl(new Follower(ELEVATOR_MOTOR_1_ID, true));

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
  }
  /**
   * Creates a zero from input
   * @param theDistance the distance that the distance sensor at the bottom of the elevator reads
   */
  public void setZero(double theDistance){//Replace with sensor return
    double rof0 = theDistance * ELEVATOR_MILLIMETERS_TO_ROTATIONS;
    zeroPoint = elevatorMotor1.getPosition().getValueAsDouble() - rof0;    
    }
  /**
   * Sets the elevator to a position relative to the 0 set by createZero. 
   * @param height double that controls how many millimeters from the distance sensor
   */
  public void setElevatorPosition(double height){
    double position = height * ELEVATOR_MILLIMETERS_TO_ROTATIONS;
    double uPos = position + zeroPoint;
    PositionVoltage voltReq = new PositionVoltage(0);
    elevatorMotor1.setControl(voltReq.withPosition(uPos));
  }
  public void setElevatorPosition(ElevatorEnums height){
    switch(height){
      case Reef1:
        setElevatorPosition(REEF_LEVEL_1_MILLIMETERS);
        break;
      case Reef2:
        setElevatorPosition(REEF_LEVEL_2_MILLIMETERS);
        break;
      case Reef3:
        setElevatorPosition(REEF_LEVEL_3_MILLIMETERS);
        break;
      case Reef4:
        setElevatorPosition(REEF_LEVEL_4_MILLIMETERS);
        break;
      case HumanPlayer:
        setElevatorPosition(HUMAN_PLAYER_STATION_MILLIMETERS);
        break;
      case Bottom:
        setElevatorPosition(BOTTOM_MILLIMETERS);
        break;
      case AlgaeInProcessor:
        setElevatorPosition(PROCESSOR_HEIGHT_MILLIMETERS);
        break;
      case Barge:
        setElevatorPosition(BARGE_SHOOT_MILLIMETERS);
        break;
    }
  }
  public boolean isElevatorAtPose() {
    return elevatorMotor1.getClosedLoopError().getValueAsDouble() < ELEVATOR_THRESHOLD;
  }
  public void stopElevator(){
    elevatorMotor1.set(0);
  }

}
