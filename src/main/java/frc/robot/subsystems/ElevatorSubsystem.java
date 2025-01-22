// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.settings.Constants.ElevatorConstants.*;
import edu.wpi.first.math.geometry.Rotation2d;

public class ElevatorSubsystem extends SubsystemBase {
  private TalonFX elevatorMotor1;
  private TalonFX elevatorMotor2;
  private TalonFXConfiguration eleMotorConfig;
  private double zeroPoint;
  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    elevatorMotor1 = new TalonFX(ELEVATOR_MOTOR_1_ID);
    elevatorMotor2 = new TalonFX(ELEVATOR_MOTOR_2_ID);

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
      elevatorMotor1.getConfigurator().apply(eleMotorConfig);
    elevatorMotor2.getConfigurator().apply(eleMotorConfig);
    elevatorMotor2.setControl(new Follower(ELEVATOR_MOTOR_1_ID, true));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
  public void stopElevator(){
    elevatorMotor1.set(0);
  }

}
