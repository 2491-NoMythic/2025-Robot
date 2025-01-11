// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants.ClimberConstants;

public class CimberSubsystem extends SubsystemBase {
  TalonFX climberMotor1;
  TalonFX climberMotor2;
  PIDController climbController;
  /** Creates a new CimberSubsystem. */
  public CimberSubsystem() {
    climberMotor1 = new TalonFX(ClimberConstants.CLIMBER_MOTOR_1_ID);
    climberMotor2 = new TalonFX(ClimberConstants.CLIMBER_MOTOR_2_ID);
    climbController = new PIDController(ClimberConstants.CLIMBER_KP, ClimberConstants.CLIMBER_KI, ClimberConstants.CLIMBER_KD);
  }
  public void runClimber(double speed){
    climberMotor1.set(speed);
    climberMotor2.set(speed);
  }
  public void stopClimber(){
    climberMotor1.set(0);
    climberMotor2.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
