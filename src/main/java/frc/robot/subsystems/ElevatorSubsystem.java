// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  TalonFX elevatorMotor1;
  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    elevatorMotor1 = new TalonFX(ElevatorConstants.ELEVATOR_MOTOR_1_ID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void runElevator(double speed){
    elevatorMotor1.set(speed);

  }
  public void stopElevator(){
    elevatorMotor1.set(0);
  }
}
