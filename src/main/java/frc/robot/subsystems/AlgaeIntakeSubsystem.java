// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants.AlgaeIntakeConstants;

public class AlgaeIntakeSubsystem extends SubsystemBase {
  TalonFX algaeIntake1;
  TalonFX algaeIntake2;
  /** Creates a new AlgaeIntakeSubsystem. */
  public AlgaeIntakeSubsystem() {
    algaeIntake1 = new TalonFX(AlgaeIntakeConstants.ALGAE_INTAKE_MOTOR_1_ID);
    algaeIntake2 = new TalonFX(AlgaeIntakeConstants.ALGAE_INTAKE_MOTOR_2_ID);
  }

 public void runAlgaeIntake(double speed){
  algaeIntake1.set(speed);
  algaeIntake2.set(speed);
 }
 public void stopAlgaeIntake(double speed){
  algaeIntake1.set(0);
  algaeIntake2.set(0);
 }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
