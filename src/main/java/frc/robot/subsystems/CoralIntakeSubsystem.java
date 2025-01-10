// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants.CoralIntakeConstants;

public class CoralIntakeSubsystem extends SubsystemBase {
  TalonFX coralIntake1;
  TalonFX coralIntake2;

  /** Creates a new Intake. */
  public CoralIntakeSubsystem() {
    coralIntake1 = new TalonFX(CoralIntakeConstants.CORAL_INTAKE_MOTOR_1_ID);
    coralIntake2 = new TalonFX(CoralIntakeConstants.CORAL_INTAKE_MOTOR_2_ID);

  }
  public void runCoralIntake(double speed){
    coralIntake1.set(speed);
    coralIntake2.set(speed);

  }
  public void stopCoralIntake(double speed){
    coralIntake1.set(0);
    coralIntake2.set(0);

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
}
