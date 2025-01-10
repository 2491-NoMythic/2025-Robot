// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants.AlgaeEndDefectorConstants;

public class AlgaeEndDefectorSubsystem extends SubsystemBase {
  TalonFX algaeEndDefectorMotor;
  /** Creates a new AlgaeEndDefectorSubsystem. */
  public AlgaeEndDefectorSubsystem() {
    algaeEndDefectorMotor = new TalonFX(AlgaeEndDefectorConstants.ALGAE_END_DEFECTOR_MOTOR_1_ID);
  }

  public void runAlgaeEndDefector(double speed){
    algaeEndDefectorMotor.set(speed);

  }
  public void stopAlgaeEndDefector(double speed){
    algaeEndDefectorMotor.set(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
