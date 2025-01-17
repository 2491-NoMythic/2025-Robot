// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.settings.Constants.CoralEndeffectorConstants.*;

public class CoralEndeffectorSubsystem extends SubsystemBase {
  TalonFX coralEndDefectorMotor;

  /** Creates a new CoralEndDefectorSubsystem. */
  public CoralEndeffectorSubsystem() {
    coralEndDefectorMotor = new TalonFX(CORAL_ENDEFFECTOR_MOTOR_1_ID);
    coralEndDefectorMotor.getConfigurator().apply(coralMotorConfigs);
  }
  public void runCoralEndDefector(double speed){
    coralEndDefectorMotor.set(speed);

  }
  public void stopCoralEndDefector(){
    coralEndDefectorMotor.set(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
